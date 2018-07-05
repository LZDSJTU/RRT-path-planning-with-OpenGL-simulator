#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <fstream>
#include <vector>
#include <math.h>
#include <time.h>
#include <string>
#include <string.h>
#include "SOIL.h"

#include "rrt_drawing.h"
RRTdrawing rrt;

using namespace std;
using namespace cv;

#define FPS 50

const float PI = 3.1415926f;
const float HPI= PI / 2.0f;
const float SIZE_X = 40.0;
const float SIZE_Y = 30.0;
const float WATCH_Z = 40.0;
const float CURVE_R2G_X = SIZE_X / 800;
const float CURVE_R2G_Y = -SIZE_Y / 600;

const float EYESPD = 0.5;
const float EYETURN= PI / 40;
const float EYEMAX_X = SIZE_X * 1.1;
const float EYEMAX_Y = SIZE_Y * 0.1;
const float EYEMAX_Z = 45.0f;
const float EYEMIN_X = -SIZE_X * 0.1;
const float EYEMIN_Y = SIZE_Y * -1.1;
const float EYEMIN_Z = 15.0f;

const int BARHIGH = 4;

int counts = 0;
float movetime = 0;
float timeStep = 1.0f;

typedef struct vec3f_s {
	union {
		struct {
			float w;
			float h;
			float th;
		};
		struct {
			float x;
			float y;
			float z;
		};
		struct {
			float Y;
			float P;
			float R;
		};
		float v[3];
	};

	vec3f_s operator + (const vec3f_s a) const {
		vec3f_s temp = {a.x + x, a.y + y, a.z + z};
		return temp;
	}

	vec3f_s operator - (const vec3f_s a) const {
		vec3f_s temp = {a.x - x, a.y - y, a.z - z};
		return temp;
	}

	vec3f_s operator * (const vec3f_s a) const {
		vec3f_s temp = {a.x * x, a.y * y, a.z * z};
		return temp;
	}

	vec3f_s operator * (const float num) const {
		vec3f_s temp;
		temp.x = x * num;
		temp.y = y * num;
		temp.z = z * num;
		return temp;
	}

	vec3f_s operator / (const float num) const {
		vec3f_s temp;
		temp.x = x / num;
		temp.y = y / num;
		temp.z = z / num;
		return temp;
	}

	friend ostream &operator << (ostream &os, const vec3f_s v) {
		os << v.x << "\t" << v.y << "\t" << v.z << "\n";
		return os;
	}
}v3f;

v3f v3fTurnXOY (v3f a, float th) {
	v3f temp = {a.x * cosf(th) - a.y * sinf(th), 
				a.x * sinf(th) + a.y * cosf(th), a.z};
	return temp;
}

v3f vzp = {0,0,1};
v3f vzn = {0,0,-1};
v3f vxp = {1,0,0};
v3f vxn = {-1,0,0};
v3f vyp = {0,1,0};
v3f vyn = {0,-1,0};

v3f Middle = {SIZE_X / 2, -SIZE_Y / 2, 0};

v3f eye;
v3f eye_dir;
v3f eye_ang;

vector<Eigen::Vector2d> curve;

float calLength() {
	double len;
	for (int i = 1; i < curve.size(); i++) {
		double x2 = (curve[i](0) - curve[i-1](0)) * (curve[i](0) - curve[i-1](0));
		double y2 = (curve[i](1) - curve[i-1](1)) * (curve[i](1) - curve[i-1](1));
		len += sqrt(x2 + y2);
	}
	cout << "The length is " << len << endl;
	return len;
}

void curveProcess () {
	curve.clear();
	curve = rrt.BezierCurvePointCallback();
	timeStep = curve.size() / (calLength() / 128.0f * FPS);
	float timeTake = curve.size() / timeStep / FPS;
	cout << "It will take " << timeTake << " seconds" << endl;
	for (int i = 0; i < curve.size(); i++) {
		curve[i](0) *= CURVE_R2G_X;
		curve[i](1) *= CURVE_R2G_Y;
	}
}

typedef struct barrier_s {
	float px;
	float py;
	int sx;
	int sy;
}barrier;

vector<barrier> bars;

void buildbar(float px, float py, float sx, float sy) {
	barrier bar;
	bar.px = px;
	bar.py = -py;
	bar.sx = sx;
	bar.sy = sy;
	bars.insert(bars.end(), bar);
}

// Mat barMat(30, 40, CV_8UC3, Scalar(255,255,255));

void buildMat() {
	Mat barMat(30, 40, CV_8UC3, Scalar(255,255,255));
	for (int i = 0; i < bars.size(); i++) {
		Mat Obstacle(bars[i].sy, bars[i].sx,CV_8UC3, Scalar(0,0,0));
		Mat imageROI = barMat(Rect(bars[i].px - 0.5, -bars[i].py - 0.5, Obstacle.cols, Obstacle.rows));
		Mat mask(Obstacle.size(),CV_8UC1, 1);
		Obstacle.copyTo(imageROI,mask);
	}
	// barMat = imread("../pic/Obstacle_resize.jpg");
	rrt.ReadObstacleFile(barMat);
	imshow("GridMap", barMat);
	waitKey(3000); 
}

enum Mode {PAUSE, EDITING, RUNNING};
Mode mode;

GLuint tex_2d0;
GLuint tex_2d1;
GLuint tex_2d2;
GLuint tex_floor;

GLUquadric* qobj;

void timerFPS(int id) {
	glutTimerFunc(1000 / FPS,timerFPS,1);
	glutPostRedisplay();
}

void drawQuad(v3f pos, v3f dir, v3f size) {
	size.w /= 2;
	size.h /= 2;

	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_QUADS);

	if (dir.x == 0.0f && dir.y == 0.0f) {
		float a = size.th;
		glTexCoord2f(0, 0);
		glVertex3f(	pos.x + (-size.w * cosf(a) + -size.h *-sinf(a)),
					pos.y + (-size.w * sinf(a) + -size.h * cosf(a)), pos.z);
		glTexCoord2f(1, 0);
		glVertex3f(	pos.x + ( size.w * cosf(a) + -size.h *-sinf(a)),
					pos.y + ( size.w * sinf(a) + -size.h * cosf(a)), pos.z);
		glTexCoord2f(1, 1);
		glVertex3f(	pos.x + ( size.w * cosf(a) + size.h *-sinf(a)),
					pos.y + ( size.w * sinf(a) + size.h * cosf(a)), pos.z);
		glTexCoord2f(0, 1);
		glVertex3f(	pos.x + (-size.w * cosf(a) + size.h *-sinf(a)),
					pos.y + (-size.w * sinf(a) + size.h * cosf(a)), pos.z);
	} else if (dir.z == 0) {
		float a = atan2f(dir.y, dir.x);
		float arm = size.w;
		glTexCoord2f(0, 0);
		glVertex3f(	pos.x + arm * sinf(a), pos.y - arm * cosf(a), pos.z - size.h);
		glTexCoord2f(1, 0);
		glVertex3f(	pos.x - arm * sinf(a), pos.y + arm * cosf(a), pos.z - size.h);
		glTexCoord2f(1, 1);
		glVertex3f(	pos.x - arm * sinf(a), pos.y + arm * cosf(a), pos.z + size.h);
		glTexCoord2f(0, 1);
		glVertex3f(	pos.x + arm * sinf(a), pos.y - arm * cosf(a), pos.z + size.h);
	} else {
		//TODO if needed + draw normal Quad
	}
	glEnd();
}

void drawQuad(int texture_id, v3f pos, v3f dir, v3f size) {
	glEnable( GL_TEXTURE_2D );
	glBindTexture ( GL_TEXTURE_2D, texture_id );
	glColor3f(1.0f, 1.0f, 1.0f);
	drawQuad(pos, dir, size);
	glDisable( GL_TEXTURE_2D );
}

void drawQuad ( float px, float py, float pz,
				float dx, float dy, float dz,
				float sw, float sh, float th) {
	v3f pos = {px, py, pz};
	v3f dir = {dx, dy, dz};
	v3f size= {sw, sh, th};
	drawQuad(pos, dir, size);
}

void drawQuad ( int texture_id,
				float px, float py, float pz,
				float dx, float dy, float dz,
				float sw, float sh, float th) {
	v3f pos = {px, py, pz};
	v3f dir = {dx, dy, dz};
	v3f size= {sw, sh, th};
	drawQuad(texture_id, pos, dir, size);
}

void drawBox ( int texture_id, v3f pos, float size, float th) {
	//TODO change size if needed
	float hs = size / 2;
	v3f size_para = {size, size, th};
	v3f pos_para;
	v3f ori_para;
	drawQuad(texture_id, pos, vzn, size_para);
	pos.z += size;
	drawQuad(texture_id, pos, vzp, size_para);
	pos.z -= hs;
	ori_para = v3fTurnXOY(vxp, th) * hs;
	pos_para = pos + ori_para;
	drawQuad(texture_id, pos_para, ori_para, size_para);
	ori_para = v3fTurnXOY(vxn, th) * hs;
	pos_para = pos + ori_para;
	drawQuad(texture_id, pos_para, ori_para, size_para);
	ori_para = v3fTurnXOY(vyp, th) * hs;
	pos_para = pos + ori_para;
	drawQuad(texture_id, pos_para, ori_para, size_para);
	ori_para = v3fTurnXOY(vyn, th) * hs;
	pos_para = pos + ori_para;
	drawQuad(texture_id, pos_para, ori_para, size_para);
}

void drawBox ( int texture_id, float px, float py, float pz, float size, float th) {
	v3f pos_para = {px, py, pz};
	drawBox(texture_id, pos_para, size, th);
}

void drawBarrier (int texture) {
	for (int i = 0; i < bars.size(); i++) {
		float x1 = bars[i].px;
		float y1 = bars[i].py;
		for (int xi = 0; xi < bars[i].sx; xi++) {
			for (int yi = 0; yi < bars[i].sy; yi++) {
				for (int zi = 0; zi < BARHIGH; zi ++) {
					drawBox(texture, x1 + xi, y1 - yi, zi, 1, 0);
				}
			}
		}
	}
}

void eyeInit() {
	eye.x = Middle.x;
	eye.y = - SIZE_Y * 19 / 20;
	eye.z = WATCH_Z;
	eye_ang.Y = PI / 2;
	eye_ang.P = - HPI + PI / 10;
}

void renderScene(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if (eye_ang.P == -HPI) {
		eye_dir.x = 0;
		eye_dir.y = 0;
		eye_dir.z = -1;
	} else {
		eye_dir.x = cosf(eye_ang.Y);
		eye_dir.y = sinf(eye_ang.Y);
		eye_dir.z = -tanf(-eye_ang.P) * sqrt( eye_dir.x * eye_dir.x + eye_dir.y * eye_dir.y);
	}
	// cout << eye << eye_dir << endl << tanf(-eye_ang.P) << endl << eye_ang.P << endl;
	v3f looked = eye + eye_dir;
	gluLookAt( 	eye.x, eye.y, eye.z,
				looked.x, looked.y, looked.z,
				// head_dir.x, head_dir.y, head_dir.z);
				cosf(eye_ang.Y), sinf(eye_ang.Y), 0);

	drawQuad ( tex_floor,
			Middle.x, Middle.y, Middle.z,
			0, 0, 1, 
			// SIZE_X, SIZE_Y, (counts++)*PI/25);
			SIZE_X, SIZE_Y, 0);
	
	drawBarrier(tex_2d2);

	if (mode != EDITING) {
		float angle;
		if (counts == 0) {
			angle = atan2f(curve[1](1) - curve[0](1), curve[1](0) - curve[0](0));
		} else {
			angle = atan2f(curve[counts](1) - curve[counts - 1](1), curve[counts](0) - curve[counts - 1](0));
		}
		drawBox(tex_2d1, curve[counts](0), curve[counts](1), 0, 1, angle);
		if (mode == RUNNING) {
			movetime += timeStep;
			counts = (int)movetime;
			if (counts >= curve.size()) {
				counts = 0;
				movetime = 0.0f;
			}
		}
	} else {
		movetime = 0;
		counts = 0;
	}

	glutSwapBuffers(); 
}

int motW = 800;
int motH = 600;
void changeSize(int w, int h) {
	if (h == 0) {
		h = 1;
	}
	motW = w;
	motH = h;
	float ratio = 1.0* w / h;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);
	gluPerspective(45, ratio, 0.1, 10000);
	glMatrixMode(GL_MODELVIEW);
}

void loadTextures(GLuint *id, const char* path) {
	glGenTextures(1, id);
    *id = SOIL_load_OGL_texture
    (
    	path,
    	SOIL_LOAD_AUTO,
    	SOIL_CREATE_NEW_ID,
		SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_COMPRESS_TO_DXT);
    
    if (*id==0){
		cout << "fatal error while loading the texture" << endl << path << endl;
	}
}

void LoadGLTextures() {
	glEnable( GL_TEXTURE_2D );
	loadTextures(&tex_2d0, "../pic/T1.png");
	loadTextures(&tex_2d1, "../pic/T2.png");
	loadTextures(&tex_2d2, "../pic/T3.png");
	loadTextures(&tex_floor, "../pic/floor.jpg");
	glDisable(GL_TEXTURE_2D );
}

bool blOnEditing = false;
void keyPressed(unsigned char key, int x, int y) {

	if (key == 27){
		exit(0);
	}

	if (mode != EDITING) {
		if (key == 'w' || key == 'W') {
			eye.x += EYESPD * cosf(eye_ang.Y);
			eye.y += EYESPD * sinf(eye_ang.Y);
		} 
		else if (key == 's' || key == 'S') {
			eye.x -= EYESPD * cosf(eye_ang.Y);
			eye.y -= EYESPD * sinf(eye_ang.Y);
		} 
		else if (key == 'a' || key == 'A') {
			eye.x -= EYESPD * cosf(eye_ang.Y - HPI);
			eye.y -= EYESPD * sinf(eye_ang.Y - HPI);
		} 
		else if (key == 'd' || key == 'D') {
			eye.x += EYESPD * cosf(eye_ang.Y - HPI);
			eye.y += EYESPD * sinf(eye_ang.Y - HPI);
		} 
		else if (key == 'q' || key == 'Q') {
			v3fTurnXOY(eye_dir, EYETURN);
			eye_ang.Y += EYETURN;
		} 
		else if (key == 'e' || key == 'E') {
			v3fTurnXOY(eye_dir, -EYETURN);
			eye_ang.Y -= EYETURN;
		}
		else if (key == 'f' || key == 'F') {
			if (mode == RUNNING) {
				mode = PAUSE;
				cout << "mode: PAUSE\n" << endl;
			} else {
				mode = RUNNING;
				cout << "mode: RUNNING\n" << endl;
			}
		}
	}

	if (mode == EDITING) {
		if (key == 'c' || key == 'C') {
			bars.clear();
		}

		if (key == '+' && !blOnEditing) {
			cout << "adding..." << endl;
			blOnEditing = true;
			buildbar(0.5, 0.5, 1, 1);
		}

		if (key == 13 && blOnEditing) {
			vector<barrier>::iterator b = bars.end() - 1;
			cout << "add complete" << endl;
			cout << "position: ( " << (int)(b->px - 0.5) << " ,\t" << (int)(-b->py - 0.5) << ")" << endl;
			cout << "size:     ( " << b->sx << " ,\t" << b->sy << ")\n" << endl;
			blOnEditing = false;
		}

		if (blOnEditing) {
			vector<barrier>::iterator b = bars.end() - 1;
			if (key == 'w' || key == 'W') {
				if (b -> py < -1) {
					b -> py += 1;
				}
			} 
			else if (key == 's' || key == 'S') {
				if (b->py > -SIZE_Y + b->sy) {
					b->py -= 1;
				}
			} 
			else if (key == 'a' || key == 'A') {
				if (b->px > 1) {
					b->px -= 1;
				}
			} 
			else if (key == 'd' || key == 'D') {
				if (b->px < SIZE_X - b->sx) {
					b->px += 1;
				}
			}
			else if (key == 'y' || key == 'Y') {
				if (b->py - b->sy > -SIZE_Y) {
					b->sy += 1;
				}
			} 
			else if (key == 'h' || key == 'H') {
				if (b->sy > 1) {
					b->sy -= 1;
				}
			}
			else if (key == 'u' || key == 'U') {
				if (b->px + b->sx < SIZE_X) {
					b->sx += 1;
				}
			} 
			else if (key == 'j' || key == 'J') {
				if (b->sx > 1) {
					b->sx -= 1;
				}
			} 
		}

		
	}

	if (key == 'i' || key == 'I') {
		if (mode != EDITING) {
			cout << "mode: EDITING\n" << endl;
			mode = EDITING;
			eyeInit();
		} else {
			cout << "Edit complete\nThere are " << bars.size() << " Obstacles\n" << endl;
			cout << "mode: PAUSE\n" << endl;
			if (blOnEditing) {
				blOnEditing = false;
				bars.erase(bars.end() - 1);
			}
			buildMat();
			curveProcess();
			mode = PAUSE;

		}
	}


	if (eye_ang.Y > 2 * PI)		eye_ang.Y -= 2 * PI;
	if (eye_ang.Y < 0)			eye_ang.Y += 2 * PI;
	if (eye.x > EYEMAX_X) eye.x = EYEMAX_X;
	if (eye.y > EYEMAX_Y) eye.y = EYEMAX_Y;
	if (eye.z > EYEMAX_Z) eye.z = EYEMAX_Z;
	if (eye.x < EYEMIN_X) eye.x = EYEMIN_X;
	if (eye.y < EYEMIN_Y) eye.y = EYEMIN_Y;
	if (eye.z < EYEMIN_Z) eye.z = EYEMIN_Z;
}

int pressOriginX = -1;
int pressOriginY = -1;
float yaw_old;
float pitch_old;
void mousePress(int button, int state, int x, int y) {
	cout << "mousePress:" << endl;
	cout << button << endl;
	cout << state << endl;
	cout << x << endl;
	cout << y << endl << endl;
	if (mode != EDITING) {
		if (button == 1) {
			if (state == 0) {
				pressOriginX = x;
				pressOriginY = y;
				yaw_old = eye_ang.Y;
				pitch_old = eye_ang.P;
			} else {
				pressOriginX = -1;
				pressOriginY = -1;
			}
		}
		
		if (button == 3) {
			if (eye.z > EYEMIN_Z) {
				eye = eye + (eye_dir / abs(eye_dir.z));				
			}
		}

		if (button == 4) {
			if (eye.z < EYEMAX_Z) {
				eye = eye - (eye_dir / abs(eye_dir.z));
			}
		}
	}
}

void mouseMove(int x, int y) {
	// cout << "mouseMove:" << endl;
	// cout << x << endl;
	// cout << y << endl << endl;
	if (mode != EDITING) {
		if (pressOriginX != -1) {
			eye_ang.Y = yaw_old + (pressOriginX - x) * PI / 2 / motW;
		}

		if (pressOriginY != -1) {
			float tempPitch = pitch_old + (pressOriginY - y) * PI / 2 / motH;
			if (tempPitch < -HPI) {
				eye_ang.P = -HPI;
			}
			else if (tempPitch < PI / 4) {
				eye_ang.P = tempPitch;
			}
		}
	}
}

void init(){
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_NORMALIZE);
	glLineWidth(1);
	glShadeModel(GL_SMOOTH);
	qobj = gluNewQuadric();
	gluQuadricNormals(qobj, GLU_SMOOTH);

	LoadGLTextures();
	eyeInit();
	mode = PAUSE;
}

int main(int argc, char **argv) {
	buildbar(10.5, 0.5, 2, 15);
	buildbar(25.5, 15.5, 2, 12);
	buildbar(10.5, 10.5, 7, 2);
	buildbar(15.5, 20.5, 12, 2);

	buildMat();
	curveProcess();

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glEnable( GL_CULL_FACE );
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(800, 600);
	glutCreateWindow("find_way");
	
	init();

	glutKeyboardFunc(keyPressed);
	glutMouseFunc(mousePress);
	glutMotionFunc(mouseMove);

	glutDisplayFunc(renderScene);
	glutReshapeFunc(changeSize);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_DEPTH_BUFFER_BIT);

	glutTimerFunc(20,timerFPS,1);
	glutMainLoop();

	return 1;
}

// int main(int argc, char** argv)
// {
//     RRTdrawing rrt;
// //    rrt.ReadObstacleFile(This is your input picture);
//     cv::Mat GridMap = rrt.GridMapCallback();
//     cv::imshow("GridMap",GridMap);
//     cv::waitKey();
// }