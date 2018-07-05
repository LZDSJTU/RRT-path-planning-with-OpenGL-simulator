#include "rrt_drawing.h"

const double VelocityDrawingMultiplier = 12;

using namespace cv;
using namespace RRT;
using namespace Eigen;
using namespace std;

//Mode
// 0：外部读取障碍信息
// 1：自定义障碍
const int Mode = 0;

RRTdrawing::RRTdrawing() {
    Vector2d size(800, 600);
    _stateSpace = make_shared<GridStateSpace>(size.x(), size.y(), 40, 30);
    _biRRT = make_unique<BiRRT<Vector2d>>(_stateSpace, RRT::hash, dimensions);

    //设置初始点与终点位置
    _biRRT->setStartState(size / 10);
//    _biRRT->setGoalState(size / 2);
    _biRRT->setGoalState(Eigen::Vector2d(700,500));
    //设置最大步长
    _biRRT->setMaxStepSize(30);
    //设置终点距离
    _biRRT->setGoalMaxDist(12);

    //朝终点的生长比例
    _biRRT->setGoalBias(0.0);
    _biRRT->setWaypointBias(0.0);
//    _biRRT->setASCEnabled(enabled);
    _biRRT->setStepSize(10);

    //设置初始点速度与终点速度
    _startVel = Vector2d(3, 0);
    _goalVel = Vector2d(0, 3);

    Mat image(600, 800, CV_8UC3, Scalar(255,255,255));
    image.copyTo(GridMap);
}

RRTdrawing::~RRTdrawing() {};

void RRTdrawing::GridMapGenerate() {
    ObstacleGenerate(300,50,Point(200,0));
//    ObstacleGenerate(200,50,Point(500,100));
    ObstacleGenerate(250,50,Point(500,300));
    ObstacleGenerate(50,150,Point(200,200));
    ObstacleGenerate(50,250,Point(300,400));
//    ObstacleGenerate(50,200,Point(500,300));
    imwrite("GridMap.jpg", GridMap);
}

void RRTdrawing::ObstacleGenerate(int height, int width, Point leftup) {
    Mat Obstacle(height,width,CV_8UC3, Scalar(0,255,0));
    Point Point = leftup;
    Mat imageROI = GridMap(cv::Rect(Point.x,Point.y,Obstacle.cols,Obstacle.rows));
    Mat mask(Obstacle.size(),CV_8UC1, 1);
    Obstacle.copyTo(imageROI,mask);
}

void RRTdrawing::GridMapDilate(){
    GridMap.copyTo(_GridMapDilate);
    Mat kernelDilateL = getStructuringElement(MORPH_RECT, Size(40, 40));
    erode(_GridMapDilate, _GridMapDilate, kernelDilateL);
}

void RRTdrawing::GenerateObstacle() {
    cvtColor( _GridMapDilate, _GridMapBinary, CV_RGB2GRAY );
    threshold(_GridMapBinary, _GridMapBinary, 250, 255, THRESH_BINARY);

    for (int row = 0; row<_GridMapDilate.rows; row++) {
        for (int column = 0; column<_GridMapDilate.cols; column++) {
            bool is_occupied;
            is_occupied = _GridMapBinary.at<uchar>(row,column)>100 ? 0 : 1;
            Vector2d pos = Vector2d(column, row);
            Vector2i gridLoc = _stateSpace->obstacleGrid().gridSquareForLocation(pos);
             _stateSpace->obstacleGrid().obstacleAt(gridLoc) = is_occupied;
        }
    }
}

void RRTdrawing::DrawObstacle() {
    int rectW = _stateSpace->obstacleGrid().width() / _stateSpace->obstacleGrid().discretizedWidth(),
            rectH = _stateSpace->obstacleGrid().height() / _stateSpace->obstacleGrid().discretizedHeight();
    for (int x = 0; x < _stateSpace->obstacleGrid().discretizedWidth(); x++) {
        for (int y = 0; y < _stateSpace->obstacleGrid().discretizedHeight(); y++) {
            if (_stateSpace->obstacleGrid().obstacleAt(x, y)) {
                rectangle(GridMap, Point(x * rectW, y * rectH),
                          Point(x * rectW + rectW, y * rectH + rectH), Scalar(0), -1);
            }
        }
    }
}

void RRTdrawing::ObstacleOutput() {
    Mat ObstacleOutput(30,
                       40,
                       CV_8UC1, Scalar::all(255));
    for (int x = 0; x < _stateSpace->obstacleGrid().discretizedWidth(); x++) {
        for (int y = 0; y < _stateSpace->obstacleGrid().discretizedHeight(); y++) {
            if (_stateSpace->obstacleGrid().obstacleAt(x, y)) {
                ObstacleOutput.at<uchar>(y,x) = 0;
            }
        }
    }
    imwrite("Obstacle_resize.png", ObstacleOutput);
}

void RRTdrawing::_step(int numTimes) {
    for (int i = 0; i < numTimes; i++) {
        _biRRT->grow();
    }
    //  store solution
    _previousSolution.clear();
    if (_biRRT->startSolutionNode() != nullptr) {
        cout<<"Successful!"<<endl;
        _previousSolution = _biRRT->getPath();
        RRT::SmoothPath<Vector2d>(_previousSolution, *_stateSpace);
    }
}

void RRTdrawing::drawTree(const Tree<Vector2d>& rrt, const Node_t<Vector2d>* solutionNode) {
    //  node drawing radius
    const double r = 1;

    //  draw all the nodes and connections
    for (const Node_t<Vector2d>& node : rrt.allNodes()) {
        Point loc(node.state().x(), node.state().y());
        circle(GridMap, loc, r, Scalar(0));

        if (node.parent()) {
            //  draw edge
            Point parentloc(node.parent()->state().x(),node.parent()->state().y());
            line(GridMap, loc, parentloc,Scalar(0,0,255));
        }
    }

    //  draw solution
    if (solutionNode) {
        const Node_t<Vector2d>* node = solutionNode,
                * parent = solutionNode->parent();
        while (parent) {
            //  draw the edge
            Point from(node->state().x(), node->state().y());
            Point to(parent->state().x(),parent->state().y());
            line(GridMap, from, to, Scalar(0,0,255),3);

            node = parent;
            parent = parent->parent();
        }
    }

    imshow("GridMap",GridMap);
    waitKey(1);
}

void RRTdrawing::reduceTree(){
    if (_previousSolution.size() > 0) {
        Vector2d prev;
        bool first = true;
        for (const Vector2d& curr : _previousSolution) {
            if (first) {
                first = false;
            } else {
                line(GridMap, Point(prev.x(),prev.y()), Point(curr.x(), curr.y()), Scalar(0,0,255),3);
            }
            prev = curr;
        }
    }
}

void RRTdrawing::smoothCurve() {
    if (_previousSolution.size() > 0) {
        //  draw cubic bezier interpolation of waypoints
        BezierCurvePoint.clear();

        Vector2d prevControlDiff = -_startVel * VelocityDrawingMultiplier;
        for (int i = 1; i < _previousSolution.size(); i++) {
            Vector2d waypoint = _previousSolution[i];
            Vector2d prevWaypoint = _previousSolution[i - 1];

            Vector2d controlDir;
            double controlLength;
            if (i == _previousSolution.size() - 1) {
                controlLength = _goalVel.norm() * VelocityDrawingMultiplier;
                controlDir = -_goalVel.normalized();
            } else {
                Vector2d nextWaypoint = _previousSolution[i + 1];
                controlLength = 0.5 * min((waypoint - prevWaypoint).norm(),
                                          (nextWaypoint - waypoint).norm());
                controlDir =
                        ((prevWaypoint - waypoint).normalized() -
                         (nextWaypoint - waypoint).normalized()).normalized();
            }

            Vector2d controlDiff = controlDir * controlLength;

            BezierControlPoint.clear();
            BezierControlPoint.push_back(prevWaypoint);
            BezierControlPoint.push_back(prevWaypoint - prevControlDiff);
            BezierControlPoint.push_back(waypoint + controlDiff);
            BezierControlPoint.push_back(waypoint);

            Vector2d CurvePoint;
            for(float i = 0 ; i <= 1.00001; i=i+0.01){
                CurvePoint = CalculateBezierPoint(BezierControlPoint, 4, i);
                BezierCurvePoint.push_back(CurvePoint);
            }
            prevControlDiff = controlDiff;
        }
    }

    for(int i = 0; i < BezierCurvePoint.size()-1; i++) {
        line(GridMap, Point(BezierCurvePoint[i].x(),BezierCurvePoint[i].y()), Point(BezierCurvePoint[i+1].x(),BezierCurvePoint[i+1].y()), Scalar(0,0,0),10);
    }
     imshow("GridMap_smooth", GridMap);
}

Vector2d RRTdrawing::CalculateBezierPoint(vector<Vector2d> ControlPoints, int n, float t)
{
    vector<Eigen::Vector2d> NextControlPoints;
    NextControlPoints.resize(ControlPoints.size());
    for(int i = 0; i < ControlPoints.size()-1; i++){
        NextControlPoints[i] =  ControlPoints[i] * (1-t) + ControlPoints[i+1] * t;
    }
    if(n == 2){
        return NextControlPoints[0];
    }
    else{
        return CalculateBezierPoint(NextControlPoints, n-1, t);
    }
}

void RRTdrawing::ReadObstacleFile(Mat obstacle_in) {
//    GridMap_resize = imread("../pic/Obstacle_resize.jpg");
    GridMap_resize = obstacle_in;
   cvtColor( GridMap_resize, GridMap_resize, CV_RGB2GRAY );
    // imshow("GridMap_1", GridMap_resize);
    // waitKey(3000);
    threshold(GridMap_resize, GridMap_resize, 100, 255, THRESH_BINARY);

    int rectW = _stateSpace->obstacleGrid().width() / _stateSpace->obstacleGrid().discretizedWidth(),
            rectH = _stateSpace->obstacleGrid().height() / _stateSpace->obstacleGrid().discretizedHeight();
    for (int y = 0; y < GridMap_resize.rows; y++) {
        for (int x = 0; x < GridMap_resize.cols; x++) {
            if (int(GridMap_resize.at<uchar>(y,x)) == 0) {
                rectangle(GridMap, Point(x * rectW, y * rectH),
                          Point(x * rectW + rectW, y * rectH + rectH), Scalar(0,255,0), -1);
            }
        }
    }

    //设置障碍
    if(Mode) {
        GridMapGenerate();
    }
    GridMapDilate();
    GenerateObstacle();
    if(Mode) {
        ObstacleOutput();
    }
//    DrawObstacle();

    //开始计算
    while(_biRRT->startSolutionNode() == nullptr){
        _step(1);
        drawTree(_biRRT->startTree(), _biRRT->startSolutionNode());
        drawTree(_biRRT->goalTree(), _biRRT->goalSolutionNode());
    }

    reduceTree();
    smoothCurve();
    waitKey();

}

