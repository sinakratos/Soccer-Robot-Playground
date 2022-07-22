#include "World.hpp"
#include <vector>

//---------------------
//--| Model Configs |--
//---------------------
//-- Robot Size (in Meter) :
double robotSize  = 0.18;
double robotLineSize = 2.5;
//-- Window Scale (in Pixels) :
//- for 3K Monitors := 222
//- for 2k Monitors := 120
//- for 1k Monitors := 80
double modelScale = 80;
//-- Font Size Scale :
double fontSize = 1.2;
//-- Lines of Model Scale :
double lineSize = 4;
//-- Blue, Green, Red
double fieldLineBlue = 100;
double fieldLineGreen = 100;
double fieldLineRed = 100;
double fieldBGBlue = 0;
double fieldBGGreen = 0;
double fieldBGRed = 0;
double enemyGoalBlue = 200;
double enemyGoalGreen = 220;
double enemyGoalRed = 0;
double teamGoalBlue = 0;
double teamGoalGreen = 220;
double teamGoalRed = 220;
//-- Refresh Rate (ms)
double refreshRate = 20.0 / 1000.0;
//-- Mathematical 
double rad = 0.0174533;
//-- Mouse Click Flag
int mouseFlag;
//-- Rotation Value with Mouse
double mouseRotationValue = 10;
//-- Click Area Radius
double clickAreaRadius = robotSize * modelScale;
//-- Mouse Distance to Center of Agent
double mouseDistance;
//-- Model Window Size
double modelLength = windowLength * half;
double modelWidth = windowWidth * half;

//---------------------
//--| Robot Configs |--
//---------------------
//-- Sharpness of Robot (in Degree) :
double robotSharpness = 140.0;
//-- Robot Spawining Position (According to Scale in Meter) :
double xSpawn = 0.5;
double ySpawn = 0.5;
double rotationSpawn = 0;
//-- Robot Colors
double robotBlue = 212;
double robotGreen = 255;
double robotRed = 127;
//-- Color Value When Clicked
int clickedColorValue = 0;
//-- Speed Increase/Decrease Value
double moveSpeedValue = 0.2;
double rotateSpeedValue = 0.4;

//-- Spawning Configuration in Constructor
World::World() {
    robot.setPosition(0, 0, M_PI);
    robot.setVelocity(0, 0, 0);
    field.fieldCreate();
    create();
    setMouseCallback("World", mouseAttacher, this);
    mouseFlag = 1;
    robot.storePosition(0, 0, 0, 0, 0, 0);
    updateWindow();
}

//-- Updates Frames of Program
int World::updateWindow() {
    while (true) {
        //-- Update Window Frame's Refresh Rate :
        sleep_for(milliseconds((int)refreshRate * 1000));
        create();
        robot.storePosition(robot.accessX(), robot.accessY(), robot.accessTheta(), robot.accessVX(), robot.accessVY(), robot.accessVTheta());
        if (robot.accessVX() != 0 || robot.accessVY() != 0 || robot.accessVTheta() != 0) {
            //status.updateHelpWindow(true);
        } else {
            //status.updateHelpWindow(false);
        }
        if (robot.borderCheck() == 0) {
            if (robot.state() == 1) {
                status.updateStatus(1, robot.accessVX());
            } else if (robot.state() == 2) {
                status.updateStatus(2, robot.accessVY());
            } else if(robot.state() == 3) {
                status.updateStatus(3, robot.accessVTheta());
            } else if (robot.state() == 0) {
                status.updateStatus(0, 0);
            }
            switch(waitKey(1)) {
                case (int('l')):
                    status.resetError();
                    robot.setVelocity(robot.accessVX(), robot.accessVY(), robot.accessVTheta() + rotateSpeedValue);
                break;
                case (int('j')):
                    status.resetError();
                    robot.setVelocity(robot.accessVX(), robot.accessVY(), robot.accessVTheta() - rotateSpeedValue);
                break;
                case (int('i')):
                    status.resetError();
                    robot.setVelocity(robot.accessVX() + moveSpeedValue, robot.accessVY(), robot.accessVTheta());
                break;
                case (int(';')):
                    status.resetError();
                    robot.setVelocity(robot.accessVX(), robot.accessVY() + moveSpeedValue, robot.accessVTheta());
                break;
                case (int(',')):
                    status.resetError();
                    robot.setVelocity(robot.accessVX() - moveSpeedValue, robot.accessVY(), robot.accessVTheta());
                break;
                case (int('h')):
                    status.resetError();
                    robot.setVelocity(robot.accessVX(), robot.accessVY() - moveSpeedValue, robot.accessVTheta());
                break;
                //-- [j] --> Reset Key
                case (int('k')):
                    robot.setVelocity(0, 0, 0);
                break;
                case (int('r')):
                    robot.setPosition(0, 0, M_PI);
                    robot.setVelocity(0, 0, 0);
                break;
                //-- [Q] --> Quit Key
                case (int('q')):
                    destroyAllWindows();
                    return 0;
                break;

            case (int('b')):
                behaviorMode *= -1;
                break;

            }
            
            if (moveSignal == 1) {

                // vector with 3 items which their values are 0 
                // cout<<"point : "<<-(point.x - (windowLength * modelScale / 2)) / modelScale<<" "<<(point.y - (windowWidth * modelScale / 2)) / modelScale<<endl;
                // cout<<"robot : "<<robot.accessX()<<" "<<robot.accessY()<<endl;
                vector <double> posGlobal(3, 0);
                posGlobal[0] = -(point.x - (windowLength * modelScale / 2)) / modelScale;
                posGlobal[1] = (point.y - (windowWidth * modelScale / 2)) / modelScale;
                posGlobal[2] = 0;
                
                vector <double> robotPos(3, 0);
                robotPos[0] = robot.accessX();
                robotPos[1] = robot.accessY();
                robotPos[2] = robot.accessTheta();
                
                vector <double> relative= poseRelative(posGlobal, robotPos);

                double vx,vy,vtheta;
                double vMax = 1,vThetaMax = 1;

                // cout<<"robot : "<<robot.accessX()<<" "<<robot.accessY()<<" "<<robot.accessTheta()<<endl;
                // cout<<"posGlobal : "<<posGlobal[0]<<" "<<posGlobal[1]<<" "<<posGlobal[2]<<endl;
                // cout<<"relative : "<<relative[0]<<" "<<relative[1]<<" "<<relative[2]<<endl;
                
                double result = sqrt(pow(relative[0], 2) + pow(relative[1], 2));
                vx = (relative[0] / result) * vMax;
                vy = (relative[1] / result) * vMax;
                vtheta = (relative[2] / M_PI) * vThetaMax;
                
                robot.setVelocity(vx, vy, vtheta);   
            }

            if ( mouseFlag == 0 || mouseFlag == -1 || mouseFlag == 2) {
                robot.setVelocity(0, 0, 0);
            }
            //robot.update();
            robot.behaviorUpdate();
            } 
            else {
                robot.resetCheck();
                status.updateError(robot.error());
            }
    }
    return 0;
}

//-- Draws the Robot on Field
void World::create() {
    //-- Creates Agent and Model
    field.Access().copyTo(realWorld);
    //-------------
    //--| World |--
    //-------------
    int agentCenterXPixel = (-robot.accessX() + (windowLength * half)) * modelScale;
    int agentCenterYPixel = (robot.accessY() + (windowWidth * half)) * modelScale;
    double agentTheta = robot.accessTheta();
    int agentDirectionXPixel;
    int agentDirectionYPixel;
    int agentRightXPixel;
    int agentRightYPixel;
    int agentLeftXPixel;
    int agentLeftYPixel;
    int robotSizePixel = robotSize * modelScale;
    //-- Point Center
    Point agentCenter(agentCenterXPixel, agentCenterYPixel);
    //-- Point Direction
    agentDirectionXPixel = agentCenterXPixel - robotSizePixel * cos(agentTheta);
    agentDirectionYPixel = agentCenterYPixel - robotSizePixel * sin(agentTheta);
    Point agentDirection(agentDirectionXPixel, agentDirectionYPixel);
    //-- Point Right
    agentRightXPixel = agentCenterXPixel - robotSizePixel * cos(agentTheta + (robotSharpness * M_PI / 180));
    agentRightYPixel = agentCenterYPixel - robotSizePixel * sin(agentTheta + (robotSharpness * M_PI / 180));
    Point agentRight(agentRightXPixel, agentRightYPixel);
    //-- Point Left
    agentLeftXPixel = agentCenterXPixel - robotSizePixel * cos(agentTheta - (robotSharpness * M_PI / 180));
    agentLeftYPixel = agentCenterYPixel - robotSizePixel * sin(agentTheta - (robotSharpness * M_PI / 180));
    Point agentLeft(agentLeftXPixel, agentLeftYPixel);
    //-- DR Line
    line(realWorld, agentDirection, agentRight, Scalar(robotBlue - clickedColorValue, robotGreen - clickedColorValue, robotRed - clickedColorValue), robotLineSize, 8, 0);
    //-- DL Line
    line(realWorld, agentDirection, agentLeft, Scalar(robotBlue - clickedColorValue, robotGreen - clickedColorValue, robotRed - clickedColorValue), robotLineSize, 8, 0);
    //-- OR Line
    line(realWorld, agentCenter, agentRight, Scalar(robotBlue - clickedColorValue, robotGreen - clickedColorValue, robotRed - clickedColorValue), robotLineSize, 8, 0);
    //-- OL Line
    line(realWorld, agentCenter, agentLeft, Scalar(robotBlue - clickedColorValue, robotGreen - clickedColorValue, robotRed - clickedColorValue), robotLineSize, 8, 0);

    if(moveSignal == 1)
        circle(realWorld, point, modelScale / 20, Scalar(200, 0, 200), 2, 8, 0);
        //arrowedLine(realWorld,agentCenter,point,Scalar(200, 0, 200),2,8,0);


    imshow("World", realWorld);
}

//-- Attach Mouse to Window
void World::mouseAttacher(int event, int x, int y, int flags, void *data){
    World *pointer = reinterpret_cast<World* >(data);
    pointer -> Mouse(event, x, y, flags);
}

//-- Offer Mouse Clicking Options
void World::Mouse(int event, int x, int y, int flags){
    if (robot.borderCheck() == 0){
        if (mouseFlag == 1){
            switch(event){
                //-- Click Left Button to Pick Agent
                case EVENT_LBUTTONDOWN:
                    mouseDistance = sqrt(pow(-robot.accessX() * modelScale - (x - windowLength * half * modelScale), 2) + pow(robot.accessY() * modelScale - (y - windowWidth * half * modelScale), 2));
                    if (mouseDistance < clickAreaRadius) {
                        clickedColorValue = 100;
                        mouseFlag = -1;
                    }
                    else {
                        if(behaviorMode == 1) {
                            point.x = x;
                            point.y = y;
                            moveSignal = 1;
                        }
                        else 
                            break;
                    }
                break;
            } 
        }
        else if (mouseFlag == -1) {
            switch(event){
                //-- Set Agent Position to Cursor Position
                case EVENT_MOUSEMOVE:
                    robot.setX(-(x - windowLength * half * modelScale) / modelScale);
                    robot.setY((y - windowWidth * half * modelScale) / modelScale);
                break;
                //-- Click Left Button to Place Agent
                case EVENT_LBUTTONUP:
                    clickedColorValue = 0;
                    mouseFlag = 1;
                break;
                case EVENT_MOUSEHWHEEL:
                    if (getMouseWheelDelta(flags) < 0){
                        robot.setTheta(robot.accessTheta() + mouseRotationValue * M_PI / 180);
                    } else {
                        robot.setTheta(robot.accessTheta() - mouseRotationValue * M_PI / 180);
                    }
                break;
                case EVENT_MBUTTONDOWN:
                    mouseFlag = 0;
                break;
            }
        } else if (mouseFlag == 0){
            switch(event){
                case EVENT_MOUSEMOVE:
                    robot.setX(-(x - windowLength * half * modelScale) / modelScale);
                    robot.setY((y - windowWidth * half * modelScale) / modelScale);
                break;
                //-- Double Click Left Button to Decrease Theta
                case EVENT_LBUTTONDOWN:
                    robot.setTheta(robot.accessTheta() - mouseRotationValue * M_PI / 180);
                break;
                //-- Double Click Right Button to Increase Theta
                case EVENT_RBUTTONDOWN:
                    robot.setTheta(robot.accessTheta() + mouseRotationValue * M_PI / 180);
                break;
                case EVENT_MBUTTONDOWN:
                    mouseFlag = -1;
                break;
            }
        }
    } else {
        robot.resetCheck();
        robot.setVelocity(0, 0, 0);
        status.updateError(robot.error());
        updateWindow();
    }
}