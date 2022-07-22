#include "Util.hpp"

double modAngle(double theta) {
    
    theta = fmod(theta, 2 * M_PI);
    if (theta >= M_PI) {
        theta -= 2 * M_PI;
    }
    return theta;
}

vector <double> poseGlobal(vector <double> pGlobal, vector <double> pRelative) {
    vector <double> resultGlobal;
    double cosT = cos(pGlobal[2]);
    double sinT = sin(pGlobal[2]);
    resultGlobal[0] = pGlobal[0] + cosT * pRelative[0] - sinT * pRelative[1];
    resultGlobal[1] = pGlobal[1] + sinT * pRelative[0] + cosT * pRelative[1];
    resultGlobal[2] = pGlobal[2] + pRelative[2];
    return resultGlobal;
}

vector <double> poseRelative(vector <double> pGlobal, vector <double> pose) {
    
    vector <double> resultRelative(3, 0);
    double cosT = cos(pose[2]);
    double sinT = sin(pose[2]);
    double pX = pGlobal[0] - pose[0];
    double pY = pGlobal[1] - pose[1];
    double pT = pGlobal[2] - pose[2];
    resultRelative[0] = cosT * pX + sinT * pY;
    resultRelative[1] = cosT * pY - sinT * pX;
    resultRelative[2] = modAngle(pT);
    return resultRelative;
}
