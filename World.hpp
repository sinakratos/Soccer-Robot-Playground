#ifndef WORLD_HPP
#define WORLD_HPP

//-- Includes Needed Files
#include "HelpWindow.hpp"
#include "Field.hpp"
#include "Robot.hpp"
#include "Util.hpp"

class World {
private:
    Field field;
    Robot robot;
    HelpWindow status;
    Mat realWorld;
    double behaviorMode = -1;
    double moveSignal = 0;
    Point point;
public:
    World();
    int updateWindow();
    void create();
    static void mouseAttacher(int ,int ,int ,int ,void *);
    void Mouse(int ,int ,int ,int);
};

#endif // WORLD_HPP
