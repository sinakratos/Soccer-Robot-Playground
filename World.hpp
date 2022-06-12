#ifndef WORLD_HPP
#define WORLD_HPP

//-- Includes Needed Files
// #include "Configs.hpp"
#include "Field.hpp"
// #include "Robot.hpp"
// #include "HelpWindow.hpp"

class World {
private:
    Field field;
    // Robot robot;
    // HelpWindow window;
public:
    World();
    void worldCreate();
};

#endif // WORLD_HPP
