#ifndef ROBOT_SIDE_H
#define ROBOT_SIDE_H

#include <Preferences.h>

enum Side {
    LEFT = 1,
    RIGHT = 2
};

class RobotSide {
public:
    static RobotSide& getInstance();
    Side getSide();
    void setSide(Side side);
    const char* getNodeName();
    const char* getSideName();
    const char* getNamespace();

private:
    RobotSide();
    ~RobotSide();
    RobotSide(const RobotSide&) = delete;
    RobotSide& operator=(const RobotSide&) = delete;

    void loadSide();
    void saveSide();
    void waitForSideInput();

    Preferences preferences;
    Side side;
    bool sideLoaded;
};

#endif // ROBOT_SIDE_H
