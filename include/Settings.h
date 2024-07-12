#ifndef ROBOT_SIDE_H
#define ROBOT_SIDE_H

#include <Preferences.h>

enum Side {
    LEFT = 1,
    RIGHT = 2
};

class Settings {
public:
    static Settings& getInstance();
    Side getSide();
    void setSide(Side side);
    const char* getNodeName();
    const char* getSideName();
    const char* getNamespace();

private:
    Settings();
    ~Settings();
    Settings(const Settings&) = delete;
    Settings& operator=(const Settings&) = delete;

    void loadSideOrWaitForSerial();
    void saveSide();
    void waitForSideInput();

    Preferences preferences;
    Side side;
    bool sideLoaded;
};

#endif // ROBOT_SIDE_H
