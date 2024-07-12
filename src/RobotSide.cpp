#include "RobotSide.h"
#include <Arduino.h>

RobotSide& RobotSide::getInstance() {
    static RobotSide instance;
    return instance;
}

RobotSide::RobotSide() : sideLoaded(false) {
    preferences.begin("robot_side", false);
    loadSide();
}

RobotSide::~RobotSide() {
    preferences.end();
}

Side RobotSide::getSide() {
    if (!sideLoaded) {
        loadSide();
    }
    return side;
}

const char* RobotSide::getNodeName() {
    String nodeName = String("motor_") + getSideName() + "_node";
    char* result = new char[nodeName.length() + 1];
    strcpy(result, nodeName.c_str());
    return result;
}

const char* RobotSide::getSideName() {
    return (getSide() == LEFT) ? "left" : "right";
}

const char* RobotSide::getNamespace() {
    String namespaceStr = String("motor/") + getSideName();
    char* result = new char[namespaceStr.length() + 1];
    strcpy(result, namespaceStr.c_str());
    return result;
}

void RobotSide::setSide(Side newSide) {
    side = newSide;
    saveSide();
}

void RobotSide::loadSide() {
    if (!sideLoaded) {
        uint storedValue = preferences.getUInt("side", 0);
        if (storedValue == LEFT || storedValue == RIGHT) {
            side = static_cast<Side>(storedValue);
        } else {
            waitForSideInput();
        }
        sideLoaded = true;
    }
}

void RobotSide::saveSide() {
    preferences.putUInt("side", static_cast<uint>(side));
}

void RobotSide::waitForSideInput() {
    Serial.println("Please enter 'left' or 'right' to set the side:");
    while (true) {
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim(); // Remove any extra whitespace
            if (input.equalsIgnoreCase("left")) {
                setSide(LEFT);
                Serial.println("Side set to LEFT.");
                break;
            } else if (input.equalsIgnoreCase("right")) {
                setSide(RIGHT);
                Serial.println("Side set to RIGHT.");
                break;
            } else {
                Serial.println("Invalid input. Please enter 'left' or 'right':");
            }
        }
    }
}
