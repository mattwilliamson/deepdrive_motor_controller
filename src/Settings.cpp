#include "Settings.h"
#include <Arduino.h>

Settings& Settings::getInstance() {
    static Settings instance;
    return instance;
}

Settings::Settings() : sideLoaded(false) {
    preferences.begin("robot_side", false);
    loadSideOrWaitForSerial();
}

Settings::~Settings() {
    preferences.end();
}

Side Settings::getSide() {
    if (!sideLoaded) {
        loadSideOrWaitForSerial();
    }
    return side;
}

const char* Settings::getSideName() {
    return (getSide() == LEFT) ? "left" : "right";
}

const char* Settings::getNamespace() {
    String namespaceStr = String("motor/") + getSideName();
    char* result = new char[namespaceStr.length() + 1];
    strcpy(result, namespaceStr.c_str());
    return result;
}

void Settings::setSide(Side newSide) {
    side = newSide;
    saveSide();
}

void Settings::loadSideOrWaitForSerial() {
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

void Settings::saveSide() {
    preferences.putUInt("side", static_cast<uint>(side));
}

/**
 * Waits for user input from the Serial port to set the motor side.
 * The user can enter 'left' or 'right' to set the side.
 * If the input is invalid, the user will be prompted to enter again.
 * If no input is received within 5 seconds, a message will be printed to prompt the user.
 */
void Settings::waitForSideInput() {
    unsigned long lastPromptTime = millis();

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

        // Print the prompt every 5 seconds
        if (millis() - lastPromptTime > 5000) {
            Serial.println("No motor side set in EEPROM. Please enter 'left' or 'right' to set the side:");
            lastPromptTime = millis();
        }
    }

    Serial.println("Configuration complete. Attempting to connect to micro-ros-agent in 10s...");
}
