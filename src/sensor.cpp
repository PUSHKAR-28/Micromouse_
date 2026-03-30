#include "sensor.h"
#include <Arduino.h>

namespace Sensor {
    // HC-SR04 Pin Assignments (ESP32)
    const int TRIG_FRONT = 30; // Trigger pins
    const int TRIG_LEFT  = 25;
    const int TRIG_RIGHT = 26;

    const int ECHO_FRONT = 32; // Echo pins (must be input)
    const int ECHO_LEFT  = 33;
    const int ECHO_RIGHT = 34;

    // Distance threshold in Centimeters
    // A standard MicroMouse cell is 20cm. 
    // If distance < 12cm, there is likely a wall.
    const int WALL_THRESHOLD_CM = (6*2)/0.0343; 

    // Helper function to get distance in CM
    inline float getDistance(int trig, int echo) {
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);
        
        // Measure pulse duration (timeout after 20ms to prevent hanging)
        long duration = pulseIn(echo, HIGH, 20000);
        
        // Speed of sound is ~343 m/s -> 0.0343 cm/us
        // distance = (time * speed) / 2
        if (duration == 0) return 20000; // No wall detected
        return duration;
    }

    void init() {
        pinMode(TRIG_FRONT, OUTPUT);
        pinMode(TRIG_LEFT,  OUTPUT);
        pinMode(TRIG_RIGHT, OUTPUT);
        
        pinMode(ECHO_FRONT, INPUT);
        pinMode(ECHO_LEFT,  INPUT);
        pinMode(ECHO_RIGHT, INPUT);
    }

    bool wallFront() { 
        // return getDistance(TRIG_FRONT, ECHO_FRONT) < WALL_THRESHOLD_CM;
        return false;
    }

    bool wallLeft() { 
        // return getDistance(TRIG_LEFT, ECHO_LEFT) < WALL_THRESHOLD_CM;
        return false;
    }

    bool wallRight() { 
        // return getDistance(TRIG_RIGHT, ECHO_RIGHT) < WALL_THRESHOLD_CM;
        return false;
    }

    // Keep raw functions for your future IR integration
    // For now, they return the distance in CM cast to int
    // int getLeftRaw() { return (int)getDistance(TRIG_LEFT, ECHO_LEFT); }
    // int getRightRaw() { return (int)getDistance(TRIG_RIGHT, ECHO_RIGHT); }
}