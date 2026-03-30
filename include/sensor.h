#ifndef SENSOR_H
#define SENSOR_H
#include <Arduino.h>

namespace Sensor {
    void init();
    bool wallFront();
    bool wallLeft();
    bool wallRight();
    // For PID:
    // int getLeftRaw();
    // int getRightRaw();
}
#endif