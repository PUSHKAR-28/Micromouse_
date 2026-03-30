#ifndef HARDWARE_H
#define HARDWARE_H
void updateYaw();
namespace Hardware {
    void init();
    void moveForward(int cell); // Moves exactly 1 cell using PID and Encoders
    void turnRight();
    void turnLeft();
    void stop();
}
#endif