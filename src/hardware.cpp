#include "hardware.h"
#include <Arduino.h>
#include <PID_v1.h>
#include <ESP32Encoder.h>
#include "MPU6050.h"
#include "Wire.h"

// left encoder A : 19
// left encoder B : 18
// right encoder A : 14
// right encoder B : 27

uint8_t encaPin[] = {19, 14};
uint8_t encbPin[] = {18, 27};
uint8_t in1[] = {13, 12};
uint8_t in2[] = {5, 15};
double pwm[] = {0, 0};

double pos[] = {0, 0};
double setpos[] = {0, 0};
double Kp = 01.5, Ki = 1.53, Kd = 0.15;
// double Kp = 0.4, Ki = 0.027, Kd = 0.025;

ESP32Encoder encoder0;
ESP32Encoder encoder1;

PID motor0(&pos[0], &pwm[0], &setpos[0], Kp, Ki, Kd, DIRECT);
// PID motor1(&pos[1], &pwm[1], &setpos[1], Kp, Ki, Kd, DIRECT);/////////removed

double yaw_target;
double yaw_current;
double pwm_yaw;
// double Kpt = 1.0, Kit = 0.02, Kdt = 0.033;

double Kpt = 0.8, Kit = 1.0, Kdt = 0.4 ;

MPU6050 mpu;

/*------------------------------------------------------------------------------------------------------------*/
unsigned long lastUpdateTime = 0;
double gyroZOffset = 0; // Set this in MPU_init

PID yaw(&yaw_current, &pwm_yaw, &yaw_target, Kpt, Kit, Kdt, DIRECT);

void MPU_init()
{
    Wire.begin(); // Standard SDA/SCL for ESP32 (usually 21/22)
    mpu.initialize();

    // if (!mpu.testConnection()) {
    //     Serial.println("MPU6050 connection failed!");
    //     while (1);
    // }
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    Serial.println("Calibrating Gyro (4000 samples)... Keep Robot Still.");

    double sumZ = 0;
    for (int i = 0; i < 4000; i++)
    {
        sumZ += mpu.getRotationZ();
        if (i % 500 == 0)
            Serial.print(".");
        delayMicroseconds(200); // Small delay for stable sampling
    }

    // Calculate the drift offset
    gyroZOffset = sumZ / 4000.0;

    // As requested: Set target and current to baseline
    yaw_target = 0;
    yaw_current = 0;
    lastUpdateTime = micros();

    // Serial.println("\nMPU Initialized.");
    // Serial.print("Offset: "); Serial.println(gyroZOffset);
}

void updateYaw()
{
    unsigned long currentTime = micros();
    double dt = (currentTime - lastUpdateTime) / 1000000.0; // Convert to seconds

    // Read raw Z-axis rotation
    double rawGZ = (double)mpu.getRotationZ();

    // 1. Subtract offset
    // 2. Convert to degrees/sec (FS_SEL=0 default is 131 LSB/deg/s)
    double gyroZrate = (rawGZ - gyroZOffset) / 65.5;

    if (abs(gyroZrate) < 3.0)
    {
        gyroZrate = 0;
    }

    // Integrate to get angle: Angle = Angle + (Rate * Time)
    yaw_current += gyroZrate * dt;
    Serial.println(yaw_current);
    lastUpdateTime = currentTime;
}

//--------------------------------------------------------------------------------------------------------

namespace Hardware
{

    void init()
    {
        ESP32Encoder::useInternalWeakPullResistors = puType::up;
        encoder0.attachFullQuad(encaPin[0], encbPin[0]);
        encoder0.clearCount();
        encoder1.attachFullQuad(encaPin[1], encbPin[1]);
        encoder1.clearCount();
        motor0.SetMode(AUTOMATIC);
        motor0.SetOutputLimits(-255, 255);
        yaw.SetSampleTime(10); 
        motor0.SetSampleTime(10);
        // motor1.SetMode(AUTOMATIC);//removed
        // motor1.SetOutputLimits(-255, 255);
        yaw.SetMode(AUTOMATIC);
        yaw.SetOutputLimits(-255, 255);
        pinMode(in1[0], OUTPUT);
        pinMode(in1[1], OUTPUT);
        pinMode(in2[0], OUTPUT);
        pinMode(in2[1], OUTPUT);

        MPU_init();

        Serial.println("h init  done");
    }

    void moveForward(int cells)
    {
        bool stop = 0;
        setpos[0] = (10000) * cells;
        // setpos[1] = (1200) * cells; //removed
        encoder0.clearCount();
        encoder1.clearCount();
        pos[0] = 0;

        motor0.SetMode(MANUAL);
        motor0.SetMode(AUTOMATIC);

        // Reset and Setup the Yaw (Heading) PID
        yaw.SetMode(MANUAL);
        yaw.SetMode(AUTOMATIC);
        updateYaw();
        yaw_target = yaw_current;
        lastUpdateTime = micros();

        // pos[1] = 0; //removed
        while (1)
        {
            pos[0] = encoder0.getCount() + encoder1.getCount();
            pos[0] /= 2.0;
            // pos[1] = encoder1.getCount();
            motor0.Compute();
            yaw.Compute();
            updateYaw() ;
            double left_pwm = pwm[0] + pwm_yaw;
            double right_pwm = pwm[0] - pwm_yaw;

            // Constrain limits to prevent weird motor behavior if PIDs spike
            left_pwm = constrain(left_pwm, -255, 255);
            right_pwm = constrain(right_pwm, -255, 255);
            // motor1.Compute();removed
            // double pwm_average = (pwm[0] + pwm[1]) / 2;
            if (right_pwm >= 10)
            {
                analogWrite(in1[0], 0);
                analogWrite(in1[1], right_pwm);
            }
            else if (right_pwm <= -10)
            {
                analogWrite(in1[0], -right_pwm);
                analogWrite(in1[1], 0);
            }
            else
            {
                analogWrite(in1[0], 255);
                analogWrite(in1[1], 255);
            }
            if (left_pwm >= 10)
            {
                analogWrite(in2[0], left_pwm);
                analogWrite(in2[1], 0);
            }
            else if (left_pwm <= -10)
            {
                analogWrite(in2[0], 0);
                analogWrite(in2[1], -left_pwm);
            }
            else
            {
                analogWrite(in2[0], 255);
                analogWrite(in2[1], 255);
            }
            delay(10);
            // break condition
            if (abs(pos[0] - setpos[0]) <= 150)
            {
                analogWrite(in1[0], 255);
                analogWrite(in1[1], 255);
                analogWrite(in2[0], 255);
                analogWrite(in2[1], 255);
                break;
            }
        }
        delay(50);
    }

    void turnRight()
    {
        // Serial.println("Turning Right");

        // // 1. Reset PID internal memory
        // yaw.SetMode(MANUAL);
        // yaw.SetMode(AUTOMATIC);
        // encoder0.clearCount();
        // encoder1.clearCount();

        // // 2. Prevent huge time jumps in dt between movements
        // lastUpdateTime = micros();
        // updateYaw();
        // yaw_target = yaw_current - 90; // Negative for Right

        // while (1)
        // {
        //     yaw.Compute();
        //     updateYaw();

        //     // Serial.print(yaw_target);
        //     // Serial.print(" ");
        //     // Serial.println(pwm_yaw);

        //     // 3. Drive Motors
        //     if (pwm_yaw >= 10)
        //     {
        //         analogWrite(in1[0], pwm_yaw);
        //         analogWrite(in1[1], 0);
        //         analogWrite(in2[0], pwm_yaw);
        //         analogWrite(in2[1], 0);
        //     }
        //     else if (pwm_yaw <= -10)
        //     {
        //         analogWrite(in1[0], 0);
        //         analogWrite(in1[1], -pwm_yaw);
        //         analogWrite(in2[0], 0);
        //         analogWrite(in2[1], -pwm_yaw);
        //     }
        //     else
        //     {
        //         // Motor Brake
        //         analogWrite(in1[0], 255);
        //         analogWrite(in1[1], 255);
        //         analogWrite(in2[0], 255);
        //         analogWrite(in2[1], 255);
        //     }
        //     delay(10);

        //     // 4. BREAK CONDITION: Exit only when we are within 2 degrees of target
        //     if (abs(yaw_target - yaw_current) <= 7.0)
        //     {
        //         analogWrite(in1[0], 255);
        //         analogWrite(in1[1], 255);
        //         analogWrite(in2[0], 255);
        //         analogWrite(in2[1], 255);
        //         yaw_current = 0;
        //         yaw_target = 0;
        //         break;
        //     }
        // }
        delay(50);
    }

    void turnLeft()
    {
        // // Serial.println("Turning Left");

        // // 1. Reset PID internal memory
        // yaw.SetMode(MANUAL);
        // yaw.SetMode(AUTOMATIC);
        // encoder0.clearCount();
        // encoder1.clearCount();


        // // 2. Prevent huge time jumps in dt between movements
        // lastUpdateTime = micros();
        // updateYaw();
        // yaw_target = yaw_current + 90; // Positive for Left

        // while (1)
        // {
        //     yaw.Compute();
        //     updateYaw();

        //     // Serial.print(yaw_target);
        //     // Serial.print(" ");
        //     // Serial.println(pwm_yaw);

        //     // 3. Drive Motors (Identical to turnRight to allow PID corrections)
        //     if (pwm_yaw >= 10)
        //     {
        //         analogWrite(in1[0], pwm_yaw);
        //         analogWrite(in1[1], 0);
        //         analogWrite(in2[0], pwm_yaw);
        //         analogWrite(in2[1], 0);
        //     }
        //     else if (pwm_yaw <= -10)
        //     {
        //         analogWrite(in1[0], 0);
        //         analogWrite(in1[1], -pwm_yaw);
        //         analogWrite(in2[0], 0);
        //         analogWrite(in2[1], -pwm_yaw);
        //     }
        //     else
        //     {
        //         // Motor Brake
        //         analogWrite(in1[0], 255);
        //         analogWrite(in1[1], 255);
        //         analogWrite(in2[0], 255);
        //         analogWrite(in2[1], 255);
        //     }
        //     delay(10);

        //     // 4. BREAK CONDITION: Exit only when we are within 2 degrees of target
        //     if (abs(yaw_target - yaw_current) <= 7.0)
        //     {
        //         analogWrite(in1[0], 255);
        //         analogWrite(in1[1], 255);
        //         analogWrite(in2[0], 255);
        //         analogWrite(in2[1], 255);
        //         yaw_current = 0;
        //         yaw_target = 0;
        //         break;
        //     }
        // }
        delay(50);
    }

    void stop()
    {
        Serial.println("Stopping");
        analogWrite(in1[0], 255);
        analogWrite(in1[1], 255);
        analogWrite(in2[0], 255);
        analogWrite(in2[1], 255);
    }
}