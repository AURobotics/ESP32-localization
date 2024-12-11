#include <Arduino.h>
#include <esp_timer.h>
// #include <esp_log.h>

#include <esp_dsp.h>

#include <Wire.h>

#include <FreeRTOSConfig.h>
#include <freertos/semphr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/atomic.h>

// #include <SparkFun_BNO080_Arduino_Library.h>
#include <ESP32Encoder.h>
#include <Odometry.h>



/** trying to improve overall speed of code execution
 *avoid using double precision floating points.
 *to measure time of a function:

void measure_important_function(void) {
    const unsigned MEASUREMENTS = 5000;
    uint64_t start = esp_timer_get_time();

    for (int retries = 0; retries < MEASUREMENTS; retries++) {
        important_function(); // This is the thing you need to measure
    }

    uint64_t end = esp_timer_get_time();

    printf("%u iterations took %llu milliseconds (%llu microseconds per invocation)\n",
           MEASUREMENTS, (end - start)/1000, (end - start)/MEASUREMENTS);
}
    or including hal/cpu_hal.h and calling the HAL function cpu_hal_get_cycle_count()

 */


#define ENCODER_RESOLUTION 11
#define GEAR_RATIO 44.912
#define ENCODER_PPR (ENCODER_RESOLUTION * GEAR_RATIO)
#define CIRCUMFERENCE (M_TWOPI * 0.065)
#define PITCH 0.5 //width of the robot measured from the center of the wheels

using EncoderPin = uint8_t;
using MotorPin = uint8_t;
using Distance = float;
using Angle = float;
using Vel = float;


enum EncoderMode {HALF_QUAD, FULL_QUAD, SINGLE};
enum MotorDriection {CW = 0, CCW = -1};

struct Differential_drive final : ESP32Encoder{
    Differential_drive(const EncoderPin pA, const EncoderPin pB, const MotorPin dirA, const MotorPin dirB, const MotorPin en,
        const EncoderMode encoderMode = FULL_QUAD){

        static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
        taskENTER_CRITICAL(&spinlock);

        //-----------Setting encoders-----------//
        useInternalWeakPullResistors = puType::up;
        switch (encoderMode)
        {
        case EncoderMode::FULL_QUAD:
            attachFullQuad(pA, pB);
            break;
        case EncoderMode::HALF_QUAD:
            attachHalfQuad(pA, pB);
            break;
        case EncoderMode::SINGLE:
            attachSingleEdge(pA, pB);
        }
        this->clearCount();

        //-----------Setting motors-----------//
        ledcSetup(ledcChannelCount, 30000, 20);
        pinMode(ledcChannelCount, OUTPUT);
        ledcAttachPin(en, ledcChannelCount);
        ledcChannelCount++;
        //------------------------------------//
        taskEXIT_CRITICAL(&spinlock);
    }

    [[nodiscard]] float wheelVel(const uint8_t samplingTime)
    {
        return (getCount() - prevCount) * CIRCUMFERENCE / samplingTime;
    }



    void drive()
    {



    }

    ~Differential_drive() = default;
public:
    int64_t prevCount = 0;

    static void setLedcChannelCount(const uint8_t n) {
        ledcChannelCount = n;
    }


private:
    MotorPin DirA;
    MotorPin DirB;
    MotorPin dir;
    static uint8_t ledcChannelCount;
};

Odometry2D odometry{};
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

auto motorFrontRight = Differential_drive(14, 27, 1,2,1);
auto motorFrontLeft = Differential_drive(26, 25,1,1,1);
auto motorBackRight = Differential_drive(33, 32,1,1,1);
auto motorBackLeft = Differential_drive(35, 34,1,1,1);
// BNO080 _bno;


inline void poseUpdate(const Distance l, const Distance r) //Using 2d odometer, inline to reduce function call overhead
{
    taskENTER_CRITICAL(&spinlock); //or mutex
    const Distance d = (l + r) / 2;
    const Angle d_theta = (r - l) / PITCH;
    odometry.pose.x = odometry.pose.x + d * cos(odometry.pose.yaw + d_theta / 2);
    odometry.pose.y = odometry.pose.y + d * sin(odometry.pose.yaw + d_theta / 2);
    odometry.pose.yaw = odometry.pose.yaw + d_theta;
    taskEXIT_CRITICAL(&spinlock);
}

// void getAllCalibration(BNO080 *BNO, uint8_t &system, uint8_t &quat, uint8_t &gyro, uint8_t &accel, uint8_t &mag)
// {
//     quat = BNO->getQuatAccuracy();
//     gyro = BNO->getGyroAccuracy();
//     accel = BNO->getAccelAccuracy();
//     mag = BNO->getMagAccuracy();
//     system = (gyro + accel + mag) / 3;
// }

 void Odom_2d(void* parameter)
{
     constexpr TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        const Distance l = (motorFrontLeft.getCount() - motorFrontLeft.prevCount) * CIRCUMFERENCE / ENCODER_RESOLUTION;
        const Distance r = (motorFrontRight.getCount() - motorFrontRight.prevCount) * CIRCUMFERENCE / ENCODER_RESOLUTION;

        poseUpdate(l, r);

        motorFrontRight.prevCount = motorFrontRight.getCount();
        motorFrontLeft.prevCount = motorFrontLeft.getCount();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

//The task will request the rpm at regular intervals. this task should have high priority as you dont want it to miss a deadline.
void deadReckoning(void *parameter)
{
    constexpr TickType_t xFrequency = pdMS_TO_TICKS(10); //maximum 2^8 - 1
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // static unsigned long long prevTime;             //or use freeRTOS timer
    for (;;)
    {
        /** request velocities from ESCs */
        constexpr auto dt = static_cast<uint8_t>(xFrequency);

        //integration functions for rover test.

        const Vel velLeft = motorFrontLeft.wheelVel(dt);
        const Vel velRight = motorFrontRight.wheelVel(dt);
        const Distance l = velLeft * dt; //comment on Rover test
        const Distance r = velRight * dt; //comment on Rover test

        poseUpdate(l, r);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


void setup(){
    Serial.begin(115200);
    esp_timer_init();
    Differential_drive::setLedcChannelCount(0);

    // Wire.begin();

    // IMPORTANT: ESP WILL NOT WORK WITHOUT THE FOLLOWING LINES OF CODE
    // PLEASE DO NOT CHANGE THE PIN CONFIGURATION
    //
    //=======================================
    /** esp32 BNO085 */
    // delay(100);
    // Wire.flush();
    // _bno.begin(BNO080_DEFAULT_ADDRESS, Wire);
    // Wire.begin(4, 5);
    // Wire.setTimeOut(4000);
    //
    // Wire.setClock(400000); //should not exceed 400khz
    //
    // _bno.enableRotationVector(10);
    // _bno.enableAccelerometer(10);
    // _bno.enableGyro(10);
    //
    // // enabling magnetometer will not return any data.
    //
    // //calibrating IMU
    // _bno.calibrateAll();
    //
    // //comment below after calibration


    // while (1)
    // {   static uint8_t system, quat, gyro, accel, mag;
    //     getAllCalibration(&_bno, system, quat, gyro, accel, mag);
    //     if (system == 3) break;
    //     Serial.println("System: " + String(system) + "quat: " + String(quat) +
    //         " gyro: " + String(gyro) + "mag: " + String(mag) + " accel: " + String(accel)
    //         );
    //     delay(500);
    // }
    /** end of BNO085 setup */
    // library example 14 shows the configuration of 2 sensors
    // I think the sensor outputs orientation relative to the magnetic north
    // Example 22 shows how to set a new reference orientation (tare)
    // Or, to get the reference quaternion, multiply it by its inverse,
    // its straight forward
    //
    //=======================================

}
    /**
    *Below is an attempt to create the full fusion algorithm based
    *it is based on the extended kalman filter, wheels encoders, and IMU
    *It will be reimplemented using tasks, one of the purposes of doing this
    *is to know the total time it is required for the full algorithm
    *as it involves multiple matrix operations and trigonometry functions
    *please do not worry about the weird implementation, every thing is documented
    */
void loop()
{
    static unsigned long prev_time = esp_timer_get_time();
    Serial.println("Time: " + String(esp_timer_get_time() - prev_time));
    prev_time = esp_timer_get_time();

    const Distance l = (motorFrontLeft.getCount() - motorFrontLeft.prevCount) * CIRCUMFERENCE / ENCODER_RESOLUTION;
    const Distance r = (motorFrontRight.getCount() - motorFrontRight.prevCount) * CIRCUMFERENCE / ENCODER_RESOLUTION;

    motorFrontRight.prevCount = motorFrontRight.getCount();
    motorFrontLeft.prevCount = motorFrontLeft.getCount();

    const Distance d = (l + r) / 2;
    const Angle d_theta = (r - l) / PITCH;

    odometry.pose.x = odometry.pose.x + d * cos(odometry.pose.yaw + d_theta / 2);
    odometry.pose.y = odometry.pose.y + d * sin(odometry.pose.yaw + d_theta / 2);
    odometry.pose.yaw = odometry.pose.yaw + d_theta;

    Serial.print("X: " + String(odometry.pose.x) + " \t");
    Serial.print("Y: " + String(odometry.pose.y) + " \t");
    Serial.print("Yaw: " + String(odometry.pose.yaw) + " \t");
    Serial.println();

    //https://github.com/xiaozhengxu/CompRobo_IMU_Sensor_fusion?tab=readme-ov-file
}



