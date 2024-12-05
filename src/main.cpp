#include <Arduino.h>
#include <ESP32Encoder.h>
#include <esp_timer.h>
#include <hal/cpu_hal.h>

#include <FreeRTOSConfig.h>
#include <freertos/semphr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/atomic.h>



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
using Distance = float;
using Angle = float;
using Vel = float;

enum EncoderMode {HALF_QUAD, FULL_QUAD, SINGLE};

struct Pose2D
{
    float x;
    float y;
    float yaw;
};

struct Velocity2D
{
    float x;
    float y;
    float yaw;
};

struct Odometry
{
    Pose2D pose;
    Velocity2D vel;
};

struct Differential_drive final : ESP32Encoder{
    Differential_drive(const EncoderPin pA, const EncoderPin pB,
        const EncoderMode encoderMode = FULL_QUAD){

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
        clearCount();
    }

    [[nodiscard]] float wheelVel(const uint8_t samplingTime)
    {
        return (getCount() - prevCount) * CIRCUMFERENCE / samplingTime;
    }

    ~Differential_drive() = default;

    int64_t prevCount = 0;
};

Odometry odometry{};
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

auto motorRight = Differential_drive(27, 26);
auto motorLeft = Differential_drive(12, 14);

inline void poseUpdate(const Distance l, const Distance r, const Vel velL, const Vel velR) //Using 2d odometer, inline to reduce function call overhead
{
    taskENTER_CRITICAL(&spinlock); //or mutex
    const Distance d = (l + r) / 2;
    const Angle d_theta = (r - l) / PITCH;
    odometry.pose.x = odometry.pose.x + d * cos(odometry.pose.yaw + d_theta / 2);
    odometry.pose.y = odometry.pose.y + d * sin(odometry.pose.yaw + d_theta / 2);
    odometry.pose.yaw = odometry.pose.yaw + d_theta;
    taskEXIT_CRITICAL(&spinlock);
}


 void Odom_2d(void* parameter)
{
     constexpr TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        const Distance l = (motorLeft.getCount() - motorLeft.prevCount) * CIRCUMFERENCE / ENCODER_RESOLUTION;
        const Distance r = (motorRight.getCount() - motorRight.prevCount) * CIRCUMFERENCE / ENCODER_RESOLUTION;

        poseUpdate(l, r, motorLeft.wheelVel(xFrequency), motorRight.wheelVel(xFrequency));

        motorRight.prevCount = motorRight.getCount();
        motorLeft.prevCount = motorLeft.getCount();

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

        const Vel velLeft = motorLeft.wheelVel(dt);
        const Vel velRight = motorRight.wheelVel(dt);
        const Distance l = velLeft * dt; //comment on Rover test
        const Distance r = velRight * dt; //comment on Rover test

        poseUpdate(l, r, velLeft, velRight);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


void setup(){
    Serial.begin(115200);

}

void loop(){


}
