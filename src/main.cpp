#include <Arduino.h>
#include <ESP32Encoder.h>
#include <esp_timer.h>
#include <hal/cpu_hal.h>

#include <FreeRTOSConfig.h>
#include <freertos/semphr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>



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



// Pose2D OdomUpdate(const Pose2D& prevPose, const Distance l, const Distance r) //Using 2d odometer
// {
//     Pose2D pose{};
//     const Distance d = (l + r) / 2;
//     const Angle d_theta = (r - l) / PITCH;
//     pose.x = prevPose.x + d * cos(prevPose.yaw + d_theta / 2);
//     pose.y = prevPose.y + d * sin(prevPose.yaw + d_theta / 2);
//     pose.yaw = prevPose.yaw + d_theta;
//     return pose;
//     }


auto motorRight = Differential_drive(27, 26);
auto motorLeft = Differential_drive(12, 14);

 void OdomUpdate_2d(void* parameter)
{

    Pose2D pose{};
    constexpr TickType_t xFrequency = pdMS_TO_TICKS(10);
    static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

    TickType_t xLastWakeTime = xTaskGetTickCount(); //used to run the task at precisely 100Hz

    for (;;) {

        const Distance l = (motorLeft.getCount() - motorLeft.prevCount) * CIRCUMFERENCE / ENCODER_RESOLUTION;
        const Distance r = (motorRight.getCount() - motorRight.prevCount) * CIRCUMFERENCE / ENCODER_RESOLUTION;

        motorRight.prevCount = motorRight.getCount();
        motorLeft.prevCount = motorLeft.getCount();

        const Distance d = (l + r) / 2;
        const Angle d_theta = (r - l) / PITCH;

        taskENTER_CRITICAL(&spinlock); //replace with mutex or semaphore (under research)

        pose.x = pose.x + d * cosf(pose.yaw + d_theta / 2);
        pose.y = pose.y + d * sinf(pose.yaw + d_theta / 2);
        pose.yaw = pose.yaw + d_theta;

        taskEXIT_CRITICAL(&spinlock);

        Serial.println(uxTaskGetStackHighWaterMark(nullptr)); //to print the number of bytes left (multiply by 4), debugging purposes only

        vTaskDelayUntil(&xLastWakeTime, xFrequency); //will try to replace all delays with suspension from other tasks, hope this doesnt turn into spahghetti
    }

}

void deadReckoning(const uint16_t samplingTime)
{






}

void setup(){
    Serial.begin(115200);

    // xTaskCreate(); //minimum stack  size for a task is aboout 750, add static allocation on top of it
}

void loop(){
    vTaskDelete(nullptr);
}
