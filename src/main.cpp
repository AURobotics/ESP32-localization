#include <Arduino.h>
#include <ESP32Encoder.h>
#include <esp_timer.h>
#include <hal/cpu_hal.h>


/**trying to improve overall speed of code execution
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
#define RADIUS (TWO_PI * 0.065)
#define PITCH 0.5

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
    ~Differential_drive() = default;

private:
    int64_t m_prevCount = 0;


};



Pose2D OdomUpdate(const Pose2D& prevPose, const Distance l, const Distance r) //Using 2d odometer
{
    Pose2D pose{};
    const Distance d = (l + r) / 2;
    const Angle d_theta = (r - l) / PITCH;
    pose.x = prevPose.x + d * cos(prevPose.yaw + d_theta / 2);
    pose.y = prevPose.y + d * sin(prevPose.yaw + d_theta / 2);
    pose.yaw = prevPose.yaw + d_theta;
    return pose;
    }


auto motorRight = Differential_drive(27, 26);
auto motorLeft = Differential_drive(12, 14);

void setup(){
    Serial.begin(115200);

    if (motorRight.isAttached() && motorLeft.isAttached())
    Serial.println("Encoders attached");

}

void loop(){

}
