#include <Arduino.h>
#include <FreeRTOSConfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <esp_dsp.h>
#include <cmath>

#include <mqtt_client.h>
#include <ESP32Encoder.h>

/** libraries bashofhom banbst*/
// #include <memory>
// #include <optional>
// #include <utility>
// #include <vector>

// #define GEAR_RATIO 44.91 //46.8
// #define ENCODER_RESOLUTION 11
// #define

/** the following setup attempts to replicate the available Rover firmware and hardware */

using encoderPin = uint8_t;
enum EncoderMode {HALF_QUAD, FULL_QUAD, SINGLE};

struct Differential_drive final : ESP32Encoder{
    Differential_drive(const encoderPin pA, const encoderPin pB,
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
    ~Differential_drive()= default;
};

auto motorRight = Differential_drive(27, 26);
auto motorLeft = Differential_drive(12, 14);

void setup(){
    Serial.begin(115200);

    if (motorRight.isAttached() && motorLeft.isAttached())
    Serial.println("Encoders attached");
    int x = cos(5.0*M_PI/180);

}

void loop(){
    Serial.println(String(motorLeft.getCount()) + " / " + String(motorRight.getCount()));
}
