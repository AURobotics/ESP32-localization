#include <Arduino.h>
#include <ESP32Encoder.h>

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

}

void loop(){
    Serial.println(String(motorLeft.getCount()) + " / " + String(motorRight.getCount()));
}
