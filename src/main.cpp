#include <Arduino.h>
#include <ESP32Encoder.h>
#include <driver/pcnt.h>

ESP32Encoder encoderRight;
ESP32Encoder encoderLeft;


void setup(){

    Serial.begin(115200);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    encoderRight.attachFullQuad(27, 26);
    encoderLeft.attachFullQuad(25, 33);


    encoderRight.clearCount();
    encoderLeft.clearCount();

    if (encoderRight.isAttached() && encoderLeft.isAttached())
    Serial.println("Encoders attached");

}

void loop(){
    Serial.println(String(encoderLeft.getCount()) + " / " + String(encoderRight.getCount()));
}
