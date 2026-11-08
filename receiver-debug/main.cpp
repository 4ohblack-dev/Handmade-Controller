#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <cstring>
#include "JoystickProtocol.h"
#include "NrfDiagnostics.h"

using namespace JoystickProtocol;
static_assert(RECEIVER_INDEX==0 || RECEIVER_INDEX==1,"Select receiver A or B");
constexpr int PIN_SCK=18, PIN_MISO=19, PIN_MOSI=23, PIN_CSN=5, PIN_CE=4, PIN_IRQ=27;
constexpr char receiverId='A'+RECEIVER_INDEX;
nRF24 radio = new Module(PIN_CSN,PIN_IRQ,PIN_CE,RADIOLIB_NC,SPI,SPISettings(1000000,MSBFIRST,SPI_MODE0));
bool ready=false, received=false;
uint32_t lastValid=0, lastReport=0, lastRetry=0;

int16_t listen() {
    // setTransmitPipe overwrites pipe 0; restore our address after every reply.
    int16_t state=radio.setReceivePipe(0,addresses[RECEIVER_INDEX]);
    if(state==RADIOLIB_ERR_NONE) state=radio.startReceive();
    ready=state==RADIOLIB_ERR_NONE;
    if(!ready) Serial.printf("RX START FAIL error=%d\n",state);
    return state;
}
void initialize() {
    lastRetry=millis();
    int16_t state=radio.begin(2400,1000,-12,5);
    ready=state==RADIOLIB_ERR_NONE;
    if(!ready) Serial.printf("RADIO INIT FAIL error=%d; retry in 1s\n",state);
    else {
        Serial.println("RADIO INIT OK / SPI 1MHz");
        listen();
    }
}
void setup() {
    Serial.begin(115200);
    SPI.begin(PIN_SCK,PIN_MISO,PIN_MOSI,PIN_CSN);
    Serial.printf("Receiver %c / 115200 baud / LX LY RX RY: -1000..1000\n",receiverId);
    diagnoseNrf();
    initialize();
}
void loop() {
    if(!ready) {
        if(millis()-lastRetry>=1000) initialize();
        delay(1);
        return;
    }
    if(digitalRead(PIN_IRQ)==LOW) {
        uint8_t packet[32]={};
        size_t length=radio.getPacketLength();
        int16_t state=radio.readData(packet,sizeof(packet));
        if(state!=RADIOLIB_ERR_NONE || length!=requestSize || packet[0]!='J' || packet[1]!=1) {
            Serial.printf("RX FAIL error=%d length=%u (expected J/version 1/14 bytes)\n",state,unsigned(length));
            listen();
        } else {
            bool valid=axesValid(packet);
            if(valid) { received=true; lastValid=millis(); }
            uint8_t reply[replySize]={};
            memcpy(reply,packet,requestSize);
            reply[0]='R'; reply[14]=receiverId; reply[15]=valid?0:1;
            // Allow the controller to switch from TX to RX before sending the echo.
            delay(3);
            state=radio.setTransmitPipe(controllerAddress);
            if(state==RADIOLIB_ERR_NONE) state=radio.transmit(reply,sizeof(reply),0);
            listen();
            Serial.printf("RX %s #%lu LX=%d LY=%d RX=%d RY=%d | REPLY %s (%d)\n",
                valid?"OK":"FAIL:AXIS_RANGE",static_cast<unsigned long>(sequenceOf(packet)),
                axisOf(packet,0),axisOf(packet,1),axisOf(packet,2),axisOf(packet,3),
                state==RADIOLIB_ERR_NONE?"OK":"FAIL",state);
        }
    }
    if(millis()-lastReport>=1000) {
        lastReport=millis();
        if(!received) Serial.println("RX WAIT: no valid packet received");
        else if(millis()-lastValid>=1000)
            Serial.printf("RX TIMEOUT: no valid packet for %lu ms\n",static_cast<unsigned long>(millis()-lastValid));
    }
    delay(1);
}
