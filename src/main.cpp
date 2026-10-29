#include<Arduino.h>
#include<RadioLib.h>
#include<SPI.h>

constexpr uint8_t SDApin = 21;
constexpr uint8_t SCLpin = 22;

constexpr int PIN_SCK  = 18;
constexpr int PIN_MISO = 19;
constexpr int PIN_MOSI = 23;
constexpr int PIN_CSN  = 5;
constexpr int PIN_CE   = 4;
constexpr int PIN_IRQ  = 27;

nRF24 radio = new Module(PIN_CSN,PIN_IRQ,PIN_CE,RADIOLIB_NC,SPI);
uint8_t address[5] = {0x12, 0x34, 0x56, 0x78, 0x90};

void check(int16_t state) {
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("RadioLib error: %d\n", state);
        while (true) {
            delay(1000);
        }
    }
}

void setup(){
    Serial.begin(115200);
    SPI.begin(PIN_SCK,PIN_MISO,PIN_MOSI,PIN_CSN);
    check(radio.begin(2400,1000,-12,5));
    check(radio.setTransmitPipe(address));
}

void loop(){
    String message;
    int16_t state = radio.receive(message);

    if (state == RADIOLIB_ERR_NONE) {
        Serial.print("RX: ");
        Serial.println(message);
    } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
        Serial.printf("RX error: %d\n", state);
    }
}