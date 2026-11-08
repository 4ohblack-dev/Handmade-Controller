#include <Arduino.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <SPI.h>
#include <cstring>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "JoystickProtocol.h"

constexpr uint8_t axisPins[] = {32, 33, 34, 35};
const char* names[] = {"LX", "LY", "RX", "RY"};
constexpr int PIN_SCK=18, PIN_MISO=19, PIN_MOSI=23, PIN_CSN=5, PIN_CE=4, PIN_IRQ=27;
Module radioModule(PIN_CSN, PIN_IRQ, PIN_CE, RADIOLIB_NC, SPI);
nRF24 radio(&radioModule);

struct Calibration {
    uint32_t version;
    uint16_t minimum[4], maximum[4], center[4];
};
Calibration measured={}, saved={}, active={};
uint16_t raw[4]={};
constexpr size_t averageSamples=8;
constexpr uint32_t samplePeriodMs=5;
uint16_t filtered[4]={};
uint16_t sampledRaw[4]={}, sampledAverage[4]={};
portMUX_TYPE sampleMux=portMUX_INITIALIZER_UNLOCKED;
bool samplesReady=false;

// Sampling continues while radio.transmit() waits for an ACK.
void sampleJoystick(void*) {
    uint16_t history[4][averageSamples]={};
    uint32_t sums[4]={};
    size_t index=0, count=0;
    TickType_t wake=xTaskGetTickCount();
    for(;;) {
        uint16_t values[4], means[4];
        for(int i=0;i<4;i++) {
            values[i]=analogRead(axisPins[i]);
            sums[i]-=history[i][index];
            history[i][index]=values[i];
            sums[i]+=values[i];
        }
        if(count<averageSamples) ++count;
        for(int i=0;i<4;i++) means[i]=(sums[i]+count/2)/count;
        index=(index+1)%averageSamples;
        portENTER_CRITICAL(&sampleMux);
        memcpy(sampledRaw,values,sizeof(values));
        memcpy(sampledAverage,means,sizeof(means));
        samplesReady=true;
        portEXIT_CRITICAL(&sampleMux);
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(samplePeriodMs));
    }
}
bool readJoystick() {
    portENTER_CRITICAL(&sampleMux);
    bool ready=samplesReady;
    memcpy(raw,sampledRaw,sizeof(raw));
    memcpy(filtered,sampledAverage,sizeof(filtered));
    portEXIT_CRITICAL(&sampleMux);
    return ready;
}
bool control=false, centerCaptured=false, savedAvailable=false;
uint32_t sequence=0, lastSend=0, lastPrint=0;
Adafruit_SSD1306 oled(128,64,&Wire,-1);
bool oledReady=false, radioReady=false, txAttempted=false;
int16_t radioError=0, txResultA=0, txResultB=0;
uint32_t lastDisplay=0;
const char* displayMessage="Sweep all axes";
struct Feedback {
    bool received=false;
    uint8_t status=0;
    uint32_t sequence=0;
    int16_t axes[4]={};
};
Feedback feedback[2];

void initDisplay() {
    Wire.begin(21,22);
    Wire.setTimeOut(20);
    // Arduino Wire uses the 7-bit I2C address (0x3C, not 0x78).
    Wire.beginTransmission(0x3C);
    if(Wire.endTransmission()!=0) {
        Serial.println("OLED not found at 0x3C; continuing with serial.");
        return;
    }
    oledReady=oled.begin(SSD1306_SWITCHCAPVCC,0x3C,false,false);
    if(!oledReady) Serial.println("OLED buffer allocation failed.");
    else {
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(SSD1306_WHITE);
        oled.setTextWrap(false);
    }
}

void drawDisplay() {
    if(!oledReady || millis()-lastDisplay<200) return;
    lastDisplay=millis();
    oled.clearDisplay();
    oled.setCursor(0,0);
    if(!control) {
        oled.printf("CAL NVS:%s C:%s",savedAvailable?"YES":"NO",centerCaptured?"OK":"--");
        oled.setCursor(0,8); oled.print("   NOW  MIN  MAX");
        for(int i=0;i<4;i++) {
            oled.setCursor(0,16+i*8);
            oled.printf("%s %4u %4u %4u",names[i],raw[i],measured.minimum[i],measured.maximum[i]);
        }
        oled.setCursor(0,48); oled.print(displayMessage);
        oled.setCursor(0,56); oled.print("c:CTR s:SAVE g:GO");
    } else {
        oled.print("CONTROL: RX ECHO");
        for(int n=0;n<2;n++) {
            const Feedback& f=feedback[n];
            oled.setCursor(0,8+n*24);
            if(!txAttempted) oled.printf("%c:WAIT",'A'+n);
            else if(!f.received) oled.printf("%c:NO REPLY TX:%d",'A'+n,n==0?txResultA:txResultB);
            else oled.printf("%c:RX %s #%lu",'A'+n,f.status==0?"OK":"BAD",static_cast<unsigned long>(f.sequence));
            oled.setCursor(0,16+n*24);
            if(f.received) oled.printf("LX%5d LY%5d",f.axes[0],f.axes[1]);
            else oled.print("LX ---- LY ----");
            oled.setCursor(0,24+n*24);
            if(f.received) oled.printf("RX%5d RY%5d",f.axes[2],f.axes[3]);
            else oled.print("RX ---- RY ----");
        }
        oled.setCursor(0,56); oled.print("r:STOP / CALIBRATE");
    }
    // Show radio initialization failures on the calibration screen as well.
    if(!control && !radioReady && radioError!=0) {
        oled.fillRect(0,48,128,8,SSD1306_BLACK);
        oled.setCursor(0,48); oled.printf("nRF INIT ERR %d",radioError);
    }
    oled.display();
}

bool valid(const Calibration& c) {
    if(c.version != 1) return false;
    for(int i=0;i<4;i++) {
        // Require a meaningful sweep and a center inside the measured range.
        if(c.maximum[i]>4095 || int(c.maximum[i])-int(c.minimum[i])<500 ||
           int(c.center[i])-int(c.minimum[i])<100 ||
           int(c.maximum[i])-int(c.center[i])<100) return false;
    }
    return true;
}
void resetMeasurement() {
    measured={}; measured.version=1; centerCaptured=false;
    displayMessage="Sweep all axes";
    radioError=0;
    for(int i=0;i<4;i++) measured.minimum[i]=4095;
}
void help() {
    Serial.println("CALIBRATION: sweep BOTH sticks through their full range.");
    Serial.println("c: capture released center / s: save and start control");
    Serial.println("r: restart calibration / g: use saved calibration / h: help");
    Serial.printf("Saved calibration: %s\n",savedAvailable ? "available":"none");
}
bool load(Calibration& c) {
    Preferences p;
    if(!p.begin("joystick",true)) return false;
    bool ok=p.getBytesLength("cal")==sizeof(c) && p.getBytes("cal",&c,sizeof(c))==sizeof(c);
    p.end();
    return ok && valid(c);
}
bool save() {
    Preferences p;
    if(!p.begin("joystick",false)) return false;
    bool ok=p.putBytes("cal",&measured,sizeof(measured))==sizeof(measured);
    p.end();
    Calibration verify={};
    if(!ok || !load(verify) || memcmp(&verify,&measured,sizeof(measured))!=0) return false;
    saved=verify; savedAvailable=true;
    return true;
}
void enterControl() {
    if(!savedAvailable || !valid(saved)) return;
    SPI.begin(PIN_SCK,PIN_MISO,PIN_MOSI,PIN_CSN);
    // nRF24 returns STATUS during the command byte, not an extra byte.
    radioModule.spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS]=Module::BITS_0;
    int16_t state=radio.begin(2400,1000,-12,5);
    radioError=state; radioReady=state==RADIOLIB_ERR_NONE;
    if(state!=RADIOLIB_ERR_NONE) {
        Serial.printf("Radio init error: %d. Calibration retained; g to retry.\n",state);
        return;
    }
    active=saved; control=true; lastSend=millis(); txAttempted=false;
    feedback[0]={}; feedback[1]={};
    Serial.println("CONTROL: sending to A and B and waiting for echo. r: stop and recalibrate");
}
void captureCenter() {
    uint32_t sum[4]={};
    for(int sample=0;sample<64;sample++) {
        readJoystick();
        for(int i=0;i<4;i++) sum[i]+=raw[i];
        delay(5);
    }
    for(int i=0;i<4;i++) measured.center[i]=sum[i]/64;
    centerCaptured=true;
    displayMessage=valid(measured)?"READY: s to save":"Sweep more / c again";
    Serial.println("Center captured (64-sample average). s: save and start");
}
void commands() {
    while(Serial.available()) {
        char cmd=Serial.read();
        if(cmd>='A' && cmd<='Z') cmd+='a'-'A';
        if(cmd=='r') {
            if(control) radio.standby();
            control=false; resetMeasurement(); help();
        } else if(cmd=='h') {
            if(control) Serial.println("CONTROL: r to stop and recalibrate.");
            else help();
        } else if(!control) {
            if(cmd=='c') captureCenter();
            else if(cmd=='s') {
                radioError=0;
                if(!centerCaptured || !valid(measured)) {
                    displayMessage="Need sweep + center";
                    Serial.println("Not saved: sweep all axes, release sticks, then c.");
                } else if(!save()) {
                    displayMessage="NVS SAVE FAILED";
                    Serial.println("NVS save/verification failed. Still calibrating.");
                } else { displayMessage="SAVED"; Serial.println("Calibration saved and verified in NVS."); enterControl(); }
            } else if(cmd=='g') {
                if(savedAvailable) enterControl();
                else { displayMessage="No saved calibration"; Serial.println("No valid saved calibration. Calibrate first."); }
            }
        }
    }
}
int16_t normalize(int i) {
    int v=constrain(int(filtered[i]),int(active.minimum[i]),int(active.maximum[i]));
    int delta=v-active.center[i];
    int span=delta<0 ? active.center[i]-active.minimum[i] : active.maximum[i]-active.center[i];
    return int32_t(delta)*1000/span;
}
int16_t sendTo(const uint8_t* address,uint8_t* packet,size_t length) {
    int16_t state=radio.setTransmitPipe(address);
    if(state==RADIOLIB_ERR_NONE) state=radio.transmit(packet,length,0);
    return state;
}
int16_t exchange(int receiver,uint8_t* packet) {
    using namespace JoystickProtocol;
    Feedback& f=feedback[receiver];
    f.received=false;
    int16_t tx=sendTo(addresses[receiver],packet,requestSize);
    // Listen even if the hardware ACK was lost: the peer may have received it.
    int16_t state=radio.setReceivePipe(0,controllerAddress);
    if(state==RADIOLIB_ERR_NONE) state=radio.startReceive();
    uint32_t started=millis();
    while(state==RADIOLIB_ERR_NONE && millis()-started<40) {
        if(digitalRead(PIN_IRQ)==LOW) {
            size_t length=radio.getPacketLength();
            uint8_t reply[32]={};
            state=radio.readData(reply,sizeof(reply));
            if(state==RADIOLIB_ERR_NONE && length==replySize && reply[0]=='R' && reply[1]==1 &&
               reply[14]=='A'+receiver && reply[15]<=1 && sequenceOf(reply)==sequenceOf(packet) &&
               memcmp(reply+2,packet+2,12)==0) {
                f.received=true; f.status=reply[15]; f.sequence=sequenceOf(reply);
                for(int i=0;i<4;i++) f.axes[i]=axisOf(reply,i);
                break;
            }
            if(state==RADIOLIB_ERR_NONE) state=radio.startReceive();
        }
        delay(1);
    }
    radio.standby();
    return tx;
}
void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    for(uint8_t pin:axisPins) analogSetPinAttenuation(pin,ADC_11db);
    if(xTaskCreate(sampleJoystick,"joystickADC",2048,nullptr,1,nullptr)!=pdPASS) {
        Serial.println("ADC sampling task failed; restart required.");
        while(true) delay(1000);
    }
    while(!readJoystick()) delay(1);
    resetMeasurement(); savedAvailable=load(saved);
    initDisplay();
    Serial.println("Joystick: LX=32 LY=33 RX=34 RY=35 / 115200 baud");
    Serial.println("Filter: 8-sample moving average / 5ms sampling");
    help();
}
void loop() {
    readJoystick();
    for(int i=0;i<4;i++) {
        if(!control) {
            if(raw[i]<measured.minimum[i]) measured.minimum[i]=raw[i];
            if(raw[i]>measured.maximum[i]) measured.maximum[i]=raw[i];
        }
    }
    commands();
    if(!control) {
        if(millis()-lastPrint>=200) {
            lastPrint=millis();
            for(int i=0;i<4;i++) {
                Serial.printf("%s=%u min=%u max=%u",names[i],raw[i],measured.minimum[i],measured.maximum[i]);
                if(centerCaptured) Serial.printf(" center=%u",measured.center[i]);
                Serial.print(i==3 ? "\n":" | ");
            }
        }
    } else if(millis()-lastSend>=50) {
        lastSend=millis();
        // 14-byte protocol: 'J', version 1, uint32 sequence (little endian),
        // then int16 LX, LY, RX, RY (little endian, -1000..1000).
        uint8_t packet[14]={'J',1};
        uint32_t seq=sequence++;
        for(int b=0;b<4;b++) packet[2+b]=seq>>(8*b);
        int16_t axes[4];
        for(int i=0;i<4;i++) {
            axes[i]=normalize(i);
            uint16_t encoded=static_cast<uint16_t>(axes[i]);
            packet[6+i*2]=encoded&0xFF; packet[7+i*2]=encoded>>8;
        }
        int16_t a=exchange(0,packet);
        int16_t b=exchange(1,packet);
        txResultA=a; txResultB=b; txAttempted=true;
        if(millis()-lastPrint>=200) {
            lastPrint=millis();
            Serial.printf("CTRL #%lu LX=%d LY=%d RX=%d RY=%d | A TX=%d ECHO=%s | B TX=%d ECHO=%s\n",
                static_cast<unsigned long>(seq),axes[0],axes[1],axes[2],axes[3],a,
                feedback[0].received?(feedback[0].status==0?"OK":"BAD"):"TIMEOUT",b,
                feedback[1].received?(feedback[1].status==0?"OK":"BAD"):"TIMEOUT");
        }
    }
    drawDisplay();
    delay(5);
}
