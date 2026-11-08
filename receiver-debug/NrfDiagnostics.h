#pragma once
#include <Arduino.h>
#include <SPI.h>

// Raw SPI diagnostics, run with CE low before RadioLib initialization.
inline uint8_t nrfRegister(uint8_t reg, uint32_t hz) {
    SPI.beginTransaction(SPISettings(hz,MSBFIRST,SPI_MODE0));
    digitalWrite(5,LOW);
    SPI.transfer(reg & 0x1F);
    uint8_t value=SPI.transfer(0xFF);
    digitalWrite(5,HIGH);
    SPI.endTransaction();
    return value;
}
inline void nrfActivateFeatures(uint32_t hz) {
    SPI.beginTransaction(SPISettings(hz,MSBFIRST,SPI_MODE0));
    digitalWrite(5,LOW); SPI.transfer(0x50); SPI.transfer(0x73); digitalWrite(5,HIGH);
    SPI.endTransaction();
    delayMicroseconds(100);
    Serial.printf("ACTIVATE 73 at %lu Hz: FEATURE=%02X AW=%02X STATUS=%02X\n",
        (unsigned long)hz,nrfRegister(0x1D,hz),nrfRegister(3,hz),nrfRegister(7,hz));
}
inline void nrfWrite(uint8_t reg,uint8_t value,uint32_t hz) {
    SPI.beginTransaction(SPISettings(hz,MSBFIRST,SPI_MODE0));
    digitalWrite(5,LOW);
    SPI.transfer(0x20 | (reg & 0x1F));
    SPI.transfer(value);
    digitalWrite(5,HIGH);
    SPI.endTransaction();
    delayMicroseconds(100);
}
inline uint8_t nrfSoftByte(uint8_t value) {
    uint8_t result=0;
    for(int bit=7;bit>=0;bit--) {
        digitalWrite(23,(value>>bit)&1);
        delayMicroseconds(50);
        digitalWrite(18,HIGH);
        delayMicroseconds(50);
        result=(result<<1)|digitalRead(19);
        digitalWrite(18,LOW);
    }
    return result;
}
inline uint8_t nrfSoftRead(uint8_t reg) {
    digitalWrite(5,LOW); delayMicroseconds(50);
    uint8_t status=nrfSoftByte(reg & 0x1F);
    uint8_t result=nrfSoftByte(0xFF);
    digitalWrite(5,HIGH); delayMicroseconds(50);
    Serial.printf("SOFT SPI reg=%02X status=%02X value=%02X\n",reg,status,result);
    return result;
}
inline void nrfSoftProbe() {
    SPI.end();
    pinMode(18,OUTPUT); digitalWrite(18,LOW);
    pinMode(23,OUTPUT); digitalWrite(23,LOW);
    for(uint8_t mode: {uint8_t(INPUT),uint8_t(INPUT_PULLUP),uint8_t(INPUT_PULLDOWN)}) {
        pinMode(19,mode); delay(2);
        Serial.printf("MISO mode=%s CSN_HIGH=%d\n",mode==INPUT?"INPUT":mode==INPUT_PULLUP?"PULLUP":"PULLDOWN",digitalRead(19));
        nrfSoftRead(3); nrfSoftRead(0); nrfSoftRead(7);
    }
    pinMode(19,INPUT);
    SPI.begin(18,19,23,5);
}
inline void diagnoseNrf() {
    pinMode(5,OUTPUT); digitalWrite(5,HIGH);
    pinMode(4,OUTPUT); digitalWrite(4,LOW);
    delay(100);
    for(uint32_t hz: {8000000UL,1000000UL,250000UL,100000UL}) {
        uint8_t aw=nrfRegister(3,hz), config=nrfRegister(0,hz), status=nrfRegister(7,hz);
        Serial.printf("SPI %lu Hz: CONFIG=%02X STATUS=%02X AW=%02X RF_CH=%02X FEATURE=%02X DYNPD=%02X\n",
            (unsigned long)hz,config,status,aw,nrfRegister(5,hz),nrfRegister(0x1D,hz),nrfRegister(0x1C,hz));
        if(aw<1 || aw>3 || status==0xFF) {
            Serial.println("SPI invalid register response; skipping write test");
            continue;
        }
        uint8_t original=nrfRegister(5,hz);
        int passed=0;
        for(int i=0;i<10;i++) {
            uint8_t expected=(i%2)?0x55:0x2A;
            nrfWrite(5,expected,hz);
            uint8_t actual=nrfRegister(5,hz);
            if(actual==expected) passed++;
            else Serial.printf("SPI RF_CH mismatch: write=%02X read=%02X\n",expected,actual);
        }
        nrfWrite(5,original,hz);
        Serial.printf("SPI write/read passed %d/10\n",passed);
    }
    nrfSoftProbe();
    // Some nRF24L01+ variants lock FEATURE until ACTIVATE 0x73.
    // Only try it when a normal writable register has been verified.
    constexpr uint32_t hz=1000000;
    uint8_t aw=nrfRegister(3,hz), original=nrfRegister(5,hz);
    if(aw<1 || aw>3) return;
    nrfWrite(5,0x2A,hz);
    bool basicOk=nrfRegister(5,hz)==0x2A;
    nrfWrite(5,original,hz);
    if(!basicOk) return;
    nrfWrite(0x1D,4,hz);
    if(nrfRegister(0x1D,hz)!=4) {
        Serial.println("FEATURE locked: trying ACTIVATE 73");
        nrfActivateFeatures(hz);
        nrfWrite(0x1D,4,hz);
    }
    nrfWrite(0x1C,0x3F,hz);
    Serial.printf("Dynamic payload registers: FEATURE=%02X (want 04) DYNPD=%02X (want 3F)\n",
        nrfRegister(0x1D,hz),nrfRegister(0x1C,hz));
}
