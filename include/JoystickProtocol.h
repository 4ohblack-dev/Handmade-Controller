#pragma once
#include <stdint.h>
#include <stddef.h>

namespace JoystickProtocol {
constexpr size_t requestSize = 14, replySize = 16;
constexpr uint8_t controllerAddress[5] = {0xC3,0x34,0x56,0x78,0x90};
constexpr uint8_t addresses[2][5] = {{0x12,0x34,0x56,0x78,0x90},{0xB2,0x34,0x56,0x78,0x90}};
inline uint32_t sequenceOf(const uint8_t* p) {
    return uint32_t(p[2]) | (uint32_t(p[3])<<8) | (uint32_t(p[4])<<16) | (uint32_t(p[5])<<24);
}
inline int16_t axisOf(const uint8_t* p, int i) {
    uint16_t v=uint16_t(p[6+i*2]) | (uint16_t(p[7+i*2])<<8);
    return v<0x8000 ? int16_t(v) : int16_t(int32_t(v)-65536);
}
inline bool axesValid(const uint8_t* p) {
    for(int i=0;i<4;i++) if(axisOf(p,i)<-1000 || axisOf(p,i)>1000) return false;
    return true;
}
}
