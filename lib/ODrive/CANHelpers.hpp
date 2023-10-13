#pragma once

#include "can.hpp"
#include <stdint.h>
#include <array>
#include <algorithm>
#include <cstring>
#include <iterator>

#include <iterator>
template <typename T>
constexpr T can_getSignal(CANFrame& frame, const uint8_t startBit, const uint8_t length, const bool isIntel)
{
    // uint64_t tempVal = 0;
    // uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;

    // if (isIntel) {
    //     std::memcpy(&tempVal, &frame.data, sizeof(tempVal));
    //     tempVal = (tempVal >> startBit) & mask;
    // } else {
    //     std::reverse(std::begin(frame.data), std::end(frame.data));
    //     std::memcpy(&tempVal, &frame.data, sizeof(tempVal));
    //     tempVal = (tempVal >> (64 - startBit - length)) & mask;
    // }

    // T retVal = T();
    // std::memcpy(&retVal, &tempVal, sizeof(T));
    // return retVal;

    const auto byte_count = length / 8;
    uint8_t tempVal[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;

    if (isIntel) {
        std::memcpy(&tempVal[8-byte_count], &frame.data[startBit], byte_count);
    }

    T retVal = T();
    std::memcpy(&retVal, &tempVal[8-byte_count], sizeof(T));
    return retVal;
}

template <typename T>
constexpr void can_setSignal(CANFrame& frame, const T& val, const uint8_t startBit, const uint8_t length, const bool isIntel) {
    uint64_t valAsBits = 0;
    std::memcpy(&valAsBits, &val, sizeof(val));

    uint64_t mask = length < 64 ? (1ULL << length) - 1ULL : -1ULL;

    if (isIntel) {
        uint64_t data = 0;
        std::memcpy(&data, frame.data, sizeof(data));

        data &= ~(mask << startBit);
        data |= valAsBits << startBit;

        std::memcpy(frame.data, &data, sizeof(data));
    } else {
        uint64_t data = 0;
        std::reverse(std::begin(frame.data), std::end(frame.data));
        std::memcpy(&data, frame.data, sizeof(data));

        data &= ~(mask << (64 - startBit - length));
        data |= valAsBits << (64 - startBit - length);

        std::memcpy(frame.data, &data, sizeof(data));
        std::reverse(std::begin(frame.data), std::end(frame.data));
    }
}

// template<typename T>
// void can_setSignal(uint8_t* buf, const T& val, const uint8_t startBit, const uint8_t length, const bool isIntel, const float factor, const float offset) {
//     T scaledVal = static_cast<T>((val - offset) / factor);
//     can_setSignal<T>(buf, scaledVal, startBit, length, isIntel);
// }

template<typename T>
float can_getSignal(CANFrame frame, const uint8_t startBit, const uint8_t length, const bool isIntel, const float factor, const float offset) {
    T retVal = can_getSignal<T>(frame, startBit, length, isIntel);
    return (retVal * factor) + offset;
}

// template <typename T>
// float can_getSignal(can_Message_t buf, const can_Signal_t& signal) {
//     return can_getSignal<T>(buf, signal.startBit, signal.length, signal.isIntel, signal.factor, signal.offset);
// }

// template <typename T>
// void can_setSignal(can_Message_t& buf, const T& val, const can_Signal_t& signal) {
//     can_setSignal(buf, val, signal.startBit, signal.length, signal.isIntel, signal.factor, signal.offset);
// }