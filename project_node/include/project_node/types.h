#ifndef TYPES_H
#define TYPES_H

#include <cstdint>
#include <vector>

using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;

using float32 = float;
using float64 = double;

enum VelocityVoltageOutputType {
	TYPE_0_5_V = 0,
	TYPE_1_5_V = 1,
	TYPE_0_10_V = 2,
	TYPE_2_10_V = 3,
	TYPE_0_20_mA = 4,
	TYPE_4_20_mA = 5
};

#endif //TYPES_H