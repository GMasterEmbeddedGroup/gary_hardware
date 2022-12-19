#pragma once

namespace utils {

    typedef unsigned short ushort;
    typedef unsigned int uint;

    float half_to_float(const ushort x);
    ushort float_to_half(const float x);
}