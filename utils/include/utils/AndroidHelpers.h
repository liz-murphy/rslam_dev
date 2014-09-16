#pragma once

// some math routines that are missing from android libm

inline double log2(double x)
{
    return (log(x) / log(2.0));
}

