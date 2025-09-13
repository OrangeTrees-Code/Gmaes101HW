#pragma once
#include "Bounds3.hpp"

struct BucketUnit
{
    Bounds3 bound;
    int primitiveCount = 0;

    BucketUnit() = default;
};