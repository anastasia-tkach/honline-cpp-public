#pragma once
#include "kernel.h"
#include <thrust/device_vector.h>
#include <thrust/copy.h>

using namespace cudax;

//-----------------------------------------------------------------//

struct ConstraintTypeFunctor{
    uchar* opencv_image_d_raw;
    int2* cnstr_indexes;
    float* e_raw;

    ConstraintTypeFunctor(uchar *opencv_image_d_raw, float* e_raw){
        this->opencv_image_d_raw = opencv_image_d_raw;
        this->e_raw = e_raw;
    }

};


