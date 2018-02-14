#pragma once
#include "tracker/Types.h"

struct TrackingError{
    float pull_error;
    float push_error;
	float weighted_error;
    static TrackingError zero(){ return {0, 0}; }
};

namespace energy{

class Energy{
protected:

public:
    static void rigid_only(LinearSystem& system);
    static VectorN solve(LinearSystem& system);
	static bool vector_has_nan(const VectorN & input);
	static bool matrix_has_nan(const Matrix_MxN & input);
};

}

