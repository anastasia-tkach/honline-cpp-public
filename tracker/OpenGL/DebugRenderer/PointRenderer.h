#pragma once
#include "tracker/Types.h"
#include "tracker/OpenGL/GeometricPrimitiveRenderer.h"

class PointRenderer : public GeometricPrimitiveRenderer {

public:
	PointRenderer(Matrix_3xN& points, std::string data_path);
	PointRenderer(const std::vector<Vector3>& points, const std::vector<Vector3>& colors, std::string data_path);
	PointRenderer(const std::vector<Vector3>& points, Vector3 color, std::string data_path);
    void render();
};
