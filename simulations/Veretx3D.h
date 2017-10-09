#pragma once
#include "Point3D.h"
#include "Color.h"

struct Vertex3D
{
	Point3D pt; // point
	Point3D norm; // normal at this point
	Color color;
};

