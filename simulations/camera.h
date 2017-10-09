#pragma once
#include "Point3D.h"

class Camera
{
public:
	Point3D location;
	Point3D up;
	Point3D lookAt;
};

class PerspectiveCamera : public Camera
{
protected:
	double fovH, fovV; // field of view (horizontal and vertical)
	double f; // focal length
public:
	void setFovH(double fov)
	{
		this->fovH = fov;
		f = (sx / 2) / tan(fov / 2.0);
	}
	void setFovV(double fov)
	{
		this->fovV = fov;
		f = (sy / 2) / tan(fov / 2.0);
	}
	void setFocal(double f)
	{
		this->f = f;
		fovH = 2 * atan2(sx / 2, f);
		fovV = 2 * atan2(sy / 2, f);
	}
	double getFovV() const {
		return fovV;
	}
	double getFovH() const
	{
		return fovH;
	}
	double getFocalLength() const {
		return f;
	}
	double sx, sy; // sensor width and height
	double near, far; // near and far planes
};

class EquirectangularCamera : public Camera
{
public:
	double horizontalFOV; // field of view (horizontal maximum 360)
	double verticalFOV; // field of view (vertical maximum 180)
	double sx, sy; // sensor width and height
	double f; // focal length
	double near, far; // near and far planes
};