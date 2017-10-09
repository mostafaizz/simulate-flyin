#pragma once

#include "Point3D.h"

class Mat3x3
{
private:
	Point3D row[3];
public:
	Mat3x3();
	Mat3x3(double a, double b, double c,
		double d, double e, double f,
		double g, double h, double i);
	Mat3x3(double elements[]);
	Mat3x3(Point3D rt[3]);
	Mat3x3(const Mat3x3& m);;
	Mat3x3& operator=(const Mat3x3& m);
	// return row vector
	Point3D& operator[](int index);
	// return column vector
	Point3D operator()(int index);
	Mat3x3 operator*(Mat3x3& m);
	Mat3x3 operator*(double factor);
	// matrix * (col) vector
	Point3D operator*(Point3D vec);
	Mat3x3 operator+(Mat3x3& m);
	Mat3x3 operator-(Mat3x3& m);
	void print();
};
