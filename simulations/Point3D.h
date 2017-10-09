#pragma once
#include <cmath>
#include <iostream>
#include <smmintrin.h>

using namespace std;

const double pi = 3.14159265359;

#define USE_SSE4 1

class Point3D
{
#if USE_SSE4
	__m128 p;
#else
	float p[3];
#endif
public:

	Point3D(double x = 0, double y = 0, double z = 0);
	float x() const;
	float y() const;
	float z() const;

	Point3D operator+(const Point3D& p) const;

	Point3D operator-() const;

	Point3D operator-(const Point3D& p) const;

	Point3D operator*(const double num) const;

	float& operator[](int index);

	// dot product
	double dot(const Point3D& p) const;

	// cross product
	Point3D cross(const Point3D& p);

	// calculate L1 norm
	double getL1Norm();

	// calculate L2 Norm
	double getL2Norm();

	// normalize the values inside the vector
	void normalize();

	friend ostream& operator<<(ostream& ostr, Point3D& pt);
	friend istream& operator>>(istream& istr, Point3D& pt);
};