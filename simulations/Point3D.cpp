#include "Point3D.h"
#include "Mat3x3.h"

Point3D::Point3D(double x, double y, double z)
{
#if USE_SSE4
	p.m128_f32[0] = x;
	p.m128_f32[1] = y;
	p.m128_f32[2] = z;
#else
	p[0] = x;
	p[1] = y;
	p[2] = z;
#endif
}

float Point3D::x() const {
#if USE_SSE4
	return p.m128_f32[0];
#else
	return p[0];
#endif
}

float Point3D::y() const {
#if USE_SSE4
	return p.m128_f32[1];
#else
	return p[1];
#endif
}

float Point3D::z() const {
#if USE_SSE4
	return p.m128_f32[2];
#else
	return p[2];
#endif
}

Point3D Point3D::operator+(const Point3D& p1) const
{
	Point3D res;
	for (int i = 0; i < 3; i++)
	{
#if USE_SSE4
		res.p.m128_f32[i] = p.m128_f32[i] + p1.p.m128_f32[i];
#else
		res.p[i] = p[i] + p1.p[i];
#endif
		
	}
	return res;
}

Point3D Point3D::operator-() const
{
	Point3D res;
	for (int i = 0; i < 3; i++)
	{
#if USE_SSE4
		res.p.m128_f32[i] = -p.m128_f32[i];
#else
		res.p[i] = -p[i];
#endif
	}
	return res;
}

Point3D Point3D::operator-(const Point3D& p) const
{
	return (*this + (-p));
}

Point3D Point3D::operator*(const double num) const
{
	Point3D res;
	for (int i = 0; i < 3; i++)
	{
#if USE_SSE4
		res.p.m128_f32[i] = p.m128_f32[i] * num;
#else
		res.p[i] = p[i] * num;
#endif
	}
	return res;
}

float& Point3D::operator[](int index)
{
#if USE_SSE4
	return p.m128_f32[index];
#else
	return p[index];
#endif
}
// normalize the values inside the vector
double Point3D::dot(const Point3D & p1) const
{
#if USE_SSE4
	const int mask = 0x71;
	__m128 res = _mm_dp_ps(p, p1.p, mask);
	return res.m128_f32[0];
#else
	double res[3];
	for (int i = 0; i < 3; i++)
	{
		res[i] = p[i] * p1.p[i];
	}
	return (res[0] + res[1] + res[2]);
#endif
}


// calculate L2 Norm
Point3D Point3D::cross(const Point3D & p1)
{
	Point3D res;
#if USE_SSE4
	res.p.m128_f32[0] = p.m128_f32[1] * p1.p.m128_f32[2] - p.m128_f32[2] * p1.p.m128_f32[1];
	res.p.m128_f32[1] = p.m128_f32[2] * p1.p.m128_f32[0] - p.m128_f32[0] * p1.p.m128_f32[2];
	res.p.m128_f32[2] = p.m128_f32[0] * p1.p.m128_f32[1] - p.m128_f32[1] * p1.p.m128_f32[0];
#else
	res.p[0] = p[1] * p1.p[2] - p[2] * p1.p[1];
	res.p[1] = p[2] * p1.p[0] - p[0] * p1.p[2];
	res.p[2] = p[0] * p1.p[1] - p[1] * p1.p[0];
#endif
	return res;
}



// calculate L1 norm
double Point3D::getL1Norm()
{
#if USE_SSE4
	return abs(p.m128_f32[0]) + abs(p.m128_f32[1]) + abs(p.m128_f32[2]);
#else
	return abs(p[0]) + abs(p[1]) + abs(p[2]);
#endif
}


// cross product
double Point3D::getL2Norm()
{
	return sqrt(this->dot(*this));
}



// dot product
void Point3D::normalize()
{
	double norm = getL2Norm();
	if (norm > 0)
	{
		norm = 1 / norm;
		for (int i = 0; i < 3; i++)
		{
#if USE_SSE4
			p.m128_f32[i] *= norm;
#else
			p[i] *= norm;
#endif
		}
	}
}


// (row) vector * matrix
Point3D operator*(Point3D& vec, Mat3x3& m)
{
	Point3D res;
	for (int i = 0; i < 3; i++)
	{
		res[i] = vec.dot(m(i));
	}
	return res;
}
// operator overloading for writing data
ostream& operator<<(ostream& ostr, Point3D& pt)
{
#if USE_SSE4
	ostr << pt.p.m128_f32[0] << " " << pt.p.m128_f32[1] << " " << pt.p.m128_f32[2];
#else
	ostr << pt.p[0] << " " << pt.p[1] << " " << pt.p[2];
#endif
	return ostr;
}


// operator overloading for reading data
istream& operator>>(istream& istr, Point3D& pt)
{
#if USE_SSE4
	istr >> pt.p.m128_f32[0] >> pt.p.m128_f32[1] >> pt.p.m128_f32[2];
#else
	istr >> pt.p[0] >> pt.p[1] >> pt.p[2];
#endif
	return istr;
}
