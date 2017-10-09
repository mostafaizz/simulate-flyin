#include "Mat3x3.h"

Mat3x3::Mat3x3() {// Default
}

Mat3x3::Mat3x3(double a, double b, double c, double d, double e, double f, double g, double h, double i) {
	row[0][0] = a;
	row[0][1] = b;
	row[0][2] = c;
	row[1][0] = d;
	row[1][1] = e;
	row[1][2] = f;
	row[2][0] = g;
	row[2][1] = h;
	row[2][2] = i;
}

Mat3x3::Mat3x3(double elements[]) {
	int index = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			row[i][j] = elements[index++];
		}
	}
}

Mat3x3::Mat3x3(Point3D rt[3]) {
	for (int i = 0; i < 3; i++)
	{
		row[i] = rt[i];
	}
}

Mat3x3::Mat3x3(const Mat3x3 & m) {
	for (int i = 0; i < 3; i++)
	{
		row[i] = m.row[i];
	}
}

void Mat3x3::print()
{
	for (int i = 0; i < 3; i++)
	{
		cout << row[i] << endl;
	}
}

Mat3x3& Mat3x3::operator=(const Mat3x3& m)
{
	for (int i = 0; i < 3; i++)
	{
		row[i] = m.row[i];
	}
	return *this;
}
// return row vector
Point3D& Mat3x3::operator[](int index)
{
	return row[index];
}
// return column vector
Point3D Mat3x3::operator()(int index)
{
	Point3D col;
	for (int i = 0; i < 3; i++)
	{
		col[i] = row[i][index];
	}
	return col;
}
Mat3x3 Mat3x3::operator*(Mat3x3& m)
{
	Mat3x3 res;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			res[i][j] = row[i].dot(m(j));
		}
	}
	return res;
}
Mat3x3 Mat3x3::operator*(double factor)
{
	Mat3x3 res;
	for (int i = 0; i < 3; i++)
	{
		res[i] = row[i] * factor;
	}
	return res;
}
// matrix * (col) vector
Point3D Mat3x3::operator*(Point3D vec)
{
	Point3D res;
	for (int i = 0; i < 3; i++)
	{
		res[i] = row[i].dot(vec);
	}
	return res;
}

Mat3x3 Mat3x3::operator+(Mat3x3& m)
{
	Mat3x3 res;
	for (int i = 0; i < 3; i++)
	{
		res[i] = row[i] + m[i];
	}
	return res;
}
Mat3x3 Mat3x3::operator-(Mat3x3& m)
{
	Mat3x3 res;
	for (int i = 0; i < 3; i++)
	{
		res[i] = row[i] - m[i];
	}
	return res;
}