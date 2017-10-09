#pragma once
#include <iostream>

using std::ostream;
using std::istream;

class Color
{
public:
	Color() { r = g = b = 0; };
	Color(unsigned char r, unsigned char g, unsigned char b);
	Color(const Color& color);
	void operator=(const Color& color);
	bool operator<(const Color& color)  const;
	bool operator==(const Color& color) const;
	~Color();
	unsigned char r, g, b;
	friend ostream& operator<<(ostream& ostr, const Color& color);
	friend istream& operator>>(istream& istr, Color& color);
};


