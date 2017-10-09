#include "Color.h"



Color::Color(unsigned char r, unsigned char g, unsigned char b)
{
	this->r = r;
	this->g = g;
	this->b = b;
}

Color::Color(const Color & color)
{
	*this = color;
}

void Color::operator=(const Color & color)
{
	this->r = color.r;
	this->g = color.g;
	this->b = color.b;
}

bool Color::operator<(const Color & color) const
{
	if (r < color.r || g < color.g || b < color.b)
	{
		return true;
	}
	return false;
}

bool Color::operator==(const Color & color) const
{
	if (r == color.r && g == color.g && b == color.b)
	{
		return true;
	}
	return false;
}

Color::~Color()
{
}

ostream & operator<<(ostream& ostr, const Color& color)
{
	// TODO: insert return statement here
	ostr << (int)color.r << " " << (int)color.g << " " << (int)color.b;

	return ostr;
}

istream & operator >> (istream & istr, Color& color)
{
	// TODO: insert return statement here
	int r, g, b;
	istr >> r >> g >> b;
	
	color.r = r;
	color.g = g;
	color.b = b;

	return istr;
}
