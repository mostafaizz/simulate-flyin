#pragma once
#include <limits>
#include <vector>

using namespace std;

enum { EMPTY = 0, VERTEX, FACE };

struct ZVertex {
	int type;
	int index;
	double zDepth; // the less the closer to the camera
	ZVertex() {
		clear();
	}
	void clear()
	{
		type = EMPTY;
		index = -1;
		zDepth = DBL_MAX;
	}
};


class ZBuffer {
	int width, hight;
	int x, y; // the starting location for width and height, usually would be -width/2 and -height/2
	double ppx, ppy; // the density in x and y directions (because this is integers and the reality is double)
	ZVertex **buffer;
	vector<bool> visVertices;

public:
	ZBuffer(int w, int h, int x, int y, double ppx, double ppy, int numVertices)
	{
		width = w;
		hight = h;
		this->x = x;
		this->y = y;
		this->ppx = ppx;
		this->ppy = ppy;
		buffer = new ZVertex*[h];
		for (int i = 0; i < h; i++)
		{
			buffer[i] = new ZVertex[w];
		}
		visVertices.resize(numVertices, false);
	}

	void clear()
	{
		for (int i = 0; i < hight; i++)
		{
			for (int j = 0; j < width; j++)
			{
				buffer[i][j].clear();
			}
		}
		visVertices.assign(visVertices.size(), false);
	}

	void insertFace()
};
