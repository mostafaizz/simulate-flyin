#pragma once
#include "TubularObject.h"
#include "camera.h"
#include <vector>
#include <fstream>
#include <algorithm>

using namespace std;
const double epsilon = 0.000001;

class Misc
{
public:
	static int pnPoly(vector<Point3D*>& poly, Point3D & pt);
	// faceInd is the face being tested right now
	static bool checkIntersection(TubularObject& tubular, int halfTag, const int j, const Point3D& loc, vector<pair<int, double> > & tmpM);

	static bool checkIntersection1(TubularObject & tubular, int halfTag, const int vInd, vector<pair<int, double>>& tmpM, vector<double>& tmpNumerator);

	static bool checkIntersection2(TubularObject & tubular, int halfTag, const int vInd, vector<pair<int, double>>& tmpM);

	// calculate visualization measure using a perspective camera
	// assume all directions are normalized already
	// halfTag is the tag for the half we are working on (0 means no halves, 1 for the first half , 2 for the second half)
	static void calcVisMeasurePerspective(
		int halfTag,
		TubularObject& data,
		const PerspectiveCamera* cam,
		const vector<Point3D>& path,
		const vector<Point3D>& pathLookAt,
		vector<double>& output,
		bool backfaceCull = false);

	static Mat3x3 getCamRotationMatrix(const Point3D & lookDir,const Point3D & upDir);

	static void calcVisMeasurePerspective1(int halfTag, TubularObject & tubular, const PerspectiveCamera * cam, 
		const vector<Point3D>& path, const vector<Point3D>& pathLookAt, const vector<Point3D>& up, vector<double>& output);

	static void calcVisMeasurePerspective2(int halfTag, TubularObject & tubular_, const PerspectiveCamera * cam, const vector<Point3D>& path, const vector<Point3D>& pathLookAt, const vector<Point3D>& up, vector<double>& output);

	// calculate visualization measure using a panoramic camera moving along centerline of a cylinder
	// assume all directions are normalized already
	static void calcVisMeasureEquirectangular(
		TubularObject& data,
		const EquirectangularCamera* cam,
		const vector<Point3D>& path,
		const vector<Point3D>& pathLookAt,
		const vector<Point3D>& upDir,
		vector<double>& output);

	// calcualte cumulative histogram for the given data using the given histogram bin locations
	// this function ignores the values greater than the largest edge
	static vector <double> calcCumulativeHistogram(const vector<double>& data1, const vector<double>& edges);

	static vector<Color> getColorPalette();

	//static void writeCylinderPLYFile(string fileName, vector<Vertex3D> cylinder, int cycleSize, bool half = false);

	// get triangle area
	static double getTriangleArea(vector<Point3D>& tri);
	static double getTriangleArea(const Point3D & p0, const  Point3D & p1, const  Point3D & p2);

	static double getTriangleArea2(const Point3D & p0, const Point3D & p1, const Point3D & p2);

	
	static double testPointInTriangle(const Point3D & p0, const Point3D & p1, const Point3D & p2);

	// line: is exactly two points (no check)
	// plan: is exactly 3 points (no check)
	static bool isLineIntersectPlan(vector<Point3D>& line, vector<Point3D>& plan);

	// line: is exactly two points (no check)
	// plan: is exactly 3 points (no check)
	static bool isLineIntersectPlan(const Point3D& line0, const Point3D& line1, const  Point3D& plan0, const  Point3D& plan1, const  Point3D& plan2);
	static bool isLineIntersectPlan(const Point3D & line0, const Point3D & line1, const Point3D & plan0, const Point3D & plan1, const Point3D & plan2, const Point3D & plan3);
	static bool isLineIntersectPlan(const Point3D & line0, const Point3D & line1, const Point3D & plan0, const Point3D & plan1, const Point3D & plan2, const Point3D & plan3, const Point3D & normal);
	static bool isLineIntersectPlan(const Point3D & line0, const Point3D & dir, double lineLength,
		const Point3D & plan0, const Point3D & plan1, const Point3D & plan2, const Point3D & plan3, const Point3D & normal,
		double denom, double numerator);
};