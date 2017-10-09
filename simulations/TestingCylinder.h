#pragma once
#include <vector>
#include "Veretx3D.h"
#include "Mat3x3.h"
#include "TubularObject.h"
#include "misc.h"

class TestingCylinder
{
public:
	/*
	vector<double> cylinderR: is the radius for each circle in the the cylinder
	rotStep: rotational step while creating the approximate cylinder
	path: is the path used to create the deformed cylinder
	*/
	static vector<Vertex3D> createCylinderFromPath(vector<double>& cylinderR, double rotStep, vector<Point3D> &path, bool half = false, double rotationShift = 0);

	/*
	cylinderR: is the radius for the cylinder
	rotStep: rotational step while creating the approximate cylinder
	path: is the path used to create the deformed cylinder
	*/
	static vector<Vertex3D> createCylinderFromPath(double cylinderR, double rotStep, vector<Point3D> &path, bool half = false, double shift = 0);
	// this is only for fly-over visualization 
	static vector<double> TestingCylinder::simulateFlyOverPerspectiveMeasure(TubularObject & cylinder, vector<double>& edges, vector<double>& m_p_data,double rotShift = 0, string oupName = "");
	static vector<double> simulateFlythrough(TubularObject & cylinder, vector<double>& edges, vector<double>& m_p_data, double rotShift, string oupName);
	static vector<double> simulateFlythroughReverese(TubularObject & cylinder, vector<double>& edges, vector<double>& m_p_data, double rotShift, string oupName);
	// this is only for Equirectangular fly-over visualization 
	static vector<double> simulateEquirectangularFlyOverPerspectiveMeasure(TubularObject& cylinder, vector<double>& edges, vector<double>& m_p_data /*output*/, string oupName = "");

	static TubularObject testCylinderCycleHalf(double shift = 0.0, double step = pi / 5, double radius = 3, double cylR = 2);
	static TubularObject testCylinderFromFile(string fileName, double rotShift);
	static TubularObject testCylinderFromPath(double shift, double rotStep, double cylRadius, double length);
};