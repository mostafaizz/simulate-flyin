#pragma once
#include "Veretx3D.h"
#include <vector>
#include "misc.h"
using namespace std;

class TestingVerticalCylinder
{
public:
	///
	/*
	double cylinderR: Cylinder Radius
	double waveR: Wave Radius
	vector<double> zRef: reference values for z direction
	double rotStep: rotational step while creating the approximate cylinder
	*/
	//static vector<Vertex3D> createDeformedCylinderSine(double cylinderR, double waveR, const vector<double>& zRef, double rotStep);


	///
	/*
	double cylinderR: Cylinder Radius
	double waveR: Wave Radius
	vector<double> zRef: reference values for z direction
	double rotStep: rotational step while creating the approximate cylinder
	shift: is used to change the starting point (must be between 0 and waveR)
	*/
	//static vector<Vertex3D> createDeformedCylinderSharp(double cylinderR, double waveR, const vector<double>& zRef, double rotStep, double shift = 0);


	// this is only for fly-over visualization 
	static vector<double> simulateFlyOverPerspectiveMeasure(TubularObject& cylinder, vector<double>& zRef, vector<double>& edges, vector<double>& m_p_data /*output*/);

	// this is only for Equirectangular fly-over visualization 
	static vector<double> simulateEquirectangularFlyOverPerspectiveMeasure(TubularObject& cylinder, vector<double>& zRef, vector<double>& edges, vector<double>& m_p_data /*output*/);

	static void compareFlyoverEquirectangular(TubularObject& cylinder, vector<double> &zRef, vector<double>& edges, string name, bool half = false);

	//static vector<Vertex3D> testSharpCylinder(vector<double>& edges);

	//static vector<Vertex3D> testSineSurfaceCylinder(vector<double>& edges);
};