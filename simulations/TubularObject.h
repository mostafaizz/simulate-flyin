#pragma once

#include "Mat3x3.h"
#include "Veretx3D.h"
#include <vector>
#include <fstream>
#include <unordered_map>

using namespace std;

struct TubularObject;

struct Face
{
	vector<int> indeces; // the indeces for the referenced vertices
	int id; // the id or index of the face
	int tag; // tag to show divisions of the object
	int size; // number of vertices
	Point3D center;
	Point3D normal;
	double totalArea;
	Color color;
	//
	TubularObject * obj;
	
	vector<Point3D*> points; // this is redundant information for the sake of optimizing the speed

	bool isPointInside(const Point3D & interPt) const;

	friend ostream& operator<<(ostream& ostr, Face& pt);
	friend istream& operator >> (istream& istr, Face& pt);
};

struct TubularObject
{
	vector<Vertex3D> vertices;
	vector<unordered_map<int, bool> > vertexFaces; // the index is the key
	// faces as indeces in the veretices
	vector<Face> faces;
	bool vertColor, faceColor;
	// faces normals
	//vector<Point3D> faceNormals;
	// centerline
	vector<Point3D> centerline;
	// calculated axis at each centerline point
	vector<Point3D> tangents;
	vector<Point3D> xAxis;
	vector<Point3D> yAxis;
	// number of vertices in one circle
	int cycleSize;
	// maximum Radius
	double maxRadius, minRadius;
	Point3D calcSurfacePointLocation(int centerLineInd, double rotationAngle, double radius);
	// assume the centerline is already intialized
	void calcCenterlineAxes();
	void init(vector<double>& cylinderR, double rotStep, vector<Point3D>& centerline, bool half, double rotationShift);
	// constructors
	/*
	Getting the up direction consistent using Sabry method
	vector<double> cylinderR: is the radius for each circle in the the cylinder
	rotStep: rotational step while creating the approximate cylinder
	path: is the path used to create the deformed cylinder
	*/
	TubularObject(const TubularObject & obj);
	TubularObject(vector<double>& cylinderR, double rotStep, vector<Point3D> &centerline, bool half = false, double rotationShift = 0);
	// read ply file ascii encoding, assuming the halves have different face colors
	TubularObject(string plyFileName, string centerFileName);
	// assume points and faces already exists
	void calcVertexFaces();
	void operator=(const TubularObject& obj);
	/*
	cylinderR: is the radius for the cylinder
	rotStep: rotational step while creating the approximate cylinder
	path: is the path used to create the deformed cylinder
	*/
	TubularObject(double cylinderR, double rotStep, vector<Point3D> &centerline, bool half = false, double shift = 0);
	// calculate the faces
	void calcFaces(bool open = false);

	void calcFaceNormals();

	void transform(Point3D translation, Mat3x3 rotation);

	void colorVertices(vector<double> measure, vector<Color> colors, bool logScale);

	void writePLYFile(string name);
	void writePLYFileColorVertices(string name, bool halves = false, bool openWithMeshLab = false);
	void writeOBJFileWithMaterial(string name, bool openWithMeshLab);
	void writeVerticesToFile(string fileName);

	int getNumberOfFaces1stHalf();
	int getNumberOfFaces2ndHalf();
	double getFaceArea(const int faceInd, const int ind0, const int ind1, const int ind2) const;
};