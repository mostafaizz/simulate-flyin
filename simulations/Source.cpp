#include <iostream>
#include <algorithm>
#include <vector>
#include "TestingVerticalCylinder.h"
#include "TestingCylinder.h"
//#include <Windows.h>
//#include <GL/gl.h>
//#include <GL/glu.h>
//#include "GL\glut.h"
//#include "display.h"

using namespace std;
//
//
//
//#define KEY_ESCAPE 27
//
//
//
//typedef struct {
//	int width; 
//	int height;
//	char* title;
//
//	float field_of_view_angle;
//	float z_near;
//	float z_far;
//} glutWindow;
//
//glutWindow win;
//
//int total;
//float *Faces_Triangles; // number of vertices
//float *Normals;
//
//void display()
//{
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		     // Clear Screen and Depth Buffer
//	glLoadIdentity();
//	glTranslatef(0.0f, 0.0f, -1000.0f);
//
//	///*
//	//* Triangle code starts here
//	//* 3 verteces, 3 colors.
//	//*/
//	glBegin(GL_TRIANGLES);
//	glColor3f(0.0f, 0.0f, 1.0f);
//	glVertex3f(0.0f, 1.0f, 0.0f);
//	glColor3f(0.0f, 1.0f, 0.0f);
//	glVertex3f(-1.0f, -1.0f, 0.0f);
//	glColor3f(1.0f, 0.0f, 0.0f);
//	glVertex3f(1.0f, -1.0f, 0.0f);
//	glEnd();
//
//	glutSwapBuffers();
//}
//
//
//void initialize()
//{
//	glMatrixMode(GL_PROJECTION);												// select projection matrix
//	glViewport(0, 0, win.width, win.height);									// set the viewport
//	glMatrixMode(GL_PROJECTION);												// set matrix mode
//	glLoadIdentity();															// reset projection matrix
//	GLfloat aspect = (GLfloat)win.width / win.height;
//	gluPerspective(win.field_of_view_angle, aspect, win.z_near, win.z_far);		// set up a perspective projection matrix
//	glMatrixMode(GL_MODELVIEW);													// specify which matrix is the current matrix
//	glShadeModel(GL_SMOOTH);
//	glClearDepth(1.0f);														// specify the clear value for the depth buffer
//	glEnable(GL_DEPTH_TEST);
//	glDepthFunc(GL_LEQUAL);
//	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);						// specify implementation-specific hints
//	glClearColor(0.0, 0.0, 0.0, 1.0);											// specify clear values for the color buffers								
//}
//
//
//void keyboard(unsigned char key, int mousePositionX, int mousePositionY)
//{
//	switch (key)
//	{
//	case KEY_ESCAPE:
//		exit(0);
//		break;
//
//	default:
//		break;
//	}
//}


void testLineCellIntersection()
{
	while (true)
	{
		vector<Point3D> line, plan;
		while (line.size() < 2)
		{
			cout << "Enter space separated point for the line:" << endl;
			Point3D pt;
			cin >> pt;
			line.push_back(pt);
		}
		while (plan.size() < 3)
		{
			cout << "Enter space separated point for the plan:" << endl;
			Point3D pt;
			cin >> pt;
			plan.push_back(pt);
		}
		cout << "line is:" << endl;
		for (int i = 0; i < line.size(); i++)
		{
			cout << "(" << line[i] << ")\t";
		}
		cout << "\r\nPlan is:" << endl;
		for (int i = 0; i < plan.size(); i++)
		{
			cout << "(" << plan[i] << ")\t";
		}
		cout << "\r\nMisc::isLineIntersectPlan() = " << Misc::isLineIntersectPlan(line, plan) << endl;
	}
}
int main(int argc, char**argv)
{
	
	vector<double> edges;
	for (double e = -1; e <= 0 + epsilon; e += 0.05) {
		edges.push_back(e);
	}
	
	//testSharpCylinder(edges);
	//TestingVerticalCylinder::testSineSurfaceCylinder(edges);
	//testCylinderFromPath();
	double shift = pi / 2;
	string name = "testCylinderCycleHalf";
	TubularObject obj = TestingCylinder::testCylinderCycleHalf(shift, pi / 2, 3, 1);
	//TubularObject obj = TestingCylinder::testCylinderFromPath(shift, pi / 16, 1, 4 * pi);
	//TubularObject obj = TestingCylinder::testCylinderFromFile("centerLineAndRadius.txt", shift);
	//TubularObject obj("rings/part_12.ply", "Normals.txt");
	//TubularObject obj("cube.ply", "centerLine.txt");
	//obj.writePLYFileColorVertices("test.ply", false, true);

	
	//obj.writeVerticesToFile(name + "_norms.csv");
	
	/*vector<double> measure(obj.vertices.size(), 1);
	TestingCylinder::simulateEquirectangularFlyOverPerspectiveMeasure(obj, edges, measure, name);
	
	obj.colorVertices(measure, Misc::getColorPalette(), true);
	obj.writePLYFileColorVertices(name + "_colors_flat", false, true);*/
	////
	vector<double> measure1(obj.vertices.size(), 1);
	TestingCylinder::simulateFlyOverPerspectiveMeasure(obj, edges, measure1, shift, name);
	obj.colorVertices(measure1, Misc::getColorPalette(), false);
	obj.writePLYFileColorVertices(name + "_colors_flyover", false, true);
	//
	//vector<double> measure2(obj.vertices.size(), 1);
	//TestingCylinder::simulateFlythrough(obj, edges, measure2, shift, name);
	//obj.colorVertices(measure2, Misc::getColorPalette(), true);
	//obj.writePLYFileColorVertices(name + "_colors_flythrough", false, true);
	////////
	////////////vector<double> measure3(obj.vertices.size(), 1);
	//TestingCylinder::simulateFlythroughReverese(obj, edges, measure2, pi / 2, name);
	//obj.colorVertices(measure2, Misc::getColorPalette(), true);
	//obj.writePLYFileColorVertices(name + "_colors_flythrough_rev", false, true);

	/*TubularObject pal = TestingCylinder::testCylinderFromPath(0, 2 * pi / 3, 4, 8 * pi * Misc::getColorPalette().size());
	double m = -1;
	vector<double> pm;
	double ind = 0;
	int div = pal.vertices.size() / Misc::getColorPalette().size();
	for (int i = 0; i < pal.vertices.size(); i++)
	{
		pm.push_back(m);
		if(i % div == (div - 1))
		{
			m += (1.0 / Misc::getColorPalette().size());
		}
	}
	pal.writePLYFile("colors", pm, Misc::getColorPalette(), false, true, false);*/

	return 0;
}