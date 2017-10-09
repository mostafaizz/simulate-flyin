#include "TestingVerticalCylinder.h"

///
/*
double cylinderR: Cylinder Radius
double waveR: Wave Radius
vector<double> zRef: reference values for z direction
double rotStep: rotational step while creating the approximate cylinder
*/

//vector<Vertex3D> TestingVerticalCylinder::createDeformedCylinderSine(double cylinderR, double waveR, const vector<double>& zRef, double rotStep)
//{
//	vector<Vertex3D> res;
//	for (int k = 0; k < zRef.size(); k++)
//	{
//		for (double theta = 0; theta < 2 * pi; theta += rotStep)
//		{
//			Vertex3D vtx;
//			vtx.pt.x = (cylinderR + waveR * sin(zRef[k])) * cos(theta);
//			vtx.pt.y = (cylinderR + waveR * sin(zRef[k])) * sin(theta);
//			vtx.pt.z = zRef[k];
//
//			vtx.norm.x = -2 * vtx.pt.x;
//			vtx.norm.y = -2 * vtx.pt.y;
//			vtx.norm.z = 2 * waveR * cos(vtx.pt.z) * (waveR * sin(vtx.pt.z) + cylinderR);
//			vtx.norm.normalize();
//			res.push_back(vtx);
//		}
//	}
//
//	return res;
//}

///
/*
double cylinderR: Cylinder Radius
double waveR: Wave Radius
vector<double> zRef: reference values for z direction
double rotStep: rotational step while creating the approximate cylinder
shift: is used to change the starting point (must be between 0 and waveR)
*/
//
//vector<Vertex3D> TestingVerticalCylinder::createDeformedCylinderSharp(double cylinderR, double waveR, const vector<double>& zRef, double rotStep, double shift)
//{
//	vector<Vertex3D> res;
//	for (int k = 0; k < zRef.size(); k++)
//	{
//		double temp = fmod(shift + zRef[k], waveR);
//		for (double theta = 0; theta < 2 * pi; theta += rotStep)
//		{
//			Vertex3D vtx;
//
//			vtx.pt.z = zRef[k];
//
//			if (temp > waveR / 2)
//			{
//				vtx.pt.x = (cylinderR + waveR - temp) * cos(theta);
//				vtx.pt.y = (cylinderR + waveR - temp) * sin(theta);
//
//				vtx.norm.z = -cylinderR - waveR + temp;
//			}
//			else
//			{
//				vtx.pt.x = (cylinderR + temp) * cos(theta);
//				vtx.pt.y = (cylinderR + temp) * sin(theta);
//
//				vtx.norm.z = cylinderR + temp;
//			}
//			vtx.norm.x = -vtx.pt.x;
//			vtx.norm.y = -vtx.pt.y;
//
//			vtx.norm.normalize();
//			res.push_back(vtx);
//		}
//	}
//
//	return res;
//}

// this is only for fly-over visualization 

vector<double> TestingVerticalCylinder::simulateFlyOverPerspectiveMeasure(TubularObject& cylinder, vector<double>& zRef, vector<double>& edges, vector<double>& m_p_data)
{
	PerspectiveCamera cam1, cam2;
	cam1.near = cam2.near = 0;
	cam1.far = cam2.far = 20;

	cam1.setFovH(2 * pi / 3);
	cam1.setFovV(2 * pi / 3);

	cam2.setFovH(2 * pi / 3);
	cam2.setFovV(2 * pi / 3);
	
	vector<Point3D> path1, path2;
	vector<Point3D> pathLookAt1, pathLookAt2;
	for (double k = 0; k < zRef.size(); k++)
	{
		path1.push_back(Point3D{ -2, 0, zRef[k] });
		path2.push_back(Point3D{ 2, 0, zRef[k] });

		pathLookAt1.push_back(Point3D{ 1, 0, 0 });
		pathLookAt2.push_back(Point3D{ -1, 0, 0 });
	}

	//Misc::calcVisMeasurePerspective(1, cylinder, &cam1, path1, pathLookAt1, m_p_data);
	//Misc::calcVisMeasurePerspective(2, cylinder, &cam2, path2, pathLookAt2, m_p_data);

	vector<double> cumMeasure = Misc::calcCumulativeHistogram(m_p_data, edges);

	return cumMeasure;
}

// this is only for Equirectangular fly-over visualization 

vector<double> TestingVerticalCylinder::simulateEquirectangularFlyOverPerspectiveMeasure(TubularObject& cylinder, vector<double>& zRef, vector<double>& edges, vector<double>& m_p_data)
{
	EquirectangularCamera cam;
	cam.near = 0;
	cam.far = 20;
	cam.verticalFOV = pi / 4;
	cam.horizontalFOV = 2 * pi;
	cam.up[2] = 1;
	vector<Point3D> path;
	vector<Point3D> pathLookAt;
	vector<Point3D> upDir;
	for (double k = 0; k < zRef.size(); k++)
	{
		path.push_back(Point3D{ 0, 0, zRef[k] });
		pathLookAt.push_back(Point3D{ 1, 0, 0 });
		upDir.push_back(Point3D{ 0,0,1 });
	}

	Misc::calcVisMeasureEquirectangular(cylinder, &cam, path, pathLookAt, upDir, m_p_data);

	vector<double> cumMeasure = Misc::calcCumulativeHistogram(m_p_data, edges);

	return cumMeasure;
}

void TestingVerticalCylinder::compareFlyoverEquirectangular(TubularObject& cylinder, vector<double>& zRef, vector<double>& edges, string name, bool half)
{
	//Misc::writeCylinderPLYFile(name, cylinder.vertices, cylinder.vertices.size() / zRef.size(), half);

	vector<double> m_p_data_flyover(cylinder.vertices.size(), 1);
	vector<double> m_p_data_flat(cylinder.vertices.size(), 1);

	vector<double> cumMeasureFlyOver = simulateFlyOverPerspectiveMeasure(cylinder, zRef, edges, m_p_data_flyover);
	vector<double> cumMeasureFlat = simulateEquirectangularFlyOverPerspectiveMeasure(cylinder, zRef, edges, m_p_data_flat);

	ofstream oup1(name + ".csv");
	for (int i = 0; i < cylinder.vertices.size(); i++)
	{
		oup1 << cylinder.vertices[i].pt << ", " << cylinder.vertices[i].norm << "," << m_p_data_flyover[i] << "," << m_p_data_flat[i] << endl;
	}
	oup1.close();

	// printing output
	ofstream oupFlyOver("oupFlyOver_" + name + ".csv");
	ofstream oupFlat("oupFlat_" + name + ".csv");
	for (int i = 0; i < cumMeasureFlat.size(); i++)
	{
		oupFlyOver << edges[i] << "," << cumMeasureFlyOver[i] << endl;
		oupFlat << edges[i] << "," << cumMeasureFlat[i] << endl;
	}
	oupFlyOver.close();
	oupFlat.close();
}

//vector<Vertex3D> TestingVerticalCylinder::testSharpCylinder(vector<double>& edges)
//{
//	vector<double> zRef;
//	for (double k = 0; k <= 2; k += 0.05) // sharp test
//	{
//		zRef.push_back(k);
//	}
//
//	vector<Vertex3D> cylinderSharp = createDeformedCylinderSharp(2, 2, zRef, pi / 12, 1);
//	compareFlyoverEquirectangular(cylinderSharp, zRef, edges, "CylinderSharp");
//	return cylinderSharp;
//}
//
//vector<Vertex3D> TestingVerticalCylinder::testSineSurfaceCylinder(vector<double>& edges)
//{
//	vector<double> zRef;
//	for (double k = 0; k <= pi; k += pi / 4)
//	{
//		zRef.push_back(k);
//	}
//	vector<Vertex3D> cylinder = createDeformedCylinderSine(2, 0, zRef, pi / 12);
//	compareFlyoverEquirectangular(cylinder, zRef, edges, "CylinderSineSurface");
//	return cylinder;
//}
