#include "TestingCylinder.h"
#include <iostream>

using namespace std;

/*
vector<double> cylinderR: is the radius for each circle in the the cylinder
rotStep: rotational step while creating the approximate cylinder
path: is the path used to create the deformed cylinder
*/

vector<Vertex3D> TestingCylinder::createCylinderFromPath(vector<double>& cylinderR, double rotStep, vector<Point3D>& path, bool half, double rotationShift)
{
	vector<Vertex3D> oup;
	double cycle = half ? pi : 2 * pi;
	double cumulativeDistance = 0;
	for (int i = 1; i < path.size() - 1; i++)
	{
		Point3D dir = path[i + 1] - path[i - 1];
		// add the distance to cumulative distance
		cumulativeDistance += dir.getL2Norm();
		// then normalize the vector
		dir.normalize();
		// find the rotation matrix
		// follownig the question in https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
		// using b == dir = [x, y, z], and a = [0, 0, 1] and rotating a to match b
		// the rotation matrix is as follows [1 0 x/(1 + z); 0 1 y/(1 + z);-x/(1 + z) -y/(1 + z) 1]
		Mat3x3 I(1, 0, 0, 0, 1, 0, 0, 0, 1);
		Point3D k = { 0, 0, 1 };
		Point3D v = k.cross(dir);
		double s = v.getL2Norm();
		double c = k.dot(dir);
		Mat3x3 vx(0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0);
		Mat3x3 rotMatrix = I + vx;
		Mat3x3 tmp1 = (vx * vx) * ((1 - c) / (s * s));
		rotMatrix = rotMatrix + tmp1;

		for (double theta = 0; theta < cycle; theta += rotStep)
		{
			Vertex3D vtx;

			vtx.pt = Point3D(0, cylinderR[i] * cos(theta + rotationShift), cylinderR[i] * sin(theta + rotationShift));

			//double factor = vtx.pt.getL2Norm();
			//vtx.pt.normalize();

			vtx.pt = (rotMatrix * vtx.pt) + path[i];

			oup.push_back(vtx);
		}
	}

	return oup;
}

/*
cylinderR: is the radius for the cylinder
rotStep: rotational step while creating the approximate cylinder
path: is the path used to create the deformed cylinder
*/

vector<Vertex3D> TestingCylinder::createCylinderFromPath(double cylinderR, double rotStep, vector<Point3D>& path, bool half, double shift)
{
	vector<double> rVector(path.size(), cylinderR);
	return createCylinderFromPath(rVector, rotStep, path, half, shift);
}

// the rotational shift is used to rotate the cutting plane of for the Fly over
vector<double> TestingCylinder::simulateFlyOverPerspectiveMeasure(TubularObject & cylinder, vector<double>& edges, vector<double>& m_p_data, double rotShift, string oupName)
{
	PerspectiveCamera cam;
	cam.near = 0;// 1.5 * cylinder.maxRadius;
	cam.far = cylinder.maxRadius * 2.5;
	cam.sy = cam.sx = 20 * cylinder.maxRadius;
	
	//cam.fov = pi / 2;
	//cam.setFocal(3 * cylinder.maxRadius);
	cam.setFovH(pi / 2);
	cam.setFovV(pi / 2);

	cout << "cam.getFovH() = " << cam.getFovH() * 180 / pi << endl;
	cout << "cam.getFocalLength() = " << cam.getFocalLength() << endl;
	vector<Point3D> path1, path2;
	vector<Point3D> lookAt1, lookAt2;
	vector<Point3D> up1, up2;

	for (int i = 0; i < cylinder.xAxis.size(); i++)
	//for(int i = 0;i < 1;i++)
	{
		path1.push_back(cylinder.calcSurfacePointLocation(i, rotShift - pi/2, 1.5 * cylinder.maxRadius));
		path2.push_back(cylinder.calcSurfacePointLocation(i, rotShift + pi / 2, 1.5 * cylinder.maxRadius));

		Point3D look1 = cylinder.centerline[i] - *(path1.end() - 1);
		look1.normalize();
		lookAt1.push_back(look1);
		
		Point3D look2 = cylinder.centerline[i] - *(path2.end() - 1);
		look2.normalize();
		lookAt2.push_back(look2);

		up1.push_back(cylinder.tangents[i]);
		up2.push_back(cylinder.tangents[i]);
	}
	Misc::calcVisMeasurePerspective(0, cylinder, &cam, path1, lookAt1, m_p_data, true);
	Misc::calcVisMeasurePerspective(0, cylinder, &cam, path2, lookAt2, m_p_data, true);

	/*Misc::calcVisMeasurePerspective1(0, cylinder, &cam, path1, lookAt1, up1, m_p_data);
	Misc::calcVisMeasurePerspective1(0, cylinder, &cam, path2, lookAt2, up2, m_p_data);*/

	vector<double> cumMeasure = Misc::calcCumulativeHistogram(m_p_data, edges);
	if (oupName.length() > 0)
	{
		// printing output
		ofstream oupFlat("oupFlyover_" + oupName + ".csv");
		cout << "oupFlyover_" << oupName.c_str() << ".csv" << endl;
		for (int i = 0; i < cumMeasure.size(); i++)
		{
			oupFlat << edges[i] << "," << cumMeasure[i] << endl;
		}
		oupFlat.close();
	}
	return cumMeasure;
}

// the rotational shift is used to rotate the camera
vector<double> TestingCylinder::simulateFlythrough(TubularObject & cylinder, vector<double>& edges, vector<double>& m_p_data, double rotShift, string oupName)
{
	PerspectiveCamera cam;
	cam.near = 0.1;// cylinder.maxRadius - epsilon;
	cam.far = 75;// cylinder.maxRadius * 7 + epsilon;
	cam.sx = cam.sy = 200;
	cam.setFovH(2 * pi / 3);
	cam.setFovV(2 * pi / 3);
	//cam.f = 2 * cylinder.maxRadius + epsilon;

	//cout << "cos(cam.fov / 2) = " << cos(cam.fov / 2) << endl;
	vector<Point3D> path;
	vector<Point3D> lookAt;
	vector<Point3D> up;
	for (int i = 0; i < cylinder.centerline.size(); i++)
	//for (int i = 1; i < 2; i++)
	{
		path.push_back(cylinder.centerline[i]);
		
		lookAt.push_back(cylinder.tangents[i]);
		(lookAt.end() - 1)->normalize();

		up.push_back(cylinder.xAxis[i]);
	}
	Misc::calcVisMeasurePerspective(0, cylinder, &cam, path, lookAt, m_p_data);
	//Misc::calcVisMeasurePerspective1(0, cylinder, &cam, path, lookAt,up, m_p_data);

	vector<double> cumMeasure = Misc::calcCumulativeHistogram(m_p_data, edges);
	if (oupName.length() > 0)
	{
		// printing output
		ofstream oupFlyt("oupFlythrough_" + oupName + ".csv");
		cout << "oupFlythrough_" << oupName.c_str() << ".csv" << endl;
		for (int i = 0; i < cumMeasure.size(); i++)
		{
			oupFlyt << edges[i] << "," << cumMeasure[i] << endl;
		}
		oupFlyt.close();
	}
	return cumMeasure;
}


// the rotational shift is used to rotate the camera
vector<double> TestingCylinder::simulateFlythroughReverese(TubularObject & cylinder, vector<double>& edges, vector<double>& m_p_data, double rotShift, string oupName)
{
	PerspectiveCamera cam;
	cam.near = 0.1;// cylinder.maxRadius - epsilon;
	cam.far = 75;// cylinder.maxRadius * 7 + epsilon;
	cam.sx = cam.sy = 200;
	cam.setFovH(2 * pi / 3);
	cam.setFovV(2 * pi / 3);
	//cam.f = 2 * cylinder.maxRadius + epsilon;

	//cout << "cos(cam.fov / 2) = " << cos(cam.fov / 2) << endl;
	vector<Point3D> path;
	vector<Point3D> lookAt;
	vector<Point3D> up;
	for (int i = cylinder.centerline.size() - 1; i >= 0; i--)
		//for (int i = 1; i < 2; i++)
	{
		path.push_back(cylinder.centerline[i]);

		lookAt.push_back(-cylinder.tangents[i]); 
		(lookAt.end() - 1)->normalize();

		up.push_back(cylinder.xAxis[i]);
	}
	Misc::calcVisMeasurePerspective(0, cylinder, &cam, path, lookAt, m_p_data);
	//Misc::calcVisMeasurePerspective1(0, cylinder, &cam, path, lookAt,up, m_p_data);

	vector<double> cumMeasure = Misc::calcCumulativeHistogram(m_p_data, edges);
	if (oupName.length() > 0)
	{
		// printing output
		ofstream oupFlyt("oupFlythroughRev_" + oupName + ".csv");
		cout << "oupFlythroughRev_" << oupName.c_str() << ".csv" << endl;
		for (int i = 0; i < cumMeasure.size(); i++)
		{
			oupFlyt << edges[i] << "," << cumMeasure[i] << endl;
		}
		oupFlyt.close();
	}
	return cumMeasure;
}



vector<double> TestingCylinder::simulateEquirectangularFlyOverPerspectiveMeasure(TubularObject & cylinder, vector<double>& edges, vector<double>& m_p_data, string oupName)
{
	EquirectangularCamera cam;
	cam.near = 0.01;
	cam.far = 2 * cylinder.maxRadius;
	cam.verticalFOV = pi;
	cam.horizontalFOV = pi;
	vector<Point3D> lookAt, path, upDir;
	for (int i = 0; i < cylinder.xAxis.size(); i++)
	//for (int i = 1000; i < 1001; i++)
	{
		path.push_back(cylinder.centerline[i]);
		upDir.push_back(cylinder.tangents[i]);
		lookAt.push_back(cylinder.xAxis[i] + cylinder.centerline[i]);
	}
	Misc::calcVisMeasureEquirectangular(cylinder, &cam, path, lookAt, upDir, m_p_data);
	
	vector<double> cumMeasure = Misc::calcCumulativeHistogram(m_p_data, edges);

	if (oupName.length() > 0)
	{
		// printing output
		ofstream oupFlat("oupFlat_" + oupName + ".csv");
		cout << "oupFlat_" << oupName.c_str() << ".csv" << endl;
		for (int i = 0; i < cumMeasure.size(); i++)
		{
			oupFlat << edges[i] << "," << cumMeasure[i] << endl;
		}
		oupFlat.close();
	}
	return cumMeasure;
}

// radius: is the radius of the U shape curve
// cylinderR: is the radius of cylinderical shape rings
TubularObject TestingCylinder::testCylinderCycleHalf(double shift, double step, double radius, double cylinderR)
{
	vector<Point3D> path;
	vector<double> cylR;
	for (double x = pi / 2; x > 0; x -= (step / radius * cylinderR))
	//for (double x = 0; x < 2.5; x += 1)
	{
		path.push_back({ radius * cos(x), radius * sin(x) , 0 });
	}
	for (double y = path[path.size() - 1].y() - step * cylinderR; y >= -2*radius; y -= (step * cylinderR))
	{
		path.push_back({ radius, y , 0 });
	}
	reverse(path.begin(), path.end());

	vector<Point3D> tmpPath = path;
	for (int i = tmpPath.size() - 2;i >= 0;i--)
	{
		tmpPath[i][0] = -tmpPath[i][0];
		path.push_back(tmpPath[i]);
	}

	// radius
	double distance = 0;
	cylR.push_back(cylinderR + 0.2 * sin(0));
	for (int i = 1; i < path.size(); i++)
	{
		distance += (path[i] - path[i - 1]).getL2Norm();
		//cylR.push_back(cylinderR + 0.2 * sin(5 * distance));
		cylR.push_back(cylinderR + 0.5 * sin(5 * distance));
	}
	TubularObject cyl0(cylinderR, step, path, false, shift);
	//TubularObject cyl0(cylR, step, path, false, shift);
	
	return cyl0;
}

TubularObject TestingCylinder::testCylinderFromFile(string fileName, double rotShift)
{
	vector<Point3D> path;
	vector<double> cylR;
	ifstream inp(fileName);
	double x, y, z, r;
	double avgR = 0;
	int cnt = 0;
	int tmpR = 0;
	int size = 4;
	while (inp >> x >> y >> z >> r)
	{
		//if (cnt > 425 && cnt < 450)
		tmpR += r;
		if((cnt%size) == (size - 1))
		{
			path.push_back({ x, y, z });
			cylR.push_back(tmpR / size);
			tmpR = 0;
			avgR += r;
		}
		cnt++;
	}
	TubularObject cyl0(cylR, pi / 8, path, false, rotShift);
	//TubularObject cyl0(avgR / cylR.size(), pi / 8, path, false);
	
	return cyl0;
}

TubularObject TestingCylinder::testCylinderFromPath(double shift, double rotStep, double cylRadius, double length)
{
	vector<Point3D> path;
	vector<double> cylR;
	for (double x = 0; x < length; x += (rotStep * cylRadius))
	{
		//path.push_back({ x, sin(x) , 0 }); // centerline sine
		path.push_back({x, 0 , 0 });
		cylR.push_back(cylRadius + 0.2 * sin(5 * x));
		//cylR.push_back(cylRadius + 0.5 * sin(5 * x)); // for the simple center
		//cylR.push_back(cylRadius + 0.35 * sin(5 * x)); // for the sine center
	}
	//TubularObject cyl0(cylR, rotStep, path, false, shift);
	TubularObject cyl0(cylRadius, rotStep, path, false, shift);

	return cyl0;
}
