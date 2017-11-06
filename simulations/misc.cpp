#include "misc.h"
#include <unordered_set>
#include <map>

#define DEBUG_INTERSECTION 0

int Misc::pnPoly(vector<Point3D*>& poly, Point3D &pt)
{
	int nvert = poly.size();
	int i, j, c = 0;
	for (i = 0, j = nvert - 1; i < nvert; j = i++) {
		if (((poly[i]->y() > pt.y()) != (poly[j]->y() > pt.y())) &&
			(pt.x() < (poly[j]->x() - poly[i]->x()) * (pt.y() - poly[i]->y()) / (poly[j]->y() - poly[i]->y()) + poly[i]->x()))
			c = !c;
	}
	return c;
}

// halfTag: 0 if not tag, 1 first half, 2 second half
bool Misc::checkIntersection(TubularObject& tubular, int halfTag, const int vInd, const Point3D& loc, vector<pair<int, double> > & tmpM)
{
	// for the line
	Point3D dir = tubular.vertices[vInd].pt - loc;
	double length = dir.getL2Norm();
	//dir.normalize();

	const vector<Face> &faces = (tubular.faces);
	//const int size = faces.size();
	//printf("%d\n", tmpM.size());
	int size = tmpM.size();
	if (!halfTag)
	{
		// take all points
		for (int i = 0; i < size; i++)
		{
			int f = tmpM[i].first;
			if (tubular.vertexFaces[vInd].find(f) == tubular.vertexFaces[vInd].end())
			{
				Face *face = &(tubular.faces[f]);
				//// check only if this is one of the correct faces to use
				// not one of the faces of this vertex
				double denom = dir.dot(face->normal);
				// if parallel the denom (dot product 
				if (denom < -epsilon || denom > epsilon)
				{
					Point3D tmpN = *face->points[0] - loc;
					double numerator = tmpN.dot(face->normal);

					/*Point3D* p0 = face->points[0];
					Point3D* normal = &(face->normal);
					double numeratorx = (p0->x - loc.x) * normal->x;
					double numeratory = (p0->x - loc.y) * normal->y;
					double numeratorz = (p0->x - loc.z) * normal->z;

					double numerator = numeratorx + numeratory + numeratorz;*/

					// if the point within the line segment
					//if (numerator > 0 && numerator <= (length * denom))
					{
						double t = numerator / denom;
						if (t >= 0 && t <= 1)
						{
							Point3D interPt = (dir * t) + loc;
							//intersection = Misc::isLineIntersectPlan(interPt, face);
							if (face->isPointInside(interPt))
							{
								return true;
							}
						}
					}
				}
			}
		}
	}
	else
	{
		for (int i = 0; i < size; i++)
		{
			int f = tmpM[i].first;
			if (/*tmpM[f] <= 1 && */tubular.vertexFaces[vInd].find(f) == tubular.vertexFaces[vInd].end())
			{
				//printf("%d\n", f);
				const Face *face = &(tubular.faces[f]);

				if (face->tag == halfTag)
				{
					//// check only if this is one of the correct faces to use
					// not one of the faces of this vertex

					double denom = dir.dot(face->normal);
					// if parallel the denom (dot product 
					if (abs(denom) > epsilon)
					{
						Point3D tmpN = face->points[0]->operator-(loc);
						double numerator = tmpN.dot(face->normal);

						// if the point within the line segment
						if (numerator > 0 && numerator <= (length * denom))
						{
							double t = numerator / denom;
							Point3D interPt = (dir * t) + loc;
							//intersection = Misc::isLineIntersectPlan(interPt, face);
							if (face->isPointInside(interPt))
							{
								return true;
							}
						}
					}
				}
			}
		}
	}
	return false;
}

// halfTag: 0 if not tag, 1 first half, 2 second half
bool Misc::checkIntersectionEn(TubularObject& tubular, int halfTag, const int vInd, const Point3D& loc, vector<int> & tmpM)
{
	// for the line
	Point3D dir = tubular.vertices[vInd].pt - loc;
	double length = dir.getL2Norm();
	//dir.normalize();

	const vector<Face> &faces = (tubular.faces);
	//const int size = faces.size();
	//printf("%d\n", tmpM.size());
	int size = tmpM.size();
	if (!halfTag)
	{
		// take all points
		for (int i = 0; i < size; i++)
		{
			int f = tmpM[i];
			if (tubular.vertexFaces[vInd].find(f) == tubular.vertexFaces[vInd].end())
			{
				Face *face = &(tubular.faces[f]);
				//// check only if this is one of the correct faces to use
				// not one of the faces of this vertex
				double denom = dir.dot(face->normal);
				// if parallel the denom (dot product 
				if (denom < -epsilon || denom > epsilon)
				{
					Point3D tmpN = *face->points[0] - loc;
					double numerator = tmpN.dot(face->normal);

					double t = numerator / denom;
					if (t >= 0 && t <= 1)
					{
						Point3D interPt = (dir * t) + loc;
						//intersection = Misc::isLineIntersectPlan(interPt, face);
						if (face->isPointInside(interPt))
						{
							return true;
						}
					}
				}
			}
		}
	}
	else
	{
		for (int i = 0; i < size; i++)
		{
			int f = tmpM[i];
			if (tubular.vertexFaces[vInd].find(f) == tubular.vertexFaces[vInd].end())
			{
				//printf("%d\n", f);
				const Face *face = &(tubular.faces[f]);

				if (face->tag == halfTag)
				{
					//// check only if this is one of the correct faces to use
					// not one of the faces of this vertex

					double denom = dir.dot(face->normal);
					// if parallel the denom (dot product 
					if (abs(denom) > epsilon)
					{
						Point3D tmpN = face->points[0]->operator-(loc);
						double numerator = tmpN.dot(face->normal);

						// if the point within the line segment
						if (numerator > 0 && numerator <= (length * denom))
						{
							double t = numerator / denom;
							Point3D interPt = (dir * t) + loc;
							//intersection = Misc::isLineIntersectPlan(interPt, face);
							if (face->isPointInside(interPt))
							{
								return true;
							}
						}
					}
				}
			}
		}
	}
	return false;
}


// halfTag: 0 if not tag, 1 first half, 2 second half
bool Misc::checkIntersection1(TubularObject& tubular, int halfTag, const int vInd, vector<pair<int, double> > & tmpM, vector<double>& tmpNumerator)
{
	// for the line
	Point3D dir = tubular.vertices[vInd].pt;
	double length = dir.z();// dir.getL2Norm();
	//dir.normalize();

	const vector<Face> &faces = (tubular.faces);
	//const int size = faces.size();
	//printf("%d\n", tmpM.size());
	int size = tmpM.size();
	if (!halfTag)
	{
		// take all points
		for (int i = 0; i < size; i++)
		{
			int f = tmpM[i].first;
			if (tubular.vertexFaces[vInd].find(f) == tubular.vertexFaces[vInd].end())
			{
				Face *face = &(tubular.faces[f]);
				//// check only if this is one of the correct faces to use
				// not one of the faces of this vertex
				double denom = dir.dot(face->normal);
				//double denom = face->normal.z();
				// if parallel the denom (dot product 
				if (denom < -epsilon || denom > epsilon)
				{
					//Point3D tmpN = *face->points[0];
					//double numerator = tmpN.dot(face->normal);
					double numerator = tmpNumerator[i];
					double t = numerator / denom;
					if (t >= 0 && t <= 1)
					{
						Point3D interPt = (dir * t);
						//intersection = Misc::isLineIntersectPlan(interPt, face);
						if (face->isPointInside(interPt))
						{
							return true;
						}
					}
				}
			}
		}
	}
	//else
	//{
	//	for (int i = 0; i < size; i++)
	//	{
	//		int f = tmpM[i].first;
	//		if (/*tmpM[f] <= 1 && */tubular.vertexFaces[vInd].find(f) == tubular.vertexFaces[vInd].end())
	//		{
	//			//printf("%d\n", f);
	//			const Face *face = &(tubular.faces[f]);

	//			if (face->tag == halfTag)
	//			{
	//				//// check only if this is one of the correct faces to use
	//				// not one of the faces of this vertex

	//				double denom = dir.dot(face->normal);
	//				// if parallel the denom (dot product 
	//				if (abs(denom) > epsilon)
	//				{
	//					Point3D tmpN = face->points[0]->operator-(loc);
	//					double numerator = tmpN.dot(face->normal);

	//					// if the point within the line segment
	//					if (numerator > 0 && numerator <= (length * denom))
	//					{
	//						double t = numerator / denom;
	//						Point3D interPt = (dir * t) + loc;
	//						//intersection = Misc::isLineIntersectPlan(interPt, face);
	//						if (face->isPointInside(interPt))
	//						{
	//							return true;
	//						}
	//					}
	//				}
	//			}
	//		}
	//	}
	//}
	return false;
}

// halfTag: 0 if not tag, 1 first half, 2 second half
bool Misc::checkIntersection2(TubularObject& tubular, int halfTag, const int vInd, vector<int> & tmpM)
{
	Point3D pt = tubular.vertices[vInd].pt;
	const vector<Face> &faces = (tubular.faces);
	const unordered_map<int, bool> & vertexFaces = tubular.vertexFaces[vInd];
	//const int size = faces.size();
	//printf("%d\n", tmpM.size());
	int size = tmpM.size();
	if (!halfTag)
	{
		// take all points

		for (int i = 0; i < size; i++)
		{
			int f = tmpM[i];
			if (vertexFaces.find(f) == vertexFaces.end())
			{
				Face *face = &(tubular.faces[f]);
				
				if (face->center[2] <= pt[2] && pnPoly(face->points, pt))
				{
					return true;
				}
			}
		}
	}
	
	return false;
}



// calculate visualization measure using a perspective camera
// assume all directions are normalized already
void Misc::calcVisMeasurePerspective(
	int halfTag,
	TubularObject& tubular, const PerspectiveCamera * cam,
	const vector<Point3D>& path, const vector<Point3D>& pathLookAt, vector<double>& output,
	bool backfaceCull)
{
	//vector<Vertex3D> data = tubular.vertices;
	const int startPath = 0;// path.size() / 2 - test;
	const int endPath = path.size();
	const int stepPath = 1;
	const double camTest = cos(cam->getFovH() / 2);
	int facesSize = tubular.faces.size();
#pragma omp parallel for
	for (int i = startPath; i < endPath; i += stepPath)
	{
		const Point3D loc = path[i];
		vector<pair<int, double> > tmpM;
		//for (int jj = facesSize - 3; jj < facesSize; jj++)
		for (int jj = 0; jj < facesSize; jj++)
		{
			if (halfTag == 0 || halfTag == tubular.faces[jj].tag)
			{
				// assuming v is already normalized
				Point3D cameraAxisDirection = pathLookAt[i];
				//cameraAxisDirection.normalize();

				Point3D pt = tubular.faces[jj].center;
				Point3D vectorFromCameraToVertex = pt;
				vectorFromCameraToVertex = vectorFromCameraToVertex - loc;
				// get the perpendicular distance from the camera 
				// by getting the projection of the vector from the camera to the point on the lookat vector
				double distance = vectorFromCameraToVertex.dot(cameraAxisDirection);
				if (distance >= cam->near && distance <= cam->far)
				{

					// normalize the projection direction
					vectorFromCameraToVertex.normalize();
					const Point3D vertexNormal = tubular.faces[jj].normal;

					// check if the point is in the field of view
					double pDotv = vectorFromCameraToVertex.dot(cameraAxisDirection);
					if (pDotv >= camTest)
					{

						// check if the point is in the View Frustum
						double normalDotCameraAxis = vectorFromCameraToVertex.dot(vertexNormal);
						if (normalDotCameraAxis <= 0 || !backfaceCull)
						{
							double m_p_temp = pDotv * normalDotCameraAxis;
							if (distance > cam->getFocalLength())
							{
								m_p_temp *= (cam->getFocalLength() / distance);
							}
							tmpM.push_back(make_pair(jj, m_p_temp));
						}
					}
				}
			}
		}
		int size = tmpM.size();
#pragma omp parallel for
		for (int f = 0;f < tmpM.size();f++)
		{
			if (tmpM[f].second < 0)
			{
				int jj = tmpM[f].first;
				// update the measurement value if the new value is better then the previous one
				int k = 0;
				for (; k < tubular.faces[jj].indeces.size(); k++)
				{
					int vInd = tubular.faces[jj].indeces[k];
					if (tmpM[f].second < output[vInd])
					{
						// check if there is intersection between camera line and cells other than this point cell
						if (!checkIntersection(tubular, halfTag, vInd, loc, tmpM))
						{
							// there is intersection and we can not get this vertex from this
							output[vInd] = tmpM[f].second;
						}
					}
				}
			}
		}
	}
}

// calculate visualization measure using a perspective camera
// assume all directions are normalized already
void Misc::calcVisMeasurePerspectiveEn(
	int halfTag,
	TubularObject& tubular, const PerspectiveCamera * cam,
	const vector<Point3D>& path, const vector<Point3D>& pathLookAt, vector<double>& output,
	bool backfaceCull)
{
	//vector<Vertex3D> data = tubular.vertices;
	const int startPath = 0;// path.size() / 2 - test;
	const int endPath = path.size();
	const int stepPath = 1;
	const double camTest = cos(cam->getFovH() / 2);
	int facesSize = tubular.faces.size();
	vector<vector<int> > pathFaces(path.size());
	vector<map<float, int> > vertMeasure(tubular.vertices.size());

#pragma omp parallel for
	for (int i = startPath; i < endPath; i += stepPath)
	{
		const Point3D loc = path[i];
		vector<int> &tmpFaces = pathFaces[i];
		//for (int jj = facesSize - 3; jj < facesSize; jj++)
		for (int jj = 0; jj < facesSize; jj++)
		{
			if (halfTag == 0 || halfTag == tubular.faces[jj].tag)
			{
				// assuming v is already normalized
				Point3D cameraAxisDirection = pathLookAt[i];
				//cameraAxisDirection.normalize();

				Point3D pt = tubular.faces[jj].center;
				Point3D vectorFromCameraToVertex = pt - loc;
				// get the perpendicular distance from the camera 
				// by getting the projection of the vector from the camera to the point on the lookat vector
				double distance = vectorFromCameraToVertex.dot(cameraAxisDirection);
				if (distance >= cam->near && distance <= cam->far)
				{

					// normalize the projection direction
					vectorFromCameraToVertex.normalize();
					const Point3D vertexNormal = tubular.faces[jj].normal;

					// check if the point is in the field of view
					double pDotv = vectorFromCameraToVertex.dot(cameraAxisDirection);
					if (pDotv >= camTest)
					{

						// check if the point is in the View Frustum
						double normalDotCameraAxis = vectorFromCameraToVertex.dot(vertexNormal);
						if (normalDotCameraAxis <= 0 || !backfaceCull)
						{
							double m_p_temp = pDotv * normalDotCameraAxis;
							if (distance > cam->getFocalLength())
							{
								m_p_temp *= (cam->getFocalLength() / distance);
							}
#pragma omp critical
							{
								tmpFaces.push_back(jj);
								for (int k = 0; k < tubular.faces[jj].indeces.size(); k++)
								{
									int vInd = tubular.faces[jj].indeces[k];
									vertMeasure[vInd][m_p_temp] = i;
								}
							}
						}
					}
				}
			}
			//pathFaces[i] = tmpFaces;
		}
	}
	int size = vertMeasure.size();
#pragma omp parallel for
	for (int vInd = 0; vInd < size; vInd++)
	{
		for (map<float, int>::iterator it = vertMeasure[vInd].begin(); it != vertMeasure[vInd].end() && it->first < output[vInd]; it++)
		{
			// check if there is intersection between camera line and cells other than this point cell
			if (!checkIntersectionEn(tubular, halfTag, vInd, path[it->second], pathFaces[it->second]))
			{
				// there is intersection and we can not get this vertex from this
				output[vInd] = it->first;
				break;
			}
		}
	}
}


// calculate visualization measure using a perspective camera
// assume all directions are normalized already
void Misc::calcVisMeasurePerspectiveEn1(
	int halfTag,
	TubularObject& tubular, const PerspectiveCamera * cam,
	const vector<Point3D>& path, const vector<Point3D>& pathLookAt, vector<double>& output,
	bool backfaceCull)
{
	const double camTest = cos(cam->getFovH() / 2);
	int facesSize = tubular.faces.size();
	
	int outerStep = 500;
	for (int outer = 0; outer < path.size(); outer += outerStep)
	{
		vector<vector<int> > pathFaces(path.size());
		vector<map<float, int> > vertMeasure(tubular.vertices.size());
		const int startPath = outer;// path.size() / 2 - test;
		const int endPath = std::min(startPath + outerStep, (int)(path.size()));
		const int stepPath = 1;
#pragma omp parallel for
		for (int i = startPath; i < endPath; i += stepPath)
		{
			const Point3D loc = path[i];
			vector<int> &tmpFaces = pathFaces[i];
			//for (int jj = facesSize - 3; jj < facesSize; jj++)
			for (int jj = 0; jj < facesSize; jj++)
			{
				if (halfTag == 0 || halfTag == tubular.faces[jj].tag)
				{
					// assuming v is already normalized
					Point3D cameraAxisDirection = pathLookAt[i];
					//cameraAxisDirection.normalize();

					Point3D pt = tubular.faces[jj].center;
					Point3D vectorFromCameraToVertex = pt - loc;
					// get the perpendicular distance from the camera 
					// by getting the projection of the vector from the camera to the point on the lookat vector
					double distance = vectorFromCameraToVertex.dot(cameraAxisDirection);
					if (distance >= cam->near && distance <= cam->far)
					{

						// normalize the projection direction
						vectorFromCameraToVertex.normalize();
						const Point3D vertexNormal = tubular.faces[jj].normal;

						// check if the point is in the field of view
						double pDotv = vectorFromCameraToVertex.dot(cameraAxisDirection);
						if (pDotv >= camTest)
						{

							// check if the point is in the View Frustum
							double normalDotCameraAxis = vectorFromCameraToVertex.dot(vertexNormal);
							if (normalDotCameraAxis <= 0 || !backfaceCull)
							{
								double m_p_temp = pDotv * normalDotCameraAxis;
								if (distance > cam->getFocalLength())
								{
									m_p_temp *= (cam->getFocalLength() / distance);
								}
#pragma omp critical
								{
									tmpFaces.push_back(jj);
									for (int k = 0; k < tubular.faces[jj].indeces.size(); k++)
									{
										int vInd = tubular.faces[jj].indeces[k];
										vertMeasure[vInd][m_p_temp] = i;
									}
								}
							}
						}
					}
				}
				//pathFaces[i] = tmpFaces;
			}
		}
		int size = vertMeasure.size();
#pragma omp parallel for
		for (int vInd = 0; vInd < size; vInd++)
		{
			for (map<float, int>::iterator it = vertMeasure[vInd].begin(); it != vertMeasure[vInd].end() && it->first < output[vInd]; it++)
			{
				// check if there is intersection between camera line and cells other than this point cell
				if (!checkIntersectionEn(tubular, halfTag, vInd, path[it->second], pathFaces[it->second]))
				{
					// there is intersection and we can not get this vertex from this
					output[vInd] = it->first;
					break;
				}
			}
		}
	}
}


Mat3x3 Misc::getCamRotationMatrix(const Point3D& lookDir, const Point3D& upDir)
{
	double a = lookDir.x();
	double b = lookDir.y();
	double c = lookDir.z();
	double d = lookDir.y()*lookDir.y() + lookDir.z()*lookDir.z();
	d = sqrt(d);

	Mat3x3 Rx = { 1,0,0,0,1,0,0,0,1 };
	Mat3x3 Ry, Rz;
	if (d != 0)
	{
		double c_d = c / d;
		double b_d = b / d;

		Rx = { 1, 0, 0, 0, c_d, -b_d,0,b_d,c_d };
	}

	/*Rotate space about the y axis so that the rotation axis lies along the positive z axis.
	Using the appropriate dot and cross product relationships as before the cosine of the angle is d, the sine of the angle is a.
	The rotation matrix about the y axis Ry and the inverse Ry-1 (required for step 5) are given below. */

	Ry = { d, 0, -a, 0, 1, 0,a,0,d };
	Point3D newXAxis = Ry * Rx * upDir;
	Rz = { newXAxis.x(), -newXAxis.y(), 0, newXAxis.y(), newXAxis.x(), 0, 0, 0, 1 };

	Mat3x3 rot = Rz * Ry * Rx;

	return rot;
}
// calculate visualization measure using a perspective camera
// assume all directions are normalized already
// transform the object to the location and direction of the camera
void Misc::calcVisMeasurePerspective1(
	int halfTag,
	TubularObject& tubular_, const PerspectiveCamera * cam,
	const vector<Point3D>& path, const vector<Point3D>& pathLookAt, const vector<Point3D>& up,
	vector<double>& output)
{
	
#pragma omp parallel for
	for (int i = 0; i < path.size(); i++)
	{
		TubularObject rotObj = tubular_;
		Mat3x3 rot = getCamRotationMatrix(pathLookAt[i], up[i]);
		rotObj.transform(path[i], rot);

		vector<pair<int, double> > tmpV;
		vector<double> tmpNumerator;
//#pragma omp critical
//		rotObj.writeVerticesToFile("test.csv");
//


		//vector<bool> flags(tubular_.faces.size(), false);

		for (int jj = 0; jj < tubular_.faces.size(); jj++)
		{
			if (halfTag == 0 || halfTag == tubular_.faces[jj].tag)
			{
				Point3D& pt = rotObj.faces[jj].center;
				// get the perpendicular distance from the camera 
				// by getting the projection of the vector from the camera to the point on the lookat vector
				if (pt.z() >= cam->near && pt.z() <= cam->far &&
					abs(pt.x() * cam->getFocalLength()) <= pt.z() * 0.5 * cam->sx &&
					abs(pt.y() * cam->getFocalLength()) <= pt.z() * 0.5 * cam->sy)
				{
					// inside the frustum
					//flags[jj] = true;
					tmpV.push_back(make_pair(jj, -1));
					//Point3D tmpN = *face->points[0];
					double numerator = rotObj.faces[jj].points[0]->dot(rotObj.faces[jj].normal);
					tmpNumerator.push_back(numerator);
				}
			}
		}
		for (vector<pair<int, double> >::iterator it = tmpV.begin(); it != tmpV.end(); it++)
		{
			int jj = it->first;

			// check if the frontal face is visible
			if (rotObj.faces[jj].normal.z() <= 0)
			{
				Point3D pt = rotObj.faces[jj].center;
				double distance = pt.z();
				// calculate the measure
				pt.normalize();
				double m_p_temp = pt.dot(rotObj.faces[jj].normal) * pt.z();
				if (distance > cam->getFocalLength())
				{
					m_p_temp *= (cam->getFocalLength() / distance);
				}
				for (int kk = 0; kk < tubular_.faces[jj].indeces.size(); kk++)
				{
					int vInd = tubular_.faces[jj].indeces[kk];
					if (m_p_temp < output[vInd])
					{
						if (!checkIntersection1(rotObj, halfTag, vInd, tmpV, tmpNumerator))
						{
							// there is no intersection

							output[vInd] = m_p_temp;
						}
					}
				}
			}

		}
	}
}

// calculate visualization measure using a perspective camera
// assume all directions are normalized already
// transform the object to the location and direction of the camera
void Misc::calcVisMeasurePerspective2(
	int halfTag,
	TubularObject& tubular_, const PerspectiveCamera * cam,
	const vector<Point3D>& path, const vector<Point3D>& pathLookAt, const vector<Point3D>& up,
	vector<double>& output)
{


	for (int i = 0; i < path.size(); i++)
	{
		TubularObject rotObj = tubular_;
		Mat3x3 rot = getCamRotationMatrix(pathLookAt[i], up[i]);
		rotObj.transform(path[i], rot);

		vector<int> tmpVert; // vertix indeces
		map<int, double> tmpVertMap; // indeces + best measure value
		vector<int> tmpFaces; // faces
#pragma omp parallel for
		for (int jj = 0; jj < tubular_.faces.size(); jj++)
		{
			if (halfTag == 0 || halfTag == tubular_.faces[jj].tag)
			{
				Point3D& pt = rotObj.faces[jj].center;
				// get the perpendicular distance from the camera 
				// by getting the projection of the vector from the camera to the point on the lookat vector
				if (pt.z() >= cam->near && pt.z() <= cam->far &&
					abs(pt.x() * cam->getFocalLength()) <= pt.z() * 0.5 * cam->sx &&
					abs(pt.y() * cam->getFocalLength()) <= pt.z() * 0.5 * cam->sy)
				{
#pragma omp critical
					{
						tmpFaces.push_back(jj);
					}
					// inside the frustum
					double distance = pt.z();
					// calculate the measure
					pt.normalize();
					double m_p_temp = pt.dot(rotObj.faces[jj].normal) * pt.z();
					if (distance > cam->getFocalLength())
					{
						m_p_temp *= (cam->getFocalLength() / distance);
					}
					for (int kk = 0; kk < tubular_.faces[jj].indeces.size(); kk++)
					{
						int vInd = tubular_.faces[jj].indeces[kk];
#pragma omp critical
						{
							if (tmpVertMap[vInd] > m_p_temp)
							{
								tmpVertMap[vInd] = m_p_temp;
							}
						}
					}
				}
			}
		}

		for (map<int, double>::iterator it = tmpVertMap.begin(); it != tmpVertMap.end(); it++)
		{
			int v = it->first;
			tmpVert.push_back(v);

			double factor = -cam->getFocalLength() / rotObj.vertices[v].pt.z();
			rotObj.vertices[v].pt[0] *= factor;
			rotObj.vertices[v].pt[1] *= factor;
		}
#pragma omp parallel for
		for (int j = 0; j < tmpVert.size(); j++)
		{
			int vInd = tmpVert[j];
			if (output[vInd] > tmpVertMap[vInd])
			{
				if (!checkIntersection2(rotObj, halfTag, vInd, tmpFaces))
				{
					// there is no intersection
					output[vInd] = tmpVertMap[vInd];
				}
			}
		}
	}
}

// calculate visualization measure using a panoramic camera moving along centerline of a cylinder
// assume all directions are normalized already

void Misc::calcVisMeasureEquirectangular(TubularObject& tubular, const EquirectangularCamera * cam, const vector<Point3D>& path,
	const vector<Point3D>& pathLookAt, const vector<Point3D>& upDir, vector<double>& output)
{
	//vector<Vertex3D> data = tubular.vertices;
	//vector<double> tmpV(tubular.faces.size(), 0);
	for (int i = 0; i < pathLookAt.size(); i++)
	{
		// location in the camera path (centerline point)
		Point3D loc = path[i];

		vector<pair<int, double> > tmpM;

#pragma omp parallel
		for (int j = 0; j < tubular.faces.size(); j++)
		{
			// point on cylinder surface
			Point3D pt = tubular.faces[j].center;	

			// projection direction
			Point3D p = pt - loc;
			Point3D originalP = p;
			// normalize the projection direction
			p.normalize();
			// surface normal
			Point3D n = tubular.faces[j].normal;
			// camera axis (different at each point because this is panormaic)
			Point3D camUp = upDir[i];
			Point3D tmp = p.cross(camUp);
			Point3D v = camUp.cross(tmp);
			v.normalize();

			// get the perpendicular distance from the camera 
			// by getting the projection of the vector from the camera to the point on the lookat vector
			double distance = std::abs(originalP.dot(v));

			// check if the point is in the View Frustum
			if (distance >= cam->near && distance <= cam->far)
			{
				// check if the point is in the field of view
				double pDotv = p.dot(v);
				if (pDotv >= cos(cam->verticalFOV / 2))
				{
					double pDotn = p.dot(n);
					if (pDotn < 0)
					{
						double m_p_temp = pDotv * pDotn;
#pragma omp critical
						tmpM.push_back(make_pair(j,m_p_temp));
					}
				}
			}
		}

#pragma omp parallel
		for (int f = 0;f < tmpM.size();f++)
		{
			if (tmpM[f].second < 0)
			{
				int j = tmpM[f].first;
				// update the measurement value if the new value is better then the previous one
				for (int k = 0; k < tubular.faces[j].indeces.size(); k++)
				{
					int vInd = tubular.faces[j].indeces[k];
					if (tmpM[f].second < output[vInd])
					{
						if (checkIntersection(tubular, 0, vInd, loc, tmpM))
						{
							continue;
						}
						else
						{
							output[vInd] = tmpM[f].second;
						}
					}
				}
			}
		}
	}
}

// calcualte cumulative histogram for the given data using the given histogram bin locations
// this function ignores the values greater than the largest edge

vector<double> Misc::calcCumulativeHistogram(const vector<double>& data1,const vector<double>& edges)
{
	vector<double> data(data1.begin(), data1.end());
	sort(data.begin(), data.end());
	vector<double> h1(edges.size(), 0);
	int edgeInd = 0;
	for (int i = 0; i < data.size(); i++)
	{
		if (data[i] <= edges[edgeInd] + epsilon)
		{
			h1[edgeInd]++;
		}
		else
		{
			while (edgeInd < edges.size() && data[i] > edges[edgeInd])
			{
				edgeInd++;
			}
			if (edgeInd >= edges.size())
			{
				// exceeded the edges
				break;
			}
			else
			{
				// repeat this iteration
				--i;
			}
		}
	}
	for (int i = 0; i < h1.size(); i++)
	{
		h1[i] /= data.size();
	}
	// create the cumulative histogram
	vector<double> res(1, h1[0]);
	for (int i = 1; i < h1.size(); i++)
	{
		res.push_back(res[i - 1] + h1[i]);
	}

	return res;
}

vector<Color> Misc::getColorPalette()
{
	int colors[][3] = {
		{ 0,0,255 },
		{ 0,25,255 },
		{ 0,51,255 },
		{ 0,76,255 },
		{ 0,102,255 },
		{ 0,127,255 },
		{ 0,153,255 },
		{ 0,178,255 },
		{ 0,204,255 },
		{ 0,229,255 },
		{ 0,255,255 },

		{ 0,255,229 },
		{ 0,255,204 },
		{ 0,255,178 },
		{ 0,255,153 },
		{ 0,255,127 },
		{ 0,255,102 },
		{ 0,255,76 },
		{ 0,255,51 },
		{ 0,255,25 },
		{ 0,255,0 },

		{ 25,255,0 },
		{ 51,255,0 },
		{ 76,255,0 },
		{ 102,255,0 },
		{ 127,255,0 },
		{ 153,255,0 },
		{ 178,255,0 },
		{ 204,255,0 },
		{ 229,255,0 },
		{ 255,255,0 },

		{ 255,229,0 },
		{ 255,204,0 },
		{ 255,178,0 },
		{ 255,153,0 },
		{ 255,127,0 },
		{ 255,102,0 },
		{ 255,76,0 },
		{ 255,51,0 },
		{ 255,25,0 },
		{ 255,0,0 }
	};

	vector<Color> res;
	for (int i = 0; i < 41; i++)
	{
		Color v(colors[i][0], colors[i][1], colors[i][2]);
		res.push_back(v);
	}

	return res;
}

// get triangle area
double Misc::getTriangleArea(vector<Point3D>& tri)
{
	return getTriangleArea(tri[0], tri[1], tri[2]);
}

// get triangle area
double Misc::getTriangleArea(const  Point3D& p0, const  Point3D&p1, const  Point3D& p2)
{
	Point3D tmp1 = p0 - p1;
	Point3D tmp2 = p0 - p2;
	
	Point3D tmp = tmp1.cross(tmp2);
	double area = 0.5 * tmp.getL2Norm();

	return area;
}

// get double triangle area
double Misc::getTriangleArea2(const  Point3D& p0, const  Point3D&p1, const  Point3D& p2)
{
	Point3D tmp1 = (p0 - p1);
	Point3D tmp2 = p0 - p2;

	Point3D tmp = tmp1.cross(tmp2);
	double area = tmp.getL2Norm();

	return area;
}

double Misc::testPointInTriangle(const  Point3D& p1, const  Point3D&p2, const  Point3D& p3)
{
	double val =
		p1.x() * (p2.y()*p3.z() - p3.y()*p2.z()) -
		p1.y() * (p2.x() * p3.z() - p3.x()*p2.z()) +
		p1.z() * (p2.x() * p3.y() - p3.x() * p2.y());
	return val;
}



// line: is exactly two points (no check)
// plan: is exactly 3 points (no check)
bool Misc::isLineIntersectPlan(vector<Point3D>& line, vector<Point3D>& plan)
{
	return isLineIntersectPlan(line[0], line[1], plan[0], plan[1], plan[2]);
}

bool Misc::isLineIntersectPlan(const Point3D & line0, const  Point3D & line1, const  Point3D & plan0, const  Point3D & plan1, const  Point3D & plan2)
{
	// the plan uses plan[0] as a point and the normal calculated below
	Point3D tmp1 = plan1;
	tmp1 = tmp1 - plan0;
	Point3D tmp2 = plan2;
	tmp2 = tmp2 - plan0;

	Point3D normal = tmp1.cross(tmp2);
	normal.normalize();
	// a point p is on the plan if (p - plan[0]) . (normal) == 0

	// for the line
	Point3D dir = line1;
	dir = dir - line0;
	double length = dir.getL2Norm();
	dir.normalize();
	// now the line is (dir * t + line[0]) where t is a paratameter
	// the point p = (x, y, z) is on the line if direction(p - line[0]) == dir and length(p - line[0]) <= length

	// now get the intersection by substituting the line in the plan equation
	// first test dir dot normal
	double denom = dir.dot(normal);

	// if denom = 0, then the line is parallel to the plan or in the same plan
	//if (abs(denom) < epsilon * epsilon)
	if (denom == 0)
	{
		// the line is parallel
		// if it is on the same plan
		Point3D tmp = line0;
		tmp = tmp - plan0;
		if (tmp.dot(normal) > epsilon)
		{
			// if the first point of the line is not in the same plan
			return false;
		}
		else
		{
			puts("// there is intersection");
			return true;
		}
	}
	else
	{
		// there is intersection somehow
		// to get the intersection 
		Point3D tmpN = plan0;
		tmpN = tmpN - line0;
		double numerator = tmpN.dot(normal);

		// if the point within the line segment
		if (numerator > 0 && numerator <= (length * denom))
			//if (t > 0 && t <= 1)
		{
			double t = numerator / denom; // t is the signed length as the dir and normal are both normalized

			// check if the point is inside the trianglular cell
			Point3D interPt = (dir * t) + line0;
			double cellArea = getTriangleArea(plan0, plan1, plan2);
			
			double accumlatedArea = 0;
			accumlatedArea += getTriangleArea(plan0, plan1, interPt);
			accumlatedArea += getTriangleArea(plan1, plan2, interPt);
			accumlatedArea += getTriangleArea(plan2, plan0, interPt);

			/*double test1 = testPointInTriangle(plan0, plan1, interPt);
			double test2 = testPointInTriangle(plan1, plan2, interPt);
			double test3 = testPointInTriangle(plan2, plan0, interPt);*/
			
#if DEBUG_INTERSECTION
			cout << "cellArea =\t" << cellArea;
			cout << "\taccumlatedArea =\t" << accumlatedArea << endl;
#endif
			//if (abs(accumlatedArea - cellArea) < 0.015){
			//if((test1 <= 0 && test2 <= 0 && test3 <= 0) || (test1 >= 0 && test2 >= 0 && test3 >= 0)){
			if (abs(accumlatedArea - cellArea) < epsilon){
				// there is intersection
				return true;
			}
		}
	}

	return false;
}



bool Misc::isLineIntersectPlan(const Point3D & line0, const  Point3D & line1, const  Point3D & plan0, const  Point3D & plan1, const  Point3D & plan2,const Point3D &plan3)
{
	// the plan uses plan[0] as a point and the normal calculated below
	Point3D tmp1 = plan1;
	tmp1 = tmp1 - plan0;
	Point3D tmp2 = plan2;
	tmp2 = tmp2 - plan0;

	Point3D normal = tmp1.cross(tmp2);
	normal.normalize();
	// a point p is on the plan if (p - plan[0]) . (normal) == 0

	// for the line
	Point3D dir = line1;
	dir = dir - line0;
	double length = dir.getL2Norm();
	dir.normalize();
	// now the line is (dir * t + line[0]) where t is a paratameter
	// the point p = (x, y, z) is on the line if direction(p - line[0]) == dir and length(p - line[0]) <= length

	// now get the intersection by substituting the line in the plan equation
	// first test dir dot normal
	double denom = dir.dot(normal);

	// if denom = 0, then the line is parallel to the plan or in the same plan
	//if (abs(denom) < epsilon * epsilon)
	if (denom == 0)
	{
		// the line is parallel
		// if it is on the same plan
		Point3D tmp = line0;
		tmp = tmp - plan0;
		if (tmp.dot(normal) > epsilon)
		{
			// if the first point of the line is not in the same plan
			return false;
		}
		else
		{
			puts("// there is intersection");
			return true;
		}
	}
	else
	{
		// there is intersection somehow
		// to get the intersection 
		Point3D tmpN = plan0;
		tmpN = tmpN - line0;
		double numerator = tmpN.dot(normal);
		
		// if the point within the line segment
		if (numerator > 0 && numerator <= (length * denom))
			//if (t > 0 && t <= 1)
		{
			double t = numerator / denom; // t is the signed length as the dir and normal are both normalized

			// check if the point is inside the trianglular cell
			Point3D interPt = (dir * t) + line0;
			double cellArea = getTriangleArea(plan0, plan1, plan2);

			double accumlatedArea = 0;
			accumlatedArea += getTriangleArea(plan0, plan1, interPt);
			accumlatedArea += getTriangleArea(plan1, plan2, interPt);
			accumlatedArea += getTriangleArea(plan2, plan0, interPt);

			/*double test1 = testPointInTriangle(plan0, plan1, interPt);
			double test2 = testPointInTriangle(plan1, plan2, interPt);
			double test3 = testPointInTriangle(plan2, plan0, interPt);*/

#if DEBUG_INTERSECTION
			cout << "cellArea =\t" << cellArea;
			cout << "\taccumlatedArea =\t" << accumlatedArea << endl;
#endif
			//if (abs(accumlatedArea - cellArea) < 0.015){
			//if((test1 <= 0 && test2 <= 0 && test3 <= 0) || (test1 >= 0 && test2 >= 0 && test3 >= 0)){
			if (abs(accumlatedArea - cellArea) < epsilon) {
				cellArea = getTriangleArea(plan0, plan2, plan3);

				accumlatedArea = 0;
				accumlatedArea += getTriangleArea(plan0, plan2, interPt);
				accumlatedArea += getTriangleArea(plan2, plan3, interPt);
				accumlatedArea += getTriangleArea(plan3, plan0, interPt);
				if (abs(accumlatedArea - cellArea) < epsilon) {
					// there is intersection
					return true;
				}
			}
		}
	}

	return false;
}



bool Misc::isLineIntersectPlan(const Point3D & line0, const  Point3D & line1, 
	const  Point3D & plan0, const  Point3D & plan1, const  Point3D & plan2, const Point3D &plan3, const Point3D& normal)
{
	// a point p is on the plan if (p - plan[0]) . (normal) == 0

	// for the line
	Point3D dir = line1;
	dir = dir - line0;
	double length = dir.getL2Norm();
	dir.normalize();
	// now the line is (dir * t + line[0]) where t is a paratameter
	// the point p = (x, y, z) is on the line if direction(p - line[0]) == dir and length(p - line[0]) <= length

	// now get the intersection by substituting the line in the plan equation
	// first test dir dot normal
	double denom = dir.dot(normal);

	// if denom = 0, then the line is parallel to the plan or in the same plan
	//if (abs(denom) < epsilon * epsilon)
	if (denom == 0)
	{
		// the line is parallel
		// if it is on the same plan
		Point3D tmp = line0;
		tmp = tmp - plan0;
		if (tmp.dot(normal) > epsilon)
		{
			// if the first point of the line is not in the same plan
			return false;
		}
		else
		{
			puts("// there is intersection");
			return true;
		}
	}
	else
	{
		// there is intersection somehow
		// to get the intersection 
		Point3D tmpN = plan0;
		tmpN = tmpN - line0;
		double numerator = tmpN.dot(normal);

		// if the point within the line segment
		if (numerator > 0 && numerator <= (length * denom))
			//if (t > 0 && t <= 1)
		{
			double t = numerator / denom; // t is the signed length as the dir and normal are both normalized

										  // check if the point is inside the trianglular cell
			Point3D interPt = (dir * t) + line0;
			double cellArea = getTriangleArea(plan0, plan1, plan2);

			double accumlatedArea = 0;
			accumlatedArea += getTriangleArea(plan0, plan1, interPt);
			accumlatedArea += getTriangleArea(plan1, plan2, interPt);
			accumlatedArea += getTriangleArea(plan2, plan0, interPt);

			/*double test1 = testPointInTriangle(plan0, plan1, interPt);
			double test2 = testPointInTriangle(plan1, plan2, interPt);
			double test3 = testPointInTriangle(plan2, plan0, interPt);*/

#if DEBUG_INTERSECTION
			cout << "cellArea =\t" << cellArea;
			cout << "\taccumlatedArea =\t" << accumlatedArea << endl;
#endif
			//if (abs(accumlatedArea - cellArea) < 0.015){
			//if((test1 <= 0 && test2 <= 0 && test3 <= 0) || (test1 >= 0 && test2 >= 0 && test3 >= 0)){
			if (abs(accumlatedArea - cellArea) < epsilon) {
				cellArea = getTriangleArea(plan0, plan2, plan3);

				accumlatedArea = 0;
				accumlatedArea += getTriangleArea(plan0, plan2, interPt);
				accumlatedArea += getTriangleArea(plan2, plan3, interPt);
				accumlatedArea += getTriangleArea(plan3, plan0, interPt);
				if (abs(accumlatedArea - cellArea) < epsilon) {
					// there is intersection
					return true;
				}
			}
		}
	}

	return false;
}




bool Misc::isLineIntersectPlan(const Point3D & line0, const  Point3D & dir, double lineLength,
	const  Point3D & plan0, const  Point3D & plan1, const  Point3D & plan2, const Point3D &plan3, const Point3D& normal
	, double denom, double numerator)
{
	// a point p is on the plan if (p - plan[0]) . (normal) == 0
// now the line is (dir * t + line[0]) where t is a paratameter
	// the point p = (x, y, z) is on the line if direction(p - line[0]) == dir and length(p - line[0]) <= length

	// now get the intersection by substituting the line in the plan equation
	// first test dir dot normal
	//double denom = dir.dot(normal);

	// if denom = 0, then the line is parallel to the plan or in the same plan
	//if (abs(denom) < epsilon * epsilon)
	//if (denom == 0)
	//{
	//	// the line is parallel
	//	// if it is on the same plan
	//	//Point3D tmp = line0 - plan0;
	//	//if (tmp.dot(normal) > epsilon)
	//	//{
	//	//	// if the first point of the line is not in the same plan
	//	//	return false;
	//	//}
	//	//else
	//	//{
	//	//	//puts("// there is intersection");
	//	//	return true;
	//	//}

	//	return false;
	//}
	//else
	{
		// there is intersection somehow
		// to get the intersection 
		//Point3D tmpN = plan0 - line0;
		//double numerator = tmpN.dot(normal);

		// if the point within the line segment
		//if (numerator > 0 && numerator <= (lineLength * denom))
			//if (t > 0 && t <= 1)
		{
			double t = numerator / denom; // t is the signed length as the dir and normal are both normalized

										  // check if the point is inside the trianglular cell
			Point3D interPt = (dir * t) + line0;
			double cellArea = getTriangleArea(plan0, plan1, plan2);

			double accumlatedArea = 0;
			accumlatedArea += getTriangleArea(plan0, plan1, interPt);
			accumlatedArea += getTriangleArea(plan1, plan2, interPt);
			accumlatedArea += getTriangleArea(plan2, plan0, interPt);

			/*double test1 = testPointInTriangle(plan0, plan1, interPt);
			double test2 = testPointInTriangle(plan1, plan2, interPt);
			double test3 = testPointInTriangle(plan2, plan0, interPt);*/

#if DEBUG_INTERSECTION
			cout << "cellArea =\t" << cellArea;
			cout << "\taccumlatedArea =\t" << accumlatedArea << endl;
#endif
			//if (abs(accumlatedArea - cellArea) < 0.015){
			//if((test1 <= 0 && test2 <= 0 && test3 <= 0) || (test1 >= 0 && test2 >= 0 && test3 >= 0)){
			if (abs(accumlatedArea - cellArea) < epsilon) {
				return true;
			}
			else{
				cellArea = getTriangleArea(plan0, plan2, plan3);

				accumlatedArea = 0;
				accumlatedArea += getTriangleArea(plan0, plan2, interPt);
				accumlatedArea += getTriangleArea(plan2, plan3, interPt);
				accumlatedArea += getTriangleArea(plan3, plan0, interPt);


#if DEBUG_INTERSECTION
				cout << "cellArea =\t" << cellArea;
				cout << "\taccumlatedArea =\t" << accumlatedArea << endl << endl;
#endif

				if (abs(accumlatedArea - cellArea) < epsilon) {
					// there is intersection
					return true;
				}
			}
		}
	}

	return false;
}

