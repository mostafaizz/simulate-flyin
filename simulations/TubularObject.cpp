#include "TubularObject.h"
#include "misc.h"
#include "Mat3x3.h"
#include <sstream>
#include <map>
// constructors

Point3D TubularObject::calcSurfacePointLocation(int centerLineInd, double rotationAngle, double radius)
{
	/*Rotate space about the x axis so that the rotation axis lies in the xz plane.
	Let U = (a,b,c) be the unit vector along the rotation axis.
	and define d = sqrt(b2 + c2) as the length of the projection onto the yz plane.
	If d = 0 then the rotation axis is along the x axis and no additional rotation is necessary.
	Otherwise rotate the rotation axis so that is lies in the xz plane.
	The rotation angle to achieve this is the angle between the projection of rotation axis in the yz plane and the z axis.
	This can be calculated from the dot product of the z component of the unit vector U and its yz projection.
	The sine of the angle is determine by considering the cross product. */
	double a = tangents[centerLineInd].x();
	double b = tangents[centerLineInd].y();
	double c = tangents[centerLineInd].z();
	double d = tangents[centerLineInd].y()*tangents[centerLineInd].y() + tangents[centerLineInd].z()*tangents[centerLineInd].z();
	d = sqrt(d);

	Point3D axis;
	Mat3x3 Rx = { 1,0,0,0,1,0,0,0,1 };
	Mat3x3 Rx_1 = Rx;
	Mat3x3 Ry, Ry_1, Rz;
	if (d != 0)
	{
		double c_d = c / d;
		double b_d = b / d;

		Rx = { 1, 0, 0, 0, c_d, -b_d,0,b_d,c_d };

		Rx_1 = { 1, 0, 0, 0, c_d, b_d,0,-b_d,c_d };
	}

	/*Rotate space about the y axis so that the rotation axis lies along the positive z axis.
	Using the appropriate dot and cross product relationships as before the cosine of the angle is d, the sine of the angle is a.
	The rotation matrix about the y axis Ry and the inverse Ry-1 (required for step 5) are given below. */

	Ry = { d, 0, -a, 0, 1, 0,a,0,d };
	Ry_1 = { d, 0, a, 0, 1, 0,-a,0,d };

	Rz = { cos(rotationAngle), -sin(rotationAngle), 0, sin(rotationAngle), cos(rotationAngle), 0, 0, 0, 1 };

	axis = Rx_1 * (Ry_1 * (Rz * (Ry * (Rx * xAxis[centerLineInd]))));
	
	
	Point3D res = axis * radius + centerline[centerLineInd];

	

	return res;
}

// assume centerline is intialized
void TubularObject::calcCenterlineAxes()
{
	for (int i = 0; i < centerline.size(); i++)
	{
		// add the new tangent
		if (i == 0)
		{
			tangents.push_back(centerline[i + 1] - centerline[i]);
		}
		else if (i == centerline.size() - 1)
		{
			tangents.push_back(centerline[i] - centerline[i - 1]);
		}
		else
		{
			tangents.push_back((centerline[i + 1] - centerline[i - 1]) * 0.5);
		}
		//cout << i << "\t: (" << centerline[i + 1] << ")\r\n";

		// then normalize the vector
		tangents[i].normalize();
	}
	for (int i = 0; i < 1/*centerline.size()*/; i++)
	{
		Point3D axis[3] = { { 0, 0, 1 },{ 0,1,0 },{ 1, 0,0 } };
		int j = 0;
		for (; j < 3; j++)
		{
			if (axis[j].cross(tangents[i]).getL2Norm() > epsilon)
			{
				break;
			}
		}

		xAxis.push_back(axis[j].cross(tangents[i]));
		xAxis[i].normalize();
		yAxis.push_back(tangents[i].cross(xAxis[i]));
		yAxis[i].normalize();
	}

	// for the rest of the points try to get consistent normal
	for (int i = 1; i < centerline.size(); i++)
	{
		if ((tangents[i].cross(tangents[i - 1])).getL2Norm() < epsilon)
		{
			// parallel and set the axis parallel too
			xAxis.push_back(xAxis[i - 1]);
			yAxis.push_back(yAxis[i - 1]);
		}
		else
		{
			// 
			Point3D tmp = tangents[i].cross(xAxis[i - 1]);
			xAxis.push_back(tmp.cross(tangents[i]));
			xAxis[i].normalize();
			yAxis.push_back(tangents[i].cross(xAxis[i]));
			yAxis[i].normalize();
		}
	}
}

/*
Getting the up direction consistent using Sabry method
vector<double> cylinderR: is the radius for each circle in the the cylinder
rotStep: rotational step while creating the approximate cylinder
path: is the path used to create the deformed cylinder
*/
void TubularObject::init(vector<double>& cylinderR, double rotStep, vector<Point3D>& centerline, bool half, double rotationShift)
{
	vertColor = true;
	faceColor = false;

	this->centerline = centerline;
	double cycle = half ? pi : 2 * pi;

	calcCenterlineAxes();

	maxRadius = 0;
	minRadius = 1000000;
	for (int i = 0; i < centerline.size(); i++)
	{
		// update maximum and minimum radius
		maxRadius = ((cylinderR[i] > maxRadius) ? cylinderR[i] : maxRadius);
		minRadius = ((cylinderR[i] < minRadius) ? cylinderR[i] : minRadius);
	}

	for (int i = 0; i < centerline.size(); i++)
	{
		for (double theta = 0; theta < cycle - epsilon; theta += rotStep)
		{
			Vertex3D vtx;
			vtx.pt = calcSurfacePointLocation(i, theta + rotationShift, cylinderR[i]);
			vertices.push_back(vtx);
		}
	}
	// calcualte cycle size
	cycleSize = std::floor(cycle / rotStep);

	// calculate faces
	calcFaces(half);
	calcFaceNormals();
	calcVertexFaces();
}


/*
Getting the up direction consistent using Sabry method
vector<double> cylinderR: is the radius for each circle in the the cylinder
rotStep: rotational step while creating the approximate cylinder
path: is the path used to create the deformed cylinder
*/
TubularObject::TubularObject(vector<double>& cylinderR, double rotStep, vector<Point3D>& centerline, bool half, double rotationShift)
{
	init(cylinderR, rotStep, centerline, half, rotationShift);
}

TubularObject::TubularObject(string plyFileName, string centerFileName)
{
	vertColor = faceColor = false;
	ifstream inpH, inpD;
	map<Color, int> parts;
	inpH.open(plyFileName.c_str());
	inpD.open(plyFileName.c_str());
	if (inpH.is_open() && inpD.is_open())
	{
		string data;
		while (inpD >> data)
		{
			if ("end_header" == data)
			{
				break;
			}
		}
		string elementType = "";
		int elementSize = 0;
		vector<string> properties;
		while (getline(inpH, data))
		{
			istringstream istr(data);
			string command;
			istr >> command;
			if (command == "ply")
			{
				// start and continue
				continue;
			}
			else if (command == "format")
			{
				string test;
				istr >> test;
				if (test == "ascii")
				{
					continue;
				}
				else
				{
					return;
				}
			}
			else if (command == "comment")
			{
				// this is just a comment
				continue;
			}
			else if (command == "element" || command == "end_header")
			{
				// read the element data first
				while (elementSize--)
				{
					string data2Str;
					for (int t = 0; t < 3 && data2Str.empty(); t++)
					{
						getline(inpD, data2Str);
					}
					istringstream istr2(data2Str);
					if (elementType == "vertex")
					{
						Vertex3D vx;
						istr2 >> vx.pt;
						if (properties.size() > 3)
						{
							vertColor = true;
							// there is color
							istr2 >> vx.color;
						}
						vertices.push_back(vx);
					}
					else if (elementType == "face")
					{
						Face face;
						istr2 >> face;
						face.size = face.indeces.size();
						face.id = faces.size();
						face.obj = this;
						if (properties.size() > 1)
						{
							faceColor = true;
							// there is color after the list of indeces
							istr2 >> face.color;

							if (!parts[face.color])
							{
								parts[face.color] = parts.size();
							}
							face.tag = parts[face.color];
						}
						faces.push_back(face);
					}
				}

				properties.clear();
				// check if this vertex/face/edge
				istr >> elementType;
				istr >> elementSize;
			}
			else if (command == "property")
			{
				string name, type, type1, type2;
				istr >> type >> name;
				if (type == "list")
				{
					type1 = name;
					istr >> type2 >> name;
				}
				properties.push_back(type);
			}
		}
		inpH.close();
		inpD.close();
	}
	maxRadius = 25;
	calcFaceNormals();
	calcVertexFaces();

	cout << "parts.size() = " << parts.size() << endl;
	
	// read centerline file
	ifstream centerStream(centerFileName.c_str());
	Point3D p, tmp1, tmp2, normalPt;
	while (centerStream >> p >> tmp1 >> tmp2 >> normalPt)
	{
		centerline.push_back(p);
		Point3D normal = normalPt - p;
		normal.normalize();
		yAxis.push_back(normal);
		if (centerline.size() > 1)
		{
			Point3D t = p - centerline[centerline.size() - 2];
			t.normalize();
			tangents.push_back(t);
			// add the yAxis
			Point3D y = normal.cross(t);
			y.normalize();
			xAxis.push_back(y);
		}
	}
	// remove the last element from normal
	yAxis.erase(yAxis.end() - 1);

	//calcCenterlineAxes();
}

void TubularObject::calcVertexFaces()
{
	// calculate vertex faces
	for (int i = 0; i < vertices.size(); i++)
	{
		vertexFaces.push_back(unordered_map<int, bool>());
		vertices[i].norm = { 0, 0, 0 };
	}
	for (int i = 0; i < faces.size(); i++)
	{
		for (int j = 0; j < faces[i].indeces.size(); j++)
		{
			int vInd = faces[i].indeces[j];
			
			vertexFaces[vInd][i] = true;
			vertices[vInd].norm = vertices[vInd].norm + faces[i].normal;
		}
	}
	// normalize the normals
	for (int i = 0; i < vertices.size(); i++)
	{
		vertices[i].norm.normalize();
	}
}

// copy constructor
TubularObject::TubularObject(const TubularObject & obj)
{
	*this = obj;
}

void TubularObject::operator=(const TubularObject & obj)
{
	this->vertColor = vertColor;
	this->faceColor = faceColor;
	this->centerline = obj.centerline;
	this->cycleSize = obj.cycleSize;
	this->maxRadius = obj.maxRadius;
	this->minRadius = obj.minRadius;
	this->vertices = obj.vertices;
	this->tangents = obj.tangents;
	this->xAxis = obj.xAxis;
	this->yAxis = obj.yAxis;
	this->faces = obj.faces;
	// update the points references
	for (int f = 0; f < faces.size(); f++)
	{
		for (int j = 0; j < faces[f].indeces.size(); j++)
		{
			faces[f].points[j] = &(vertices[faces[f].indeces[j]].pt);
		}
	}
	this->vertexFaces = obj.vertexFaces;
}

/*
cylinderR: is the radius for the cylinder
rotStep: rotational step while creating the approximate cylinder
path: is the path used to create the deformed cylinder
*/

TubularObject::TubularObject(double cylinderR, double rotStep, vector<Point3D>& centerline, bool half, double shift)
{
	vector<double> tmp(centerline.size(), cylinderR);
	init(tmp, rotStep, centerline, half, shift);
}

// calculate the faces

void TubularObject::calcFaces(bool open)
{
	// calculating faces
	int numFaces = 0;
	if (open)
	{
		numFaces = vertices.size() - (vertices.size() / cycleSize);
	}
	else
	{
		numFaces = vertices.size() - cycleSize;
	}
	for (int i = 0; i < vertices.size() - cycleSize; i++)
	{
		int cycleIndex = i / cycleSize;
		if (open && (i + 1) / cycleSize != cycleIndex)
		{
			continue;
		}
		vector<int> face;
		face.push_back(i);
		face.push_back(i + cycleSize);
		if ((i + 1) / cycleSize == cycleIndex)
		{
			face.push_back(i + cycleSize + 1);
			face.push_back(i + 1);
		}
		else
		{
			face.push_back(i + 1);
			face.push_back(i + 1 - cycleSize);
		}
		// add the index of the face to all vertices connected to this face
		
		// and push the normal to its vector
		//faceNormals.push_back(normal);
		// don't forget to push the face
		Face faceObj;
		faceObj.indeces = face;
		faceObj.id = faces.size();
		if ((faceObj.id % cycleSize) <= cycleSize / 2)
		{
			faceObj.tag = 1;
		}
		else
		{
			faceObj.tag = 2;
		}
		
		int index = faces.size();
		faceObj.size = face.size();
		// then save the face
		faces.push_back(faceObj);
	}
	
}

void TubularObject::calcFaceNormals()
{
	for (int i = 0; i < faces.size(); i++)
	{
		Face& face = faces[i];
		// calculating faces
		Point3D vec1 = vertices[face.indeces[1]].pt - vertices[face.indeces[0]].pt;
		Point3D vec2 = vertices[face.indeces[2]].pt - vertices[face.indeces[0]].pt;
		vec1.normalize();
		vec2.normalize();
		Point3D normal = vec1.cross(vec2);
		normal.normalize();
		// and push the normal to its vector
		face.normal = normal;

		// get face center
		face.center = vertices[face.indeces[0]].pt;
		for (int j = 1; j < face.indeces.size(); j++)
		{
			face.center = face.center + vertices[face.indeces[j]].pt;
		}
		face.center = face.center * (1.0 / face.indeces.size());

		// set pointer to the parent 3D object
		for (int v = 0; v < face.indeces.size(); v++)
		{
			face.points.push_back(&vertices[face.indeces[v]].pt);
		}

		// then calculate the area
		face.totalArea = getFaceArea(face.id, 0, 1, 2);
		if (face.size > 3)
		{
			face.totalArea += getFaceArea(face.id, 0, 2, 3);
		}
	}

}


void TubularObject::transform(Point3D translation, Mat3x3 rotation)
{
	// vertices
	for (int i = 0; i < vertices.size(); i++)
	{
		vertices[i].norm = rotation * (vertices[i].norm + vertices[i].pt - translation);
		vertices[i].pt = rotation * (vertices[i].pt - translation);

		vertices[i].norm = vertices[i].norm - vertices[i].pt;
		vertices[i].norm.normalize();
	}

	// faces
	for (int i = 0; i < faces.size(); i++)
	{
		faces[i].normal = rotation * (faces[i].normal + faces[i].center - translation);
		faces[i].center = rotation * (faces[i].center - translation);

		faces[i].normal = faces[i].normal - faces[i].center;
		faces[i].normal.normalize();
	}
}


void TubularObject::colorVertices(vector<double> measure, vector<Color> colors, bool logScale)
{
	vertColor = true;
	//
	Color black(0, 0,0);
	for (int i = 0; i < vertices.size(); i++)
	{
		if (measure.size() == 0)
		{
			vertices[i].color = black;
		}
		else
		{
			if (colors.size())
			{
				// use color palette
				//int ind = colors.size() * ((measure[i] - minM) / diff);
				double tempMeasure = measure[i] + 1;
				if (logScale)
				{
					tempMeasure = std::log10(1 + tempMeasure * 9);
				}
				int ind = colors.size() * tempMeasure;
				if (measure[i] > 0)
				{
					ind = colors.size() - 1;
				}
				vertices[i].color = colors[ind];
			}
			else
			{
				double tempMeasure = measure[i] + 1;
				if (logScale)
				{
					tempMeasure = std::log10(1 + tempMeasure * 9);
				}
				//grey scale
				int blue = 255 * (1 - tempMeasure);
				int red = 255 * (tempMeasure);
				vertices[i].color = Color(red, 0, blue);
			}
		}
	}
}

void TubularObject::writePLYFileColorVertices(string name, bool halves, bool openWithMeshLab)
{
	ofstream oup[3]; // complete cylinder, first half, second half
	oup[0].open(name + ".ply");
	int cnt = 1;
	if (halves)
	{
		oup[1].open(name + "_1st" + ".ply");
		oup[2].open(name + "_2nd" + ".ply");
		cnt = 3;
	}
	for (int c = 0; c < cnt; c++)
	{
		oup[c] << "ply" << endl;
		oup[c] << "format ascii 1.0" << endl;
		oup[c] << "comment object: vertex cloud" << endl;
		oup[c] << "element vertex " << vertices.size() << endl;
		oup[c] << "property double x" << endl;
		oup[c] << "property double y" << endl;
		oup[c] << "property double z" << endl;
		if (vertColor)
		{
			oup[c] << "property uchar red" << endl;
			oup[c] << "property uchar green" << endl;
			oup[c] << "property uchar blue" << endl;
		}
		if (c == 0)
		{
			// full cylinder
			oup[c] << "element face " << faces.size() << endl;
		}
		else
		{
			// half
			oup[c] << "element face ";
			if (c == 1)
			{
				oup[c] << getNumberOfFaces1stHalf() << endl;
			}
			else if (c == 2)
			{
				oup[c] << getNumberOfFaces2ndHalf() << endl;
			}
		}
		oup[c] << "property list uchar int vertex_indices" << endl;
		if (faceColor)
		{
			oup[c] << "property uchar red" << endl;
			oup[c] << "property uchar green" << endl;
			oup[c] << "property uchar blue" << endl;
		}
		oup[c] << "end_header" << endl;

		for (int i = 0; i < vertices.size(); i++)
		{
			oup[c] << vertices[i].pt;
			if (vertColor)
			{
				oup[c] << " " << vertices[i].color;
			}
			oup[c] << endl;
		}
		for (int i = 0; i < faces.size(); i++)
		{
			Face face = faces[i];
			if (c == 0 || face.tag == c)
			{
				oup[c] << face.size << " " << face;
				if (faceColor)
				{
					oup[c] << " " << face.color;
				}
				oup[c] << endl;
			}
		}
		oup[c].close();
	}
	if (openWithMeshLab)
	{
		string cmd = "\"C:\\Program Files\\VCG\\MeshLab\\meshlab.exe\" " + name + ".ply";
		system(cmd.c_str());
	}
}


void TubularObject::writeOBJFileWithMaterial(string name, bool openWithMeshLab)
{
	// open the file and write the data
	ofstream obj(name + ".obj");
	obj << "mtllib " << name << ".mtl\n\n";

	for (int i = 0; i < vertices.size(); i++)
	{
		obj << "v " << vertices[i].pt << endl;
	}

	// calculate face colors
	map<Color, int> colors;
	vector<Color> vc;
	for (int i = 0; i < faces.size(); i++)
	{
		//obj << "v";
		int r = 0, g = 0, b = 0;
		for (int j = 0; j < faces[i].indeces.size(); j++)
		{
			//obj << " " << faces[i].indeces[j] + 1;

			r += vertices[faces[i].indeces[j]].color.r;
			g += vertices[faces[i].indeces[j]].color.g;
			b += vertices[faces[i].indeces[j]].color.b;
		}
		faces[i].color.r = r / faces[i].size;
		faces[i].color.g = g / faces[i].size;
		faces[i].color.b = b / faces[i].size;

		if (!colors[faces[i].color])
		{
			vc.push_back(faces[i].color);
			colors[faces[i].color] = vc.size();
		}
		obj << "usemtl color_" << colors[faces[i].color] << endl;

		obj << "f";
		for (int j = 0; j < faces[i].indeces.size(); j++)
		{
			obj << " " << faces[i].indeces[j] + 1;
		}
		obj << endl;
	}
	obj.close();
	
	ofstream mtl(name + ".mtl");
	// write the material file
	for (int i = 0; i < vc.size(); i++)
	{
		mtl << "newmtl color_" << i << endl;
		mtl << "Kd " << vc[i].r / 255.0 << " " << vc[i].g / 255.0 << " " << vc[i].b / 255.0 << endl;
		cout << endl;
	}
	mtl.close();
}

void TubularObject::writeVerticesToFile(string fileName)
{
	ofstream oup(fileName);
	for (int i = 0; i < vertices.size(); i++)
	{
		oup << vertices[i].pt << "," << vertices[i].norm << endl;
	}
	oup.close();
}

int TubularObject::getNumberOfFaces1stHalf() {
	int count = 0;
	for (int i = 0; i < faces.size(); i++)
	{
		if (faces[i].tag == 1)
		{
			count++;
		}
	}
	return count;
}

int TubularObject::getNumberOfFaces2ndHalf() {
	int count = 0;
	for (int i = 0; i < faces.size(); i++)
	{
		if (faces[i].tag == 2)
		{
			count++;
		}
	}
	return count;
}

// get face triangle area
double TubularObject::getFaceArea(const int faceInd, const int ind0, const int ind1, const int ind2) const
{
	return Misc::getTriangleArea(vertices[faces[faceInd].indeces[ind0]].pt,
		vertices[faces[faceInd].indeces[ind1]].pt,
		vertices[faces[faceInd].indeces[ind2]].pt);
}


// if the point in the same plane as this face, is it inside the face or not
bool Face::isPointInside(const Point3D & interPt) const
{
	double accumlatedArea = 0;
	if (size == 4)
	{
		double a = Misc::getTriangleArea2(*points[0], *points[1], interPt);
		double c = Misc::getTriangleArea2(*points[2], *points[3], interPt);
		double b = Misc::getTriangleArea2(*points[1], *points[2], interPt);
		double d = Misc::getTriangleArea2(*points[3], *points[0], interPt);

		accumlatedArea = a + b + c + d;
	}
	else if (size == 3)
	{
		double a = Misc::getTriangleArea2(*points[0], *points[1], interPt);
		double b = Misc::getTriangleArea2(*points[1], *points[2], interPt);
		double c = Misc::getTriangleArea2(*points[2], *points[0], interPt);
		
		accumlatedArea = a + b + c;
	}
	double test = epsilon - (accumlatedArea - 2 * totalArea);
	return (test < 2 * epsilon);
}

ostream & operator<<(ostream & ostr, Face & fc)
{
	// TODO: insert return statement here
	for (int i = 0; i < fc.indeces.size(); i++)
	{
		ostr << " " << fc.indeces[i];
	}

	return ostr;
}

istream & operator >> (istream & istr, Face & fc)
{
	// TODO: insert return statement here
	int size;
	istr >> size;
	fc.indeces.clear();
	for (int i = 0; i < size; i++)
	{
		int temp;
		istr >> temp;
		fc.indeces.push_back(temp);
	}
	return istr;
}
