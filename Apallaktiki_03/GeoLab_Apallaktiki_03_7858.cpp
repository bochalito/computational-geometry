#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <MathGeoLib/MathGeoLib.h>
#include "utils.h"
#include "canvas.h"
#include "GeoLab_Apallaktiki_03_7858.h"

#define  NOMINMAX  1// Fixes a problem on windows

#define CONFIG_FILE_PATH "../Apallaktiki_03/config.txt"
#define POLYGON_FILENAME  "../Apallaktiki_03/polygons/polygon.txt"
#define MIN_POINT_DIST_PIXELS 10

#define FLAG_SHOW_AXES			1
#define FLAG_SHOW_AABB			2
#define FLAG_SHOW_WIRE			4
#define FLAG_SHOW_SOLID			8
#define FLAG_SHOW_NORMALS		32768
#define FLAG_SHOW_CURV2D		32
#define FLAG_SHOW_CURV3D		64
#define FLAG_SHOW_CONVEX		128	
#define FLAG_SHOW_CONVEX_CURV3D_GAUSS 256
#define FLAG_TRIAN_AGAIN		512
#define FLAG_SMOOTH_GAUSS		1024
#define FLAG_SHOW_LOCAL_MAX_GAUSS		2048
#define FLAG_SMOOTH_MEAN		4096
#define FLAG_SHOW_CURV3D_MEAN   8192
#define FLAG_SHOW_CONVEX_CURV3D_MEAN 16384
#define FLAG_SHOW_LOCAL_MAX_MEAN 16


using namespace math;
using namespace vvr;

bool CH_draw_flag = false;
bool Triang_Convex_Hull = false;
bool Smoothing = false;
bool Button_6 = false;
bool Button_m = false;
bool Smoothing_Factor_Gauss_Read = false;
bool Smoothing_Factor_Gauss_Flag = false;
bool Smoothing_Factor_Mean_Read = false;
bool Smoothing_Factor_Mean_Flag = false;
bool Gauss_Threshold_Read = false;
bool Mean_Threshold_Read = false;
double Gauss_Threshold;
double Mean_Threshold;
double Gauss_Smoothing;
double Mean_Smoothing;
string Gauss_Factor = "200.0";
string Mean_Factor = "200.0";

typedef struct {
	double red, green, blue;
} COLOUR;

COLOUR GetRGBValue(double value, double vmin, double vmax)
{
	////////////////////////////////////////////
	/////     Hot-to-Cold Colour Ramp      /////
	////////////////////////////////////////////

	COLOUR color = { 1.0, 1.0, 1.0 }; // white
	double dv;

	// Push Colour inside the range if it is outside

	if (value < vmin)
		value = vmin;
	if (value > vmax)
		value = vmax;
	dv = vmax - vmin;

	// Depending on our value I give specific color to the point
	// based on which space of values my value lies on

	if (value < (vmin + 0.25 * dv)) {
		color.red = 0;
		color.green = 4 * (value - vmin) / dv;
	}
	else if (value < (vmin + 0.5 * dv)) {
		color.red = 0;
		color.blue = 1 + 4 * (vmin + 0.25 * dv - value) / dv;
	}
	else if (value < (vmin + 0.75 * dv)) {
		color.red = 4 * (value - vmin - 0.5 * dv) / dv;
		color.blue = 0;
	}
	else {
		color.green = 1 + 4 * (vmin + 0.75 * dv - value) / dv;
		color.blue = 0;
	}

	return(color);
}

Simple3DScene::Simple3DScene()
{
	// Load settings.
	m_settings = Settings(getExePath() + CONFIG_FILE_PATH);
	m_bg_col = Colour(m_settings.getStr("color_bg"));
	m_obj_col = Colour(m_settings.getStr("color_obj"));
	m_perspective_proj = m_settings.getBool("perspective_proj");
	m_style_flag = FLAG_SHOW_SOLID;

	// Scene rotation.
	const double def_rot_x = m_settings.getDbl("def_rot_x");
	const double def_rot_y = m_settings.getDbl("def_rot_y");
	const double def_rot_z = m_settings.getDbl("def_rot_z");
	m_globRot_def = Vec3d(def_rot_x, def_rot_y, def_rot_z);
	m_globRot = m_globRot_def;

	// Load 3D models.
	const string objDir = getExePath() + m_settings.getStr("obj_dir");
	const string objFile = getExePath() + m_settings.getStr("obj_file");

	// H teleytaia parametros kathorizei an ta trigwna dinontai se CW/CCW fora.
	// An to 3D antikeimenou emfanizetai xwris skies, allakse ayti tin parametro.
	m_model = Mesh(objDir, objFile, "", false);

	// Fortwse to polygwno apo tin mnimi
	loadPolygonFromFile(getExePath() + POLYGON_FILENAME);
	b_show_pts = false;

}

void Simple3DScene::resize()
{
	// Making FIRST PASS static and initialing it to true we make
	// sure that the if block will be executed only once.

	static bool FLAG_FIRST_PASS = true;
	if (FLAG_FIRST_PASS)
	{
		m_sphere_rad = getSceneWidth() / 8;
		m_model.setBigSize(getSceneWidth() / 6);
		m_model.centerAlign();
		consoleMenu();
		FLAG_FIRST_PASS = false;
	}
}

void Simple3DScene::curvature2D()
{
	enterPixelMode();

	// Initialize variables
	double curv;
	vector<double> curvature;
	double max_curv = -10000;

	// Checking all points of polygon to calculate curvature for each one 
	// In order to calculate curvature I get 3 neighbor points and find the triangle from these 3 points
	// Then we can find the circumcircle of the triangle
	// So 1/radius_of_circumcircle will be the curvature on the middle point
	// Although the method I used to find curvature was K = (4*triangle_area)/(|AB|*|AC|*|BC|)
	// With the second method I got better results
	C2DPoint p1, p2, p3;

	for (int i = 0; i < m_pts.size(); i++)
	{
		// Getting neighbor points to calculate curvature on the middle one
		if (i == 0)
		{
			// If we are at first point we need to take the last one first
			p1 = m_pts[m_pts.size() - 1];
			p2 = m_pts[i];
			p3 = m_pts[i + 1];
		}
		else if (i == m_pts.size() - 1)
		{
			// If we are at the last point of our polygon we need to take
			// the last one, the one before that and the first point of our polygon
			p1 = m_pts[m_pts.size() - 2];
			p2 = m_pts[m_pts.size() - 1];
			p3 = m_pts[0];
		}
		else
		{
			p1 = m_pts[i - 1];
			p2 = m_pts[i];
			p3 = m_pts[i + 1];
		}

		// Getting lines of the triangle 
		C2DLine l1(p1, p2);
		C2DLine l2(p1, p3);
		C2DLine l3(p2, p3);
		C2DTriangle t(p1, p2, p3);

		// Calculating curvature with my formula
		curv = (4.0 * t.GetAreaSigned()) / (l1.GetLength()*l2.GetLength()*l3.GetLength());
		curvature.push_back(curv);

		// Finding max curvature to use it for the coloring of our line  
		if (curv > max_curv)
			max_curv = curv;
	}

	for (int i = 0; i < curvature.size(); i++)
	{
		C2DPoint p1, p2, p3;

		if (i == 0)
		{
			// If we are at first point we need to take the last one first
			p1 = m_pts[m_pts.size() - 1];
			p2 = m_pts[i];
			p3 = m_pts[i + 1];
		}
		else if (i == m_pts.size() - 1)
		{
			// If we are at the last point of our polygon we need to take
			// the last one, the one before that and the first point of our polygon
			p1 = m_pts[m_pts.size() - 2];
			p2 = m_pts[m_pts.size() - 1];
			p3 = m_pts[0];
		}
		else
		{
			p1 = m_pts[i - 1];
			p2 = m_pts[i];
			p3 = m_pts[i + 1];
		}

		// Calculating percentange of curvature in order to use it for coloring
		// I use abs because negative curvature indicates indicates the direction 
		// In which the unit tangent vector rotates as a function of the parameter
		// Along the curve. If the unit tangent rotates counterclockwise, then curv > 0
		// If it rotates clockwise, then curv < 0. 
		double percent = abs((curvature.at(i) / (max_curv)));

		// Coloring the polygon using my formula, based on percentange of curvature on each point 
		Colour cper((GetRGBValue(percent, 0.0, 1.0).red)*255.0, (GetRGBValue(percent, 0.0, 1.0).green)*255.0, (GetRGBValue(percent, 0.0, 1.0).blue)*255.0);

		// Drawing a new polygon above the first one but this time showing the curvature
		// I draw lines from midpoint to midpoint with the color found above to show the curvature on each point
		if (i < m_pts.size() - 2)
		{
			C2DLine l1(p1, p2);
			C2DLine l2(p2, p3);
			Point2D midlast(l1.GetMidPoint().x, l1.GetMidPoint().y);
			Point2D midnext(l2.GetMidPoint().x, l2.GetMidPoint().y);
			LineSeg2D l1new(midlast.x, midlast.y, p2.x, p2.y, cper);
			LineSeg2D l2new(midnext.x, midnext.y, p2.x, p2.y, cper);
			l1new.draw();
			l2new.draw();
		}
	}
	returnFromPixelMode();
}

double Simple3DScene::meanCurvatureThreshold(Mesh &mesh)
{
	// Here we calculate the threshold to cut the spikes of the curvature
	// To do this we run the algorithm once to find the curvature on each 
	// vertex and then we calculate the mean value and the max value.
	// Then the user is asked to determine the threshold. 
	// Usually it is the mean value multiplied by 2

	vector<Vec3d> &vertices = mesh.getVertices();
	vector<vvr::Triangle> &tri = mesh.getTriangles();
	pair<vvr::Vec3d, vvr::Vec3d> temp_Vertices;
	vector<pair<double, Vec3d>> curv_at_vertex;
	Vec3d temp_Vertex;
	vector<vvr::Triangle> star;
	string threshold;
	double temp_Curv;
	double sum_of_curv = 0;
	double sumCotan;
	double sum;
	double max = 0;
	double meanvalue;
	double thresh;
	bool flag = false;

	for (int i = 0; i < vertices.size(); i++)
	{
		sum = 0;
		findStar(vertices[i], mesh, star);
		for (int j = 0; j < star.size(); j++)
		{
			if ((star[j].v1().x == vertices[i].x)
				&& (star[j].v1().y == vertices[i].y)
				&& (star[j].v1().z == vertices[i].z))
			{
				temp_Vertices = (make_pair(vertices[i], star[j].v2()));
				sumCotan = findCotaCotb(star, temp_Vertices);
				temp_Vertex.x = sumCotan * (star[j].v2().x - vertices[i].x);
				temp_Vertex.y = sumCotan * (star[j].v2().y - vertices[i].y);
				temp_Vertex.z = sumCotan * (star[j].v2().z - vertices[i].z);
				sum += sqrt(pow(temp_Vertex.x, 2) + pow(temp_Vertex.y, 2) + pow(temp_Vertex.z, 2));



				temp_Vertices = (make_pair(vertices[i], star[j].v3()));
				sumCotan = findCotaCotb(star, temp_Vertices);
				temp_Vertex.x = sumCotan * (star[j].v3().x - vertices[i].x);
				temp_Vertex.y = sumCotan * (star[j].v3().y - vertices[i].y);
				temp_Vertex.z = sumCotan * (star[j].v3().z - vertices[i].z);
				sum += sqrt(pow(temp_Vertex.x, 2) + pow(temp_Vertex.y, 2) + pow(temp_Vertex.z, 2));


			}
			else if ((star[j].v2().x == vertices[i].x)
				&& (star[j].v2().y == vertices[i].y)
				&& (star[j].v2().z == vertices[i].z))
			{
				temp_Vertices = (make_pair(vertices[i], star[j].v1()));
				sumCotan = findCotaCotb(star, temp_Vertices);
				temp_Vertex.x = sumCotan * (star[j].v1().x - vertices[i].x);
				temp_Vertex.y = sumCotan * (star[j].v1().y - vertices[i].y);
				temp_Vertex.z = sumCotan * (star[j].v1().z - vertices[i].z);
				sum += sqrt(pow(temp_Vertex.x, 2) + pow(temp_Vertex.y, 2) + pow(temp_Vertex.z, 2));



				temp_Vertices = (make_pair(vertices[i], star[j].v3()));
				sumCotan = findCotaCotb(star, temp_Vertices);
				temp_Vertex.x = sumCotan * (star[j].v3().x - vertices[i].x);
				temp_Vertex.y = sumCotan * (star[j].v3().y - vertices[i].y);
				temp_Vertex.z = sumCotan * (star[j].v3().z - vertices[i].z);
				sum += sqrt(pow(temp_Vertex.x, 2) + pow(temp_Vertex.y, 2) + pow(temp_Vertex.z, 2));


			}
			else if ((star[j].v3().x == vertices[i].x)
				&& (star[j].v3().y == vertices[i].y)
				&& (star[j].v3().z == vertices[i].z))
			{
				temp_Vertices = (make_pair(vertices[i], star[j].v2()));
				sumCotan = findCotaCotb(star, temp_Vertices);
				temp_Vertex.x = sumCotan * (star[j].v2().x - vertices[i].x);
				temp_Vertex.y = sumCotan * (star[j].v2().y - vertices[i].y);
				temp_Vertex.z = sumCotan * (star[j].v2().z - vertices[i].z);
				sum += sqrt(pow(temp_Vertex.x, 2) + pow(temp_Vertex.y, 2) + pow(temp_Vertex.z, 2));



				temp_Vertices = (make_pair(vertices[i], star[j].v1()));
				sumCotan = findCotaCotb(star, temp_Vertices);
				temp_Vertex.x = sumCotan * (star[j].v1().x - vertices[i].x);
				temp_Vertex.y = sumCotan * (star[j].v1().y - vertices[i].y);
				temp_Vertex.z = sumCotan * (star[j].v1().z - vertices[i].z);
				sum += sqrt(pow(temp_Vertex.x, 2) + pow(temp_Vertex.y, 2) + pow(temp_Vertex.z, 2));


			}
		}
		temp_Curv = sum / (8.0*findAreaOfVoronoiRegion(star, vertices[i]));
		sum_of_curv += temp_Curv;
		if (temp_Curv > max)
			max = temp_Curv;

		curv_at_vertex.push_back(make_pair(temp_Curv, vertices[i]));
	}

	meanvalue = sum_of_curv / vertices.size();

	// Here user is asked about curvature's threshold
	cout << "\n		Mean Curvature\n";
	cout << "\n	...Max value of mean curvature = " << max << "\n";
	cout << "\n	...Mean value of mean curvature = " << meanvalue << "\n";
	cout << "\n	...Please type threshold: ";
	cin >> threshold;
	while (!flag)
	{
		try
		{
			thresh = stod(threshold);
			flag = true;
		}
		catch (exception e)
		{
			cout << "\n		WRONG INPUT!!! \n";
			cout << "\n	...Please type threshold(double value): ";
			cin >> threshold;
		}
	}

	return thresh;
}

void Simple3DScene::curvature3DMean(Mesh &mesh)
{
	/////////////////////////////////////////////////////////////////////////
	//////          COMPUTATION OF MEAN CURVATURE USING              ////////
	//////          DISCRETE LAPLACE-BELTRAMI OPERATOR               ////////
	/////////////////////////////////////////////////////////////////////////

	vector<Vec3d> &vertices = mesh.getVertices();
	vector<vvr::Triangle> &tri = mesh.getTriangles();
	pair<vvr::Vec3d, vvr::Vec3d> temp_Vertices;
	double sumCotan;
	double sum;
	Vec3d temp_Vertex;
	vector<vvr::Triangle> star;
	double temp_Curv;
	vector<pair<double, Vec3d>> curv_at_vertex;

	for (int i = 0; i < vertices.size(); i++)
	{
		// For each vertex of our model we find the star of triangles at that vertex

		sum = 0;
		findStar(vertices[i], mesh, star);
		for (int j = 0; j < star.size(); j++)
		{
			// For each triangle of the star we calculate cot(a),cot(b)
			// We do that for each edge of the triangle, so we have to divide with 2 
			// when we find the curvature at point p

			if ((star[j].v1().x == vertices[i].x)
				&& (star[j].v1().y == vertices[i].y)
				&& (star[j].v1().z == vertices[i].z))
			{
				temp_Vertices = (make_pair(vertices[i], star[j].v2()));
				sumCotan = findCotaCotb(star, temp_Vertices);
				temp_Vertex.x = sumCotan * (star[j].v2().x - vertices[i].x);
				temp_Vertex.y = sumCotan * (star[j].v2().y - vertices[i].y);
				temp_Vertex.z = sumCotan * (star[j].v2().z - vertices[i].z);
				sum += sqrt(pow(temp_Vertex.x, 2) + pow(temp_Vertex.y, 2) + pow(temp_Vertex.z, 2));

				temp_Vertices = (make_pair(vertices[i], star[j].v3()));
				sumCotan = findCotaCotb(star, temp_Vertices);
				temp_Vertex.x = sumCotan * (star[j].v3().x - vertices[i].x);
				temp_Vertex.y = sumCotan * (star[j].v3().y - vertices[i].y);
				temp_Vertex.z = sumCotan * (star[j].v3().z - vertices[i].z);
				sum += sqrt(pow(temp_Vertex.x, 2) + pow(temp_Vertex.y, 2) + pow(temp_Vertex.z, 2));


			}
			else if ((star[j].v2().x == vertices[i].x)
				&& (star[j].v2().y == vertices[i].y)
				&& (star[j].v2().z == vertices[i].z))
			{
				temp_Vertices = (make_pair(vertices[i], star[j].v1()));
				sumCotan = findCotaCotb(star, temp_Vertices);
				temp_Vertex.x = sumCotan * (star[j].v1().x - vertices[i].x);
				temp_Vertex.y = sumCotan * (star[j].v1().y - vertices[i].y);
				temp_Vertex.z = sumCotan * (star[j].v1().z - vertices[i].z);
				sum += sqrt(pow(temp_Vertex.x, 2) + pow(temp_Vertex.y, 2) + pow(temp_Vertex.z, 2));



				temp_Vertices = (make_pair(vertices[i], star[j].v3()));
				sumCotan = findCotaCotb(star, temp_Vertices);
				temp_Vertex.x = sumCotan * (star[j].v3().x - vertices[i].x);
				temp_Vertex.y = sumCotan * (star[j].v3().y - vertices[i].y);
				temp_Vertex.z = sumCotan * (star[j].v3().z - vertices[i].z);
				sum += sqrt(pow(temp_Vertex.x, 2) + pow(temp_Vertex.y, 2) + pow(temp_Vertex.z, 2));


			}
			else if ((star[j].v3().x == vertices[i].x)
				&& (star[j].v3().y == vertices[i].y)
				&& (star[j].v3().z == vertices[i].z))
			{
				temp_Vertices = (make_pair(vertices[i], star[j].v2()));
				sumCotan = findCotaCotb(star, temp_Vertices);
				temp_Vertex.x = sumCotan * (star[j].v2().x - vertices[i].x);
				temp_Vertex.y = sumCotan * (star[j].v2().y - vertices[i].y);
				temp_Vertex.z = sumCotan * (star[j].v2().z - vertices[i].z);
				sum += sqrt(pow(temp_Vertex.x, 2) + pow(temp_Vertex.y, 2) + pow(temp_Vertex.z, 2));



				temp_Vertices = (make_pair(vertices[i], star[j].v1()));
				sumCotan = findCotaCotb(star, temp_Vertices);
				temp_Vertex.x = sumCotan * (star[j].v1().x - vertices[i].x);
				temp_Vertex.y = sumCotan * (star[j].v1().y - vertices[i].y);
				temp_Vertex.z = sumCotan * (star[j].v1().z - vertices[i].z);
				sum += sqrt(pow(temp_Vertex.x, 2) + pow(temp_Vertex.y, 2) + pow(temp_Vertex.z, 2));


			}
		}

		// Calculate total curvature at point p

		temp_Curv = sum / (8.0*findAreaOfVoronoiRegion(star, vertices[i]));
		if (temp_Curv > Mean_Threshold)
			temp_Curv = Mean_Threshold;

		curv_at_vertex.push_back(make_pair(temp_Curv, vertices[i]));
	}

	double max = 0.0;
	for (int i = 0; i < curv_at_vertex.size(); i++)
	{
		if (curv_at_vertex[i].first > max)
			max = curv_at_vertex[i].first;
	}

	// Drawing of the model

	for (int j = 0; j < tri.size(); j++){
		double sum = 0;
		vvr::Triangle triangle = tri.at(j);
		Vec3d tri_v1, tri_v2, tri_v3, ver;

		tri_v1.x = triangle.v1().x;
		tri_v1.y = triangle.v1().y;
		tri_v1.z = triangle.v1().z;

		tri_v2.x = triangle.v2().x;
		tri_v2.y = triangle.v2().y;
		tri_v2.z = triangle.v2().z;

		tri_v3.x = triangle.v3().x;
		tri_v3.y = triangle.v3().y;
		tri_v3.z = triangle.v3().z;

		for (int k = 0; k < curv_at_vertex.size(); k++){
			ver.x = curv_at_vertex.at(k).second.x;
			ver.y = curv_at_vertex.at(k).second.y;
			ver.z = curv_at_vertex.at(k).second.z;

			if ((tri_v1 == ver) || (tri_v2 == ver) || (tri_v3 == ver)) {
				sum += curv_at_vertex.at(k).first;
			}
		}

		double percent = ((sum / 3.0) / max);

		Colour cper((GetRGBValue(percent, 0.0, 1.0).red)*255.0, (GetRGBValue(percent, 0.0, 1.0).green)*255.0, (GetRGBValue(percent, 0.0, 1.0).blue)*255.0);
		Triangle3D(triangle.v1().x, triangle.v1().y, triangle.v1().z,
			triangle.v2().x, triangle.v2().y, triangle.v2().z,
			triangle.v3().x, triangle.v3().y, triangle.v3().z, cper).draw();
	}

	findLocalMaxOfCurvature(curv_at_vertex, vertices_with_maxMeanCurv);
	curv_at_vertex.clear();
}

double Simple3DScene::findCotaCotb(vector<vvr::Triangle> star, pair<vvr::Vec3d, vvr::Vec3d> temp_Vertices)
{
	double angle_a, angle_b;
	double edge_1_lenght;
	double edge_2_lenght;
	double edge_3_length;
	double cota, cotb;
	int flag = 0;
	pair<vvr::Vec3d, vvr::Vec3d> edge_1, edge_2, edge_3;

	for (int i = 0; i < star.size(); i++)
	{
		edge_1.first = star[i].v1();
		edge_1.second = star[i].v2();

		edge_2.first = star[i].v1();
		edge_2.second = star[i].v3();

		edge_3.first = star[i].v2();
		edge_3.second = star[i].v3();

		edge_1_lenght = sqrt(pow((edge_1.first.x - edge_1.second.x), 2)
			+ pow((edge_1.first.y - edge_1.second.y), 2)
			+ pow((edge_1.first.z - edge_1.second.z), 2));
		edge_2_lenght = sqrt(pow((edge_2.first.x - edge_2.second.x), 2)
			+ pow((edge_2.first.y - edge_2.second.y), 2)
			+ pow((edge_2.first.z - edge_2.second.z), 2));
		edge_3_length = sqrt(pow((edge_3.first.x - edge_3.second.x), 2)
			+ pow((edge_3.first.y - edge_3.second.y), 2)
			+ pow((edge_3.first.z - edge_3.second.z), 2));

		if (((temp_Vertices.first == edge_1.first) && (temp_Vertices.second == edge_1.second))
			|| ((temp_Vertices.first == edge_1.second) && (temp_Vertices.second == edge_1.first)))
		{

			if (flag == 0){
				angle_a = acos((pow(edge_2_lenght, 2) + pow(edge_3_length, 2) - pow(edge_1_lenght, 2))
					/ (2.0*edge_2_lenght*edge_3_length));
				flag++;
			}
			if (flag == 1){
				angle_b = acos((pow(edge_2_lenght, 2) + pow(edge_3_length, 2) - pow(edge_1_lenght, 2))
					/ (2.0*edge_2_lenght*edge_3_length));
				flag++;
			}
		}
		else if (((temp_Vertices.first == edge_2.first) && (temp_Vertices.second == edge_2.second))
			|| ((temp_Vertices.first == edge_2.second) && (temp_Vertices.second == edge_2.first)))
		{

			if (flag == 0){
				angle_a = acos((pow(edge_1_lenght, 2) + pow(edge_3_length, 2) - pow(edge_2_lenght, 2))
					/ (2.0*edge_1_lenght*edge_3_length));
				flag++;
			}
			if (flag == 1){
				angle_b = acos((pow(edge_1_lenght, 2) + pow(edge_3_length, 2) - pow(edge_2_lenght, 2))
					/ (2.0*edge_1_lenght*edge_3_length));
				flag++;
			}
		}
		else if (((temp_Vertices.first == edge_3.first) && (temp_Vertices.second == edge_3.second))
			|| ((temp_Vertices.first == edge_3.second) && (temp_Vertices.second == edge_3.first)))
		{

			if (flag == 0){
				angle_a = acos((pow(edge_1_lenght, 2) + pow(edge_2_lenght, 2) - pow(edge_3_length, 2))
					/ (2.0*edge_1_lenght*edge_2_lenght));
				flag++;
			}
			if (flag == 1){
				angle_b = acos((pow(edge_1_lenght, 2) + pow(edge_2_lenght, 2) - pow(edge_3_length, 2))
					/ (2.0*edge_1_lenght*edge_2_lenght));
				flag++;
			}
		}
	}
	cota = 1.0 / tan(angle_a);
	cotb = 1.0 / tan(angle_b);


	return (cota * 180 / pi) + (cotb * 180 / pi);
}

void Simple3DScene::findStar(vvr::Vec3d p, Mesh &mesh, vector<vvr::Triangle> &star)
{
	// Here we find the star around vertex p
	// To do that we find all the triangles with top same as vertex p

	vector<Vec3d> &vertices = mesh.getVertices();
	vector<vvr::Triangle> &triangles = mesh.getTriangles();
	star.clear();

	for (int i = 0; i < triangles.size(); i++)
	{
		if ((triangles.at(i).v1().x == p.x)
			&& (triangles.at(i).v1().y == p.y)
			&& (triangles.at(i).v1().z == p.z))
		{
			star.push_back(triangles.at(i));
		}
		else if ((triangles.at(i).v2().x == p.x)
			&& (triangles.at(i).v2().y == p.y)
			&& (triangles.at(i).v2().z == p.z))
		{
			star.push_back(triangles.at(i));
		}
		else if ((triangles.at(i).v3().x == p.x)
			&& (triangles.at(i).v3().y == p.y)
			&& (triangles.at(i).v3().z == p.z))
		{
			star.push_back(triangles.at(i));
		}
	}
}

double Simple3DScene::findAreaOfVoronoiRegion(vector<vvr::Triangle> &star, Vec3d p)
{
	// Here we caculate the area of voronoi region which is necessary
	// for mean curvature. To do this we find the center of each triangle that
	// belongs to vertex's p star and then we create a polygon with the center of
	// each triangle, the middle point from PA, the middle point from PB and point P
	// Then we sum each area and find the total area of voronoi region

	Polygon polygon;
	math::LineSegment edge;
	math::Triangle triangle;
	math::vec point;
	double area = 0.0;

	point.x = p.x;
	point.y = p.y;
	point.z = p.z;
	for (int i = 0; i < star.size(); i++)
	{
		polygon.p.clear();

		// Transform vvr::Triangle to math::Triangle 
		triangle.a.x = star.at(i).v1().x;
		triangle.a.y = star.at(i).v1().y;
		triangle.a.z = star.at(i).v1().z;

		triangle.b.x = star.at(i).v2().x;
		triangle.b.y = star.at(i).v2().y;
		triangle.b.z = star.at(i).v2().z;

		triangle.c.x = star.at(i).v3().x;
		triangle.c.y = star.at(i).v3().y;
		triangle.c.z = star.at(i).v3().z;

		// Check to find which point of the triangle is point p
		if ((triangle.a.x == point.x) && (triangle.a.y == point.y) && (triangle.a.z == point.z)){

			// If triangle.a = point p we create the polugon and we calculate the area 
			// of the small polygon created by the vertex p and the mid point of the two
			// edges and the center od the triangle.
			polygon.p.clear();
			polygon.p.push_back(point);
			edge.a = triangle.a;
			edge.b = triangle.b;
			polygon.p.push_back(edge.CenterPoint());
			polygon.p.push_back(triangle.CenterPoint());
			edge.a = triangle.a;
			edge.b = triangle.c;
			polygon.p.push_back(edge.CenterPoint());
			area += polygon.Area();
		}
		else if ((triangle.b.x == point.x) && (triangle.b.y == point.y) && (triangle.b.z == point.z)){

			// If triangle.b = point p we create the polugon and we calculate the area 
			// of the small polygon created by the vertex p and the mid point of the two
			// edges and the center od the triangle.
			polygon.p.clear();
			polygon.p.push_back(point);
			edge.a = triangle.b;
			edge.b = triangle.a;
			polygon.p.push_back(edge.CenterPoint());
			polygon.p.push_back(triangle.CenterPoint());
			edge.a = triangle.b;
			edge.b = triangle.c;
			polygon.p.push_back(edge.CenterPoint());
			area += polygon.Area();
		}
		else if ((triangle.c.x == point.x) && (triangle.c.y == point.y) && (triangle.c.z == point.z)){

			// If triangle.c = point p we create the polugon and we calculate the area 
			// of the small polygon created by the vertex p and the mid point of the two
			// edges and the center od the triangle.
			polygon.p.clear();
			polygon.p.push_back(point);
			edge.a = triangle.c;
			edge.b = triangle.a;
			polygon.p.push_back(edge.CenterPoint());
			polygon.p.push_back(triangle.CenterPoint());
			edge.a = triangle.c;
			edge.b = triangle.b;
			polygon.p.push_back(edge.CenterPoint());
			area += polygon.Area();
		}

	}

	// Returning the total area of voronoi region for the star at vertex p
	return area;
}

double Simple3DScene::gaussianCurvatureThreshold(Mesh &mesh)
{
	// Here we calculate the threshold to cut the spikes of the curvature
	// To do this we run the algorithm once to find the curvature on each 
	// vertex and then we calculate the mean value and the max value.
	// Then the user is asked to determine the threshold. 
	// Usually it is the mean value multiplied by 2
	vector<Vec3d> &vertices = mesh.getVertices();
	vector<vvr::Triangle> tri = mesh.getTriangles();
	Vec3d vertex;
	Vec3d ab, ac;
	vector<pair<double, Vec3d>> curv_at_vertex;
	string threshold;
	double max = -1000.0;
	double abac;
	double met_ab;
	double met_ac;
	double cos_theta;
	double theta;
	double area;
	double gauss;
	double sum_curv = 0.0;
	double meanvalue;
	double thresh;
	bool flag = false;

	for (int vi = 0; vi < vertices.size(); vi++)
	{
		vertex = vertices[vi];
		double sum_area = 0.0;
		double sum_degrees = 0.0;

		// Calculate sum of area and sum of angles of all the triangles with the same vertex

		for (int i = 0; i < tri.size(); i++)
		{
			vvr::Triangle trian = tri.at(i);

			// If statements for every triangle of our model to find those with the same vertex
			// Inside every if statement calculations for angles and area of every triangle 

			if ((trian.v1().x == vertex.x)&(trian.v1().y == vertex.y)&(trian.v1().z == vertex.z))
			{
				// When we find the triangle with the same vertex we calculate |AB|,|AC| + angle theta between AB,AC

				ab.x = trian.v3().x - trian.v1().x;
				ab.y = trian.v3().y - trian.v1().y;
				ab.z = trian.v3().z - trian.v1().z;
				ac.x = trian.v2().x - trian.v1().x;
				ac.y = trian.v2().y - trian.v1().y;
				ac.z = trian.v2().z - trian.v1().z;
				abac = (ab.x * ac.x) + (ab.y * ac.y) + (ab.z * ac.z);

				met_ab = sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2));
				met_ac = sqrt(pow(ac.x, 2) + pow(ac.y, 2) + pow(ac.z, 2));

				// Calculating angle_theta using dot product between AB,AC and |AB|,|AC|
				cos_theta = abac / (met_ab*met_ac);
				theta = acos(cos_theta);

				// Calculating area using |AB|*|AC|*sin(theta)
				area = (met_ab*met_ac* sin((theta*pi) / 180)) / 2.0;

				// Finding sum of area, sum of degrees of all triangles with same vertex  
				sum_area += area;
				sum_degrees += theta;
			}
			else if ((trian.v2().x == vertex.x)&(trian.v2().y == vertex.y)&(trian.v2().z == vertex.z))
			{
				// When we find the triangle with the same vertex we calculate |AB|,|AC| + angle theta between AB,AC

				ab.x = trian.v3().x - trian.v2().x;
				ab.y = trian.v3().y - trian.v2().y;
				ab.z = trian.v3().z - trian.v2().z;
				ac.x = trian.v1().x - trian.v2().x;
				ac.y = trian.v1().y - trian.v2().y;
				ac.z = trian.v1().z - trian.v2().z;
				abac = (ab.x * ac.x) + (ab.y * ac.y) + (ab.z * ac.z);

				met_ab = sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2));
				met_ac = sqrt(pow(ac.x, 2) + pow(ac.y, 2) + pow(ac.z, 2));

				// Calculating angle_theta using dot product between AB,AC and |AB|,|AC|
				cos_theta = abac / (met_ab*met_ac);
				theta = acos(cos_theta);

				// Calculating area using |AB|*|AC|*sin(theta)
				area = (met_ab*met_ac* sin((theta*pi) / 180)) / 2.0;

				// Finding sum of area, sum of degrees of all triangles with same vertex  
				sum_area += area;
				sum_degrees += theta;

			}
			else if ((trian.v3().x == vertex.x)&(trian.v3().y == vertex.y)&(trian.v3().z == vertex.z))
			{
				// When we find the triangle with the same vertex we calculate |AB|,|AC| + angle theta between AB,AC

				ab.x = trian.v1().x - trian.v3().x;
				ab.y = trian.v1().y - trian.v3().y;
				ab.z = trian.v1().z - trian.v3().z;
				ac.x = trian.v2().x - trian.v3().x;
				ac.y = trian.v2().y - trian.v3().y;
				ac.z = trian.v2().z - trian.v3().z;
				abac = (ab.x * ac.x) + (ab.y * ac.y) + (ab.z * ac.z);

				met_ab = sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2));
				met_ac = sqrt(pow(ac.x, 2) + pow(ac.y, 2) + pow(ac.z, 2));

				// Calculating angle_theta using dot product between AB,AC and |AB|,|AC|
				cos_theta = abac / (met_ab*met_ac);
				theta = acos(cos_theta);

				// Calculating area using |AB|*|AC|*sin(theta)
				area = (met_ab*met_ac* sin((theta*pi) / 180)) / 2.0;

				// Finding sum of area, sum of degrees of all triangles with same vertex  
				sum_area += area;
				sum_degrees += theta;
			}
		}

		// Calculating gaussian curvature  using K= (2*pi-sum_of_angles)/((1/3)*sum_of_area)

		gauss = abs((2.0 * pi - sum_degrees) / (sum_area / 3.0));
		sum_curv += gauss;

		if (gauss > max)
			max = gauss;
	}
	meanvalue = sum_curv / vertices.size();

	// Here user is asked about curvature's threshold
	cout << "\n		Gaussian Curvature\n";
	cout << "\n	...Max value of gaussian curvature = " << max << "\n";
	cout << "\n	...Mean value of gaussian curvature = " << meanvalue << "\n";
	cout << "\n	...Please type threshold: ";
	cin >> threshold;
	while (!flag)
	{
		try
		{
			thresh = stod(threshold);
			flag = true;
		}
		catch (exception e)
		{
			cout << "\n		WRONG INPUT!!! \n";
			cout << "\n	...Please type threshold(double value): ";
			cin >> threshold;
		}
	}
	return thresh;
}

void Simple3DScene::curvature3DGauss(Mesh &mesh)
{
	/////////////////////////////////////////////////////////////////////////
	//////       COMPUTATION OF GAUSSIAN CURVATURE USING             ////////
	//////               GAUSS BONNET THEORREM                       ////////
	/////////////////////////////////////////////////////////////////////////

	vector<Vec3d> &vertices = mesh.getVertices();
	vector<vvr::Triangle> tri = mesh.getTriangles();
	Vec3d vertex;
	Vec3d ab, ac;
	vector<pair<double, Vec3d>> curv_at_vertex;
	double max = -1000.0;
	double abac;
	double met_ab;
	double met_ac;
	double cos_theta;
	double theta;
	double area;
	double gauss;

	for (int vi = 0; vi < vertices.size(); vi++)
	{
		vertex = vertices[vi];
		double sum_area = 0.0;
		double sum_degrees = 0.0;

		// Calculate sum of area and sum of angles of all the triangles with the same vertex

		for (int i = 0; i < tri.size(); i++)
		{
			vvr::Triangle trian = tri.at(i);

			// If statements for every triangle of our model to find those with the same vertex
			// Inside every if statement calculations for angles and area of every triangle 

			if ((trian.v1().x == vertex.x)&(trian.v1().y == vertex.y)&(trian.v1().z == vertex.z))
			{
				// When we find the triangle with the same vertex we calculate |AB|,|AC| + angle theta between AB,AC

				ab.x = trian.v3().x - trian.v1().x;
				ab.y = trian.v3().y - trian.v1().y;
				ab.z = trian.v3().z - trian.v1().z;
				ac.x = trian.v2().x - trian.v1().x;
				ac.y = trian.v2().y - trian.v1().y;
				ac.z = trian.v2().z - trian.v1().z;
				abac = (ab.x * ac.x) + (ab.y * ac.y) + (ab.z * ac.z);

				met_ab = sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2));
				met_ac = sqrt(pow(ac.x, 2) + pow(ac.y, 2) + pow(ac.z, 2));

				// Calculating angle_theta using dot product between AB,AC and |AB|,|AC|
				cos_theta = abac / (met_ab*met_ac);
				theta = acos(cos_theta);

				// Calculating area using |AB|*|AC|*sin(theta)
				area = (met_ab*met_ac* sin((theta*pi) / 180)) / 2.0;

				// Finding sum of area, sum of degrees of all triangles with same vertex  
				sum_area += area;
				sum_degrees += theta;
			}
			else if ((trian.v2().x == vertex.x)&(trian.v2().y == vertex.y)&(trian.v2().z == vertex.z))
			{
				// When we find the triangle with the same vertex we calculate |AB|,|AC| + angle theta between AB,AC

				ab.x = trian.v3().x - trian.v2().x;
				ab.y = trian.v3().y - trian.v2().y;
				ab.z = trian.v3().z - trian.v2().z;
				ac.x = trian.v1().x - trian.v2().x;
				ac.y = trian.v1().y - trian.v2().y;
				ac.z = trian.v1().z - trian.v2().z;
				abac = (ab.x * ac.x) + (ab.y * ac.y) + (ab.z * ac.z);

				met_ab = sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2));
				met_ac = sqrt(pow(ac.x, 2) + pow(ac.y, 2) + pow(ac.z, 2));

				// Calculating angle_theta using dot product between AB,AC and |AB|,|AC|
				cos_theta = abac / (met_ab*met_ac);
				theta = acos(cos_theta);

				// Calculating area using |AB|*|AC|*sin(theta)
				area = (met_ab*met_ac* sin((theta*pi) / 180)) / 2.0;

				// Finding sum of area, sum of degrees of all triangles with same vertex  
				sum_area += area;
				sum_degrees += theta;

			}
			else if ((trian.v3().x == vertex.x)&(trian.v3().y == vertex.y)&(trian.v3().z == vertex.z))
			{
				// When we find the triangle with the same vertex we calculate |AB|,|AC| + angle theta between AB,AC

				ab.x = trian.v1().x - trian.v3().x;
				ab.y = trian.v1().y - trian.v3().y;
				ab.z = trian.v1().z - trian.v3().z;
				ac.x = trian.v2().x - trian.v3().x;
				ac.y = trian.v2().y - trian.v3().y;
				ac.z = trian.v2().z - trian.v3().z;
				abac = (ab.x * ac.x) + (ab.y * ac.y) + (ab.z * ac.z);

				met_ab = sqrt(pow(ab.x, 2) + pow(ab.y, 2) + pow(ab.z, 2));
				met_ac = sqrt(pow(ac.x, 2) + pow(ac.y, 2) + pow(ac.z, 2));

				// Calculating angle_theta using dot product between AB,AC and |AB|,|AC|
				cos_theta = abac / (met_ab*met_ac);
				theta = acos(cos_theta);

				// Calculating area using |AB|*|AC|*sin(theta)
				area = (met_ab*met_ac* sin((theta*pi) / 180)) / 2.0;

				// Finding sum of area, sum of degrees of all triangles with same vertex  
				sum_area += area;
				sum_degrees += theta;
			}
		}

		// Calculating gaussian curvature  using K= (2*pi-sum_of_angles)/((1/3)*sum_of_area)

		gauss = abs((2.0 * pi - sum_degrees) / (sum_area / 3.0));
		// Threshold to cut spikes of curvature
		if (gauss > Gauss_Threshold)
			gauss = Gauss_Threshold;
		if (gauss > max)
			max = gauss;

		curv_at_vertex.push_back(make_pair(gauss, vertices[vi]));
	}

	for (int j = 0; j < tri.size(); j++)
	{
		double sum = 0;
		vvr::Triangle triangle = tri.at(j);
		Vec3d tri_v1, tri_v2, tri_v3, ver;

		tri_v1.x = triangle.v1().x;
		tri_v1.y = triangle.v1().y;
		tri_v1.z = triangle.v1().z;

		tri_v2.x = triangle.v2().x;
		tri_v2.y = triangle.v2().y;
		tri_v2.z = triangle.v2().z;

		tri_v3.x = triangle.v3().x;
		tri_v3.y = triangle.v3().y;
		tri_v3.z = triangle.v3().z;

		for (int k = 0; k < curv_at_vertex.size(); k++)
		{
			ver.x = curv_at_vertex.at(k).second.x;
			ver.y = curv_at_vertex.at(k).second.y;
			ver.z = curv_at_vertex.at(k).second.z;

			if ((tri_v1 == ver) || (tri_v2 == ver) || (tri_v3 == ver)) {
				sum += curv_at_vertex.at(k).first;

			}
		}

		double percent = ((sum / 3.0) / max);

		Colour cper((GetRGBValue(percent, 0.0, 1.0).red)*255.0, (GetRGBValue(percent, 0.0, 1.0).green)*255.0, (GetRGBValue(percent, 0.0, 1.0).blue)*255.0);
		Triangle3D(triangle.v1().x, triangle.v1().y, triangle.v1().z,
			triangle.v2().x, triangle.v2().y, triangle.v2().z,
			triangle.v3().x, triangle.v3().y, triangle.v3().z, cper).draw();
	}

	findLocalMaxOfCurvature(curv_at_vertex, vertices_with_maxGaussCurv);
	curv_at_vertex.clear();

}

void Simple3DScene::convexHull3D(Mesh &mesh)
{
	// Using clock to find the time our algorithm needs to construct convex hull of model
	Clock start_of_algorithm, end_of_algorithm;
	math::tick_t tickstart = start_of_algorithm.Tick();

	vector<Vec3d> &vertices = m_model.getVertices();
	vector<Vec3d> points = vertices;
	LineSegment p1_p2;
	Polygon poly;
	Polyhedron convex_hull;
	Polyhedron::Face facet1, facet2, facet3, facet4, temp_face;
	math::vec p1, p2, p3, p4, p;
	bool flag;
	int index_of_3rd_point, index_of_4th_point;
	vector<int> visiblefacetsindices, nonvisiblefacetsindices;
	vector<pair<int, int>> vertices_pairs, remaining_vertices_pair;
	vector<Polyhedron::Face> final_faces;

	////////////////////////////////////////////////////////////////////////////////
	/////        INITIALIZATION OF CONVEX HULL 3D INCREMENTAL           ////////////
	////////////////////////////////////////////////////////////////////////////////

	// First and Second point of starting tetrahedron
	// Conversion from Vec3d to math::vec in order to use mathgeolib
	p1.x = vertices[0].x;
	p1.y = vertices[0].y;
	p1.z = vertices[0].z;
	p2.x = vertices[1].x;
	p2.y = vertices[1].y;
	p2.z = vertices[1].z;

	// Create a line with two first points in order to find a 3rd one not contained inside that line
	p1_p2.a.x = vertices[0].x;
	p1_p2.a.y = vertices[0].y;
	p1_p2.a.z = vertices[0].z;
	p1_p2.b.x = vertices[1].x;
	p1_p2.b.y = vertices[1].y;
	p1_p2.b.z = vertices[1].z;
	Line initial_line = p1_p2.ToLine();


	for (int ctr2 = 2; ctr2 < vertices.size(); ctr2++)
	{
		// Finding a point not contained inside initial line in order to create starting polygon 
		// of convexHull3D

		p.x = vertices[ctr2].x;
		p.y = vertices[ctr2].y;
		p.z = vertices[ctr2].z;
		if (!initial_line.Contains(p, 0.0001)){
			p3.Set(p.x, p.y, p.z);
			index_of_3rd_point = ctr2;
			break;
		}
	}

	// Creating a polygon with the first three points

	poly.p.push_back(p1);
	poly.p.push_back(p2);
	poly.p.push_back(p3);

	// Converting polygon to plane in order to find the first point not contained inside polygon's plane
	// and create starting tetrahedron of convex hull

	Plane planep1p2p3 = poly.PlaneCCW();

	for (int ctr3 = 2; ctr3 < vertices.size(); ctr3++)
	{
		// Checking all points to find first point away of plane

		p.x = vertices[ctr3].x;
		p.y = vertices[ctr3].y;
		p.z = vertices[ctr3].z;
		if (ctr3 != index_of_3rd_point)
		{
			if (!(planep1p2p3.Contains(p, 0.0001)))
			{
				p4.Set(p.x, p.y, p.z);
				index_of_4th_point = ctr3;
				break;
			}
		}
	}

	// Delete the starting 4 points of initial convex hull

	points.erase(points.begin(), points.begin() + 2);
	points.erase(points.begin() + (index_of_3rd_point - 2));
	points.erase(points.begin() + (index_of_4th_point - 3));

	// Push back 4 first points of initial convex hull as vertices and as faces

	convex_hull.v.push_back(p1);
	convex_hull.v.push_back(p2);
	convex_hull.v.push_back(p3);
	convex_hull.v.push_back(p4);

	facet1.v.push_back(0);
	facet1.v.push_back(1);
	facet1.v.push_back(2);

	facet2.v.push_back(0);
	facet2.v.push_back(1);
	facet2.v.push_back(3);

	facet3.v.push_back(0);
	facet3.v.push_back(2);
	facet3.v.push_back(3);

	facet4.v.push_back(1);
	facet4.v.push_back(2);
	facet4.v.push_back(3);

	convex_hull.f.push_back(facet1);
	convex_hull.f.push_back(facet2);
	convex_hull.f.push_back(facet3);
	convex_hull.f.push_back(facet4);

	// Using OrientNormalsOutsideConvex() in order every face of convex hull to have normal oriented outside
	// so we can use IsOnPositiveSide() to find horizon with ease

	//****************************************************************************************************//
	convex_hull.OrientNormalsOutsideConvex();
	//****************************************************************************************************//

	////////////////////////////////////////////////////////////////////////////////
	/////                    INITIAL CONVEX HULL IS READY                      /////
	////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////
	/////              STARTING COMPUTATION OF 3D CONVEX HULL                  /////
	////////////////////////////////////////////////////////////////////////////////


	// Checking all other points of model

	for (int i = 0; i < points.size(); i++)
	{
		p.x = points.at(i).x;
		p.y = points.at(i).y;
		p.z = points.at(i).z;
		if (convex_hull.Contains(p))
		{
			// If the point at(i) is inside convex hull we continue 
			continue;
		}
		else
		{

			// If point at(i) is outside convex hull we check to see which face our point "sees"

			for (int ctr4 = 0; ctr4 < convex_hull.NumFaces(); ctr4++)
			{
				// Checking if a point is on positive side of each face
				// If it is then the face is visible from our point and it belongs in the horizon
				// If not the face is not visible
				Plane plane = convex_hull.FacePlane(ctr4);
				if (plane.IsOnPositiveSide(p))
				{
					visiblefacetsindices.push_back(ctr4);
				}
				else
				{
					nonvisiblefacetsindices.push_back(ctr4);
				}
			}

			// For every visible face we create a vector with vertices in pairs

			for (int ctr5 = 0; ctr5 < visiblefacetsindices.size(); ctr5++)
			{
				vertices_pairs.push_back(make_pair(convex_hull.f[visiblefacetsindices[ctr5]].v[0], convex_hull.f[visiblefacetsindices[ctr5]].v[1]));
				vertices_pairs.push_back(make_pair(convex_hull.f[visiblefacetsindices[ctr5]].v[1], convex_hull.f[visiblefacetsindices[ctr5]].v[2]));
				vertices_pairs.push_back(make_pair(convex_hull.f[visiblefacetsindices[ctr5]].v[2], convex_hull.f[visiblefacetsindices[ctr5]].v[0]));
			}

			flag = false;

			// Computing the horizon of our point by checking how many times an edge 
			// belongs to the visible faces. If it belongs 1 time then the edge
			// belongs to the horizon

			for (int ctr6 = 0; ctr6 < vertices_pairs.size(); ctr6++)
			{
				for (int ctr8 = 0; ctr8 < vertices_pairs.size(); ctr8++)
				{
					if (ctr8 != ctr6)
					{
						if ((vertices_pairs[ctr6].first == vertices_pairs[ctr8].first && vertices_pairs[ctr6].second == vertices_pairs[ctr8].second) ||
							(vertices_pairs[ctr6].first == vertices_pairs[ctr8].second && vertices_pairs[ctr6].second == vertices_pairs[ctr8].first))
						{
							flag = true;
						}

					}
				}
				if (flag == false)
				{
					remaining_vertices_pair.push_back(vertices_pairs[ctr6]);
				}
				flag = false;
			}

			// Push back in our convex hull all non visible faces
			for (int ctr7 = 0; ctr7 < nonvisiblefacetsindices.size(); ctr7++)
			{
				final_faces.push_back(convex_hull.f[nonvisiblefacetsindices[ctr7]]);
			}

			// Pushing back our point in the points of convex hull
			convex_hull.v.push_back(p);
			convex_hull.f.clear();

			convex_hull.f = final_faces;

			// Create a face with our point and the horizon edges and push it back  in convex hull faces

			for (int l = 0; l<remaining_vertices_pair.size(); l++)
			{
				temp_face.v.push_back(convex_hull.v.size() - 1);
				temp_face.v.push_back(remaining_vertices_pair[l].first);
				temp_face.v.push_back(remaining_vertices_pair[l].second);
				convex_hull.f.push_back(temp_face);
				temp_face.v.clear();
			}

			// Using OrientNormalsOutsideConvex() in order every face of our new convex hull 
			// to have normal oriented outside  so we can use IsOnPositiveSide() 
			// to find horizon with ease
			convex_hull.OrientNormalsOutsideConvex();

			// Clear our vectors in order for the procedure to work for our next point
			visiblefacetsindices.clear();
			nonvisiblefacetsindices.clear();
			vertices_pairs.clear();
			remaining_vertices_pair.clear();
			final_faces.clear();
		}
	}

	// Making our convex hull from polyhedron to mesh
	vector<Vec3d> &verts = mesh.getVertices();
	vector<vvr::Triangle> &tris = mesh.getTriangles();
	for (int c = 0; c < convex_hull.v.size(); c++)
	{
		Vec3d vertex;
		vertex.x = convex_hull.v.at(c).x;
		vertex.y = convex_hull.v.at(c).y;
		vertex.z = convex_hull.v.at(c).z;

		// Pushing back vertices of our convex hull inside mesh

		verts.push_back(vertex);
	}
	// Creating faces of our mesh
	for (int j = 0; j < convex_hull.f.size(); j++)
	{
		tris.push_back(vvr::Triangle(&verts, convex_hull.f.at(j).v[0], convex_hull.f.at(j).v[1], convex_hull.f.at(j).v[2]));
	}

	math::tick_t tickend = end_of_algorithm.Tick();

	// Calculating duration of convex hull construction
	cout << "	\n	... Convex hull duration: " << end_of_algorithm.TicksToSecondsD(end_of_algorithm.TicksInBetween(tickend, tickstart)) << " seconds\n";
	cout << "	\n	_ Press (5) to see gaussian curvature of 3D model's Convex Hull" << "\n";
	cout << "	\n	_ Press (6) to see mean curvature of 3D model's Convex Hull" << "\n";
	cout << "	\n	_ Press (7) to re-triangulate Convex Hull" << "\n";
}

void Simple3DScene::triangulationOfConvexHull(Mesh &mesh)
{
	vector<Vec3d> &vertices = mesh.getVertices();
	vector<vvr::Triangle> &tris = mesh.getTriangles();
	LineSegment edge_1, edge_2, edge_3;
	Vec3d mid1, mid2, mid3;
	int vector_of_tris_size = tris.size();
	double max_area = 0;

	// Retriangulation of convex hull by breaking each triangle into four smallers
	// To do that we get the midpoint of every edge of the triangle
	// And put one new triangle with those vertices
	// I do this till every triangle of the model have area smaller than the max area
	// of one triangle of the model
	for (int i = 0; i < m_model.getTriangles().size(); i++){
		math::Triangle tri;
		vvr::Triangle triangle = m_model.getTriangles().at(i);
		tri.a.x = triangle.v1().x;
		tri.a.y = triangle.v1().y;
		tri.a.z = triangle.v1().z;

		tri.b.x = triangle.v2().x;
		tri.b.y = triangle.v2().y;
		tri.b.z = triangle.v2().z;

		tri.c.x = triangle.v3().x;
		tri.c.y = triangle.v3().y;
		tri.c.z = triangle.v3().z;

		if (tri.Area() > max_area){
			max_area = tri.Area();
		}
	}


	for (int ctr1 = 0; ctr1 < vector_of_tris_size; ctr1++){
		vvr::Triangle tri = tris[ctr1];
		math::Triangle trian;

		trian.a.x = tri.v1().x;
		trian.a.y = tri.v1().y;
		trian.a.z = tri.v1().z;

		trian.b.x = tri.v2().x;
		trian.b.y = tri.v2().y;
		trian.b.z = tri.v2().z;

		trian.c.x = tri.v3().x;
		trian.c.y = tri.v3().y;
		trian.c.z = tri.v3().z;

		if (trian.Area() < max_area){
			continue;
		}
		else{
			edge_1.a.x = tri.v1().x;
			edge_1.a.y = tri.v1().y;
			edge_1.a.z = tri.v1().z;

			edge_1.b.x = tri.v2().x;
			edge_1.b.y = tri.v2().y;
			edge_1.b.z = tri.v2().z;

			edge_2.a.x = tri.v1().x;
			edge_2.a.y = tri.v1().y;
			edge_2.a.z = tri.v1().z;

			edge_2.b.x = tri.v3().x;
			edge_2.b.y = tri.v3().y;
			edge_2.b.z = tri.v3().z;

			edge_3.a.x = tri.v2().x;
			edge_3.a.y = tri.v2().y;
			edge_3.a.z = tri.v2().z;

			edge_3.b.x = tri.v3().x;
			edge_3.b.y = tri.v3().y;
			edge_3.b.z = tri.v3().z;

			mid1.x = edge_1.CenterPoint().x;
			mid1.y = edge_1.CenterPoint().y;
			mid1.z = edge_1.CenterPoint().z;

			mid2.x = edge_2.CenterPoint().x;
			mid2.y = edge_2.CenterPoint().y;
			mid2.z = edge_2.CenterPoint().z;

			mid3.x = edge_3.CenterPoint().x;
			mid3.y = edge_3.CenterPoint().y;
			mid3.z = edge_3.CenterPoint().z;

			int v1 = tri.vi1;
			int v2 = tri.vi2;
			int v3 = tri.vi3;

			// Keeping indices to push back our new triangles

			vertices.push_back(mid1);
			int pointer_of_mid1 = vertices.size() - 1;
			vertices.push_back(mid2);
			int pointer_of_mid2 = vertices.size() - 1;
			vertices.push_back(mid3);
			int pointer_of_mid3 = vertices.size() - 1;


			tris[ctr1].vi1 = (pointer_of_mid1);
			tris[ctr1].vi2 = (pointer_of_mid2);
			tris[ctr1].vi3 = (pointer_of_mid3);

			// Pushing back new triangles

			tris.push_back(vvr::Triangle(&vertices, v1, pointer_of_mid1, pointer_of_mid2));
			tris.push_back(vvr::Triangle(&vertices, pointer_of_mid2, pointer_of_mid3, v3));
			tris.push_back(vvr::Triangle(&vertices, pointer_of_mid1, v2, pointer_of_mid3));
		}
	}
}

void Simple3DScene::findLocalMaxOfCurvature(vector<pair<double, Vec3d>> curvature_values, vector<pair<double, Vec3d>> &vertices_with_max_curv)
{
	vertices_with_max_curv.clear();
	double previous_curv = curvature_values.at(0).first;
	bool going_up;

	// My algorithm here is checking all values to find local maxima
	// I check every new value of curvature with the previous one
	// If the previous value is greater than the next one 
	// Then we have a local maxima and I save that maxima (both value and vertex coordinates)
	// Inside a vector to use it for other processes

	if (curvature_values.at(1).first >= curvature_values.at(0).first) going_up = true;
	else going_up = false;

	for (int i = 0; i < curvature_values.size(); i++){

		if (going_up){
			if ((previous_curv > curvature_values.at(i).first) && (i > 0)){
				going_up = false;
				vertices_with_max_curv.push_back(curvature_values.at(i - 1));
			}
		}
		else
		{
			if (previous_curv < curvature_values.at(i).first){
				going_up = true;
			}
		}
		previous_curv = curvature_values.at(i).first;
	}
}

void Simple3DScene::smoothingGauss(Mesh &mesh, double factor)
{

	vector<Vec3d> &vertices = mesh.getVertices();
	vector<vvr::Triangle> &triangles = mesh.getTriangles();
	double sum;
	int sum_of_vertices;

	/////////////////////////////////////////////////////////////////
	///////              SMOOTHING CURVATURE                 ////////
	/////////////////////////////////////////////////////////////////

	// I decrease the value of curvature on each local max
	// Using 1-ring and a factor that is given by the user
	// The factor must have values between 0(ultra smoothing) and 200(no smoothing)
	// When I get the factor i check all local maxima to see if they have curvature 
	// greater than that. If thats true I scale the vertex of the model based on the ring
	// Vertices distance from (0,0,0)


	// Checking all vertices from my model
	for (int i = 0; i < vertices.size(); i++)
	{
		// Checking all vertices on local maxima
		for (int j = 0; j < vertices_with_maxGaussCurv.size(); j++)
		{
			// If Ι found the same vertex on my model and it has curvature greater than
			// the factor given I scale the vertex 
			if ((vertices[i] == vertices_with_maxGaussCurv[j].second) && (vertices_with_maxGaussCurv[j].first > factor)){
				sum = 0.0;
				sum_of_vertices = 0;
				for (int k = 0; k < triangles.size(); k++){
					Vec3d v1 = triangles.at(k).v1();
					Vec3d v2 = triangles.at(k).v2();
					Vec3d v3 = triangles.at(k).v3();

					if ((v1 == vertices[i])){
						sum_of_vertices = sum_of_vertices + 2;
						sum = sum + sqrt(pow(triangles.at(k).v2().x, 2) + pow(triangles.at(k).v2().y, 2) + pow(triangles.at(k).v2().z, 2));
						sum = sum + sqrt(pow(triangles.at(k).v3().x, 2) + pow(triangles.at(k).v3().y, 2) + pow(triangles.at(k).v3().z, 2));
					}
					else if ((v2 == vertices[i])){
						sum_of_vertices = sum_of_vertices + 2;
						sum = sum + sqrt(pow(triangles.at(k).v1().x, 2) + pow(triangles.at(k).v1().y, 2) + pow(triangles.at(k).v1().z, 2));
						sum = sum + sqrt(pow(triangles.at(k).v3().x, 2) + pow(triangles.at(k).v3().y, 2) + pow(triangles.at(k).v3().z, 2));
					}
					else if ((v3 == vertices[i])){
						sum_of_vertices = sum_of_vertices + 2;
						sum = sum + sqrt(pow(triangles.at(k).v2().x, 2) + pow(triangles.at(k).v2().y, 2) + pow(triangles.at(k).v2().z, 2));
						sum = sum + sqrt(pow(triangles.at(k).v1().x, 2) + pow(triangles.at(k).v1().y, 2) + pow(triangles.at(k).v1().z, 2));
					}
				}
				double middle_r = sqrt(pow(vertices[i].x, 2) + pow(vertices[i].y, 2) + pow(vertices[i].z, 2));
				double mid = middle_r - (sum / sum_of_vertices);
				vertices[i].normalize();
				vertices[i].scale(middle_r + ((sum / (double)sum_of_vertices - middle_r) / 2.0));

			}
		}
	}
	vertices_with_maxGaussCurv.clear();
	curvature3DGauss(mesh);

	// For the scaling of the vertex I compute average sum of the distance
	// of each vertex on the 1-ring and the distance on the center of the ring
	// Then Ι scale the center with a scale factor = |p| +(((sum|xi|/sum of vertices)-|p|)/2)
}

void Simple3DScene::smoothingMean(Mesh &mesh, double factor)
{

	vector<Vec3d> &vertices = mesh.getVertices();
	vector<vvr::Triangle> &triangles = mesh.getTriangles();
	double sum;
	int sum_of_vertices;

	/////////////////////////////////////////////////////////////////
	///////              SMOOTHING CURVATURE                 ////////
	/////////////////////////////////////////////////////////////////

	// I decrease the value of curvature on each local max
	// Using 1-ring and a factor that is given by the user
	// The factor must have values between 0(ultra smoothing) and 200(no smoothing)
	// When I get the factor i check all local maxima to see if they have curvature 
	// greater than that. If thats true I scale the vertex of the model based on the ring
	// Vertices distance from (0,0,0)


	// Checking all vertices from my model
	for (int i = 0; i < vertices.size(); i++)
	{
		// Checking all vertices on local maxima
		for (int j = 0; j < vertices_with_maxMeanCurv.size(); j++)
		{
			// If i found the same vertex on my model and it has curvature greater than
			// the factor given I scale the vertex 
			if ((vertices[i] == vertices_with_maxMeanCurv[j].second) && (vertices_with_maxMeanCurv[j].first > factor))
			{
				sum = 0.0;
				sum_of_vertices = 0;
				for (int k = 0; k < triangles.size(); k++){
					Vec3d v1 = triangles.at(k).v1();
					Vec3d v2 = triangles.at(k).v2();
					Vec3d v3 = triangles.at(k).v3();

					if ((v1 == vertices[i])){
						sum_of_vertices = sum_of_vertices + 2;
						sum = sum + sqrt(pow(triangles.at(k).v2().x, 2) + pow(triangles.at(k).v2().y, 2) + pow(triangles.at(k).v2().z, 2));
						sum = sum + sqrt(pow(triangles.at(k).v3().x, 2) + pow(triangles.at(k).v3().y, 2) + pow(triangles.at(k).v3().z, 2));
					}
					else if ((v2 == vertices[i])){
						sum_of_vertices = sum_of_vertices + 2;
						sum = sum + sqrt(pow(triangles.at(k).v1().x, 2) + pow(triangles.at(k).v1().y, 2) + pow(triangles.at(k).v1().z, 2));
						sum = sum + sqrt(pow(triangles.at(k).v3().x, 2) + pow(triangles.at(k).v3().y, 2) + pow(triangles.at(k).v3().z, 2));
					}
					else if ((v3 == vertices[i])){
						sum_of_vertices = sum_of_vertices + 2;
						sum = sum + sqrt(pow(triangles.at(k).v2().x, 2) + pow(triangles.at(k).v2().y, 2) + pow(triangles.at(k).v2().z, 2));
						sum = sum + sqrt(pow(triangles.at(k).v1().x, 2) + pow(triangles.at(k).v1().y, 2) + pow(triangles.at(k).v1().z, 2));
					}
				}
				double middle_r = sqrt(pow(vertices[i].x, 2) + pow(vertices[i].y, 2) + pow(vertices[i].z, 2));
				double mid = middle_r - (sum / sum_of_vertices);
				vertices[i].normalize();
				vertices[i].scale(middle_r + ((sum / (double)sum_of_vertices - middle_r) / 2.0));

			}
		}
	}
	vertices_with_maxMeanCurv.clear();
	curvature3DMean(mesh);

	// For the scaling of the vertex I compute average sum of the distance
	// of each vertex on the 1-ring and the distance on the center of the ring
	// Then i scale the center with a scale factor = |p| +(((sum|xi|/sum of vertices)-|p|)/2)
}

void Simple3DScene::resetProgram()
{
	reset();
	CH_draw_flag = false;
	Triang_Convex_Hull = false;
	Smoothing = false;
	Button_6 = false;
	Button_m = false;
	Smoothing_Factor_Gauss_Read = false;
	Smoothing_Factor_Gauss_Flag = false;
	Smoothing_Factor_Mean_Read = false;
	Smoothing_Factor_Mean_Flag = false;
	Gauss_Threshold_Read = false;
	Mean_Threshold_Read = false;
	vertices_with_maxGaussCurv.clear();
	vertices_with_maxMeanCurv.clear();
	curvature_values.clear();
	system("cls");
	consoleMenu();

	m_style_flag = FLAG_SHOW_CONVEX;
	m_style_flag = FLAG_SHOW_CONVEX_CURV3D_GAUSS;
	m_style_flag = FLAG_SHOW_CONVEX_CURV3D_MEAN;
	m_style_flag = FLAG_SHOW_CURV2D;
	m_style_flag = FLAG_SHOW_CURV3D;
	m_style_flag = FLAG_SHOW_CURV3D_MEAN;
	m_style_flag = FLAG_SHOW_LOCAL_MAX_GAUSS;
	m_style_flag = FLAG_SHOW_LOCAL_MAX_MEAN;
	m_style_flag = FLAG_SMOOTH_GAUSS;
	m_style_flag = FLAG_SMOOTH_MEAN;
	m_style_flag = FLAG_TRIAN_AGAIN;
	m_style_flag = FLAG_SHOW_SOLID;


}

void Simple3DScene::consoleMenu()
{
	cout << "\n\n";
	cout << "     Computational Geometry Project" << "\n" << "Curvature, Convex Hulls and Level of Detail" << "\n\n";
	cout << "_ User Interface and buttons:" << "\n";
	cout << "_ Press (1) to see curvature on 2D polygon" << "\n";
	cout << "_ Press (2) to see gaussian curvature on 3D model" << "\n";
	cout << "_ Press (3) to see mean curvature on 3D model" << "\n";
	cout << "_ Press (4) to see convex hull of the 3D model" << "\n";
	cout << "_ Press (9) to smooth gaussian curvature of the model" << "\n";
	cout << "_ Press (0) to smooth mean curvature of the model" << "\n";
	cout << "_ Press (g) to show local max of gaussian curvature of the model" << "\n";
	cout << "_ Press (m) to show local max of mean curvature of the model" << "\n";
	cout << "_ Press (R) to reset program" << "\n";
}

void Simple3DScene::draw()
{
	if (m_style_flag & FLAG_SHOW_SOLID)     m_model.draw(m_obj_col, SOLID);
	if (m_style_flag & FLAG_SHOW_WIRE)      m_model.draw(Colour::black, WIRE);
	if (m_style_flag & FLAG_SHOW_NORMALS)   m_model.draw(Colour::black, NORMALS);
	if (m_style_flag & FLAG_SHOW_AXES)      m_model.draw(Colour::black, AXES);
	if (m_style_flag & FLAG_SHOW_AABB)      m_model.draw(Colour::black, BOUND);

	drawPolygon();

	if (m_style_flag & FLAG_SHOW_CURV2D)
	{
		curvature2D();
	}


	if (m_style_flag & FLAG_SHOW_CURV3D)
	{
		if (Gauss_Threshold_Read == false)
		{
			Gauss_Threshold = gaussianCurvatureThreshold(m_model);
			Gauss_Threshold_Read = true;
		}
		curvature3DGauss(m_model);
	}
	if (m_style_flag & FLAG_SHOW_CURV3D_MEAN)
	{
		if (Mean_Threshold_Read == false)
		{
			Mean_Threshold = meanCurvatureThreshold(m_model);
			Mean_Threshold_Read = true;
		}
		curvature3DMean(m_model);
	}

	if (m_style_flag & FLAG_SHOW_CONVEX)
	{
		if (CH_draw_flag == false)
		{
			cout << "\n	... Computing Convex Hull from our model\n";
			convexHull3D(m_model_convex_hull);
			CH_draw_flag = true;
		}
		m_model_convex_hull.draw(Colour::grey, SOLID);
		m_model_convex_hull.draw(Colour::black, WIRE);
		if (m_style_flag & FLAG_SHOW_CONVEX_CURV3D_GAUSS)
		{
			curvature3DGauss(m_model_convex_hull);
		}
		if (m_style_flag & FLAG_SHOW_CONVEX_CURV3D_MEAN)
		{

			curvature3DMean(m_model_convex_hull);
		}
		if (m_style_flag & FLAG_TRIAN_AGAIN)
		{
			if (Triang_Convex_Hull == false)
			{
				triangulationOfConvexHull(m_model_convex_hull);
				Triang_Convex_Hull = true;
			}
			m_model_convex_hull.draw(Colour::black, Style::WIRE);
		}
	}
	if (m_style_flag & FLAG_SMOOTH_GAUSS)
	{
		if (Smoothing_Factor_Gauss_Read == false)
		{
			cout << "\n_ Please type the factor of smoothing(Gaussian Curvature):";
			cin >> Gauss_Factor;
			while (!Smoothing_Factor_Gauss_Flag)
			{
				try
				{
					Gauss_Smoothing = stod(Gauss_Factor);
					Smoothing_Factor_Gauss_Flag = true;
					Smoothing_Factor_Gauss_Read = true;
				}
				catch (exception e)
				{
					cout << "\n		WRONG INPUT!!! \n";
					cout << "\n	...Please type smoothing factor again(double value): ";
					cin >> Gauss_Factor;
				}
			}
		}
		smoothingGauss(m_model, Gauss_Smoothing);
	}

	if (m_style_flag & FLAG_SMOOTH_MEAN)
	{
		if (Smoothing_Factor_Mean_Read == false)
		{
			cout << "\n_ Please type the factor of smoothing(Mean Curvature):";
			cin >> Mean_Factor;
			while (!Smoothing_Factor_Mean_Flag)
			{
				try
				{
					Mean_Smoothing = stod(Mean_Factor);
					Smoothing_Factor_Mean_Flag = true;
					Smoothing_Factor_Mean_Read = true;
				}
				catch (exception e)
				{
					cout << "\n		WRONG INPUT!!! \n";
					cout << "\n	...Please type smoothing factor again(double value): ";
					cin >> Mean_Factor;
				}
			}
		}
		smoothingMean(m_model, Mean_Smoothing);
	}

	if (m_style_flag & FLAG_SHOW_LOCAL_MAX_GAUSS)
	{
		for (int i = 0; i < vertices_with_maxGaussCurv.size(); i++)
		{
			Point3D(vertices_with_maxGaussCurv.at(i).second.x, vertices_with_maxGaussCurv.at(i).second.y, vertices_with_maxGaussCurv.at(i).second.z, Colour::red).draw();
		}
	}

	if (m_style_flag & FLAG_SHOW_LOCAL_MAX_MEAN)
	{

		for (int i = 0; i < vertices_with_maxMeanCurv.size(); i++)
		{
			Point3D(vertices_with_maxMeanCurv.at(i).second.x, vertices_with_maxMeanCurv.at(i).second.y, vertices_with_maxMeanCurv.at(i).second.z, Colour::orange).draw();
		}
	}
}

void Simple3DScene::drawPolygon()
{
	enterPixelMode();

	for (int pi = 0; pi < m_pts.size(); pi++)
	{
		const C2DPoint &p1 = m_pts[pi];
		const C2DPoint &p2 = m_pts[(pi + 1) % m_pts.size()];
		Colour line_col = Colour::yellow;
		LineSeg2D(p1.x, p1.y, p2.x, p2.y, line_col).draw();
		if (b_show_pts) Point2D(p1.x, p1.y, Colour::yellow).draw();
	}

	returnFromPixelMode();

}

void Simple3DScene::keyEvent(unsigned char key, bool up, int modif)
{
	Scene::keyEvent(key, up, modif);
	key = tolower(key);

	switch (key)
	{
	case 'a': m_style_flag ^= FLAG_SHOW_AXES; break;
	case 'w': m_style_flag ^= FLAG_SHOW_WIRE; break;
	case 's': m_style_flag ^= FLAG_SHOW_SOLID; break;
	case 'n': m_style_flag ^= FLAG_SHOW_NORMALS; break;
	case 'b': m_style_flag ^= FLAG_SHOW_AABB; break;
	case 'p': b_show_pts ^= true; break;
	case 'f': savePolygonToFile(); break;
	case 'r': m_pts.clear(); resetProgram(); break;

	case '1':
		m_style_flag ^= FLAG_SHOW_CURV2D;
		break;

	case '2':
		m_style_flag ^= FLAG_SHOW_CURV3D;
		break;

	case '3':
		m_style_flag ^= FLAG_SHOW_CURV3D_MEAN;
		break;

	case '4':
		m_style_flag ^= FLAG_SHOW_CONVEX;
		break;

	case '5':
		m_style_flag ^= FLAG_SHOW_CONVEX_CURV3D_GAUSS;
		break;

	case '6':
		m_style_flag ^= FLAG_SHOW_CONVEX_CURV3D_MEAN;
		break;

	case '7':
		m_style_flag ^= FLAG_TRIAN_AGAIN;
		if (m_style_flag ^= FLAG_TRIAN_AGAIN && (Button_6 == true))
			cout << "	\n	_ Press (8) to re-triangulate again" << "\n";
		Button_6 = true;
		break;

	case '8':
		Triang_Convex_Hull = false;
		break;

	case '9':
		m_style_flag ^= FLAG_SMOOTH_GAUSS;
		break;

	case '0':
		m_style_flag ^= FLAG_SMOOTH_MEAN;
		break;

	case 'g':
		m_style_flag ^= FLAG_SHOW_LOCAL_MAX_GAUSS;
		break;

	case 'm':
		m_style_flag ^= FLAG_SHOW_LOCAL_MAX_MEAN;
		break;

	}

}

void Simple3DScene::mousePressed(int x, int y, int modif)
{
	if (ctrlDown(modif)) {
		Scene::mousePressed(x, y, modif);
		return;
	}

	if (altDown(modif)) {
		float xf = x; float yf = y;
		m_pts.push_back(C2DPoint(xf, yf));
	}

}

void Simple3DScene::mouseMoved(int x, int y, int modif)
{
	if (!altDown(modif)) {
		Scene::mouseMoved(x, y, modif);
		return;
	}

	float xf = x; float yf = y;

	float d, dmin;
	dmin = MIN_POINT_DIST_PIXELS;

	if (!m_pts.empty() && !m_pts.empty()) {
		double lx = m_pts.back().x;
		double ly = m_pts.back().y;
		d = sqrt((double)(SQUARE(lx - xf) + SQUARE(ly - yf)));
	}
	else {
		d = 10000;
	}

	if (d > dmin) {
		m_pts.push_back(C2DPoint(xf, yf));
	}
}

void Simple3DScene::savePolygonToFile()
{
	string filename = POLYGON_FILENAME;
	filename = getExePath() + filename;
	std::cout << "Saving to " << filename << std::endl;

	FILE* file = fopen(filename.c_str(), "w");
	if (!file) throw "Cannot open <" + filename + "> for writing";

	for (int pi = 0; pi < m_pts.size(); pi++) {
		C2DPoint &p = m_pts[pi];
		fprintf(file, "%f %f \n", p.x, p.y);
	}

	fclose(file);
}

void Simple3DScene::loadPolygonFromFile(string filename)
{
	FILE* file = fopen(filename.c_str(), "r");
	if (!file) throw "Cannot open <" + filename + "> for reading";

	m_pts.clear();

	char line[1024];
	while (fgets(line, 1023, file)) {
		int len;
		if ((len = strlen(line))<1) continue;
		if (line[len - 1] == '\n') line[len - 1] = 0;
		float x, y;
		sscanf(line, "%f %f", &x, &y);
		m_pts.push_back(C2DPoint(x, y));
	}

}

void Simple3DScene::pixelCoordsToSceneCoords(float &x, float &y)
{
	x = getSceneWidth() / getViewportWidth()  *  x;
	y = getSceneHeight() / getViewportHeight() *  y;
}

int main(int argc, char* argv[])
{
	try {

		return vvr::mainLoop(argc, argv, new Simple3DScene);
	}
	catch (std::string exc) {
		std::cerr << exc << std::endl;
		return 1;
	}
}