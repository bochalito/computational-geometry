#include "settings.h"
#include "scene.h"
#include "mesh.h"
#include <vector>
#include <string>
#include <MathGeoLib/MathGeoLib.h>
#include <C2DPoint.h>

class Simple3DScene : public vvr::Scene
{
public:
	Simple3DScene();
	const char* getName() const { return "Simple 3D Scene"; }
	void keyEvent(unsigned char key, bool up, int modif) override;
	void mousePressed(int x, int y, int modif) override;
	void mouseMoved(int x, int y, int modif) override;

protected:
	void draw() override;
	void resize() override;

private:
	void drawPolygon();
	void savePolygonToFile();
	void loadPolygonFromFile(string filename);
	void pixelCoordsToSceneCoords(float &x, float &y);
	void curvature2D();
	void curvature3DGauss(vvr::Mesh &mesh);
	void curvature3DMean(vvr::Mesh &mesh);
	void convexHull3D(vvr::Mesh &mesh);
	void triangulationOfConvexHull(vvr::Mesh &mesh);
	void findLocalMaxOfCurvature(vector<pair<double, vvr::Vec3d>> curvature_values, vector<pair<double, vvr::Vec3d>> &vertices_with_max_curv);
	void smoothingGauss(vvr::Mesh &mesh,double factor);
	void smoothingMean(vvr::Mesh &mesh, double factor);
	void consoleMenu();
	void findStar(vvr::Vec3d p, vvr::Mesh &mesh, vector<vvr::Triangle> &star);
	void resetProgram();
	double findCotaCotb(vector<vvr::Triangle> star, pair<vvr::Vec3d, vvr::Vec3d> temp_Vertices);
	double findAreaOfVoronoiRegion(vector<vvr::Triangle> &star, vvr::Vec3d p);
	double gaussianCurvatureThreshold(vvr::Mesh &mesh);
	double meanCurvatureThreshold(vvr::Mesh &mesh);

private:
	bool                    b_show_pts;
	vector<C2DPoint>        m_pts;
	vvr::Mesh               m_model;
	vvr::Mesh				m_model_convex_hull;
	vvr::Settings           m_settings;
	vvr::Colour             m_obj_col;
	float                   m_sphere_rad;
	vector<vvr::Sphere3D>	m_spheres;
	int                     m_style_flag;
	vector<pair<double,vvr::Vec3d>> vertices_with_maxGaussCurv;
	vector<pair<double, vvr::Vec3d>> vertices_with_maxMeanCurv;
	vector<pair<double, vvr::Vec3d>> curvature_values;
};
