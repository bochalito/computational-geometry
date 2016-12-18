#ifndef GEOM_H
#define GEOM_H

#include "vvrscenedll.h"
#include "canvas.h"

#include <vector>
#include <cstdio>

using std::vector;

namespace vvr {

struct VVRScene_API Vec3d
{
    union
    {
        struct { double x,y,z; };
        double data[3];
    };

    Vec3d(double _x = 0, double _y = 0, double _z = 0) : x(_x), y(_y), z(_z) {}

    /**
     * Prints the values of all dimensions is stdout.
     */
    void print();

    /**
     * Compares vectors for equality in all dimensions.
     */
    bool operator== (const Vec3d &p);

    /**
     * Compares vectors for inequality in all dimensions.
     */
    bool operator!= (const Vec3d &p);

    /**
     * Returns the length of this vector.
     */
    double length();

    /**
     * Normalize the vector so that its length is 1.
     */
    Vec3d &normalize();

    /**
     * Adds a vector to this vector.
     * @param [in] v The vector that is added to the original vector.
     * @return referrence to original vector.
     */
    Vec3d &add(const Vec3d &v);

    /**
     * Subs a vector from this vector.
     * @param [in] v The vector that is subtracted from the original vector.
     * @return referrence to original vector.
     */
    Vec3d &sub(const Vec3d &v);

    /**
     * Multiplies the vector with a scalar double value
     * @param [in] s The double value with which all dimensions are multiplied
     * @return referrence to original vector.
     */
    Vec3d &scale(const double s);
};

struct VVRScene_API Box
{
    Vec3d min, max;

    Box();

    Box(const Vec3d &vmin, const Vec3d &vmax);

    /**
     * Constructs the smallest box that contains 3 points.
     * Bounding box of triangle.
     **/
    Box(const Vec3d &v1, const Vec3d &v2, const Vec3d &v3);

    /**
     * Construct the smallest box that contains all the Vector3fs of a vector.
     * (Bounding Box of the Vector3fs)
     * @param [in] vertices Vector that contains all the Vector3fs to be tested.
     */
    Box(const vector<Vec3d> &vertices);

    /**
     * Crops a box.
     *
     * Ensures that every dimension of the original box is
     * less or equal from the respective dimension of the cropBox.
     * @param [in] cropBox The box that is used for the crop.
     */
    Box &cropBox(Box &cropBox);

    double getXSize() const;

    double getYSize() const;

    double getZSize() const;

    Vec3d getSize() const;

    /**
     * Get the minimum of the 3 dimensions.
     */
    double getMinSize() const;

    /**
     * Get the maximum of the 3 dimensions.
     */
    double getMaxSize() const;

    /**
     * Get the volume of the box.
     */
    double getVolume() const;

    /**
     * Translates the box by adding the Vector3f v to both corners
     * that define the box.
     * @param v Translation distnace.
     */
    Box &add(const Vec3d &v);

    /**
     * Translates the box by adding the Vector3f -v to both corners
     * that define the box.
     * @param v Translation distnace * (-1).
     */
    Box &sub(const Vec3d &v);

    /**
     * Scales the box by multiplying both corners with a value.
     * @param s The value with which is the box's corners are multiplied.
     */
    Box &scale(const double s);

    void draw(const Colour &col, unsigned char a = 0) const;
};

/**
 * Struct that contains a triangle.
 *
 * Doesn't contain the actual data for the 3 vertices.
 * Instead, contains indices to a vector and a Vector3fer
 * to that vector.
 *
 * Also, it contains A,B,C,D coefficients of the plane equation of
 * the triangles plane. This is done in order to calculate fast the
 * plane equation and the normal.
 */
struct VVRScene_API Triangle
{
    /**
     * Indices to the veclist
     */
    union
    {
        struct { int vi1, vi2, vi3;};
        int v[3];
    };

    /**
     * Vector3fer to the vector containing the vertices
     */
    vector<Vec3d> *vecList;

    /**
     * Plane equation coefficients
     */
    double A,B,C,D;

    /**
     * Bounding box of the triangle
     */
    Box box;

    Triangle(vector<Vec3d> *vecList, int v1 = 0, int v2 = 0, int v3 = 0) :
        vi1(v1), vi2(v2), vi3(v3), vecList(vecList)
    {
        update();
    }

    /** Calculate the coefficients of the plane from the vertices. */
    void update();

    const Vec3d &v1() const;

    const Vec3d &v2() const;

    const Vec3d &v3() const;

    /** Returns the bounding box of the triangle */
    const Box &getBox() const;

    /** Returns the normal of this triangle */
    const Vec3d getNormal() const;

    /** Returns the center Vector3f of this triangle */
    const Vec3d getCenter() const;

    /**
    * Evaluates the plane equation of this triangle's plane
    * for the given Vector3f.
    * @param r The Vector3f at which we find the value of the plane equation.
    * @return The value of the plane equation at the given Vector3f.
    */
    double planeEquation(const Vec3d &r) const;
};

}

#endif
