#ifndef __SHAPE2D_H__
#define __SHAPE2D_H__

#include "vvrscenedll.h"
#include "utils.h"
#include <string>
#include <vector>
#include <cstdlib>
#include <GeoLib.h>

using std::string;
using std::vector;

static void drawSphere(double r, int lats, int longs);

namespace vvr {

struct VVRScene_API Colour
{
    union
    {
        struct { unsigned char r, g, b;};
        unsigned char data[3];
    };

    Colour () : r(0), g(0), b(0) {}

    Colour (unsigned char red, unsigned char green, unsigned char blue) :
        r(red), g(green), b(blue) {}

    Colour(string hex_str) {
        r = strtol(hex_str.substr(0,2).c_str(), 0, 16);
        g = strtol(hex_str.substr(2,2).c_str(), 0, 16);
        b = strtol(hex_str.substr(4,2).c_str(), 0, 16);
    }

    static Colour white;
    static Colour red;
    static Colour green;
    static Colour blue;
    static Colour black;
    static Colour yellow;
    static Colour grey;
    static Colour orange;
    static Colour cyan;
    static Colour magenta;
    static Colour darkOrange;
    static Colour darkRed;
};

/* Renderables */

class VVRScene_API IRenderable {
public:
    virtual ~IRenderable() {}

    virtual void draw() const = 0;
};

struct VVRScene_API Shape : public IRenderable
{
    Colour colour;
    bool b_render_solid;

protected:
    Shape() : b_render_solid(false) {}
    Shape(const Colour &rgb) : colour(rgb), b_render_solid(false) {}
    virtual void drawShape() const = 0;

public:
    virtual ~Shape() {}
    void draw() const override;
    void setColour(const Colour &col) {colour = col;}
    void setSolidRender(bool render_solid) {b_render_solid = render_solid;}
};

struct VVRScene_API Point2D : public Shape
{
    double x,y;

protected:
    void drawShape() const override;

public:
    Point2D(){}
    Point2D(double x, double y, const Colour &rgb=Colour()) :
        x(x), y(y), Shape(rgb) {}
};

struct VVRScene_API Point3D : public Shape
{
    double x,y,z;

protected:
    void drawShape() const override;

public:
    Point3D(){}
    Point3D(double x, double y, double z, const Colour &rgb=Colour()) :
        x(x), y(y), z(z), Shape(rgb) {}
};

struct VVRScene_API LineSeg2D : public Shape
{
    double x1,y1;
    double x2,y2;

protected:

    void drawShape() const override;

public:
    LineSeg2D(){}
    LineSeg2D(double _x1, double _y1, double _x2, double _y2, const Colour &rgb=Colour()) :
        x1(_x1), y1(_y1), x2(_x2), y2(_y2), Shape(rgb) {}
};

struct VVRScene_API Line2D : public LineSeg2D
{
protected:
    void drawShape() const override;

public:
    Line2D(){}
    Line2D(double _x1, double _y1, double _x2, double _y2, const Colour &rgb=Colour()) :
        LineSeg2D(_x1, _y1, _x2, _y2, rgb) {}
};

struct VVRScene_API LineSeg3D : public Shape
{
    double x1,y1,z1;
    double x2,y2,z2;

protected:

    void drawShape() const override;

public:
    LineSeg3D(){}
    LineSeg3D(double x1, double y1, double z1,
              double x2, double y2, double z2, const Colour &rgb=Colour()) :
        x1(x1), y1(y1), z1(z1), x2(x2), y2(y2), z2(z2), Shape(rgb) {}
};

struct VVRScene_API Circle2D : public Shape
{
    double x,y,r;
    double rad_from, rad_to;
    bool closed_loop;
protected:
    void drawShape() const override;

public:
    Circle2D() : rad_from(0), rad_to(6.28318530718), closed_loop(true) {}
    Circle2D(double x, double y, double rad, const Colour &rgb=Colour()) : Shape(rgb),
        x(x), y(y), r(rad), rad_from(0), rad_to(6.28318530718), closed_loop(true) {}
    void setRange(double Rad_from, double Rad_to) {rad_from = Rad_from;rad_to = Rad_to;}
    void setClosedLoop(bool Closed_loop) {closed_loop = Closed_loop;}
};

struct VVRScene_API Sphere3D : public Shape
{
    double x, y, z, rad;

protected:
    void drawShape() const override;

public:
    Sphere3D(){}
    Sphere3D(double x, double y, double z, double rad, const Colour &rgb = Colour()) :
        x(x), y(y), z(z), rad(rad), Shape(rgb) {}
};

struct VVRScene_API Box3D : public Shape
{
    double x1, y1, z1;
    double x2, y2, z2;

protected:
    void drawShape() const override;

public:
    Box3D() {}
    Box3D(double xmin, double ymin, double zmin,
          double xmax, double ymax, double zmax,
          const Colour &col = Colour()) :
        x1(xmin), y1(ymin), z1(zmin),
        x2(xmax), y2(ymax), z2(zmax), Shape(col) {}
};

struct VVRScene_API Triangle2D : public Shape
{
    double x1,y1;
    double x2,y2;
    double x3,y3;

protected:
    void drawShape() const override;

public:
    Triangle2D(){b_render_solid = false;}
    Triangle2D(double x1, double y1, double x2, double y2, double x3, double y3,
               const Colour &rgb=Colour()) :
        x1(x1), y1(y1), x2(x2), y2(y2), x3(x3), y3(y3), Shape(rgb) {b_render_solid = false;}
};

struct VVRScene_API Triangle3D : public Shape
{
    double x1,y1,z1;
    double x2,y2,z2;
    double x3,y3,z3;

protected:
    void drawShape() const override;

public:
    Triangle3D(){b_render_solid = true;}
    Triangle3D(double x1, double y1, double z1,
               double x2, double y2, double z2,
               double x3, double y3, double z3,
               const Colour &rgb=Colour()) :
        x1(x1), y1(y1), z1(z1),
        x2(x2), y2(y2), z2(z2),
        x3(x3), y3(y3), z3(z3),
        Shape(rgb) {b_render_solid = true;}
};

/* Canvas */
struct VVRScene_API Frame {
    vector<Shape*> shapes;
    bool show_old;
    Frame ();
    Frame (bool show_old);
};

class VVRScene_API Canvas2D {
    vector<Frame> frames;
    unsigned fi;

public:
    Canvas2D();
    ~Canvas2D();

    unsigned size() { return frames.size(); }
    unsigned frameIndex() { return fi; }
    bool isAtStart() { return fi == 0; }
    bool isAtEnd() { return fi == frames.size()-1; }

    void newFrame(bool show_old_frames=true);
    void add(Shape *shape_ptr);
    void draw();
    void next();
    void prev();
    void rew();
    void ff();
    void resize(int i);
    void clear();

    /* Utilities to directly add GeoLib objects to canvas */

    void add(const C2DPoint &p, const Colour &col) {
        add(new Point2D(p.x, p.y, col));
    }

    void add(const C2DPoint &p1, const C2DPoint &p2, const Colour &col, bool inf_line=false) {
        if (inf_line)
            add(new Line2D(p1.x, p1.y, p2.x, p2.y, col));
        else
            add(new LineSeg2D(p1.x, p1.y, p2.x, p2.y, col));
    }

    void add(const C2DLine &line, const Colour &col, bool inf_line=false) {
        const C2DPoint &p1 = line.GetPointFrom();
        const C2DPoint &p2 = line.GetPointTo();
        add(p1, p2, col, inf_line);
    }

    void add(const C2DCircle &circle, const Colour &col, bool solid=false) {
        Shape * s = new Circle2D(circle.GetCentre().x, circle.GetCentre().y, circle.GetRadius(), col);
        s->setSolidRender(solid);
        add(s);
    }

    void add(const C2DTriangle &tri, const Colour &col, bool solid=false) {
        Shape *s = new Triangle2D(
                    tri.GetPoint1().x,
                    tri.GetPoint1().y,
                    tri.GetPoint2().x,
                    tri.GetPoint2().y,
                    tri.GetPoint3().x,
                    tri.GetPoint3().y,
                    col);
        s->setSolidRender(solid);
        add(s);
    }

};

VVRScene_API void draw(C2DPointSet &point_set, Colour &col=Colour::black);

VVRScene_API void draw(C2DLineSet &line_set, Colour &col=Colour::black);

VVRScene_API void draw(C2DPolygon &polygon, Colour &col=Colour::black, bool filled=false);

}

#endif
