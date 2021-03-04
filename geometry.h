#pragma once

#include <cassert>
#include <list>
#include <vector>
#include <cmath>
#include <utility>
#include <stdexcept>
#include <iostream>

namespace geometry {

enum g_obj_pos {COMMON, PARALLEL, MATCH};
//enum g_obj_type {TRIANGLE, CUT, POINT};

const double DOUBLE_GAP = 0.000001;



//----------------------------------------point-----------------------------------

class vec;
class vec_2d;

class point {
protected:
    double x_;
    double y_;
    double z_;
public:
    double x() const { return x_; }
    double y() const { return y_; }
    double z() const { return z_; }

    point(double x, double y, double z): x_(x), y_(y), z_(z) {}
    virtual ~point() {}

    point& operator+=(const vec& v)&;
    bool is_real_point() const;
};

point operator+(const point& p, const vec& v);
point operator+(const vec& v, const point& p);

bool is_points_match(const point &p1, const point &p2);
bool is_points_on_one_line(const point &p1, const point &p2, const point &p3);


class point_2d final {
private:
    double x_;
    double y_;
public:
    double x() const { return x_; }
    double y() const { return y_; }

    point_2d(double x, double y): x_(x), y_(y) {}

    point_2d& operator+=(const vec_2d& v)&;
    bool is_real_point() const;
};

point_2d operator+(const point_2d& p, const vec_2d& v);
point_2d operator+(const vec_2d& v, const point_2d& p);

bool is_points_match(const point_2d &p1, const point_2d &p2);
bool is_points_on_one_line(const point_2d &p1, const point_2d &p2, const point_2d &p3);




//--------------------------------------vector------------------------------------

class Normalized_Quaternion;

class vec final {
private:
    double x_;
    double y_;
    double z_;
public:
    double x() const { return x_; }
    double y() const { return y_; }
    double z() const { return z_; }

    vec(const point &p1, const point &p2):
        x_(p2.x() - p1.x()),
        y_(p2.y() - p1.y()),
        z_(p2.z() - p1.z()) {}
    vec(double x, double y, double z): x_(x), y_(y), z_(z) {}

    vec& operator+=(const vec& v)&;
    vec& operator-=(const vec& v)&;
    vec& operator*=(double a)&;
    vec& operator/=(double a)&;

    double length() const { return sqrt(x_ * x_ + y_ * y_ + z_ * z_); }
    void normalize();
    bool is_null() const { return fabs(length()) < DOUBLE_GAP; }
    static bool is_parallel(const vec& v1, const vec& v2);
    vec rotate_vec(const Normalized_Quaternion& quat) const;
};

vec operator+(const vec& v1, const vec& v2);
vec operator-(const vec& v1, const vec& v2);
vec operator*(double a, const vec& v);
vec operator*(const vec& v, double a);
vec operator/(const vec& v, double a);

vec mult_vec(const vec &v1, const vec &v2);


class vec_2d final {
private:
    double x_;
    double y_;
public:
    double x() const { return x_; }
    double y() const { return y_; }

    vec_2d(const point_2d &p1, const point_2d &p2):
        x_(p2.x() - p1.x()),
        y_(p2.y() - p1.y()) {}
    vec_2d(double x, double y): x_(x), y_(y) {}

    vec_2d& operator+=(const vec_2d& v)&;
    vec_2d& operator-=(const vec_2d& v)&;
    vec_2d& operator*=(double a)&;
    vec_2d& operator/=(double a)&;

    double length() const { return sqrt(x_ * x_ + y_ * y_); }
    void normalize();
    bool is_null() const { return fabs(length()) < DOUBLE_GAP; }
    static bool is_parallel(const vec_2d& v1, const vec_2d& v2);
};

vec_2d operator+(const vec_2d& v1, const vec_2d& v2);
vec_2d operator-(const vec_2d& v1, const vec_2d& v2);
vec_2d operator*(double a, const vec_2d& v);
vec_2d operator*(const vec_2d& v, double a);
vec_2d operator/(const vec_2d& v, double a);




//--------------------------------------Normalized_Quaternion----------------------

class Normalized_Quaternion final {
private:
    double a0_, a1_, a2_, a3_;
public:
    double a0() const { return a0_; }
    double a1() const { return a1_; }
    double a2() const { return a2_; }
    double a3() const { return a3_; }

    Normalized_Quaternion(double rotate_angle, const vec& axis);

    Normalized_Quaternion adj() const;

    Normalized_Quaternion& operator*=(const Normalized_Quaternion& rhs)&;
};

Normalized_Quaternion operator*(const Normalized_Quaternion& lhs, const Normalized_Quaternion& rhs);




//------------------------------------------Cut------------------------------------

class Cut {
protected:
    point p_;
    vec v_;
public:
    Cut(const point &p1, const point &p2): p_(p1), v_(p1, p2) {
        if(v_.is_null()) throw std::invalid_argument("Cut length = 0");
    }
    Cut(const point &p, const vec &v): p_(p), v_(v) {
        if(v_.is_null()) throw std::invalid_argument("Cut length = 0");
    }
    virtual ~Cut() {}

    const point& p_begin() const { return p_; }
    point p_end() const { return p_ + v_; }
    const vec& vec() const { return v_; }

    double length() const { return v_.length(); }
};

class Cut_2d final {
private:
    point_2d p_;
    vec_2d v_;
public:
    Cut_2d(const point_2d &p1, const point_2d &p2): p_(p1), v_(p1, p2) {
        if(v_.is_null()) throw std::invalid_argument("Cut_2d length = 0");
    }
    Cut_2d(const point_2d &p, const vec_2d &v): p_(p), v_(v) {
        if(v_.is_null()) throw std::invalid_argument("Cut_2d length = 0");
    }

    const point_2d& p_begin() const { return p_; }
    point_2d p_end() const { return p_ + v_; }
    const vec_2d& vec() const { return v_; }

    double length() const { return v_.length(); }
};

g_obj_pos lines_pos_2d(const Cut_2d &c1, const Cut_2d &c2);
bool is_cut_2d_intersects(const Cut_2d &c1, const Cut_2d &c2);

//------------------------------------------Plane----------------------------------

class Triangle;

class Plane final {
private:
    double a;
    double b;
    double c;
    double d;
public:
    double A() const { return a; }
    double B() const { return b; }
    double C() const { return c; }
    double D() const { return d; }

    Plane(const point &p1, const point &p2, const point &p3);

    int point_side_plane(const point& p) const {
        double k = p.x() * a + p.y() * b + p.z() * c + d;
        if(k > DOUBLE_GAP) return 1;
        if(k < -DOUBLE_GAP) return -1;
        return 0;
    }
    int cut_side_plane(const Cut& c) const;
    int triangle_side_plane(const Triangle& t) const;
    vec normal() const { return vec(a, b, c); }
};

g_obj_pos planes_pos(const Plane &pl1, const Plane &pl2);
bool is_vec_parallel_plane(const Plane &pl, const vec &v);
bool is_point_on_plane(const Plane &pl, const point& p);
g_obj_pos cut_and_plane_pos(const Plane &pl, const Cut &c);
point intersection_plane_and_line(const Plane &pl, const Cut &c);



//---------------------------------------Triangle----------------------------------

class Triangle {
protected:
    point p1_;
    point p2_;
    point p3_;
    Plane pl_;
public:
    const point& p1() const { return p1_; }
    const point& p2() const { return p2_; }
    const point& p3() const { return p3_; }
    const Plane& pl() const { return pl_; }

    Triangle(const point& p1, const point& p2, const point& p3):
        p1_(p1), p2_(p2), p3_(p3), pl_(Plane(p1, p2, p3)) {
        //matching points checked in Plane constrcutor
    }
    virtual ~Triangle() {}

    void print() const {
        std::cout << p1_.x() << " " << p1_.y() << " " << p1_.z() << std::endl;
        std::cout << p2_.x() << " " << p2_.y() << " " << p2_.z() << std::endl;
        std::cout << p3_.x() << " " << p3_.y() << " " << p3_.z() << std::endl;
    }
};

class Triangle_2d final {
private:
    point_2d p1_;
    point_2d p2_;
    point_2d p3_;
public:
    const point_2d& p1() const { return p1_; }
    const point_2d& p2() const { return p2_; }
    const point_2d& p3() const { return p3_; }

    Triangle_2d(const point_2d& p1, const point_2d& p2, const point_2d& p3):
        p1_(p1), p2_(p2), p3_(p3) {
        if((is_points_match(p1, p2)) || (is_points_match(p2, p3)))
            throw std::invalid_argument("Matching points in triangle");
    }

    bool is_in_triangle(const point_2d &p) const;

    void print() const {
        std::cout << p1_.x() << " " << p1_.y() << std::endl;
        std::cout << p2_.x() << " " << p2_.y() << std::endl;
        std::cout << p3_.x() << " " << p3_.y() << std::endl;
    }
};

bool is_cut_and_triangle_intersects_on_plane(const Triangle &t, const Cut &c);
bool is_cut_and_triangle_intersects_2d(const Triangle_2d &t, const Cut_2d &c);
bool is_triangles_intersects_on_plane(const Triangle &t1, const Triangle &t2);
bool is_triangles_intersects_2d(const Triangle_2d &t1, const Triangle_2d &t2);

} //namespace geometry
