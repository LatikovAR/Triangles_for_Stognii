#include <cmath>
#include <cassert>

#include "geometry.h"

namespace geometry {


//--------------------------------------point-------------------------------------

point& point::operator+=(const vec& v)& {
    x_ += v.x();
    y_ += v.y();
    z_ += v.z();
    return *this;
}

point operator+(const point& p, const vec& v) {
    point tmp {p};
    tmp += v;
    return tmp;
}

point operator+(const vec& v, const point& p) {
    point tmp {p};
    tmp += v;
    return tmp;
}

bool point::is_real_point() const {
    if((std::isinf(x_)) || (std::isnan(x_))) {
        return false;
    }
    if((std::isinf(y_)) || (std::isnan(y_))) {
        return false;
    }
    if((std::isinf(z_)) || (std::isnan(z_))) {
        return false;
    }
    return true;
}

bool is_points_match(const point &p1, const point &p2) {
    if((fabs(p1.x() - p2.x()) > DOUBLE_GAP) ||
       (fabs(p1.y() - p2.y()) > DOUBLE_GAP) ||
       (fabs(p1.z() - p2.z()) > DOUBLE_GAP)) {
        return false;
    }
    return true;
}

bool is_points_on_one_line(const point &p1, const point &p2, const point &p3) {
    vec v1(p1, p2);
    vec v2(p1, p3);
    v1.normalize();
    v2.normalize();
    if(fabs(fabs(v1.x()) - fabs(v2.x())) > DOUBLE_GAP) {
        return false;
    }
    if(fabs(fabs(v1.y()) - fabs(v2.y())) > DOUBLE_GAP) {
        return false;
    }
    if(fabs(fabs(v1.z()) - fabs(v2.z())) > DOUBLE_GAP) {
        return false;
    }
    return true;
}

point_2d& point_2d::operator+=(const vec_2d& v)& {
    x_ += v.x();
    y_ += v.y();
    return *this;
}

point_2d operator+(const point_2d& p, const vec_2d& v) {
    point_2d tmp {p};
    tmp += v;
    return tmp;
}

point_2d operator+(const vec_2d& v, const point_2d& p) {
    point_2d tmp {p};
    tmp += v;
    return tmp;
}

bool point_2d::is_real_point() const {
    if((std::isinf(x_)) || (std::isnan(x_))) {
        return false;
    }
    if((std::isinf(y_)) || (std::isnan(y_))) {
        return false;
    }
    return true;
}

bool is_points_match(const point_2d &p1, const point_2d &p2) {
    if((fabs(p1.x() - p2.x()) > DOUBLE_GAP) ||
       (fabs(p1.y() - p2.y()) > DOUBLE_GAP)) {
        return false;
    }
    return true;
}

bool is_points_on_one_line(const point_2d &p1, const point_2d &p2, const point_2d &p3) {
    vec_2d v1(p1, p2);
    vec_2d v2(p1, p3);
    v1.normalize();
    v2.normalize();
    if(fabs(fabs(v1.x()) - fabs(v2.x())) > DOUBLE_GAP) {
        return false;
    }
    if(fabs(fabs(v1.y()) - fabs(v2.y())) > DOUBLE_GAP) {
        return false;
    }
    return true;
}



//------------------------------------------vector--------------------------------

vec& vec::operator+=(const vec& v)& {
    x_ += v.x();
    y_ += v.y();
    z_ += v.z();
    return *this;
}

vec& vec::operator-=(const vec& v)& {
    x_ -= v.x();
    y_ -= v.y();
    z_ -= v.z();
    return *this;
}

vec& vec::operator*=(double a)& {
    x_ *= a;
    y_ *= a;
    z_ *= a;
    return *this;
}

vec& vec::operator/=(double a)& {
    a = 1 / a;
    return (*this *= a);
}

vec operator+(const vec& v1, const vec& v2) {
    vec tmp {v1};
    tmp += v2;
    return tmp;
}

vec operator-(const vec& v1, const vec& v2) {
    vec tmp {v1};
    tmp -= v2;
    return tmp;
}

vec operator*(double a, const vec& v) {
    vec tmp {v};
    tmp *= a;
    return tmp;
}

vec operator*(const vec& v, double a) {
    vec tmp {v};
    tmp *= a;
    return tmp;
}

vec operator/(const vec& v, double a) {
    vec tmp {v};
    tmp /= a;
    return tmp;
}

void vec::normalize() {
    double n = length();
    assert(n > 0);
    *this /= n;
}

bool vec::is_parallel(const vec& v1, const vec& v2) {
    if(v1.is_null() || v2.is_null()) return true;

    vec v1_dir {v1};
    vec v2_dir {v2};
    v1_dir.normalize();
    v2_dir.normalize();

    if((fabs(v1_dir.x_ - v2_dir.x_) < DOUBLE_GAP) &&
       (fabs(v1_dir.y_ - v2_dir.y_) < DOUBLE_GAP)) { return true; }

    return false;
}

vec mult_vec(const vec &v1, const vec &v2) {
    double a, b, c;
    a = v1.y() * v2.z() - v1.z() * v2.y();
    b = v1.z() * v2.x() - v1.x() * v2.z();
    c = v1.x() * v2.y() - v1.y() * v2.x();
    return vec(a, b, c);
}

vec vec::rotate_vec(const Normalized_Quaternion& quat) const {
    Normalized_Quaternion adj_quat = quat.adj();

    //(A * v) * A(adj) = T * A(adj);     A, T - quaternions
    double t0, t1, t2, t3;
    t0 = -quat.a1() * x_ - quat.a2() * y_ - quat.a3() * z_;
    t1 = quat.a0() * x_ + quat.a2() * z_ - quat.a3() * y_;
    t2 = quat.a0() * y_ + quat.a3() * x_ - quat.a1() * z_;
    t3 = quat.a0() * z_ + quat.a1() * y_ - quat.a2() * x_;

    //T * A(adj) = rotated_v
    double x, y, z;
    x = t0 * adj_quat.a1() + t1 * adj_quat.a0() + t2 * adj_quat.a3() - t3 * adj_quat.a2();
    y = t0 * adj_quat.a2() + t2 * adj_quat.a0() + t3 * adj_quat.a1() - t1 * adj_quat.a3();
    z = t0 * adj_quat.a3() + t3 * adj_quat.a0() + t1 * adj_quat.a2() - t2 * adj_quat.a1();

    return vec(x, y, z);
}


vec_2d& vec_2d::operator+=(const vec_2d& v)& {
    x_ += v.x();
    y_ += v.y();
    return *this;
}

vec_2d& vec_2d::operator-=(const vec_2d& v)& {
    x_ -= v.x();
    y_ -= v.y();
    return *this;
}

vec_2d& vec_2d::operator*=(double a)& {
    x_ *= a;
    y_ *= a;
    return *this;
}

vec_2d& vec_2d::operator/=(double a)& {
    a = 1 / a;
    return (*this *= a);
}

vec_2d operator+(const vec_2d& v1, const vec_2d& v2) {
    vec_2d tmp {v1};
    tmp += v2;
    return tmp;
}

vec_2d operator-(const vec_2d& v1, const vec_2d& v2) {
    vec_2d tmp {v1};
    tmp -= v2;
    return tmp;
}

vec_2d operator*(double a, const vec_2d& v) {
    vec_2d tmp {v};
    tmp *= a;
    return tmp;
}

vec_2d operator*(const vec_2d& v, double a) {
    vec_2d tmp {v};
    tmp *= a;
    return tmp;
}

vec_2d operator/(const vec_2d& v, double a) {
    vec_2d tmp {v};
    tmp /= a;
    return tmp;
}

void vec_2d::normalize() {
    double n = length();
    assert(n > 0);
    *this /= n;
}

bool vec_2d::is_parallel(const vec_2d& v1, const vec_2d& v2) {
    if(v1.is_null() || v2.is_null()) return true;

    vec_2d v1_dir {v1};
    vec_2d v2_dir {v2};
    v1_dir.normalize();
    v2_dir.normalize();

    if(fabs(v1_dir.x_ - v2_dir.x_) < DOUBLE_GAP) { return true; }

    return false;
}




//----------------------------------Normalized_Quaternion--------------------------

Normalized_Quaternion::Normalized_Quaternion(double rotate_angle, const vec& axis) {
    vec v{axis};
    v.normalize();
    double k = sin(rotate_angle / 2);

    a0_ = cos(rotate_angle / 2);
    a1_ = k * v.x();
    a2_ = k * v.y();
    a3_ = k * v.z();
}

Normalized_Quaternion Normalized_Quaternion::adj() const {
    Normalized_Quaternion tmp{*this};
    tmp.a1_ *= -1;
    tmp.a2_ *= -1;
    tmp.a3_ *= -1;
    return tmp;
}

Normalized_Quaternion& Normalized_Quaternion::operator*=(const Normalized_Quaternion& rhs)& {
    Normalized_Quaternion lhs{*this};

    a0_ = lhs.a0_ * rhs.a0_ - lhs.a1_ * rhs.a1_ - lhs.a2_ * rhs.a2_ - lhs.a3_ * rhs.a3_;
    a1_ = lhs.a0_ * rhs.a1_ + lhs.a1_ * rhs.a0_ + lhs.a2_ * rhs.a3_ - lhs.a3_ * rhs.a2_;
    a2_ = lhs.a0_ * rhs.a2_ + lhs.a2_ * rhs.a0_ + lhs.a3_ * rhs.a1_ - lhs.a1_ * rhs.a3_;
    a3_ = lhs.a0_ * rhs.a3_ + lhs.a3_ * rhs.a0_ + lhs.a1_ * rhs.a2_ - lhs.a2_ * rhs.a1_;

    return *this;
}

Normalized_Quaternion operator*(const Normalized_Quaternion& lhs, const Normalized_Quaternion& rhs) {
    Normalized_Quaternion tmp{lhs};
    tmp *= rhs;
    return tmp;
}

} //namespace geometry
