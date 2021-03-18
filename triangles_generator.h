#pragma once

#include <string>
#include <vector>

#include "geometry.h"

namespace geometry {

class Triangles_Generator final {
private:
    const size_t T_NUM = 2000;
    const double AREA_SIZE = 50.0;
    const double MAX_T_SIZE = 5.0;
    const int ACCURACY_COEF = 100;

    double gen_number(int max_num) const;
    Triangle gen_triangle() const;
    void write_to_file(const std::vector<Triangle>& trs,
                       const std::string& filename) const;
public:
    Triangles_Generator() = default;
    ~Triangles_Generator() = default;

    void generate(const std::string& filename) const;
};

} //namespace geometry
