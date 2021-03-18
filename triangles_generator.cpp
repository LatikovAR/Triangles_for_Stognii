#include <cstdlib>
#include <ctime>
#include <vector>
#include <fstream>
#include <cassert>

#include "triangles_generator.h"

namespace geometry {

void Triangles_Generator::generate(const std::string& filename) const {
    std::srand(std::time(0));

    std::vector<Triangle> trs;
    trs.reserve(T_NUM);
    for(size_t i = 0; i < T_NUM; ++i) {
        trs.push_back(gen_triangle());
    }

    write_to_file(trs, filename);
}

Triangle Triangles_Generator::gen_triangle() const {
    double x, y, z;
    x = gen_number(AREA_SIZE);
    y = gen_number(AREA_SIZE);
    z = gen_number(AREA_SIZE);
    vec shift{x, y, z};

    x = gen_number(MAX_T_SIZE);
    y = gen_number(MAX_T_SIZE);
    z = gen_number(MAX_T_SIZE);
    point p1 = point{x, y, z} + shift;

    x = gen_number(MAX_T_SIZE);
    y = gen_number(MAX_T_SIZE);
    z = gen_number(MAX_T_SIZE);
    point p2 = point{x, y, z} + shift;

    while (is_points_match(p1, p2)) {
        x = gen_number(MAX_T_SIZE);
        y = gen_number(MAX_T_SIZE);
        z = gen_number(MAX_T_SIZE);
        p2 = point{x, y, z} + shift;
    }

    x = gen_number(MAX_T_SIZE);
    y = gen_number(MAX_T_SIZE);
    z = gen_number(MAX_T_SIZE);
    point p3 = point{x, y, z} + shift;

    while((is_points_match(p1, p3)) ||
          (is_points_match(p2, p3)) ||
          (is_points_on_one_line(p1, p2, p3)))
    {
        x = gen_number(MAX_T_SIZE);
        y = gen_number(MAX_T_SIZE);
        z = gen_number(MAX_T_SIZE);
        p3 = point{x, y, z} + shift;
    }

    return Triangle{p1, p2, p3};
}

double Triangles_Generator::gen_number(int max_num) const {
    return static_cast<double>(std::rand() % (max_num * ACCURACY_COEF)) /
           static_cast<double>(ACCURACY_COEF);
}

void Triangles_Generator::write_to_file(const std::vector<Triangle> &trs,
                                        const std::string& filename) const
{
    std::ofstream out;
    out.open(filename);

    out << T_NUM << std::endl;

    for(const Triangle& t : trs) {
        out << t.p1().x() << " " << t.p1().y() << " " << t.p1().z() << " ";
        out << t.p2().x() << " " << t.p2().y() << " " << t.p2().z() << " ";
        out << t.p3().x() << " " << t.p3().y() << " " << t.p3().z() << std::endl;
    }
}

} //namespace geometry
