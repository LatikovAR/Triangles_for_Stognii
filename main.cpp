#include <iostream>
#include <list>
#include <cassert>
#include <vector>

#include "geometry.h"
#include "intersection_finder.h"
#include "vulkan_drawing.h"

using namespace geometry;

Undefined_Object input_geometry_object() {
    double x, y, z;
    std::cin >> x >> y >> z;
    point p1(x, y, z);
    std::cin >> x >> y >> z;
    point p2(x, y, z);
    std::cin >> x >> y >> z;
    point p3(x, y, z);
    return Undefined_Object(p1, p2, p3);
}

int main() {
    size_t n;
    std::cin >> n;

    std::vector<Undefined_Object> objects;
    objects.reserve(n);

    for(size_t i = 0; i < n; i++) {
        objects.push_back(input_geometry_object());
    }

    std::cout << "Input complete.\n";

    Intersection_Finder intersection_finder{Geometry_Object_Storage(objects)};
    Objects_and_Intersections intersection_defined_objects = intersection_finder.compute_intersections();
    const std::vector<bool>& intersection_flags = intersection_defined_objects.intersection_flags();

    std::cout << "Intersected objects:" << std::endl;
    for(size_t i = 0; i < intersection_flags.size(); ++i) {
        if(intersection_flags[i] == true) {
            std::cout << i << std::endl;
        }
    }
    std::cout << std::endl;

    draw_triangles_driver(std::move(intersection_defined_objects));
    return 0;
}

