#include <cstdlib>
#include <vector>
#include <typeinfo>

#include "intersection_finder.h"
#include "geometry.h"

namespace geometry {

bool Geometry_Object::check_intersection(const Triangle &t1, const Triangle &t2) {
    g_obj_pos p_pos = planes_pos(t1.pl(), t2.pl());

    if(p_pos == PARALLEL) return false;
    if(p_pos == MATCH) return is_triangles_intersects_on_plane(t1, t2);
    assert(p_pos == COMMON);

    //next two blocks can be deleted, but they give a VERY STRONG optimization
    //checking is t2 have common points with t1.pl()
    const Plane& pl_base1 = t1.pl();
    int i11, i12, i13;
    i11 = pl_base1.point_side_plane(t2.p1());
    i12 = pl_base1.point_side_plane(t2.p2());
    i13 = pl_base1.point_side_plane(t2.p3());
    if((i11 * i12 > 0) && (i12 * i13 > 0)) return false;

    //checking is t1 have common points with t2.pl()
    const Plane& pl_base2 = t2.pl();
    int i21, i22, i23;
    i21 = pl_base2.point_side_plane(t1.p1());
    i22 = pl_base2.point_side_plane(t1.p2());
    i23 = pl_base2.point_side_plane(t1.p3());
    if((i21 * i22 > 0) && (i22 * i23 > 0)) return false;

    Cut c1(t2.p1(), t2.p2());
    Cut c2(t2.p1(), t2.p3());
    Cut c3(t2.p2(), t2.p3());

    g_obj_pos c1_ind, c2_ind, c3_ind;
    c1_ind = cut_and_plane_pos(pl_base1, c1);
    c2_ind = cut_and_plane_pos(pl_base1, c2);
    c3_ind = cut_and_plane_pos(pl_base1, c3);

    //cases if some t2 cut lies on t1.pl()
    if(c1_ind == MATCH) return is_cut_and_triangle_intersects_on_plane(t1, c1);
    if(c2_ind == MATCH) return is_cut_and_triangle_intersects_on_plane(t1, c2);
    if(c3_ind == MATCH) return is_cut_and_triangle_intersects_on_plane(t1, c3);

    //common way: searching two points which t2 cuts intersect t1.pl()
    std::vector <point> arr_p;
    if(i11 * i12 < 0) {
        arr_p.push_back(intersection_plane_and_line(pl_base1, c1));
    }
    if(i11 * i13 < 0) {
        arr_p.push_back(intersection_plane_and_line(pl_base1, c2));
    }
    if(i12 * i13 < 0) {
        arr_p.push_back(intersection_plane_and_line(pl_base1, c3));
    }
    if(i11 == 0) {
        arr_p.push_back(t2.p1());
     }
    if(i12 == 0) {
        arr_p.push_back(t2.p2());
     }
    if(i13 == 0) {
        arr_p.push_back(t2.p3());
     }

    assert((arr_p.size() <= 2) && (arr_p.size() >= 1));

    //case if triangle corner lies on pl
    if(arr_p.size() == 1) {
        return check_intersection(t1, arr_p[0]);
    }

    Cut c(arr_p[0], arr_p[1]);
    return is_cut_and_triangle_intersects_on_plane(t1, c);
}

bool Geometry_Object::check_intersection(const Triangle &t, const Cut &c) {
    g_obj_pos pos = cut_and_plane_pos(t.pl(), c);

    if(pos == PARALLEL) return false;
    if(pos == MATCH) return is_cut_and_triangle_intersects_on_plane(t, c);
    assert(pos == COMMON);
    point p = intersection_plane_and_line(t.pl(), c);

    const vec& v = c.vec();
    if((v.x() >= v.y()) && (v.x() >= v.z())) {
        double k = (p.x() - c.p_begin().x()) / v.x();
        if((k < 0) || (k > 1)) return false;
    }
    else if((v.y() >= v.z()) && (v.y() >= v.z())) {
        double k = (p.y() - c.p_begin().y()) / v.y();
        if((k < 0) || (k > 1)) return false;
    }
    else {
        double k = (p.z() - c.p_begin().z()) / v.z();
        if((k < 0) || (k > 1)) return false;
    }

    return check_intersection(t, p);
}

bool Geometry_Object::check_intersection(const Triangle &t, const point &p) {
    if(is_point_on_plane(t.pl(), p) == false) return false;

    vec a1 = mult_vec(vec(t.p1(), t.p2()), vec(t.p1(), p));
    vec a2 = mult_vec(vec(t.p2(), t.p3()), vec(t.p2(), p));
    vec a3 = mult_vec(vec(t.p3(), t.p1()), vec(t.p3(), p));

    if((fabs(a1.x()) >= fabs(a1.y())) && (fabs(a1.x()) >= fabs(a1.z()))) {
        if(a1.x() * a2.x() < 0) return false;
        if(a2.x() * a3.x() < 0) return false;
        return true;
    }

    if((fabs(a1.y()) >= fabs(a1.x())) && (fabs(a1.y()) >= fabs(a1.z()))) {
        if(a1.y() * a2.y() < 0) return false;
        if(a2.y() * a3.y() < 0) return false;
        return true;
    }

    if(a1.z() * a2.z() < 0) return false;
    if(a2.z() * a3.z() < 0) return false;
    return true;
}

bool Geometry_Object::check_intersection(const Cut &c1, const Cut &c2) {
    if(is_points_on_one_line(c1.p_begin(), c1.p_end(), c2.p_begin())) {
        if(check_intersection(c1, c2.p_begin()) == true) return true;
        else return check_intersection(c1, c2.p_end());
    }

    Plane pl(c1.p_begin(), c1.p_end(), c2.p_begin());
    double i = pl.point_side_plane(c2.p_end());
    if(fabs(i) > DOUBLE_GAP) {
        return false;
    }

    if((fabs(pl.A()) >= fabs(pl.B())) && (fabs(pl.A()) >= fabs(pl.C()))) {
        return is_cut_2d_intersects(Cut_2d(point_2d(c1.p_begin().y(), c1.p_begin().z()),
                                           vec_2d(c1.vec().y(), c1.vec().z())),
                                    Cut_2d(point_2d(c2.p_begin().y(), c2.p_begin().z()),
                                           vec_2d(c2.vec().y(), c2.vec().z())));
    }

    if((fabs(pl.B()) >= fabs(pl.A())) && (fabs(pl.B()) >= fabs(pl.C()))) {
        return is_cut_2d_intersects(Cut_2d(point_2d(c1.p_begin().x(), c1.p_begin().z()),
                                           vec_2d(c1.vec().x(), c1.vec().z())),
                                    Cut_2d(point_2d(c2.p_begin().x(), c2.p_begin().z()),
                                           vec_2d(c2.vec().x(), c2.vec().z())));
    }

    return is_cut_2d_intersects(Cut_2d(point_2d(c1.p_begin().x(), c1.p_begin().y()),
                                       vec_2d(c1.vec().x(), c1.vec().y())),
                                Cut_2d(point_2d(c2.p_begin().x(), c2.p_begin().y()),
                                       vec_2d(c2.vec().x(), c2.vec().y())));
}

bool Geometry_Object::check_intersection(const Cut &c, const point &p) {
    const vec& v_c = c.vec();
    const vec v_p(c.p_begin(), p);
    if(vec::is_parallel(v_c, v_p) == false) return false;

    if((fabs(v_c.x()) >= fabs(v_c.y())) && (fabs(v_c.x()) >= fabs(v_c.z()))) {
        assert(fabs(v_c.x()) > DOUBLE_GAP);

        double k = v_p.x() / v_c.x();
        if((k < 0) || (k > 1)) return false;
        return true;
    }

    if((fabs(v_c.y()) >= fabs(v_c.x())) && (fabs(v_c.y()) >= fabs(v_c.z()))) {
        assert(fabs(v_c.y()) > DOUBLE_GAP);

        double k = v_p.y() / v_c.y();
        if((k < 0) || (k > 1)) return false;
        return true;
    }

    assert(fabs(v_c.z()) > DOUBLE_GAP);

    double k = v_p.z() / v_c.z();
    if((k < 0) || (k > 1)) return false;
    return true;
}

bool Geometry_Object::check_intersection(const point &p1, const point &p2) {
    return is_points_match(p1, p2);
}




//------------------------------Geometry_Objects_Storage---------------------------

Geometry_Object_Storage::Geometry_Object_Storage(const std::vector<Undefined_Object>& undef_objects) {
    for(size_t i = 0; i < undef_objects.size(); ++i) {
        const Undefined_Object& cur_obj = undef_objects[i];

        if(is_points_match(cur_obj.p1(), cur_obj.p2())) {

            if(is_points_match(cur_obj.p1(), cur_obj.p3())) {
                Object_Point p(cur_obj.p1(), i);
                obj_point_storage_.push_back(p);
                continue;
            }

            Object_Cut c(Cut(cur_obj.p1(), cur_obj.p3()), i);
            obj_cut_storage_.push_back(c);
            continue;;
        }

        if((is_points_match(cur_obj.p1(), cur_obj.p3())) ||
           (is_points_match(cur_obj.p2(), cur_obj.p3()))) {
            Object_Cut c(Cut(cur_obj.p1(), cur_obj.p2()), i);
            obj_cut_storage_.push_back(c);
            continue;;
        }

        Object_Triangle t(Triangle(cur_obj.p1(), cur_obj.p2(), cur_obj.p3()), i);
        obj_triangle_storage_.push_back(t);
    }

}




//-------------------------------------Intersection_Finder------------------------

Intersection_Finder::Intersection_Finder(Geometry_Object_Storage&& objects):
    num_of_objects_(objects.capacity()),
    objects_(objects)
{
    intersection_flags_.reserve(num_of_objects_);
    for(size_t i = 0; i < num_of_objects_; ++i) {
        intersection_flags_.push_back(false);
    }
}

Objects_and_Intersections Intersection_Finder::compute_intersections() {

    std::vector<Geometry_Object*> p_objects(num_of_objects_);
    size_t i = 0, j = 0;
    while(i < objects_.triangles().size()) {
        p_objects[j] = &((objects_.triangles())[i]);
        ++i;
        ++j;
    }
    i = 0;
    while(i < objects_.cuts().size()) {
        p_objects[j] = &((objects_.cuts())[i]);
        ++i;
        ++j;
    }
    i = 0;
    while(i < objects_.points().size()) {
        p_objects[j] = &((objects_.points())[i]);
        ++i;
        ++j;
    }
    assert(j == num_of_objects_);

    compute_intersections_recursive_algorithm(p_objects);

    std::cout << iter1 << " " << iter2 << " " << iter3 << " " << iter4 << std::endl;

    Objects_and_Intersections answer(std::move(objects_),
                                     std::move(intersection_flags_));
    return answer;
}

void Intersection_Finder::compute_intersections_recursive_algorithm(
        const std::vector<Geometry_Object*>& p_objects)
{
    if(p_objects.size() > 1) {
        Geometry_Object* root_object = p_objects[0];

        if(typeid(*root_object) == typeid(Object_Triangle)) {
            const Object_Triangle* root_t = static_cast<Object_Triangle*>(root_object);

            root_triangle_case(p_objects, root_t);
        }

        else if(typeid(*root_object) == typeid(Object_Cut)) {
            const Object_Cut* root_c = static_cast<Object_Cut*>(root_object);

            root_cut_case(p_objects, root_c);
        }

        else {
            assert(typeid(*root_object) == typeid(Object_Point));
            const Object_Point* root_p = static_cast<Object_Point*>(root_object);

            root_point_case(p_objects, root_p);
        }
    }
}

void Intersection_Finder::root_triangle_case(const std::vector<Geometry_Object*>& p_objects,
                                             const Object_Triangle* root_t) {

    const Plane& pl = root_t->pl();
    std::vector<Geometry_Object*> p_objects_new1;
    std::vector<Geometry_Object*> p_objects_new2;

    for(size_t i = 1; i < p_objects.size(); ++i) {
        Geometry_Object* cur_obj = p_objects[i];

        if(typeid(*cur_obj) == typeid(Object_Triangle)) {
            const Object_Triangle* t = static_cast<Object_Triangle*>(cur_obj);
            int k = pl.triangle_side_plane(*t);

            if(k != 1) p_objects_new2.push_back(cur_obj);
            if(k != -1) p_objects_new1.push_back(cur_obj);

            if(k == 0) {
                if(Geometry_Object::check_intersection(*root_t, *t)) {
                    intersection_flags_[p_objects[0]->number()] = true;
                    intersection_flags_[cur_obj->number()] = true;
                }
            }

            continue;
        }

        if(typeid(*cur_obj) == typeid(Object_Cut)) {
            const Object_Cut* c = static_cast<Object_Cut*>(cur_obj);
            int k = pl.cut_side_plane(*c);

            if(k != 1) p_objects_new2.push_back(cur_obj);
            if(k != -1) p_objects_new1.push_back(cur_obj);

            if(k == 0) {
                if(Geometry_Object::check_intersection(*root_t, *c)) {
                    intersection_flags_[p_objects[0]->number()] = true;
                    intersection_flags_[cur_obj->number()] = true;
                }
            }

            continue;
        }

        assert(typeid(*cur_obj) == typeid(Object_Point));
        const Object_Point* p = static_cast<Object_Point*>(cur_obj);
        int k = pl.point_side_plane(*p);

        if(k != 1) p_objects_new2.push_back(cur_obj);
        if(k != -1) p_objects_new1.push_back(cur_obj);

        if(k == 0) {
            if(Geometry_Object::check_intersection(*root_t, *p)) {
                intersection_flags_[p_objects[0]->number()] = true;
                intersection_flags_[cur_obj->number()] = true;
            }
        }
    }

    ++iter1;

    if(p_objects_new1.size() == (p_objects.size() - 1)) {
        ++iter2;
        compute_intersections_recursive_algorithm(p_objects_new1);
    }
    else if (p_objects_new2.size() == (p_objects.size() - 1)) {
        ++iter3;
        compute_intersections_recursive_algorithm(p_objects_new2);
    }
    else {
        ++iter4;
        compute_intersections_recursive_algorithm(p_objects_new1);
        compute_intersections_recursive_algorithm(p_objects_new2);
    }
}

void Intersection_Finder::root_cut_case(const std::vector<Geometry_Object*>& p_objects,
                                        const Object_Cut* root_c) {
    std::vector<Geometry_Object*> p_objects_new;

    for(size_t i = 1; i < p_objects.size(); ++i) {
        Geometry_Object* cur_obj = p_objects[i];
        p_objects_new.push_back(cur_obj);

        if(typeid(*cur_obj) == typeid(Object_Triangle)) {
            const Object_Triangle* t = static_cast<Object_Triangle*>(cur_obj);

            if(Geometry_Object::check_intersection(*root_c, *t)) {
                intersection_flags_[p_objects[0]->number()] = true;
                intersection_flags_[cur_obj->number()] = true;
            }

            continue;
        }

        if(typeid(*cur_obj) == typeid(Object_Cut)) {
            const Object_Cut* c = static_cast<Object_Cut*>(cur_obj);

            if(Geometry_Object::check_intersection(*root_c, *c)) {
                intersection_flags_[p_objects[0]->number()] = true;
                intersection_flags_[cur_obj->number()] = true;
            }

            continue;
        }

        assert(typeid(*cur_obj) == typeid(Object_Point));
        const Object_Point* p = static_cast<Object_Point*>(cur_obj);

        if(Geometry_Object::check_intersection(*root_c, *p)) {
            intersection_flags_[p_objects[0]->number()] = true;
            intersection_flags_[cur_obj->number()] = true;
        }
    }

    compute_intersections_recursive_algorithm(p_objects_new);
}

void Intersection_Finder::root_point_case(const std::vector<Geometry_Object*>& p_objects,
                                          const Object_Point* root_p) {
    std::vector<Geometry_Object*> p_objects_new;

    for(size_t i = 1; i < p_objects.size(); ++i) {
        Geometry_Object* cur_obj = p_objects[i];
        p_objects_new.push_back(cur_obj);

        if(typeid(*cur_obj) == typeid(Object_Triangle)) {
            const Object_Triangle* t = static_cast<Object_Triangle*>(cur_obj);

            if(Geometry_Object::check_intersection(*root_p, *t)) {
                intersection_flags_[p_objects[0]->number()] = true;
                intersection_flags_[cur_obj->number()] = true;
            }

            continue;
        }

        if(typeid(*cur_obj) == typeid(Object_Cut)) {
            const Object_Cut* c = static_cast<Object_Cut*>(cur_obj);

            if(Geometry_Object::check_intersection(*root_p, *c)) {
                intersection_flags_[p_objects[0]->number()] = true;
                intersection_flags_[cur_obj->number()] = true;
            }

            continue;
        }

        assert(typeid(*cur_obj) == typeid(Object_Point));
        const Object_Point* p = static_cast<Object_Point*>(cur_obj);

        if(Geometry_Object::check_intersection(*root_p, *p)) {
            intersection_flags_[p_objects[0]->number()] = true;
            intersection_flags_[cur_obj->number()] = true;
        }
    }

    compute_intersections_recursive_algorithm(p_objects_new);
}


} //namespace geometry
