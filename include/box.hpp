#ifndef BOX_H
#define BOX_H
#include <string>
typedef std::string string;

struct Point {
    float x;
    float y;
    float z;
};
struct Dimension {
    float height;
    float width;
    float length;
};
struct Box2d {
    float xmin;
    float ymin;
    float xmax;
    float ymax;
    int id;
    string obj_class;
    Dimension dims_reg;
    float r_y_reg;
};
struct Box3d {
    float length;
    float width;
    float height;
    float heading;
    Point pos;
    float corner_x;
    float corner_y;
    int id;
    string obj_class;
    bool optimized = false;
};
struct Label {
    string obj_class;
    float truncation;
    float occlusion;
    float alpha;
    Box2d box2d;
    Box3d box3d;
};

#endif