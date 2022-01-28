#ifndef LABEL_READ_H
#define LABEL_READ_H
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include "box.hpp"

typedef std::string string;

class label_read {
private:
    string label_file;
    std::vector<Label> vehicle_labels;
    std::vector<Label> object_labels;
public:
    label_read(string label_file_);
    ~label_read();
    std::vector<Label> get_vehicles();
    std::vector<Label> get_objects();
};
/*****************************************************
*功能：读取label文件中的目标真值，排除被截断的车辆
*输入：
*label_file_: label文件名
*****************************************************/
label_read::label_read(string label_file_) : label_file(label_file_) {
    std::ifstream input_file(label_file.c_str(), std::ifstream::in);
    if(!input_file.is_open()) {std::cout << "Detection File doesn't exist."; return;}
    string line;
    string obj_class;
    while(getline(input_file, line)) {
        std::istringstream iss(line);
        iss >> obj_class;
        if(obj_class == "DontCare") continue;
        Label label;
        iss >> label.truncation >> label.occlusion >> label.alpha;
        iss >> label.box2d.xmin >> label.box2d.ymin >> label.box2d.xmax >> label.box2d.ymax;
        iss >> label.box3d.height >> label.box3d.width >> label.box3d.length;
        iss >> label.box3d.pos.x >> label.box3d.pos.y >> label.box3d.pos.z;
        iss >> label.alpha;
        label.obj_class = obj_class;
        label.box2d.obj_class = obj_class;
        label.box3d.obj_class = obj_class;

        if(obj_class.compare("Car") == 0 || obj_class.compare("Van") == 0 || obj_class.compare("Truck") == 0) {
            if(label.truncation != 0) continue;
            label.box2d.id = vehicle_labels.size();
            vehicle_labels.push_back(label);
        } else {
            label.box2d.id = object_labels.size();
            label.box3d.id = object_labels.size();
            object_labels.push_back(label);
        }
    }
    input_file.close();
}
/*****************************************************
*功能：析构函数
*****************************************************/
label_read::~label_read() {}

/*****************************************************
*功能：获取车辆的label真值
*****************************************************/
std::vector<Label> label_read::get_vehicles() {
    return vehicle_labels;
}

/*****************************************************
*功能：获取其他物体的label真值
*****************************************************/
std::vector<Label> label_read::get_objects() {
    return object_labels;
}
#endif