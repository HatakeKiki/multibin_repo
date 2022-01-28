#ifndef CALIB_READ_H
#define CALIB_READ_H
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <Eigen/Eigen>

typedef std::string string;
typedef Eigen::Matrix<float, 3, 3> Matrix3f;
typedef Eigen::Matrix<float, 3, 4> Matrix34f;
typedef Eigen::Matrix<float, 3, 1> Matrix31f;

struct Calibration {
    Matrix34f P;
    Matrix3f R;
    Matrix31f T;
};
/*************************************************************************
*功能：获取校准参数
*************************************************************************/
class calib_read {
private:
    Matrix34f P2;
    Matrix3f R0_rect;
    Matrix34f Tr_velo_to_cam;
    string cal_file;

public:
    calib_read(string calib_file);
    ~calib_read();
    Matrix34f get_P();
    Matrix3f get_R();
    Matrix34f get_T();
};
/*****************************************************
*功能：读取相机00到另一个相机的坐标转换矩阵包括velo到cam00的
转移矩阵，cam00到cam02的旋转矩阵，cam02的投影矩阵
*输入：
*calib_file: 存放校准参数的文件名
*****************************************************/
calib_read::calib_read(string calib_file) : cal_file(calib_file) {
    std::ifstream input_file(calib_file.c_str(), std::ifstream::in);
    if(!input_file.is_open()) {std::cout << "Failed to open calibration file in: " << calib_file << std::endl; return;}
    // 开始读取参数
    string line;
    while(getline(input_file, line))  {
        std::stringstream iss;
        string data_type;
        iss << line;
        iss >> data_type;
        if(data_type.compare("P2:") == 0) {
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 4; b++)
                    iss >> P2(a, b);
        } else if(data_type.compare("R0_rect:") == 0) {
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++)
                    iss >> R0_rect(a, b);
        } else if(data_type.compare("Tr_velo_to_cam:") == 0) {
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 4; b++)
                    iss >> Tr_velo_to_cam(a, b);
        }
    }
    input_file.close();
}

/*****************************************************
*功能：析构函数
*****************************************************/
calib_read::~calib_read() {}

/*****************************************************
*功能：获取cam02的投影矩阵
*****************************************************/
Matrix34f calib_read::get_P() {
    return P2;
}

/*****************************************************
*功能：获取cam00到cam02的旋转矩阵
*****************************************************/
Matrix3f calib_read::get_R() {
    return R0_rect;
}

/*****************************************************
*功能：获取velo到cam00的转移矩阵
*****************************************************/
Matrix34f calib_read::get_T() {
    return Tr_velo_to_cam;
}

#endif