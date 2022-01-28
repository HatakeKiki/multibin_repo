#ifndef NET_OUTPUT_PROCESS_H
#define NET_OUTPUT_PROCESS_H
#include <vector>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <Eigen/Eigen>
class net_output_process {
private:
    std::vector<float> dimensions;
    float heading;
    Eigen::Matrix<float, 3, 4> P;
    std::vector<float> box;
    Eigen::Matrix<float, 3, 8> box_3d_init;
    Eigen::Matrix<float, 3, 1> center_min_error;
    Eigen::Matrix<float, 3, 8> box_3d_comp;
    Eigen::Matrix<float, 3, 8> box_3d_pixel;
    Eigen::Matrix<float, 3, 8> pixel;
    float error_min;
    
public:
    net_output_process() {};
    net_output_process(std::vector<float> dims, float heading_, Eigen::Matrix<float, 3, 4> P_, std::vector<float> box_);
    ~net_output_process() {};
    void init_3d_box_pose();
    float compute_error(Eigen::Matrix<float, 3, 1> center);
    void compute_center_min_error();
    Eigen::Matrix<float, 3, 1> compute_center(Eigen::Matrix<float, 3, 4> points);
    Eigen::Matrix<float, 3, 1> get_center();
    Eigen::Matrix<float, 3, 8> get_box();
    Eigen::Matrix<float, 3, 8> get_box_pixel();
};

net_output_process::net_output_process(std::vector<float> dims, float heading_, Eigen::Matrix<float, 3, 4> P_, std::vector<float> box_) : dimensions(dims), heading(heading_), P(P_), box(box_) {
    center_min_error = Eigen::Matrix<float, 3, 1>::Zero();
    box_3d_init = Eigen::Matrix<float, 3, 8>::Zero();
    error_min = 100000;
    init_3d_box_pose();
    box_3d_comp = box_3d_init;
    compute_center_min_error();
    for(int i = 0; i < 8; i++) {
        box_3d_comp.block<3, 1>(0, i) += center_min_error;
    }
    //std::cout << "box_3d_init: " << std::endl;
    //std::cout << box_3d_init << std::endl;
}

void net_output_process::init_3d_box_pose() {
    Eigen::MatrixXf rotate = Eigen::MatrixXf(3, 3);
    rotate << cos(heading), 0, sin(heading), 0, 1, 0, -sin(heading), 0, cos(heading);
    Eigen::MatrixXf box_3d = Eigen::Matrix<float, 3, 8>::Zero();
    size_t cnt = 0;
    for(int i = -1; i <= 1; i += 2) {
        for(int j = 1; j >= -1; j -= 2) {
            for(int k = 1; k >= -1; k -= 2) {
                box_3d(0, cnt) = dimensions[0] / 2 * j;
                box_3d(1, cnt) = dimensions[1] / 2 * i;
                box_3d(2, cnt) = dimensions[2] / 2 * k;
                cnt++;
            }
        }
    }
    box_3d_init = rotate * box_3d;
}

Eigen::Matrix<float, 3, 1> net_output_process::compute_center(Eigen::Matrix<float, 3, 4> points) {
    float fx = P(0, 0);
    float fy = P(1, 1);
    float u0 = P(0, 2);
    float v0 = P(1, 2);
    float bz = -P(2, 3);
    Eigen::MatrixXf W = Eigen::MatrixXf(4, 3);
    Eigen::MatrixXf y = Eigen::MatrixXf(4, 1);
    W << fx, 0, u0 - box[0],
         fx, 0, u0 - box[1],
         0, fy, v0 - box[2],
         0, fy, v0 - box[3];
         
    for(int i = 0; i < 4; i++) {
        Eigen::Matrix<float, 1, 1> tmp = P.block<1, 3>(i/2 ,0) * points.block<3, 1>(0, i);
        y(i, 0) = box[i] * (points(2, i) - bz) - tmp(0,0) - P(i/2 ,3);
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<float, 4, 4> U = svd.matrixU();
    Eigen::Matrix<float, 3, 3> V = svd.matrixV();
    auto S = svd.singularValues();

    Eigen::MatrixXf S_pinv = Eigen::Matrix<float, 3, 4>::Zero();
    for(int i = 0; i < 3; i++) S_pinv(i, i) = 1/S[i];
    Eigen::Matrix<float, 3, 4> W_pinv = V * S_pinv * U.transpose();
    Eigen::Matrix<float, 3, 1> center = W_pinv * y;
    if(center(2, 0) < 0) {
        center = -center;
    }
    /*
    std::cout <<  "y: " << std::endl << y << std::endl;
    std::cout << "U: " << std::endl << U << std::endl;
    std::cout << "V: " << std::endl << V << std::endl;
    std::cout << "S pinv" << std::endl << S_pinv << std::endl;
    std::cout << "W pinv." << std::endl << W_pinv << std::endl;
    std::cout << "center: " << std::endl << center << std::endl;
    */
    return center;
}

float net_output_process::compute_error(Eigen::Matrix<float, 3, 1> center) {
    Eigen::Matrix<float, 4, 8> box_3d = Eigen::Matrix<float, 4, 8>::Ones();
    box_3d.block<3, 8>(0, 0) = box_3d_init;
    for(int i = 0; i < 8; i++) {
        box_3d.block<3, 1>(0, i) += center;
    }
    pixel = P * box_3d;
    for(int i = 0; i < 8; i++) {
        pixel(0, i) = pixel(0, i) / pixel(2, i);
        pixel(1, i) = pixel(1, i) / pixel(2, i);
        pixel(2, i) = 1;
    }
    std::vector<float> box_new = {pixel.row(0).minCoeff(), pixel.row(0).maxCoeff(), 
                                  pixel.row(1).minCoeff(), pixel.row(1).maxCoeff()};
    
    float error = 0;
    for(int i = 0; i < 4; i++) error += abs(box_new[i] -box[i]);
    /*
    std::cout << "pixel: " << pixel << std::endl;
    std::cout << "new box: " << box_new[0] << ' ' << box_new[1] << ' '  << box_new[2] << ' '  << box_new[3] << std::endl;
    std::cout << "error: " << error << std::endl;
    */
    return error;
}

void net_output_process::compute_center_min_error() {
    std::vector<std::vector<size_t>> candidate = {{0, 1, 2, 3, 4, 5, 6, 7}, {0, 1, 2, 3, 4, 5, 6, 7}}; 
    for(int m = 0; m < 8; m++) {
        for(int n = 0; n < 8; n++) {
            for(int a = 0; a < 8; a++) {
                for(int b = 0; b < 8; b++) {
                    Eigen::Matrix<float, 3, 4> points_3d = Eigen::Matrix<float, 3, 4>::Zero();
                    points_3d.block<3,1>(0,0) = box_3d_init.block<3,1>(0, candidate[1][m]);
                    points_3d.block<3,1>(0,1) = box_3d_init.block<3,1>(0, candidate[1][n]);
                    points_3d.block<3,1>(0,2) = box_3d_init.block<3,1>(0, candidate[1][a]);
                    points_3d.block<3,1>(0,3) = box_3d_init.block<3,1>(0, candidate[0][b]);
                    auto center = compute_center(points_3d);
                    float error = compute_error(center);
                    //std::cout << m << n << a << b << std::endl << center << ' ' << error << std::endl;
                    //std::cout << "points_3d: " << std::endl << points_3d << std::endl;
                    if(error < error_min) {
                        center_min_error = center;
                        error_min = error;
                        box_3d_pixel = pixel;
                    }
                }
            }
        }
    }
}

Eigen::Matrix<float, 3, 1> net_output_process::get_center(){
    return center_min_error;
}

Eigen::Matrix<float, 3, 8> net_output_process::get_box(){
    return box_3d_comp;
}

Eigen::Matrix<float, 3, 8> net_output_process::get_box_pixel(){
    return box_3d_pixel;
}
#endif