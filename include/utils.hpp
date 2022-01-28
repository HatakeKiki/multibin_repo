#ifndef UTILS_H
#define UTILS_H
#include <string>
typedef std::string string;
typedef std::vector<Box2d> Boxes2d;
typedef std::vector<Box3d> Boxes3d;
const string base_dir = "/media/kiki/520EA48E0EA46CA3/KITTI Dataset/object_detection/training/";
const std::vector<std::vector<float>> vehicle_mean_dims = {{1.52131309, 1.64441358, 3.85728004}, 
                                                           {2.18560847, 1.91077601, 5.08042328}, 
                                                           {3.07044968, 2.62877944, 11.17126338}};
string name_generate(const int frame, const int length) {
    string file_name = std::to_string(frame);
    int cur_length = length-file_name.size();
    for (int a = 0; a < cur_length; a++)
        file_name = "0" + file_name;
    return file_name;
}
void draw_box_3d_pic(cv::Mat& image, Eigen::Matrix<float, 3, 8> pixel) {
    // Headstock of vehicle
    cv::rectangle(image, cv::Point(pixel(0,0), pixel(1,0)), cv::Point(pixel(0,5), pixel(1,5)), cv::Scalar(0, 0, 255), 4);
    cv::line(image, cv::Point(pixel(0,0), pixel(1,0)), cv::Point(pixel(0,5), pixel(1,5)), cv::Scalar(0, 0, 255), 4);
    cv::line(image, cv::Point(pixel(0,1), pixel(1,1)), cv::Point(pixel(0,4), pixel(1,4)), cv::Scalar(0, 0, 255), 4);
    // Tailstock of vehicle
    cv::rectangle(image, cv::Point(pixel(0,2), pixel(1,2)), cv::Point(pixel(0,7), pixel(1,7)), cv::Scalar(255, 0, 0), 4);
    // Bodywork of vehicle
    cv::line(image, cv::Point(pixel(0,0), pixel(1,0)), cv::Point(pixel(0,2), pixel(1,2)), cv::Scalar(255, 0, 0), 4);
    cv::line(image, cv::Point(pixel(0,1), pixel(1,1)), cv::Point(pixel(0,3), pixel(1,3)), cv::Scalar(255, 0, 0), 4);
    cv::line(image, cv::Point(pixel(0,4), pixel(1,4)), cv::Point(pixel(0,6), pixel(1,6)), cv::Scalar(255, 0, 0), 4);
    cv::line(image, cv::Point(pixel(0,5), pixel(1,5)), cv::Point(pixel(0,7), pixel(1,7)), cv::Scalar(255, 0, 0), 4);
}
#endif