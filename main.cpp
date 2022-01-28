#include <fdeep/fdeep.hpp>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <ctime>
#include <algorithm>
#include <math.h>
#include "calib_read.hpp"
#include "label_read.hpp"
#include "net_output_process.hpp"
#include "utils.hpp"

#define FDEEP_FLOAT_TYPE double
#define PI 3.14159265

int main() {
    // load the net model
    const auto model = fdeep::load_model("../model/multibin.json");
    for(int name = 0; name <= 7480; name++) {
        // Read the pointcloud， calibration and 2d detection infos
        string file_name = name_generate(name, 6);
        calib_read calib(base_dir + "calib/" + file_name + ".txt");
        label_read label(base_dir + "label_2/" + file_name + ".txt");
        Calibration calib_velo_2_cam;
        calib_velo_2_cam.P = calib.get_P();

        // get the detection of vehicles and other objects;
        auto vehicle_labels = label.get_vehicles();
        auto object_labels = label.get_objects();
        Boxes2d vehicles;
        Boxes2d objects;

        cv::Mat image = cv::imread(base_dir + "image_2/" + file_name + ".png");
        if(image.empty()) {
            std::cout << "Failed to read the picture!";
            return -1;
        }
        cv::Size dsize = cv::Size(224, 224);
        for(auto& label : vehicle_labels) {
            std::vector<int> box = {(int)label.box2d.xmin, (int)label.box2d.xmax, (int)label.box2d.ymin, (int)label.box2d.ymax};
            cv::Mat patch = image(cv::Rect(box[0], box[2], box[1] - box[0], box[3] - box[2]));
            cv::resize(patch, patch, dsize);
            // Every channel will subtract a fixed value seperately
            // Need to refine the code in fdeep
            const auto input = fdeep::tensor_from_bytes_no_scale(patch.ptr(),
                static_cast<std::size_t>(patch.rows),
                static_cast<std::size_t>(patch.cols),
                static_cast<std::size_t>(patch.channels()));
            // Time cost for one prediction
            // clock_t start_time, end_time;
            // start_time = clock();
            const auto output = model.predict({input});
            // std::cout << fdeep::show_tensor(input) << std::endl;
            // std::cout <<  fdeep::show_tensor(output) << std::endl;

            // end_time = clock();
            // std::cout << "The running time is: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;

            // Add mean dimensions of certain sub-catogory to regressed dimensions output
            std::vector<float> dims_reg = output[0].to_vector();
            std::vector<float> dims_mean;
            if(label.obj_class == "Car") dims_mean = vehicle_mean_dims[0];
            else if (label.obj_class == "Van") dims_mean = vehicle_mean_dims[1];
            else dims_mean = vehicle_mean_dims[2];
            for(size_t i = 0; i < 3; i++) dims_reg[i] += dims_mean[i];

            // Locate the bin which is most possible range for orientation
            std::vector<float> orientation = output[1].to_vector();
            std::vector<float> confidence = output[2].to_vector();
            size_t bin_num = confidence.size();
            size_t max_bin = 0;
            float max_bin_conf = 0;
            for(size_t i = 0; i < bin_num; i++) {
                if(confidence[i] > max_bin_conf) max_bin = i;
                max_bin_conf = std::max(max_bin_conf, confidence[i]);
            }
            std::vector<float> anchor = {orientation[max_bin*2], orientation[max_bin*2 + 1]};

            // Calculate local orientation
            float angle_offset = 0;
            if(anchor[1] > 0) angle_offset = acos(anchor[0] / sqrt(anchor[0] * anchor[0] + anchor[1] * anchor[1]));
            else angle_offset = -acos(anchor[0] / sqrt(anchor[0] * anchor[0] + anchor[1] * anchor[1]));
            float wedge = 2 * PI / bin_num;
            float theta_loc = angle_offset + max_bin * wedge;

            // Calculate global orientation
            float theta_ray = atan(calib_velo_2_cam.P(0,0)/((box[0] + box[1])/2 - calib_velo_2_cam.P(0,2)));
            if(theta_ray < 0) theta_ray += PI;
            float heading = -theta_loc - theta_ray;
            while(heading < -PI) heading += 2 * PI;
            while(heading > PI) heading -= 2 * PI;

            /*
            label.box2d.dims_reg.height = dims_reg[0];
            label.box2d.dims_reg.width = dims_reg[1];
            label.box2d.dims_reg.length = dims_reg[2];
            label.box2d.r_y_reg = heading;
            vehicles.push_back(label.box2d);
            */

            // Post-processing of output to get the distane in 3d world
            // Input clarifying:
            // box: xmin-xmax-ymin-ymax
            // dimensions: length-height-width
            std::vector<float> box_input = {label.box2d.xmin, label.box2d.xmax, label.box2d.ymin, label.box2d.ymax};
            net_output_process post_pro({dims_reg[2], dims_reg[0], dims_reg[1]}, heading, calib_velo_2_cam.P, box_input);

            // result visualization
            Eigen::Matrix<float, 3, 8> pixel = post_pro.get_box_pixel();
            draw_box_3d_pic(image, pixel);
        }
        cv::imshow("img", image);
        cv::waitKey(0);
    }
}