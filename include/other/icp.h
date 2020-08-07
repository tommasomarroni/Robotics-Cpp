#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <Eigen/Eigen>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

namespace ICP_2D {

// TODO: sistemare e usare by reference by pointer when necessary to use less memory and make code cleaner

class icp_svd {
private:
    std::vector<Eigen::Vector2f> Q;
    std::vector<Eigen::Vector2f> P;
    bool draw;
    std::array<float, 2> x_lim;
    std::array<float, 2> y_lim;
    int window_x;
    int window_y;
    float scaling_x;
    float scaling_y;
    cv::Mat image;

public:

    icp_svd(std::vector<Eigen::Vector2f> Q_, std::vector<Eigen::Vector2f> P_, bool draw_ = true, std::array<float, 2> x_lim_ = {-20.0, 20.0}, std::array<float, 2> y_lim_ = {-20.0, 20.0}, int window_x_ = 800, int window_y_ = 800);

    cv::Mat draw_init();

    void make_draw();

    std::vector<int> scale(Eigen::Vector2f point) { return {(window_x/2) - (int) round(point[0]*scaling_x), window_y - ((window_y/2) - (int) round(point[1]*scaling_y))}; };

    Eigen::Vector2f descale(std::vector<int> pos) { return { x_lim[0] - (pos[0]/scaling_x), y_lim[0] - ((window_y - pos[1])/scaling_y)}; };
};

}
