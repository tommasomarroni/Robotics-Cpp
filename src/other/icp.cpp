#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <Eigen/Eigen>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include "other/icp.h"

#define DELAY_DRAW 1000

// TODO put const where necessary
// TODO make parent class for drawing and so on
// TODO pass by ref when necessary

using namespace ICP_2D;

icp_svd::icp_svd(std::vector<Eigen::Vector2f> Q_, std::vector<Eigen::Vector2f> P_, float epsilon_, bool draw_, bool verbose_,  std::array<float, 2> x_lim_, std::array<float, 2> y_lim_, int window_x_, int window_y_): Q(Q_), P(P_), epsilon(epsilon_), draw(draw_), verbose(verbose_), x_lim(x_lim_), y_lim(y_lim_), window_x(window_x_), window_y(window_y_) {
    if (Q_.size() != P_.size()) exit(EXIT_FAILURE);
    image = draw_init();
}

Eigen::Matrix3f icp_svd::start() {

    std::vector<Eigen::Vector2f> Q_centered;
    std::vector<Eigen::Vector2f> P_centered;
    std::vector<Eigen::Vector2f> P_iter = P;

    Eigen::Vector2f center_of_Q;
    Eigen::Vector2f center_of_P;

    std::vector<int> correspondences;
    Eigen::Matrix2f cov;
    Eigen::JacobiSVD<Eigen::Matrix2f> svd;

    Eigen::Matrix2f R;
    Eigen::Vector2f t;
    Eigen::Matrix3f T_temp = Eigen::Matrix3f::Identity();

    int iter = 0;
    float error = std::numeric_limits<float>::max();

    while (error > epsilon) {
        Q_centered.clear();
        P_centered.clear();
        correspondences.clear();
        cov = Eigen::Matrix2f::Zero();
        T_temp = Eigen::Matrix3f::Identity();
        error = 0;

        // Step 1: center data
        center_of_Q = center_data(Q, Q_centered);
        center_of_P = center_data(P_iter, P_centered);

        // Step 2: compute correspondences
        compute_correspondences(Q_centered, P_centered, correspondences);

        // Step 3: compute cov
        compute_cov(Q_centered, P_centered, Eigen::Vector2f(0.0, 0.0), Eigen::Vector2f(0.0, 0.0), correspondences, cov);

        // Step 4: perform SVD and compute coordinate transformation rotation matrix and translation vector
        svd = Eigen::JacobiSVD<Eigen::Matrix2f>(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R = svd.matrixU()*(svd.matrixV().transpose());
        t = center_of_Q - R*center_of_P;

        // Draw
        if (draw) {
            draw_points(Q, P_iter);
            draw_correspondences(Q, P_iter, correspondences);
        }

        // Step 5: set new P by performing coordinate transformation
        for (int i = 0; i < P_iter.size(); i++) P_iter[i] = R*P_iter[i] + t;

        T_temp.block<2,2>(0,0) = R;
        T_temp.block<2,1>(0,2) = t;
        T = T_temp*T;

        for (int i = 0; i < P.size(); i++) {
            error += sqrtf(pow(Q_centered[i][0] - P_centered[i][0], 2) + pow(Q_centered[i][1] - P_centered[i][1], 2));
        }

        iter++;
    }

    if (verbose) {
        std::cout << "\n[!] Performed " << iter << " iterations." << std::endl;
        std::cout << "\n[!] Final error: " << error << std::endl;
        std::cout << "\n[!] Homogeneous coordinate transformation:\n" << T << std::endl;
        std::cout << "\n[!] Rotation matrix:\n" << T.block<2,2>(0,0) << std::endl;
        std::cout << "\n[!] Translation vector:\n" << T.block<2,1>(0,2) << std::endl;
    }

    if (draw) {
        draw_points(Q, P_iter);
        cv::waitKey(0);
    }

    return T;
}

Eigen::Vector2f icp_svd::center_data(std::vector<Eigen::Vector2f>& PC, std::vector<Eigen::Vector2f>& PC_centered) {
    Eigen::Vector2f mean(0.0, 0.0);

    for (Eigen::Vector2f point : PC) {
        mean += point;
    }
    mean = mean/PC.size();
    for (Eigen::Vector2f point : PC) {
        PC_centered.push_back(point - mean);
    }

    return mean;
}

void icp_svd::compute_correspondences(std::vector<Eigen::Vector2f>& Q_, std::vector<Eigen::Vector2f>& P_, std::vector<int>& correspondences_) {
    float min_dist, dist, index;

    for (Eigen::Vector2f point_P : P_) {
        min_dist = std::numeric_limits<float>::max();
        for (int i = 0; i < Q_.size(); i++) {
            dist = sqrtf(pow(Q_[i][0] - point_P[0], 2) + pow(Q_[i][1] - point_P[1], 2));
            if (dist < min_dist) {
                min_dist = dist;
                index = i;
            }
        }
        correspondences_.push_back(index);
    }
}

void icp_svd::compute_cov(std::vector<Eigen::Vector2f>& Q_, std::vector<Eigen::Vector2f>& P_, Eigen::Vector2f mu_Q_, Eigen::Vector2f mu_P_, std::vector<int>& correspondences_, Eigen::Matrix2f& cov_) {
    for (int i = 0; i < Q_.size(); i++) {
        cov_ += (Q_[correspondences_[i]] - mu_Q_)*((P_[i] - mu_P_).transpose());
    }
    cov_ = cov_/Q_.size();
}

cv::Mat icp_svd::draw_init() {
    cv::Mat image(window_y, window_x, CV_8UC3, cv::Scalar(0, 0, 0));

    if (!image.data) {
        std::cout <<  "Could not open or find the image!" << std::endl ;
        exit(EXIT_FAILURE);
    }

    return image;
}

void icp_svd::draw_points(std::vector<Eigen::Vector2f>& Q_, std::vector<Eigen::Vector2f>& P_) {
    image = cv::Mat::zeros(image.size(), image.type());

    for (Eigen::Vector2f point : Q_) {
        cv::Point centerStart(scale(point)[0], scale(point)[1]);
        cv::circle(image, centerStart, 2, cv::Scalar(0, 0, 180), cv::FILLED); // CV_FILLED
    }
    for (Eigen::Vector2f point : P_) {
        cv::Point centerStart(scale(point)[0], scale(point)[1]);
        cv::circle(image, centerStart, 2, cv::Scalar(180, 0, 0), cv::FILLED); // CV_FILLED
    }

    cv::imshow("ICP: SVD", image);
    cv::waitKey(DELAY_DRAW);
}

void icp_svd::draw_correspondences(std::vector<Eigen::Vector2f>& Q_, std::vector<Eigen::Vector2f>& P_, std::vector<int>& correspondences_) {
    for (int i = 0; i < P_.size(); i++) {
        cv::Point p1(scale(P_[i])[0], scale(P_[i])[1]), p2(scale(Q_[correspondences_[i]])[0], scale(Q_[correspondences_[i]])[1]);
        cv::line(image, p1, p2, cv::Scalar(240, 240, 240), 1);
    }

    cv::imshow("ICP: SVD", image);
    cv::waitKey(DELAY_DRAW);
}
