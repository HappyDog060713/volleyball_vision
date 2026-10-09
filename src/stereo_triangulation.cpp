#include "volleyball_vision/stereo_triangulation.hpp"
#include <cmath>
#include <limits>
#include <iostream>

namespace volleyball_vision {

StereoTriangulator::StereoTriangulator() {}

void StereoTriangulator::setProjectionMatrices(const cv::Mat& P1, const cv::Mat& P2, double baseline) {
    P1_ = P1.clone();
    P2_ = P2.clone();
    baseline_ = baseline;
}

void StereoTriangulator::setEpipolarThreshold(double max_vertical_diff_px) {
    epipolar_threshold_ = max_vertical_diff_px;
}

bool StereoTriangulator::matchAndTriangulate(const std::vector<Detection>& left_dets,
                                             const std::vector<Detection>& right_dets,
                                             TriangulatedPoint& out_point) {
    out_point.is_valid = false;
    if (left_dets.empty() || right_dets.empty() || P1_.empty() || P2_.empty()) {
        return false;
    }

    double best_score = -1.0;
    int best_l = -1;
    int best_r = -1;
    double best_epi_err = 0.0;
    double best_disp = 0.0;

    for (size_t l = 0; l < left_dets.size(); ++l) {
        for (size_t r = 0; r < right_dets.size(); ++r) {
            double epi_err = std::abs(left_dets[l].center.y - right_dets[r].center.y);
            if (epi_err > epipolar_threshold_) {
                continue; // Violates horizontal epipolar line constraint
            }

            double disp = left_dets[l].center.x - right_dets[r].center.x;
            if (disp <= 0.0) {
                continue; // In front of cameras requires positive disparity
            }

            // Score based on confidence and epipolar alignment
            double pair_score = (left_dets[l].confidence + right_dets[r].confidence) - (epi_err * 0.1);
            if (pair_score > best_score) {
                best_score = pair_score;
                best_l = l;
                best_r = r;
                best_epi_err = epi_err;
                best_disp = disp;
            }
        }
    }

    if (best_l < 0 || best_r < 0) {
        return false;
    }

    // Triangulate
    cv::Mat pt_l = (cv::Mat_<double>(2, 1) << left_dets[best_l].center.x, left_dets[best_l].center.y);
    cv::Mat pt_r = (cv::Mat_<double>(2, 1) << right_dets[best_r].center.x, right_dets[best_r].center.y);
    cv::Mat pt_4d;

    cv::triangulatePoints(P1_, P2_, pt_l, pt_r, pt_4d);

    double w = pt_4d.at<double>(3, 0);
    if (std::abs(w) < 1e-6) return false;

    double X = pt_4d.at<double>(0, 0) / w;
    double Y = pt_4d.at<double>(1, 0) / w;
    double Z = pt_4d.at<double>(2, 0) / w;

    // Plausibility check for volleyball court range (2m ~ 35m)
    if (Z < 1.0 || Z > 40.0) {
        return false;
    }

    out_point.point_3d = cv::Point3d(X, Y, Z);
    out_point.disparity = best_disp;
    out_point.epipolar_error = best_epi_err;
    out_point.confidence = (left_dets[best_l].confidence + right_dets[best_r].confidence) * 0.5f;
    out_point.is_valid = true;
    return true;
}

} // namespace volleyball_vision
