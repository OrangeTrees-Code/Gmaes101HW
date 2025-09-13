//
// Created by LEI XU on 4/27/19.
//

#include "Texture.hpp"
#include <eigen3/Eigen/Eigen>
Eigen::Vector3f Texture::getColorBilinear(float u, float v)
{
    u *= width-1;
    v = (1 - v) * (height-1);
    int xLow = std::floorf(u);
    int yLow = std::floorf(v);
    int xHigh = std::min(xLow+1, width - 1);
    int yHigh = std::min(yLow + 1, height - 1);

    Eigen::Vector2i u00_coord(xLow,yLow);
    Eigen::Vector2i u01_coord(xLow,yHigh);
    Eigen::Vector2i u10_coord(xHigh,yLow);
    Eigen::Vector2i u11_coord(xHigh,yHigh);

    auto lerp = [](float x, cv::Vec3b v0, cv::Vec3b v1) -> cv::Vec3b
    { return v0 + ((v1 - v0) * x); };

    auto u00 = image_data.at<cv::Vec3b>(u00_coord.y(), u00_coord.x());
    auto u01 = image_data.at<cv::Vec3b>(u01_coord.y(), u01_coord.x());
    auto u10 = image_data.at<cv::Vec3b>(u10_coord.y(), u10_coord.x());
    auto u11 = image_data.at<cv::Vec3b>(u11_coord.y(), u11_coord.x());

    auto u0 = lerp(u - xLow, u00, u10);
    auto u1 = lerp(u - xLow, u01, u11);
    auto color = lerp(v - yLow, u0, u1);

    return Eigen::Vector3f(color[0], color[1], color[2]);
}