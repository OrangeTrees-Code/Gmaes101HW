#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

std::vector<cv::Point2f> control_points;
cv::Point2f lerp(cv::Point2f a, cv::Point2f b, float t)
{
    return a + t * (b - a);
}

void swap_point(cv::Point2f& a,cv::Point2f& b)
{
    cv::Point2f temp = a;
    a = b;
    b = temp;
}

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
                  << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                     3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1)
    {
        return control_points[0];
    }
    std::vector<cv::Point2f> result{};
    int length = control_points.size();
    for (int i = 0; i < length - 1; i++)
    {
        result.emplace_back(lerp(control_points[i], control_points[i + 1], t));
    }
    return recursive_bezier(result, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    for (float t = 0; t < 1.0f; t += 0.00001)
    {
        auto point = recursive_bezier(control_points, t);
        for (int i = static_cast<int>(point.x) - 1; i < static_cast<int>(point.x) + 1;++i)
        {
            for (int j = static_cast<int>(point.y) - 1; j < static_cast<int>(point.y) + 1; ++j)
            {
                if(i>0&&i<window.cols&&j>0&&j<window.rows)
                {
                    float d = std::sqrtf(std::powf((i + 0.5 - point.x), 2) + std::powf((j + 0.5 - point.y), 2));
                    window.at<cv::Vec3b>(j,i)[1] = 255 * std::max(0.0f, 1 - d / 2);
                }
            }
        }
        // window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        // window.at<cv::Vec3b>(point.y, point.x+1)[1] = 255;
    }
    // recursive Bezier algorithm.
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27)
    {
        for (auto &point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4)
        {
            swap(control_points[2], control_points[3]);
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
