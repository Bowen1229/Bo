#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <cmath>
#include <iostream>

std::vector<std::vector<cv::Rect>> clusterRects(const std::vector<cv::Rect>& rects, double distance_threshold) {
    std::vector<std::vector<cv::Rect>> clusteredRects;
    std::vector<bool> visited(rects.size(), false);

    for (size_t i = 0; i < rects.size(); ++i) {
        if (visited[i]) continue;

        std::vector<cv::Rect> cluster;
        cluster.push_back(rects[i]);
        visited[i] = true;

        for (size_t j = i + 1; j < rects.size(); ++j) {
            if (!visited[j]) {
                double dist = std::sqrt(std::pow(rects[i].x - rects[j].x, 2) + std::pow(rects[i].y - rects[j].y, 2));
                if (dist < distance_threshold) {
                    cluster.push_back(rects[j]);
                    visited[j] = true;
                }
            }
        }

        clusteredRects.push_back(cluster);
    }

    return clusteredRects;
}

void processFrame(const cv::Mat& inputImage) {
    // 降低图像亮度
    double alpha = 0.6;
    cv::Mat adjustedImage = inputImage * alpha;

    // 将图像分割为红、绿、蓝三个通道
    std::vector<cv::Mat> channels;
    cv::split(adjustedImage, channels);

    // 对红蓝通道相减
    cv::Mat diffImage;
    cv::subtract(channels[0], channels[1], diffImage);

    // 对差值图像进行阈值化来识别蓝色
    cv::Mat mask;
    cv::threshold(diffImage, mask, 40, 255, cv::THRESH_BINARY);

    // 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 获取每个轮廓的外接矩形
    std::vector<cv::Rect> boundingRects;
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Rect boundingRect = cv::boundingRect(contours[i]);
        boundingRects.push_back(boundingRect);
    }

    // 将矩形根据位置和大小进行聚类
    double distanceThreshold = 50.0;
    std::vector<std::vector<cv::Rect>> clusteredRects = clusterRects(boundingRects, distanceThreshold);

    // 在图像上绘制合并后的矩形，并输出位置信息
    for (const auto& cluster : clusteredRects) {
        if (!cluster.empty()) {
            cv::Rect mergedBoundingRect = cluster[0];
            for (size_t i = 1; i < cluster.size(); ++i) {
                mergedBoundingRect |= cluster[i];
            }
            cv::rectangle(inputImage, mergedBoundingRect, cv::Scalar(0, 255, 0), 2);

            // 输出位置参数信息
            std::cout << "Detected light strip at position: x = " << mergedBoundingRect.x << ", y = " << mergedBoundingRect.y << std::endl;
        }
    }

    // 在图像上绘制合并后的矩形，并输出位置信息和角度信息
    for (const auto& cluster : clusteredRects) {
        if (!cluster.empty()) {
            cv::Rect mergedBoundingRect = cluster[0];
            for (size_t i = 1; i < cluster.size(); ++i) {
                mergedBoundingRect |= cluster[i];
            }
            cv::rectangle(inputImage, mergedBoundingRect, cv::Scalar(0, 255, 0), 2);

            // 获取旋转矩形
            std::vector<cv::Point> contour;
            contour.emplace_back(mergedBoundingRect.tl());
            contour.emplace_back(mergedBoundingRect.br());
            cv::RotatedRect rotatedRect = cv::minAreaRect(contour);

            // 输出位置参数信息
            std::cout << "Detected light strip at position: x = " << mergedBoundingRect.x << ", y = " << mergedBoundingRect.y << std::endl;

            // 输出角度信息
            std::cout << "Angle of the detected light strip: " << rotatedRect.angle << " degrees" << std::endl;
        }
    }


    // 显示识别后的图像
    cv::imshow("Detected Colors", inputImage);
    cv::waitKey(33.3);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "color_detection_node");
    ros::NodeHandle nh;

    std::string videoPath = "/home/bowen/aim/stream.mp4"; //这里是我自己的视频路径
    cv::VideoCapture cap(videoPath);
    cv::Mat frame;

    while (true) {
        cap.read(frame);
        if (frame.empty()) break;

        processFrame(frame);
        ros::spinOnce();
    }

    return 0;
}
