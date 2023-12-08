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
    // ����ͼ������
    double alpha = 0.6;
    cv::Mat adjustedImage = inputImage * alpha;

    // ��ͼ��ָ�Ϊ�졢�̡�������ͨ��
    std::vector<cv::Mat> channels;
    cv::split(adjustedImage, channels);

    // �Ժ���ͨ�����
    cv::Mat diffImage;
    cv::subtract(channels[0], channels[1], diffImage);

    // �Բ�ֵͼ�������ֵ����ʶ����ɫ
    cv::Mat mask;
    cv::threshold(diffImage, mask, 40, 255, cv::THRESH_BINARY);

    // Ѱ������
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // ��ȡÿ����������Ӿ���
    std::vector<cv::Rect> boundingRects;
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::Rect boundingRect = cv::boundingRect(contours[i]);
        boundingRects.push_back(boundingRect);
    }

    // �����θ���λ�úʹ�С���о���
    double distanceThreshold = 50.0;
    std::vector<std::vector<cv::Rect>> clusteredRects = clusterRects(boundingRects, distanceThreshold);

    // ��ͼ���ϻ��ƺϲ���ľ��Σ������λ����Ϣ
    for (const auto& cluster : clusteredRects) {
        if (!cluster.empty()) {
            cv::Rect mergedBoundingRect = cluster[0];
            for (size_t i = 1; i < cluster.size(); ++i) {
                mergedBoundingRect |= cluster[i];
            }
            cv::rectangle(inputImage, mergedBoundingRect, cv::Scalar(0, 255, 0), 2);

            // ���λ�ò�����Ϣ
            std::cout << "Detected light strip at position: x = " << mergedBoundingRect.x << ", y = " << mergedBoundingRect.y << std::endl;
        }
    }

    // ��ͼ���ϻ��ƺϲ���ľ��Σ������λ����Ϣ�ͽǶ���Ϣ
    for (const auto& cluster : clusteredRects) {
        if (!cluster.empty()) {
            cv::Rect mergedBoundingRect = cluster[0];
            for (size_t i = 1; i < cluster.size(); ++i) {
                mergedBoundingRect |= cluster[i];
            }
            cv::rectangle(inputImage, mergedBoundingRect, cv::Scalar(0, 255, 0), 2);

            // ��ȡ��ת����
            std::vector<cv::Point> contour;
            contour.emplace_back(mergedBoundingRect.tl());
            contour.emplace_back(mergedBoundingRect.br());
            cv::RotatedRect rotatedRect = cv::minAreaRect(contour);

            // ���λ�ò�����Ϣ
            std::cout << "Detected light strip at position: x = " << mergedBoundingRect.x << ", y = " << mergedBoundingRect.y << std::endl;

            // ����Ƕ���Ϣ
            std::cout << "Angle of the detected light strip: " << rotatedRect.angle << " degrees" << std::endl;
        }
    }


    // ��ʾʶ����ͼ��
    cv::imshow("Detected Colors", inputImage);
    cv::waitKey(33.3);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "color_detection_node");
    ros::NodeHandle nh;

    std::string videoPath = "/home/bowen/aim/stream.mp4"; //���������Լ�����Ƶ·��
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
