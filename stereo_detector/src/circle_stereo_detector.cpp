#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_geometry/stereo_camera_model.h>

using namespace message_filters;

class CircleStereoDetector
{
public:
    CircleStereoDetector(ros::NodeHandle& nh)
        : nh_(nh),
          left_img_sub_(nh_, "/zed/zed_node/left/image_rect_color", 1),
          left_info_sub_(nh_, "/zed/zed_node/left/camera_info", 1),
          right_info_sub_(nh_, "/zed/zed_node/right/camera_info", 1),
          disp_sub_(nh_, "/zed/zed_node/disparity/disparity_image", 1),
          sync_(SyncPolicy(10), left_img_sub_, left_info_sub_, right_info_sub_, disp_sub_)
    {
        sync_.registerCallback(boost::bind(&CircleStereoDetector::callback, this, _1, _2, _3, _4));

        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("stereo_pattern", 1);
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("stereo_pattern_markers", 1);

        ROS_INFO("Circulo + Estereo detector inicializado.");
    }

private:

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image,
        sensor_msgs::CameraInfo,
        sensor_msgs::CameraInfo,
        stereo_msgs::DisparityImage
    > SyncPolicy;

    ros::NodeHandle nh_;

    message_filters::Subscriber<sensor_msgs::Image> left_img_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_sub_;
    message_filters::Subscriber<stereo_msgs::DisparityImage> disp_sub_;
    Synchronizer<SyncPolicy> sync_;

    ros::Publisher cloud_pub_;
    ros::Publisher markers_pub_;

    // ---------------------------------------
    // CALLBACK PRINCIPAL
    // ---------------------------------------
    void callback(
        const sensor_msgs::ImageConstPtr& left_img_msg,
        const sensor_msgs::CameraInfoConstPtr& left_info,
        const sensor_msgs::CameraInfoConstPtr& right_info,
        const stereo_msgs::DisparityImageConstPtr& disp_msg)
    {
        ROS_INFO("➡ Procesando frame...");

        // Convertir imagen izquierda
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(left_img_msg, sensor_msgs::image_encodings::BGR8);
        } catch (...) {
            ROS_ERROR("Error en cv_bridge al convertir left image.");
            return;
        }

        cv::Mat img_color = cv_ptr->image;
        cv::Mat img_gray;
        cv::cvtColor(img_color, img_gray, cv::COLOR_BGR2GRAY);

        // ---------------------------
        // 1) DETECCIÓN DE CÍRCULOS
        // ---------------------------
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(img_gray, circles, cv::HOUGH_GRADIENT,
                         1.2,        // dp
                         20,         // minDist
                         100, 30,    // param1, param2
                         5, 80);     // minRadius, maxRadius

        ROS_INFO("Circulos detectados: %lu", circles.size());

        // ---------------------------
        // 2) CONVERTIR DISPARIDAD
        // ---------------------------
        cv_bridge::CvImageConstPtr disp_ptr;
        try {
            disp_ptr = cv_bridge::toCvCopy(
                disp_msg->image,
                sensor_msgs::image_encodings::TYPE_32FC1
            );
        } catch (...) {
            ROS_ERROR("Error leyendo disparity");
            return;
        }

        cv::Mat disparity = disp_ptr->image;

        if (disparity.empty())
        {
            ROS_ERROR("Disparity vacío.");
            return;
        }
        // ---------------------------
        // 3) PROYECTAR A 3D CON StereoCameraModel
        // ---------------------------
        image_geometry::StereoCameraModel model;
        model.fromCameraInfo(left_info, right_info);

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.header.frame_id = left_img_msg->header.frame_id;

        for (auto& c : circles)
        {
            int u = (int)c[0];
            int v = (int)c[1];

            float d = disparity.at<float>(v, u);
            if (!std::isfinite(d) || d <= 0.0) continue;

            cv::Point3d P;
            model.projectDisparityTo3d(cv::Point2d(u, v), d, P);

            pcl::PointXYZRGB pt;
            pt.x = P.x;
            pt.y = P.y;
            pt.z = P.z;

            cv::Vec3b color = img_color.at<cv::Vec3b>(v, u);
            pt.r = color[2];
            pt.g = color[1];
            pt.b = color[0];

            cloud.push_back(pt);
        }

        ROS_INFO("📌 Puntos 3D generados: %lu", cloud.size());

        // --------------------------------
        // 4) Publicar nube
        // --------------------------------
        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(cloud, msg_cloud);
        msg_cloud.header = left_img_msg->header;
        cloud_pub_.publish(msg_cloud);

        // --------------------------------
        // 5) Publicar marcadores RViz
        // --------------------------------
        visualization_msgs::MarkerArray arr;
        int id = 0;

        for (auto& p : cloud)
        {
            visualization_msgs::Marker m;
            m.header = msg_cloud.header;
            m.id = id++;
            m.type = visualization_msgs::Marker::SPHERE;
            m.action = visualization_msgs::Marker::ADD;

            m.pose.position.x = p.x;
            m.pose.position.y = p.y;
            m.pose.position.z = p.z;
            m.pose.orientation.w = 1.0;

            m.scale.x = m.scale.y = m.scale.z = 0.03;

            m.color.a = 1.0;
            m.color.r = p.r / 255.0;
            m.color.g = p.g / 255.0;
            m.color.b = p.b / 255.0;

            arr.markers.push_back(m);
        }

        markers_pub_.publish(arr);

        ROS_INFO("✅ Frame procesado");
    }
};
