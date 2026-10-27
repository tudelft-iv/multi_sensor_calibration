#include <ros/ros.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DisparityNormalizer {
public:
    DisparityNormalizer(ros::NodeHandle& nh)
    {
        sub_ = nh.subscribe("/zed/zed_node/disparity/disparity_image", 1, &DisparityNormalizer::callback, this);
        pub_gray_ = nh.advertise<sensor_msgs::Image>("/disparity_normalized", 1);
        pub_color_ = nh.advertise<sensor_msgs::Image>("/disparity_colored", 1);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_gray_, pub_color_;

    void callback(const stereo_msgs::DisparityImageConstPtr& msg)
    {
        try {
            // Convertir a cv::Mat
            cv_bridge::CvImageConstPtr bridge_ptr = cv_bridge::toCvShare(msg->image, msg);
            cv::Mat disparity = bridge_ptr->image;

            if (disparity.empty() || disparity.type() != CV_32FC1) {
                ROS_WARN("Imagen vacía o tipo inesperado: %d", disparity.type());
                return;
            }

            // Mostrar estadísticas antes de limpiar
            double min_raw, max_raw;
            cv::minMaxLoc(disparity, &min_raw, &max_raw);
            ROS_INFO_STREAM_THROTTLE(2.0, "Disparity raw min: " << min_raw << " max: " << max_raw);

            // Limpiar valores inválidos
            cv::Mat cleaned = disparity.clone();
            cv::patchNaNs(cleaned, 0.0);
            cleaned.setTo(0.0f, cleaned < 0); // quitar negativos

            // Estadísticas después de limpiar
            double min_cleaned, max_cleaned;
            cv::minMaxLoc(cleaned, &min_cleaned, &max_cleaned);
            ROS_INFO_STREAM_THROTTLE(2.0, "Cleaned disparity min: " << min_cleaned << " max: " << max_cleaned);

            // Normalizar a 8 bits
            cv::Mat normalized;
            double max_disp = (max_cleaned > 0.0) ? max_cleaned : 1.0;  // evitar división por cero
            cleaned.convertTo(normalized, CV_8UC1, 255.0 / max_disp);

            // Publicar imagen en gris
            sensor_msgs::ImagePtr msg_gray = cv_bridge::CvImage(msg->image.header, "mono8", normalized).toImageMsg();
            pub_gray_.publish(msg_gray);

            // Colormap
            cv::Mat colored;
            cv::applyColorMap(normalized, colored, cv::COLORMAP_JET);
            sensor_msgs::ImagePtr msg_colored = cv_bridge::CvImage(msg->image.header, "bgr8", colored).toImageMsg();
            pub_color_.publish(msg_colored);

        } catch (const std::exception& e) {
            ROS_ERROR("Error processing disparity image: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "disparity_normalizer");
    ros::NodeHandle nh;
    DisparityNormalizer normalizer(nh);
    ros::spin();
    return 0;
}

