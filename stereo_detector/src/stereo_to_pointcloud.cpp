#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class StereoProjector {
public:
    StereoProjector(ros::NodeHandle& nh)
        : nh_(nh),
          it_(nh),
          left_image_sub_(it_, "/zed/zed_node/left/image_rect_color", 1),
          disparity_sub_(nh_, "/zed/zed_node/disparity/disparity_image", 1),
          left_info_sub_(nh_, "/zed/zed_node/left/camera_info", 1),
          right_info_sub_(nh_, "/zed/zed_node/right/camera_info", 1),
          sync_(SyncPolicy(10), left_image_sub_, disparity_sub_, left_info_sub_, right_info_sub_)
    {
        sync_.registerCallback(boost::bind(&StereoProjector::callback, this, _1, _2, _3, _4));
        pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/stereo_cloud", 1);

        ROS_INFO("Stereo 3D projector iniciado.");
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image,
        stereo_msgs::DisparityImage,
        sensor_msgs::CameraInfo,
        sensor_msgs::CameraInfo> SyncPolicy;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter left_image_sub_;
    
    message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
    message_filters::Synchronizer<SyncPolicy> sync_;
    ros::Publisher pub_cloud_;

    void callback(const sensor_msgs::ImageConstPtr& left_msg,
                  const stereo_msgs::DisparityImageConstPtr& disp_msg,
                  const sensor_msgs::CameraInfoConstPtr& left_info,
                  const sensor_msgs::CameraInfoConstPtr& right_info)
    {
        ROS_INFO("Ejecutando callback...");

        // Convert images
        cv_bridge::CvImageConstPtr left_cv;
        try {
            left_cv = cv_bridge::toCvShare(left_msg, "bgr8");
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Error en cv_bridge: %s", e.what());
            return;
        }

        // Stereo model
        image_geometry::StereoCameraModel model;
        model.fromCameraInfo(*left_info, *right_info);

        int width = disp_msg->image.width;
        int height = disp_msg->image.height;

        ROS_INFO("Ancho: %d; alto: %d ", width, height);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        cloud->header.frame_id = "zed_left_camera_optical_frame";
        cloud->width = width;
        cloud->height = height;
        cloud->is_dense = false;
        cloud->points.resize(width * height);

        int idx = 0;
        
        cv::Mat disp_cv = cv_bridge::toCvCopy(disp_msg->image, "32FC1")->image;
        for (int v = 0; v < height; v++) {
            for (int u = 0; u < width; u++) {
                float disparity_value = disp_cv.at<float>(v, u);


                if (std::isnan(disparity_value) || disparity_value <= disp_msg->min_disparity) {
                    cloud->points[idx].x = cloud->points[idx].y = cloud->points[idx].z = NAN;
                    idx++;
                    continue;
                }

                cv::Point3d pt;
                model.projectDisparityTo3d(cv::Point2d(u, v), disparity_value, pt);

                cloud->points[idx].x = -pt.x;
                cloud->points[idx].y = -pt.y;
                cloud->points[idx].z = -pt.z;

                cv::Vec3b color = left_cv->image.at<cv::Vec3b>(v, u);
                cloud->points[idx].r = color[2];
                cloud->points[idx].g = color[1];
                cloud->points[idx].b = color[0];

                idx++;
            }
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.stamp = left_msg->header.stamp;
        pub_cloud_.publish(output);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereo_to_pointcloud");
    ros::NodeHandle nh;
    StereoProjector projector(nh);
    ros::spin();
    return 0;
}