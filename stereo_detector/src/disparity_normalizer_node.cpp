#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h> // Para publicar imágenes de ROS
#include <stereo_msgs/DisparityImage.h> // Para suscribirse a la disparidad de la ZED
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> // Para cv::imshow y cv::waitKey
#include <cmath> // Para std::isfinite

// Declaración global de los publishers para que sean accesibles desde disparityCallback
ros::Publisher normalized_disparity_pub;
ros::Publisher colored_disparity_pub;

void disparityCallback(const stereo_msgs::DisparityImageConstPtr& msg) {
    try {
        // Convertir imagen de ROS a OpenCV float32
        // Usamos toCvCopy con msg->image directamente.
        // stereo_msgs::DisparityImage::image es de tipo sensor_msgs::Image
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            // CORRECCIÓN AQUÍ: Usar toCvCopy directamente con msg->image
            cv_ptr = cv_bridge::toCvCopy(msg->image, "32FC1");
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat original_disparity = cv_ptr->image;

        std::cout << "\n[INFO] --- Disparity Image Info ---" << std::endl;
        std::cout << "Encoding: " << msg->image.encoding << std::endl;
        std::cout << "Size: " << original_disparity.cols << " x " << original_disparity.rows << std::endl;
        std::cout << "Type (Original): " << original_disparity.type() << " | Depth: " << original_disparity.depth() << " | Channels: " << original_disparity.channels() << std::endl;

        // Mostrar estadísticas crudas
        double min_raw, max_raw;
        cv::minMaxLoc(original_disparity, &min_raw, &max_raw);
        std::cout << "[INFO] Raw disparity: min=" << min_raw << " max=" << max_raw
                  << " mean=" << cv::mean(original_disparity)[0] << std::endl;

        // Visualizar la imagen de disparidad original (será negra si no está normalizada)
        cv::imshow("Original Disparity", original_disparity);

        // Clonar y limpiar (eliminar NaN e Inf)
        cv::Mat cleaned_disparity = original_disparity.clone();
        for (int y = 0; y < cleaned_disparity.rows; ++y) {
            for (int x = 0; x < cleaned_disparity.cols; ++x) {
                float& val = cleaned_disparity.at<float>(y, x);
                if (!std::isfinite(val)) {
                    val = 0.0f; // Asignar 0 a valores no finitos (NaN, Inf)
                }
                // Si quisieras también ignorar valores negativos de disparidad:
                // if (!std::isfinite(val) || val < 0.0f) val = 0.0f;
            }
        }

        double min_clean, max_clean;
        cv::minMaxLoc(cleaned_disparity, &min_clean, &max_clean);
        std::cout << "[INFO] Cleaned disparity: min=" << min_clean << " max=" << max_clean
                  << " mean=" << cv::mean(cleaned_disparity)[0] << std::endl;

        // Visualizar la imagen de disparidad limpia (seguirá siendo negra si no está normalizada)
        cv::imshow("Cleaned Disparity", cleaned_disparity);

        // Normalizar a 0-255 y convertir a CV_8UC1
        cv::Mat normalized_disparity;
        // Solo normalizar si hay un rango válido de valores
        if (max_clean > min_clean) {
            cv::normalize(cleaned_disparity, normalized_disparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        } else {
            // Si todos los valores son iguales (ej. todo 0.0f), crea una imagen negra
            normalized_disparity = cv::Mat::zeros(cleaned_disparity.size(), CV_8UC1);
        }
        

        double min_norm, max_norm;
        cv::minMaxLoc(normalized_disparity, &min_norm, &max_norm);
        std::cout << "[INFO] Normalized disparity: min=" << min_norm << " max=" << max_norm
                  << " mean=" << cv::mean(normalized_disparity)[0] << std::endl;

        // Visualizar la imagen de disparidad normalizada
        cv::imshow("Normalized Disparity", normalized_disparity);

        // Aplicar colormap (JET es una buena opción)
        cv::Mat colored_disparity;
        cv::applyColorMap(normalized_disparity, colored_disparity, cv::COLORMAP_JET);

        // Visualizar la imagen de disparidad coloreada
        cv::imshow("Colored Disparity", colored_disparity);

        // --- Publicar las imágenes procesadas en ROS ---
        // Usamos el mismo header que el mensaje original para la estampilla de tiempo y el frame_id

        // 1. Publicar la imagen normalizada (mono8)
        sensor_msgs::ImagePtr normalized_msg = cv_bridge::CvImage(msg->header, "mono8", normalized_disparity).toImageMsg();
        normalized_disparity_pub.publish(normalized_msg);

        // 2. Publicar la imagen coloreada (bgr8, ya que OpenCV usa BGR por defecto)
        sensor_msgs::ImagePtr colored_msg = cv_bridge::CvImage(msg->header, "bgr8", colored_disparity).toImageMsg();
        colored_disparity_pub.publish(colored_msg);

        // Esto es necesario para que las ventanas de cv::imshow se actualicen
        cv::waitKey(1); 

    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("Standard exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "disparity_normalizer_node");
    ros::NodeHandle nh;

    // Crear los publishers
    // El tamaño del buffer (1) es suficiente para un flujo de imágenes continuo
    normalized_disparity_pub = nh.advertise<sensor_msgs::Image>("disparity/normalized", 1);
    colored_disparity_pub = nh.advertise<sensor_msgs::Image>("disparity/colored", 1);

    // Suscribirse al tópico de disparidad de la ZED
    ros::Subscriber sub = nh.subscribe("/zed/zed_node/disparity/disparity_image", 1, disparityCallback);

    ROS_INFO("Disparity normalizer node started. Publishing to /disparity/normalized and /disparity/colored");

    ros::spin(); // Mantener el nodo corriendo y procesando callbacks

    // Limpiar ventanas de OpenCV al salir
    cv::destroyAllWindows();
    return 0;
}