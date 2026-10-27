#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> // Necesario para cv::applyColorMap y cv::imshow

// Publicador global para que pueda ser accedido desde el callback
ros::Publisher colored_image_pub;

/**
 * @brief Callback para la imagen de la cámara térmica.
 * Convierte la imagen, la normaliza a 8 bits, aplica un colormap y la publica.
 * @param msg Mensaje de imagen entrante de la cámara térmica.
 */
void thermalImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        // Convertir el mensaje de imagen de ROS a una imagen de OpenCV (cv::Mat).
        // Hacemos una copia para poder modificarla.
        // El encoding se mantiene para la conversión inicial, luego lo manejamos.
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat original_image = cv_ptr->image;
    cv::Mat processed_image; // Esta será la imagen de 8 bits en escala de grises antes del colormap

    // 1. Manejar diferentes profundidades y canales de la imagen de entrada
    if (original_image.depth() == CV_16U) {
        // Si la imagen es de 16 bits (CV_16UC1), normalizar a 8 bits (CV_8UC1)
        // cv::NORM_MINMAX mapea el valor mínimo a 0 y el máximo a 255.
        // Si conoces el rango de temperatura específico de tu sensor y quieres mapear
        // solo ese rango (ej. 20C-50C), puedes calcular los valores min/max del sensor
        // y usar: original_image.convertTo(processed_image, CV_8UC1, 255.0/(max_val-min_val), -min_val*255.0/(max_val-min_val));
        cv::normalize(original_image, processed_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    } else if (original_image.depth() == CV_8U) {
        if (original_image.channels() > 1) {
            // Si es una imagen a color de 8 bits (ej. bgr8 pseudo-color), convertir a escala de grises
            cv::cvtColor(original_image, processed_image, cv::COLOR_BGR2GRAY);
        } else {
            // Si ya es una imagen de 8 bits en escala de grises (mono8), usar directamente
            processed_image = original_image.clone();
        }
    } else {
        ROS_ERROR("Unsupported image depth: %d. Expected 8U or 16U for thermal images.", original_image.depth());
        return;
    }

    // 2. Aplicar el Colormap (paleta de colores) a la imagen de 8 bits en escala de grises
    cv::Mat colored_image;
    // Aquí puedes elegir diferentes colormaps disponibles en OpenCV:
    // cv::COLORMAP_AUTUMN, cv::COLORMAP_BONE, cv::COLORMAP_JET (el más común para térmicas),
    // cv::COLORMAP_WINTER, cv::COLORMAP_RAINBOW, cv::COLORMAP_OCEAN, cv::COLORMAP_SUMMER,
    // cv::COLORMAP_SPRING, cv::COLORMAP_COOL, cv::COLORMAP_HSV, cv::COLORMAP_PINK,
    // cv::COLORMAP_HOT, cv::COLORMAP_PARULA, cv::COLORMAP_MAGMA, cv::COLORMAP_INFERNO,
    // cv::COLORMAP_TURBO, cv::COLORMAP_VIRIDIS, cv::COLORMAP_CIVIDIS, cv::COLORMAP_TWILIGHT,
    // cv::COLORMAP_TWILIGHT_SHIFTED.
    cv::applyColorMap(processed_image, colored_image, cv::COLORMAP_JET);

    // 3. Opcional: Mostrar la imagen coloreada para depuración local
    // Asegúrate de que tu entorno gráfico ROS esté funcionando (ej. RViz, rqt_image_view)
    // o que tengas X forwarding si estás en SSH.
    cv::imshow("Thermal Colored Image (thermal_colorizer_node)", colored_image);
    cv::waitKey(1); // Necesario para que la ventana se actualice

    // 4. Convertir la imagen coloreada de OpenCV de vuelta a un mensaje de imagen de ROS
    // La imagen coloreada es BGR de 3 canales y 8 bits (CV_8UC3), que corresponde a la codificación "bgr8".
    // Mantenemos el mismo encabezado (header) del mensaje original para preservar el timestamp y frame_id.
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(msg->header, "bgr8", colored_image).toImageMsg();

    // 5. Publicar la imagen coloreada en el nuevo tópico
    colored_image_pub.publish(output_msg);
}

int main(int argc, char** argv) {
    // Inicializar el nodo ROS
    ros::init(argc, argv, "thermal_colorizer_node");
    ros::NodeHandle nh;

    // Suscribirse al tópico de imagen de la cámara térmica
    // Asumiendo que tu cámara térmica publica en "/phm/thermal_camera/image"
    ros::Subscriber sub = nh.subscribe("/phm/thermal_camera/image", 1, thermalImageCallback);

    // Publicitar el nuevo tópico para la imagen coloreada
    colored_image_pub = nh.advertise<sensor_msgs::Image>("/thermal_colored", 1);

    ROS_INFO("Thermal colorizer node started. Subscribing to /phm/thermal_camera/image and publishing to /thermal_colored.");

    // Mantener el nodo corriendo y procesando callbacks
    ros::spin();

    // Limpiar las ventanas de OpenCV al cerrar el nodo
    cv::destroyAllWindows();

    return 0;
}
