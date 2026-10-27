/**
 * @file lidar_circle_detector.cpp
 * @brief Nodo ROS1 Noetic – Detección de plano + círculos sobre nube PointXYZI.
 *
 * Novedades respecto a la versión anterior:
 *  - ROI angular: filtra la nube de entrada para quedarse solo con el sector
 *    azimutal [azimuth_min_deg, azimuth_max_deg] y el rango radial
 *    [min_range, max_range] antes de ejecutar RANSAC.  Esto reduce el tiempo
 *    de cómputo y evita que planos ajenos a la zona de interés "ganen" el RANSAC.
 *  - Visualización RViz completa:
 *      · Sector angular ROI (LINE_LIST verde) en el plano XY del sensor.
 *      · Normal del plano detectado (ARROW naranja).
 *      · Círculos detectados (CYLINDER rojo-naranja, disco fino).
 *    Todos los markers se publican en el mismo tópico "lidar_markers" con ns
 *    distintos para poder activar/desactivar cada grupo en RViz.
 */

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <cmath>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Dense>

// ─────────────────────────────────────────────────────────────────────────────
//  Tipos
// ─────────────────────────────────────────────────────────────────────────────
typedef pcl::PointXYZI PointT;

// ─────────────────────────────────────────────────────────────────────────────
//  Publishers globales
// ─────────────────────────────────────────────────────────────────────────────
ros::Publisher g_plane_pub;    // nube del plano extraído
ros::Publisher g_marker_pub;   // todos los markers de visualización

// ─────────────────────────────────────────────────────────────────────────────
//  Configuración cargada desde YAML
// ─────────────────────────────────────────────────────────────────────────────
struct LidarCircleConfig
{
    // Plano RANSAC
    double plane_distance_threshold;
    int    plane_max_iterations;
    int    plane_min_inliers;
    int    table_min_points;

    // Clustering
    double cluster_tolerance;
    int    cluster_min_size;
    int    cluster_max_size;

    // Círculos
    double circle_min_radius;
    double circle_max_radius;
    double circle_alpha;

    // ROI angular
    bool   roi_enabled;
    double roi_azimuth_min_rad;   // convertido a radianes al cargar
    double roi_azimuth_max_rad;
    double roi_min_range;
    double roi_max_range;

    // Visualización
    bool   show_roi_sector;
    double roi_color_r, roi_color_g, roi_color_b, roi_color_a;
    double roi_sector_height;

    bool   show_plane_normal;
    double normal_arrow_length;
    double normal_scale_shaft;
    double normal_scale_head;

    double circle_color_r, circle_color_g, circle_color_b;
    double circle_thickness;

    // Tópico de entrada
    std::string input_topic;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Ajuste mínimo-cuadrático de círculo 2D
// ─────────────────────────────────────────────────────────────────────────────
static bool fitCircleLS(const std::vector<Eigen::Vector2d>& pts,
                        Eigen::Vector2d& center,
                        double& radius)
{
    if (pts.size() < 3) return false;

    const int n = static_cast<int>(pts.size());
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd b(n);

    for (int i = 0; i < n; ++i)
    {
        const double x = pts[i].x(), y = pts[i].y();
        A(i, 0) = 2.0 * x;
        A(i, 1) = 2.0 * y;
        A(i, 2) = 1.0;
        b(i)    = x * x + y * y;
    }

    const Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);
    center = sol.head<2>();

    const double rSq = sol(2) + center.squaredNorm();
    if (rSq <= 0.0) return false;

    radius = std::sqrt(rSq);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Filtrado de la nube por ROI angular y de rango
// ─────────────────────────────────────────────────────────────────────────────
/**
 * Devuelve una nueva nube con solo los puntos dentro del sector azimutal y el
 * rango radial configurados.
 *
 * Convención de ángulo: atan2(y, x)  →  [-π, π]
 * Si azimuth_min > azimuth_max (sector que cruza los ±180°) se maneja
 * correctamente con la condición OR.
 */
static pcl::PointCloud<PointT>::Ptr applyROI(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const LidarCircleConfig& cfg)
{
    pcl::PointCloud<PointT>::Ptr roi_cloud(new pcl::PointCloud<PointT>);
    roi_cloud->reserve(cloud->size());

    const bool wraps = cfg.roi_azimuth_min_rad > cfg.roi_azimuth_max_rad;
    const bool use_range = (cfg.roi_max_range > 0.0);

    for (const auto& p : cloud->points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;

        // Rango radial 2D (en el plano XY)
        const double range = std::sqrt(static_cast<double>(p.x) * p.x +
                                       static_cast<double>(p.y) * p.y);

        if (range < cfg.roi_min_range) continue;
        if (use_range && range > cfg.roi_max_range) continue;

        // Ángulo azimutal
        const double az = std::atan2(static_cast<double>(p.y),
                                     static_cast<double>(p.x));

        const bool in_sector = wraps
            ? (az >= cfg.roi_azimuth_min_rad || az <= cfg.roi_azimuth_max_rad)
            : (az >= cfg.roi_azimuth_min_rad && az <= cfg.roi_azimuth_max_rad);

        if (in_sector)
            roi_cloud->push_back(p);
    }

    roi_cloud->width    = static_cast<uint32_t>(roi_cloud->points.size());
    roi_cloud->height   = 1;
    roi_cloud->is_dense = true;
    return roi_cloud;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Generación de markers de visualización
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Marker LINE_LIST que dibuja el sector ROI en RViz:
 *   - dos radios (líneas desde el origen hasta el borde del sector)
 *   - un arco aproximado con N_ARC_SEGS segmentos de línea
 */
static visualization_msgs::Marker makeSectorMarker(
    const std_msgs::Header& header,
    const LidarCircleConfig& cfg,
    int marker_id)
{
    visualization_msgs::Marker m;
    m.header    = header;
    m.ns        = "roi_sector";
    m.id        = marker_id;
    m.type      = visualization_msgs::Marker::LINE_LIST;
    m.action    = visualization_msgs::Marker::ADD;
    m.lifetime  = ros::Duration(0.5);

    m.scale.x   = 0.02;  // grosor de línea [m]
    m.color.r   = static_cast<float>(cfg.roi_color_r);
    m.color.g   = static_cast<float>(cfg.roi_color_g);
    m.color.b   = static_cast<float>(cfg.roi_color_b);
    m.color.a   = static_cast<float>(cfg.roi_color_a);
    m.pose.orientation.w = 1.0;

    const double R       = cfg.roi_max_range > 0.0 ? cfg.roi_max_range : 6.0;
    const double z_lo    = -cfg.roi_sector_height * 0.5;
    const double z_hi    =  cfg.roi_sector_height * 0.5;
    const int    N_SEGS  = 32;   // segmentos del arco

    // ── radio izquierdo ──────────────────────────────────────────────
    geometry_msgs::Point p0, p1;
    p0.x = 0; p0.y = 0; p0.z = z_lo;
    p1.x = R * std::cos(cfg.roi_azimuth_min_rad);
    p1.y = R * std::sin(cfg.roi_azimuth_min_rad);
    p1.z = z_lo;
    m.points.push_back(p0); m.points.push_back(p1);
    p0.z = p1.z = z_hi;
    m.points.push_back(p0); m.points.push_back(p1);

    // ── radio derecho ────────────────────────────────────────────────
    p0.x = 0; p0.y = 0; p0.z = z_lo;
    p1.x = R * std::cos(cfg.roi_azimuth_max_rad);
    p1.y = R * std::sin(cfg.roi_azimuth_max_rad);
    p1.z = z_lo;
    m.points.push_back(p0); m.points.push_back(p1);
    p0.z = p1.z = z_hi;
    m.points.push_back(p0); m.points.push_back(p1);

    // ── arco ─────────────────────────────────────────────────────────
    double ang_span = cfg.roi_azimuth_max_rad - cfg.roi_azimuth_min_rad;
    if (ang_span <= 0.0) ang_span += 2.0 * M_PI;  // sector que cruza ±π

    for (int i = 0; i < N_SEGS; ++i)
    {
        const double a0 = cfg.roi_azimuth_min_rad + ang_span * i       / N_SEGS;
        const double a1 = cfg.roi_azimuth_min_rad + ang_span * (i + 1) / N_SEGS;

        for (double z : {z_lo, z_hi})
        {
            geometry_msgs::Point pa, pb;
            pa.x = R * std::cos(a0); pa.y = R * std::sin(a0); pa.z = z;
            pb.x = R * std::cos(a1); pb.y = R * std::sin(a1); pb.z = z;
            m.points.push_back(pa);
            m.points.push_back(pb);
        }
    }

    return m;
}

/**
 * Marker ARROW que representa la normal del plano detectado.
 * La flecha arranca del centroide del plano y apunta en dirección de la normal.
 */
static visualization_msgs::Marker makePlaneNormalMarker(
    const std_msgs::Header& header,
    const Eigen::Vector3d& centroid,
    const Eigen::Vector3d& normal,
    const LidarCircleConfig& cfg,
    int marker_id)
{
    visualization_msgs::Marker m;
    m.header   = header;
    m.ns       = "plane_normal";
    m.id       = marker_id;
    m.type     = visualization_msgs::Marker::ARROW;
    m.action   = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0.5);

    m.scale.x = static_cast<float>(cfg.normal_scale_shaft);
    m.scale.y = static_cast<float>(cfg.normal_scale_head);
    m.scale.z = 0.0;  // ignorado para ARROW con dos puntos

    m.color.r = 1.0f;  // naranja
    m.color.g = 0.55f;
    m.color.b = 0.0f;
    m.color.a = 1.0f;

    geometry_msgs::Point tail, tip;
    tail.x = centroid.x(); tail.y = centroid.y(); tail.z = centroid.z();
    tip.x  = centroid.x() + normal.x() * cfg.normal_arrow_length;
    tip.y  = centroid.y() + normal.y() * cfg.normal_arrow_length;
    tip.z  = centroid.z() + normal.z() * cfg.normal_arrow_length;
    m.points.push_back(tail);
    m.points.push_back(tip);

    return m;
}

/**
 * Marker CYLINDER (disco fino) para cada círculo detectado.
 * La orientación del cilindro se alinea con la normal del plano usando
 * la rotación mínima desde +Z hasta 'normal'.
 */
static visualization_msgs::Marker makeCircleMarker(
    const std_msgs::Header& header,
    const Eigen::Vector3d& center,
    double radius,
    const Eigen::Vector3d& normal,
    const LidarCircleConfig& cfg,
    int marker_id)
{
    visualization_msgs::Marker m;
    m.header   = header;
    m.ns       = "detected_circles";
    m.id       = marker_id;
    m.type     = visualization_msgs::Marker::CYLINDER;
    m.action   = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0.5);

    m.pose.position.x = center.x();
    m.pose.position.y = center.y();
    m.pose.position.z = center.z();

    // Orientación: rotar el eje +Z del cilindro hacia la normal del plano
    // usando la rotación mínima (eje-ángulo vía producto vectorial).
    const Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    const Eigen::Vector3d axis  = z_axis.cross(normal);
    const double          angle = std::acos(
        std::max(-1.0, std::min(1.0, z_axis.dot(normal))));

    if (axis.norm() > 1e-6)
    {
        Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis.normalized()));
        m.pose.orientation.x = q.x();
        m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z();
        m.pose.orientation.w = q.w();
    }
    else
    {
        m.pose.orientation.w = 1.0;  // normal ya es +Z
    }

    m.scale.x = radius * 2.0;                               // diámetro
    m.scale.y = radius * 2.0;
    m.scale.z = static_cast<float>(cfg.circle_thickness);   // disco delgado

    m.color.r = static_cast<float>(cfg.circle_color_r);
    m.color.g = static_cast<float>(cfg.circle_color_g);
    m.color.b = static_cast<float>(cfg.circle_color_b);
    m.color.a = static_cast<float>(cfg.circle_alpha);

    return m;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Callback principal
// ─────────────────────────────────────────────────────────────────────────────
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                   const LidarCircleConfig& cfg)
{
    // ── 1. Conversión ROS → PCL ───────────────────────────────────────────
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    // ── 2. Filtrado por ROI angular ───────────────────────────────────────
    pcl::PointCloud<PointT>::Ptr working_cloud = cloud;
    if (cfg.roi_enabled)
    {
        working_cloud = applyROI(cloud, cfg);
        if (working_cloud->empty())
        {
            ROS_WARN_THROTTLE(2.0, "ROI angular vacía – sin puntos en el sector.");
            return;
        }
        ROS_DEBUG_THROTTLE(1.0, "ROI: %zu → %zu puntos",
                           cloud->size(), working_cloud->size());
    }

    // ── 3. Inicializar array de markers ───────────────────────────────────
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;

    // ── 4. Marker del sector ROI ─────────────────────────────────────────
    if (cfg.roi_enabled && cfg.show_roi_sector)
        marker_array.markers.push_back(
            makeSectorMarker(msg->header, cfg, marker_id++));

    // ── 5. Segmentación RANSAC del plano dominante ────────────────────────
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(cfg.plane_distance_threshold);
    seg.setMaxIterations(cfg.plane_max_iterations);
    seg.setInputCloud(working_cloud);

    pcl::PointIndices::Ptr      plane_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
    seg.segment(*plane_inliers, *plane_coeff);

    if (static_cast<int>(plane_inliers->indices.size()) < cfg.plane_min_inliers)
    {
        ROS_WARN_THROTTLE(2.0, "Plano no detectado (inliers: %zu < %d).",
                          plane_inliers->indices.size(), cfg.plane_min_inliers);
        // Publicar al menos el marker de ROI aunque no haya plano
        if (!marker_array.markers.empty())
            g_marker_pub.publish(marker_array);
        return;
    }

    // ── 6. Extraer nube del plano ─────────────────────────────────────────
    pcl::PointCloud<PointT>::Ptr table_cloud(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(working_cloud);
    extract.setIndices(plane_inliers);
    extract.setNegative(false);
    extract.filter(*table_cloud);

    if (static_cast<int>(table_cloud->size()) < cfg.table_min_points)
        return;

    // Publicar nube del plano para inspeccionarla en RViz
    sensor_msgs::PointCloud2 plane_msg;
    pcl::toROSMsg(*table_cloud, plane_msg);
    plane_msg.header = msg->header;
    g_plane_pub.publish(plane_msg);

    // ── 7. Normal del plano ───────────────────────────────────────────────
    Eigen::Vector3d normal(plane_coeff->values[0],
                           plane_coeff->values[1],
                           plane_coeff->values[2]);
    normal.normalize();
    if (normal.z() < 0.0) normal = -normal;   // normal apuntando hacia +Z

    const Eigen::Vector3d u = normal.unitOrthogonal();
    const Eigen::Vector3d v = normal.cross(u);

    // Centroide del plano (para el arrow de la normal)
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : table_cloud->points)
        centroid += Eigen::Vector3d(p.x, p.y, p.z);
    centroid /= static_cast<double>(table_cloud->size());

    // Marker: normal del plano
    if (cfg.show_plane_normal)
        marker_array.markers.push_back(
            makePlaneNormalMarker(msg->header, centroid, normal, cfg, marker_id++));

    // ── 8. Clustering euclidiano sobre el plano ──────────────────────────
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(table_cloud);

    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cfg.cluster_tolerance);
    ec.setMinClusterSize(cfg.cluster_min_size);
    ec.setMaxClusterSize(cfg.cluster_max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(table_cloud);
    ec.extract(clusters);

    // Proyector compartido para todos los clusters
    pcl::ProjectInliers<PointT> projector;
    projector.setModelType(pcl::SACMODEL_PLANE);
    projector.setModelCoefficients(plane_coeff);

    // ── 9. Ajuste de círculo por cluster ──────────────────────────────────
    int circles_found = 0;
    for (const auto& c_indices : clusters)
    {
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        cluster->reserve(c_indices.indices.size());
        for (int idx : c_indices.indices)
            cluster->push_back(table_cloud->points[idx]);

        // Proyectar al plano
        pcl::PointCloud<PointT>::Ptr projected(new pcl::PointCloud<PointT>);
        projector.setInputCloud(cluster);
        projector.filter(*projected);
        if (projected->empty()) continue;

        // Centroide del cluster
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (const auto& p : projected->points)
            mean += Eigen::Vector3d(p.x, p.y, p.z);
        mean /= static_cast<double>(projected->size());

        // Coordenadas 2D en el frame local del plano
        std::vector<Eigen::Vector2d> pts2d;
        pts2d.reserve(projected->size());
        for (const auto& p : projected->points)
        {
            Eigen::Vector3d P(p.x, p.y, p.z);
            P -= mean;
            pts2d.emplace_back(P.dot(u), P.dot(v));
        }

        // Ajuste LS
        Eigen::Vector2d c2d;
        double r;
        if (!fitCircleLS(pts2d, c2d, r)) continue;
        if (r < cfg.circle_min_radius || r > cfg.circle_max_radius) continue;

        // Centro en 3D
        const Eigen::Vector3d center3d = mean + c2d.x() * u + c2d.y() * v;

        marker_array.markers.push_back(
            makeCircleMarker(msg->header, center3d, r, normal, cfg, marker_id++));

        ++circles_found;
    }

    if (circles_found > 0)
        ROS_INFO_THROTTLE(1.0, "Círculos detectados: %d", circles_found);

    // ── 10. Publicar todos los markers ────────────────────────────────────
    g_marker_pub.publish(marker_array);
}

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_circle_detector");
    ros::NodeHandle nh("~");

    // ── Cargar YAML ───────────────────────────────────────────────────────
    std::string config_file;
    if (!nh.getParam("config_file", config_file))
    {
        ROS_FATAL("Parámetro 'config_file' no encontrado.");
        return 1;
    }

    YAML::Node cfg_yaml = YAML::LoadFile(config_file);

    LidarCircleConfig cfg;

    // Plano
    cfg.plane_distance_threshold = cfg_yaml["plane"]["distance_threshold"].as<double>();
    cfg.plane_max_iterations     = cfg_yaml["plane"]["max_iterations"].as<int>();
    cfg.plane_min_inliers        = cfg_yaml["plane"]["min_inliers"].as<int>();
    cfg.table_min_points         = cfg_yaml["table"]["min_points"].as<int>();

    // Clustering
    cfg.cluster_tolerance = cfg_yaml["clustering"]["tolerance"].as<double>();
    cfg.cluster_min_size  = cfg_yaml["clustering"]["min_size"].as<int>();
    cfg.cluster_max_size  = cfg_yaml["clustering"]["max_size"].as<int>();

    // Círculos
    cfg.circle_min_radius = cfg_yaml["circle"]["min_radius"].as<double>();
    cfg.circle_max_radius = cfg_yaml["circle"]["max_radius"].as<double>();
    cfg.circle_alpha      = cfg_yaml["circle"]["alpha"].as<double>();

    // ROI angular
    cfg.roi_enabled = cfg_yaml["roi"]["enabled"].as<bool>();
    {
        const double deg2rad = M_PI / 180.0;
        cfg.roi_azimuth_min_rad = cfg_yaml["roi"]["azimuth_min_deg"].as<double>() * deg2rad;
        cfg.roi_azimuth_max_rad = cfg_yaml["roi"]["azimuth_max_deg"].as<double>() * deg2rad;
        cfg.roi_min_range       = cfg_yaml["roi"]["min_range"].as<double>();
        cfg.roi_max_range       = cfg_yaml["roi"]["max_range"].as<double>();
    }

    // Visualización
    const YAML::Node& vis         = cfg_yaml["visualization"];
    cfg.show_roi_sector           = vis["show_roi_sector"].as<bool>();
    cfg.roi_color_r               = vis["roi_sector_color_r"].as<double>();
    cfg.roi_color_g               = vis["roi_sector_color_g"].as<double>();
    cfg.roi_color_b               = vis["roi_sector_color_b"].as<double>();
    cfg.roi_color_a               = vis["roi_sector_alpha"].as<double>();
    cfg.roi_sector_height         = vis["roi_sector_height"].as<double>();
    cfg.show_plane_normal         = vis["show_plane_normal"].as<bool>();
    cfg.normal_arrow_length       = vis["normal_arrow_length"].as<double>();
    cfg.normal_scale_shaft        = vis["normal_scale_shaft"].as<double>();
    cfg.normal_scale_head         = vis["normal_scale_head"].as<double>();
    cfg.circle_color_r            = vis["circle_color_r"].as<double>();
    cfg.circle_color_g            = vis["circle_color_g"].as<double>();
    cfg.circle_color_b            = vis["circle_color_b"].as<double>();
    cfg.circle_thickness          = vis["circle_thickness"].as<double>();

    // Tópico de entrada
    cfg.input_topic = cfg_yaml["topics"]["input"].as<std::string>();

    ROS_INFO("Configuración cargada correctamente.");
    ROS_INFO("  ROI: %s  [%.1f°, %.1f°]  rango [%.1f, %.1f] m",
             cfg.roi_enabled ? "ON" : "OFF",
             cfg.roi_azimuth_min_rad * 180.0 / M_PI,
             cfg.roi_azimuth_max_rad * 180.0 / M_PI,
             cfg.roi_min_range, cfg.roi_max_range);

    // ── Publishers ────────────────────────────────────────────────────────
    g_plane_pub  = nh.advertise<sensor_msgs::PointCloud2>("detected_plane", 1);
    g_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("lidar_markers", 1);

    // ── Subscriber ────────────────────────────────────────────────────────
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
        cfg.input_topic, 1,
        boost::bind(cloudCallback, _1, cfg));

    ROS_INFO("Nodo en ejecución. Escuchando '%s'.", cfg.input_topic.c_str());
    ros::spin();
    return 0;
}