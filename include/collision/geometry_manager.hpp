#ifndef GEOMETRY_MANAGER_HPP
#define GEOMETRY_MANAGER_HPP

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <geometric_shapes/shapes.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.h>
#include <tf2_eigen/tf2_eigen.h>

#include "imarker.hpp"
#include "../parameters.h"

// Map of shapes that match a string ID with the corresponding parameters
const std::map<std::string, std::pair<shapes::ShapeType, std::vector<std::string>>> SHAPES_MAP = {
    {"sphere", std::make_pair(shapes::SPHERE, std::vector<std::string>{"d"})},
    {"cylinder", std::make_pair(shapes::CYLINDER, std::vector<std::string>{"d", "l"})},
    {"box", std::make_pair(shapes::BOX, std::vector<std::string>{"x", "y", "z"})},
    {"plane", std::make_pair(shapes::PLANE, std::vector<std::string>{"orientation", "w", "l"})}};

/**
 *  CLASS Geometry 
 */

class Geometry
{
private:
    std::string m_id;
    std::string m_type;
    shapes::ShapeType m_shape_type;
    std::vector<double> m_size;
    Eigen::Isometry3d m_pose;
    shapes::ShapePtr m_shape;
    visualization_msgs::MarkerPtr m_marker;

public:
    Geometry();
    Geometry(std::string id, std::string type,std::vector<double> size, Eigen::Isometry3d init_pose);
    ~Geometry();

    std::string getId();
    std::string getType();
    std::string getIMarkerTopic();
    std::vector<double> getSize();
    float getMaxAxisSize();
    Eigen::Isometry3d getPose();
    shapes::ShapePtr getShape();
    visualization_msgs::MarkerPtr getMarker();
    void updatePose(Eigen::Isometry3d pose);
    void plane2marker(std::vector<double> size, visualization_msgs::Marker &marker);
};

/**
 *  CLASS GeometryManager 
 */

class GeometryManager
{
private:
    bool loadGeometryConfigFile();
    void createIMarker(boost::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)> imarker_callback);

    const std::string DEFAULT_PATH = "/config/geometry.yaml";

    std::string m_geometry_path;
    std::shared_ptr<Geometry> m_geometry;
    planning_scene::PlanningScenePtr m_planning_scene;
    std::shared_ptr<IMarker> m_imarker_world;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> m_inter_marker_srv;

    bool m_obj_added = false;

public:
    GeometryManager(planning_scene::PlanningScenePtr &planning_scene,
                    boost::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)> imarker_callback,
                    std::string relative_path = "");
    ~GeometryManager();

    std::shared_ptr<Geometry> &getGeometry();
    void updateWorld();

    visualization_msgs::Marker getMarker();
};

#endif // GEOMETRY_MANAGER_HPP