#include <yaml-cpp/yaml.h>
#include <algorithm>

#include "../../include/collision/geometry_manager.hpp"

/** ======================>
 *  CLASS Geometry
 *  ^^^^^^^^^^^^^^^^^^^^^
 */

Geometry::Geometry()
{
}

Geometry::Geometry(std::string id, std::string type, std::vector<double> size, Eigen::Isometry3d init_pose)
    : m_id(id),
      m_type(type),
      m_size(size),
      m_pose(init_pose)
{
    m_shape_type = SHAPES_MAP.at(m_type).first;

    // Marker initialization
    m_marker.reset(new visualization_msgs::Marker);
    m_marker->header.frame_id = "/base_link";
    m_marker->header.stamp = ros::Time::now();
    m_marker->ns = "world_box";
    m_marker->id = 0;
    m_marker->action = visualization_msgs::Marker::ADD;
    m_marker->color.r = 1.0f;
    m_marker->color.g = 1.0f;
    m_marker->color.b = 0.0f;
    m_marker->color.a = 0.6f;
    m_marker->lifetime = ros::Duration();
    m_marker->pose = tf2::toMsg(m_pose);

    // Geometry and Marker creation
    switch (m_shape_type)
    {
    case shapes::ShapeType::SPHERE:
        m_shape.reset(new shapes::Sphere(size[0] / 2));
        m_marker->type = visualization_msgs::Marker::SPHERE;
        m_marker->scale.x = size[0];
        m_marker->scale.y = size[0];
        m_marker->scale.z = size[0];
        break;

    case shapes::ShapeType::CYLINDER:
        m_shape.reset(new shapes::Cylinder(size[0] / 2, size[1]));
        m_marker->type = visualization_msgs::Marker::CYLINDER;
        m_marker->scale.x = size[0];
        m_marker->scale.y = size[0];
        m_marker->scale.z = size[1];
        break;

    case shapes::ShapeType::BOX:
        m_shape.reset(new shapes::Box(size[0], size[1], size[2]));
        m_marker->type = visualization_msgs::Marker::CUBE;
        m_marker->scale.x = size[0];
        m_marker->scale.y = size[1];
        m_marker->scale.z = size[2];
        break;

    case shapes::ShapeType::PLANE:
        plane2marker(size, *m_marker);
        break;

    default:
        throw std::invalid_argument("Invalid shapre type");
        break;
    }
}

Geometry::~Geometry()
{
    m_shape.reset();
    m_marker.reset();
}

std::string Geometry::getId()
{
    return m_id;
}

std::string Geometry::getType()
{
    return m_type;
}

std::string Geometry::getIMarkerTopic()
{
    return "imarker_" + m_id;
}

std::vector<double> Geometry::getSize()
{
    return m_size;
}

float Geometry::getMaxAxisSize()
{
    float max_axes_size = 0;
    // for plane geometries the first element indicates the orientation
    unsigned int i = m_shape_type != shapes::ShapeType::PLANE ? 0 : 1;
    for (; i < m_size.size(); i++)
        if (m_size[i] > max_axes_size)
            max_axes_size = m_size[i];

    return max_axes_size;
}

Eigen::Isometry3d Geometry::getPose()
{
    return m_pose;
}

shapes::ShapePtr Geometry::getShape()
{
    return m_shape;
}

visualization_msgs::MarkerPtr Geometry::getMarker()
{
    return m_marker;
}

void Geometry::updatePose(Eigen::Isometry3d pose)
{
    m_pose = pose;
    m_marker->pose = tf2::toMsg(m_pose);
}

void Geometry::plane2marker(std::vector<double> size, visualization_msgs::Marker &marker)
{
    double x, y, z;
    double thickness = 0.1;
    if (size[0] == 0)
    {
        // horizontal plane
        z = thickness;
        x = size[1];
        y = size[2];
    }
    else if (size[0] == 1)
    {
        // vertical (in x direction) plane
        y = thickness;
        x = size[1];
        z = size[2];
    }
    else // if (size[0] == 2) and defalt configuration
    {
        // vertical (in y direction) plane
        x = thickness;
        y = size[1];
        z = size[2];
    }

    m_shape.reset(new shapes::Box(x, y, z));
    m_marker->type = visualization_msgs::Marker::CUBE;
    marker.scale.x = x;
    marker.scale.y = y;
    marker.scale.z = z;
}

/** ======================>
 *  CLASS GeometryManager 
 *  ^^^^^^^^^^^^^^^^^^^^^
 */

GeometryManager::GeometryManager(planning_scene::PlanningScenePtr &planning_scene,
                                 boost::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)> imarker_callback,
                                 std::string relative_path)
{
    m_planning_scene = planning_scene;

    // get geometry config absolute path
    std::string path = ros::package::getPath(PACKAGE_NAME);
    if (relative_path == "")
    {
        m_geometry_path = path + GeometryManager::DEFAULT_PATH;
    }
    else
    {
        m_geometry_path = path + relative_path;
    }

    // Load Geometry from file
    if (loadGeometryConfigFile())
        ROS_INFO("Loaded Yaml file: %s", m_geometry_path.c_str());
    else
        throw std::logic_error("Yaml file error: " + m_geometry_path);

    // Associate Interactive Marker to geometry
    createIMarker(imarker_callback);

    // Add object to world
    updateWorld();
}

GeometryManager::~GeometryManager()
{
    m_geometry.reset();
    m_planning_scene.reset();
    m_inter_marker_srv.reset();
}

bool GeometryManager::loadGeometryConfigFile()
{
    YAML::Node config = YAML::LoadFile(m_geometry_path);
    std::string id;
    std::string imarker_topic;
    std::string type;
    std::vector<double> size;
    std::vector<double> pose;

    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
    {
        // Get the Geometry ID
        // ^^^^^^^^^^^^^^^^^^^
        id = it->first.as<std::string>();

        // Get the Geometry TYPE
        // ^^^^^^^^^^^^^^^^^^^^^
        if (!it->second["type"])
        {
            ROS_ERROR("[%s] Wrong id configuration, skipped.", id.c_str());
            continue;
        }
        type = it->second["type"].as<std::string>();

        // Get the Geometry SIZE
        // ^^^^^^^^^^^^^^^^^^^^^
        try
        {
            unsigned int count = 0;
            std::vector<std::string> size_types = SHAPES_MAP.at(type).second;
            std::vector<double> tmp_size(size_types.size()); // can throw out_of_range except.

            for (YAML::const_iterator it_size = it->second["size"].begin(); it_size != it->second["size"].end(); ++it_size)
            {
                // get right position of parameters from SHAPES_MAP
                ptrdiff_t pos = std::distance(size_types.begin(), std::find(size_types.begin(), size_types.end(), it_size->first.as<std::string>()));

                // If the parameter is recognized in relation with the specified
                // geometry, then it is placed in the vect. in the right position.
                if (0 <= pos && (unsigned)pos < size_types.size())
                {
                    tmp_size[pos] = it_size->second.as<double>();
                    count++;
                }
            }

            // Check that the number of parameters that
            // correspond to the geometry are the right amount.
            if (count++ != size_types.size())
            {
                ROS_ERROR("[%s] Invalid size configuration, skipped.", id.c_str());
                continue;
            }

            size = tmp_size;
        }
        catch (const std::out_of_range &e)
        {
            //std::cout << e << std::endl;
            ROS_ERROR("[%s] Geometry type (%s) not known, skipped.", id.c_str(), type.c_str());
            continue;
        }

        // Get the Geometry POSE
        // ^^^^^^^^^^^^^^^^^^^^^
        unsigned int count = 0;
        std::vector<std::string> pose_types = {"x", "y", "z"};
        std::vector<double> tmp_pose(pose_types.size());

        for (YAML::const_iterator it_pose = it->second["pose"].begin(); it_pose != it->second["pose"].end(); ++it_pose)
        {
            // get right position of parameters from SHAPES_MAP
            ptrdiff_t pos = std::distance(pose_types.begin(), std::find(pose_types.begin(), pose_types.end(), it_pose->first.as<std::string>()));

            // If the parameter is recognized in relation with the specified
            // geometry, then it is placed in the vect. in the right position.
            if (0 <= pos && (unsigned)pos < pose_types.size())
            {
                tmp_pose[pos] = it_pose->second.as<double>();
                count++;
            }
        }

        // Check that the number of parameters that indicates
        // the pose correspond to a 3d dimension world.
        if (count != pose_types.size())
        {
            ROS_ERROR("[%s] Invalid pose configuration, skipped.", id.c_str());
            continue;
        }

        pose = tmp_pose;
        Eigen::Isometry3d init_pose(Eigen::Isometry3d(Eigen::Translation3d(tmp_pose[0], tmp_pose[1], tmp_pose[2])));
        m_geometry.reset(new Geometry(id, type, size, init_pose));
        break; // supported only for one geometry
    }

    return m_geometry != nullptr ? true : false;
}

void GeometryManager::createIMarker(boost::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)> imarker_callback)
{
    m_inter_marker_srv.reset(new interactive_markers::InteractiveMarkerServer(m_geometry->getIMarkerTopic()));
    // create an interactive marker to control the world geometry
    m_imarker_world.reset(new IMarker(*m_inter_marker_srv, "world", m_geometry->getPose(), imarker_callback,
                                      m_geometry->getMaxAxisSize(), "/base_link", IMarker::POS));
}

std::shared_ptr<Geometry> &GeometryManager::getGeometry()
{
    return m_geometry;
}

// Update geometry to the PlanningScene
void GeometryManager::updateWorld()
{
    if (!m_obj_added)
    {
        m_planning_scene->getWorldNonConst()->addToObject(m_geometry->getId(), m_geometry->getShape(), m_geometry->getPose());
        m_obj_added = true;
    }
    else
    {
        m_planning_scene->getWorldNonConst()->moveShapeInObject(m_geometry->getId(), m_geometry->getShape(), m_geometry->getPose());
    }
}

visualization_msgs::Marker GeometryManager::getMarker()
{
    return *m_geometry->getMarker();
}
