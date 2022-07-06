#include "smb_navigation/terrain_adapter.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <algorithm>
#include <fstream>
#include <boost/filesystem.hpp>

#define CUSTOM_INF 10000

namespace fsys = boost::filesystem;

namespace smb_navigation
{

    TerrainAdapter::TerrainAdapter(const ros::NodeHandle &nh)
        : nodeHandle_(nh), initialized_(false)
    {
        // Make sure we have the parameters
        initialized_ = readParameters();

        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }
    }

    bool TerrainAdapter::readParameters()
    {
        if (!nodeHandle_.getParam("terrain_adapter/csv_polygon_full_path", csvPolygonFullPath_))
        {
            ROS_ERROR("[Terrain Adapter] csv file not specified");
            return false;
        }
        if (!nodeHandle_.getParam("terrain_adapter/source_frame", sourceFrame_))
        {
            ROS_ERROR("[Terrain Adapter] source frame not specified");
            return false;
        }
        if (!nodeHandle_.getParam("terrain_adapter/target_frame", targetFrame_))
        {
            ROS_ERROR("[Terrain Adapter] target frame not specified");
            return false;
        }

        if (!nodeHandle_.getParam("terrain_adapter/inside_param_full_path", inParamsFullPath_))
        {
            ROS_ERROR("[Terrain Adapter] target frame not specified");
            return false;
        }

        if (!nodeHandle_.getParam("terrain_adapter/outside_param_full_path", outParamsFullPath_))
        {
            ROS_ERROR("[Terrain Adapter] target frame not specified");
            return false;
        }

        ROS_INFO("[Terrain Adapter] Parameters parsed correctly");
        return true;
    }

    void TerrainAdapter::readCsv()
    {
        std::fstream inputFile(csvPolygonFullPath_);
        if (!inputFile.is_open())
        {
            ROS_ERROR("[Terrain Adapter] Cannot open given CSV file");
            return;
        }

        double x, y;

        for (;;)
        { /* loop continually */
            inputFile >> x >> y;
            if (inputFile.fail() || inputFile.eof())
                break;

            Point tmpPoint;
            tmpPoint.x = x;
            tmpPoint.y = y;
            polygonVertices_.push_back(tmpPoint);
        }
        inputFile.close();
        ROS_INFO_STREAM("[Terrain Adapter] Finished loading csv file\n");
    }

    void TerrainAdapter::run()
    {

        ros::Rate loopRate(1000);

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        bool lastState = false;
        bool firstTime = true;
        while (nodeHandle_.ok())
        {
            geometry_msgs::TransformStamped transformStamped;
            try
            {
                transformStamped = tfBuffer.lookupTransform(targetFrame_, sourceFrame_,
                                                            ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            auto translation = transformStamped.transform.translation;

            Point currentPoint;
            currentPoint.x = translation.x;
            currentPoint.y = translation.y;

            bool checkPointPolygon = isInside(polygonVertices_, polygonVertices_.size(), currentPoint);
            bool load = false;
            if (checkPointPolygon != lastState || firstTime == true)
            {
                firstTime = false;
                lastState = checkPointPolygon;
                load = true;
                // If the point is inside the polygon
                if (checkPointPolygon)
                {
                    loadYAMLFile(inParamsFullPath_);
                }
                // Otherwise
                else
                {
                    loadYAMLFile(outParamsFullPath_);
                }
            }
            ROS_INFO_STREAM("isIN: " << checkPointPolygon << " x: "<< translation.x << " y: " << translation.y << " load: " << load<< " \n");
            loopRate.sleep();
        }
    }
    // Define Infinite (Using INT_MAX caused overflow problems)

    // Given three collinear points p, q, r, the function checks if
    // point q lies on line segment 'pr'
    bool TerrainAdapter::onSegment(Point p, Point q, Point r)
    {
        if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
            return true;
        return false;
    }

    // To find orientation of ordered triplet (p, q, r).
    // The function returns following values
    // 0 --> p, q and r are collinear
    // 1 --> Clockwise
    // 2 --> Counterclockwise
    int TerrainAdapter::orientation(Point p, Point q, Point r)
    {
        int val = (q.y - p.y) * (r.x - q.x) -
                  (q.x - p.x) * (r.y - q.y);

        if (val == 0)
            return 0;             // collinear
        return (val > 0) ? 1 : 2; // clock or counterclock wise
    }

    // The function that returns true if line segment 'p1q1'
    // and 'p2q2' intersect.
    bool TerrainAdapter::doIntersect(Point p1, Point q1, Point p2, Point q2)
    {
        // Find the four orientations needed for general and
        // special cases
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases
        // p1, q1 and p2 are collinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1))
            return true;

        // p1, q1 and p2 are collinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1))
            return true;

        // p2, q2 and p1 are collinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2))
            return true;

        // p2, q2 and q1 are collinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2))
            return true;

        return false; // Doesn't fall in any of the above cases
    }

    // Returns true if the point p lies inside the polygon[] with n vertices
    bool TerrainAdapter::isInside(std::vector<Point> polygon, int n, Point p)
    {
        // There must be at least 3 vertices in polygon[]
        if (n < 3)
            return false;

        // Create a point for line segment from p to infinite
        Point extreme = {CUSTOM_INF, p.y};

        // Count intersections of the above line with sides of polygon
        int count = 0, i = 0;
        do
        {
            int next = (i + 1) % n;

            // Check if the line segment from 'p' to 'extreme' intersects
            // with the line segment from 'polygon[i]' to 'polygon[next]'
            if (doIntersect(polygon[i], polygon[next], p, extreme))
            {
                // If the point 'p' is collinear with line segment 'i-next',
                // then check if it lies on segment. If it lies, return true,
                // otherwise false
                if (orientation(polygon[i], p, polygon[next]) == 0)
                    return onSegment(polygon[i], p, polygon[next]);

                count++;
            }
            i = next;
        } while (i != 0);

        // Return true if count is odd, false otherwise
        return count & 1; // Same as (count%2 == 1)
    }

    void TerrainAdapter::loadYAMLFile(std::string filePath)
    {
        if (!fsys::exists(filePath) ||
            !(fsys::is_regular_file(filePath) || fsys::is_symlink(filePath)))
        {
            ROS_WARN_STREAM("Could not open camera param file " << filePath);
        }
        else
        {
            // load the YAML file using the external rosparam command
            std::string command = "rosparam load " + filePath;
            int result = std::system(command.c_str());
            if (result != 0)
            {
                ROS_WARN_STREAM("Could not set config file " << filePath << " to the parameter server.");
            }
        }
    }
} // end namespace smb_navigation
