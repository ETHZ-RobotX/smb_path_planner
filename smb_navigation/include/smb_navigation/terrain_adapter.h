#include <ros/ros.h>
#include <vector>

namespace smb_navigation
{
    class TerrainAdapter
    {

    public:
        typedef struct Point
        {
            int x;
            int y;
        } Point;

        TerrainAdapter(const ros::NodeHandle &nh);

        ~TerrainAdapter() {}

        bool isInitialized() const { return initialized_; }
        void readCsv();
        void run();

    private:
        /**
         * @brief Function to read parameters from server
         * @return True if all parameters have been found, False otherwise
         */
        bool readParameters();

        bool onSegment(Point p, Point q, Point r);
        int orientation(Point p, Point q, Point r);
        bool doIntersect(Point p1, Point q1, Point p2, Point q2);
        bool isInside(std::vector<Point> polygon, int n, Point p);
        void loadYAMLFile(std::string filePath);

        ros::NodeHandle nodeHandle_;

        bool initialized_;

        std::string csvPolygonFullPath_;
        std::string sourceFrame_;
        std::string targetFrame_;
        std::string outParamsFullPath_;
        std::string inParamsFullPath_;
        std::vector<Point> polygonVertices_;

    }; // end class GridMapConverter

} // end namespace smb_navigation
