#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
//#include <bspline_opt/uniform_bspline.h>
#include <iostream>
//#include <bspline_opt/polynomial_traj.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <stdlib.h>

using std::vector;
namespace ego_planner
{
  class PlanningVisualization
  {
  private:
    using MarkerPub = rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr;
    using MarkerArrayPub = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;

    rclcpp::Node::SharedPtr node;

    MarkerPub goal_point_pub;
    MarkerPub global_list_pub;
    MarkerPub init_list_pub;
    MarkerPub optimal_list_pub;
    MarkerPub a_star_list_pub;

  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(const rclcpp::Node::SharedPtr &nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(const MarkerPub &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
    void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(const MarkerArrayPub &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    // void displayIntermediateState(MarkerPub& intermediate_pub, ego_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);
    // void displayNewArrow(MarkerPub& guide_vector_pub, ego_planner::BsplineOptimizer::Ptr optimizer);
  };
} // namespace ego_planner
#endif