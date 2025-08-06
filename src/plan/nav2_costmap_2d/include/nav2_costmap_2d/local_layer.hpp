/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__LOCAL_LAYER_HPP_
#define NAV2_COSTMAP_2D__LOCAL_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/occ_grid_values.hpp"

namespace nav2_costmap_2d
{

/**
 * @class LocalLayer
 * @brief Takes in real-time occupancy grid from perception system to update costmap
 * This layer subscribes to /perception/occupancy_grid_map/local_map topic
 * and updates the costmap with real-time perception data.
 * FREE and OCCUPIED values are trusted and overwrite existing data,
 * while UNKNOWN values are ignored to avoid overwriting with sparse point cloud data.
 */
class LocalLayer : public CostmapLayer
{
public:
  /**
    * @brief Local Layer constructor
    */
  LocalLayer();
  /**
    * @brief Local Layer destructor
    */
  virtual ~LocalLayer();

  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize();

  /**
   * @brief Activate this layer
   */
  virtual void activate();
  /**
   * @brief Deactivate this layer
   */
  virtual void deactivate();

  /**
   * @brief Reset this costmap
   */
  virtual void reset();

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() {return false;}

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y);

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief Match the size of the master costmap
   */
  virtual void matchSize();

protected:
  /**
   * @brief Get parameters of layer
   */
  void getParameters();

  /**
   * @brief Process a new local map coming from perception topic
   */
  void processLocalMap(const nav_msgs::msg::OccupancyGrid & new_map);

  /**
   * @brief  Callback to update the costmap's map from the perception system
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * local map are overwritten.
   */
  void incomingLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map);

  /**
   * @brief Interpret the value in the local map given on the topic to
   * convert into costs for the costmap to utilize
   * @param value The occupancy grid value (FREE, OCCUPIED, or UNKNOWN)
   * @return The cost value for the costmap
   */
  unsigned char interpretValue(int8_t value);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters Vector of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  bool footprint_clearing_enabled_;
  /**
   * @brief Clear costmap layer info below the robot's footprint
   */
  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string map_frame_;  /// @brief frame that local map is located in

  bool has_updated_data_{false};

  unsigned int x_{0};
  unsigned int y_{0};
  unsigned int width_{0};
  unsigned int height_{0};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_sub_;

  // Parameters
  std::string local_map_topic_;
  bool track_unknown_space_;
  bool use_maximum_;
  unsigned char lethal_threshold_;
  unsigned char unknown_cost_value_;
  bool trinary_costmap_;
  bool map_received_{false};
  bool map_received_in_update_bounds_{false};
  tf2::Duration transform_tolerance_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_buffer_;
  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__LOCAL_LAYER_HPP_ 