#include "nav2_costmap_2d/local_layer.hpp"

#include <algorithm>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::LocalLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

LocalLayer::LocalLayer()
: map_buffer_(nullptr)
{
}

LocalLayer::~LocalLayer()
{
}

void
LocalLayer::onInitialize()
{
  global_frame_ = layered_costmap_->getGlobalFrameID();
  
  getParameters();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  local_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_map_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&LocalLayer::incomingLocalMap, this, std::placeholders::_1));
}

void
LocalLayer::activate()
{
}

void
LocalLayer::deactivate()
{
  dyn_params_handler_.reset();
}

void
LocalLayer::reset()
{
  has_updated_data_ = true;
  current_ = false;
}

void
LocalLayer::getParameters()
{
  int temp_lethal_threshold = 0;
  double temp_tf_tol = 0.0;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.0));
  declareParameter("local_map_topic", rclcpp::ParameterValue("/perception/occupancy_grid_map/local_map"));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(false));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
  node->get_parameter(name_ + "." + "local_map_topic", local_map_topic_);
  node->get_parameter("track_unknown_space", track_unknown_space_);
  node->get_parameter("use_maximum", use_maximum_);
  node->get_parameter("lethal_cost_threshold", temp_lethal_threshold);
  node->get_parameter("unknown_cost_value", unknown_cost_value_);
  node->get_parameter("trinary_costmap", trinary_costmap_);
  node->get_parameter("transform_tolerance", temp_tf_tol);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  map_received_ = false;
  map_received_in_update_bounds_ = false;
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &LocalLayer::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void
LocalLayer::processLocalMap(const nav_msgs::msg::OccupancyGrid & new_map)
{
  unsigned int size_x = new_map.info.width;
  unsigned int size_y = new_map.info.height;

  // Store the frame_id for later use
  map_frame_ = new_map.header.frame_id;
  
  // Check if the map is in base_link frame
  bool is_base_link_frame = (map_frame_ == "base_link");
  
  Costmap2D * master = layered_costmap_->getCostmap();
  
  // For base_link frame, we don't resize the master costmap as it should be in map frame
  // For map frame, we follow the original logic
  if (!is_base_link_frame && !layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
    master->getSizeInCellsY() != size_y ||
    master->getResolution() != new_map.info.resolution ||
    master->getOriginX() != new_map.info.origin.position.x ||
    master->getOriginY() != new_map.info.origin.position.y ||
    !layered_costmap_->isSizeLocked()))
  {
    layered_costmap_->resizeMap(
      size_x, size_y, new_map.info.resolution,
      new_map.info.origin.position.x,
      new_map.info.origin.position.y,
      true);
  } else if (size_x_ != size_x || size_y_ != size_y ||
    resolution_ != new_map.info.resolution ||
    origin_x_ != new_map.info.origin.position.x ||
    origin_y_ != new_map.info.origin.position.y)
  {
    resizeMap(
      size_x, size_y, new_map.info.resolution,
      new_map.info.origin.position.x, new_map.info.origin.position.y);
  }

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // Process map data efficiently
  const size_t total_size = size_x * size_y;
  for (size_t index = 0; index < total_size; ++index) {
    costmap_[index] = interpretValue(new_map.data[index]);
  }

  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  has_updated_data_ = true;
  current_ = true;
}

void
LocalLayer::matchSize()
{
  // If we are using rolling costmap, the local map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling()) {
    Costmap2D * master = layered_costmap_->getCostmap();
    resizeMap(
      master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
      master->getOriginX(), master->getOriginY());
  }
  // In rolling window mode, no resize needed as local_layer has its own map size
}

unsigned char
LocalLayer::interpretValue(int8_t value)
{
  if (value == nav2_util::OCC_GRID_FREE) {
    return FREE_SPACE;
  } else if (value == nav2_util::OCC_GRID_OCCUPIED) {
    return LETHAL_OBSTACLE;
  } else if (value == nav2_util::OCC_GRID_UNKNOWN) {
    return NO_INFORMATION;
  } else {
    // For other values, use threshold-based judgment
    if (track_unknown_space_ && value == unknown_cost_value_) {
      return NO_INFORMATION;
    } else if (!track_unknown_space_ && value == unknown_cost_value_) {
      return FREE_SPACE;
    } else if (value >= lethal_threshold_) {
      return LETHAL_OBSTACLE;
    } else if (trinary_costmap_) {
      return FREE_SPACE;
    } else {
      // Linear scaling
      double scale = static_cast<double>(value) / lethal_threshold_;
      unsigned char result = static_cast<unsigned char>(scale * LETHAL_OBSTACLE);
      return result;
    }
  }
}

void
LocalLayer::incomingLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map)
{
  // Simple message validation
  if (!new_map) {
    RCLCPP_ERROR(logger_, "Received local map message is null. Rejecting.");
    return;
  }
  
  if (new_map->data.empty()) {
    RCLCPP_ERROR(logger_, "Received local map message has empty data. Rejecting.");
    return;
  }
  
  if (!map_received_) {
    processLocalMap(*new_map);
    map_received_ = true;
    return;
  }
  
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  map_buffer_ = new_map;
}

void
LocalLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  if (!map_received_) {
    map_received_in_update_bounds_ = false;
    return;
  }
  map_received_in_update_bounds_ = true;

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // If there is a new available map, load it.
  if (map_buffer_) {
    processLocalMap(*map_buffer_);
    map_buffer_ = nullptr;
  }

  if (!layered_costmap_->isRolling() ) {
    if (!(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;

  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void
LocalLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y,
  double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {return;}

  transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void
LocalLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }
  if (!map_received_in_update_bounds_) {
    return;
  }

  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  // Check if the map is in base_link frame
  bool is_base_link_frame = (map_frame_ == "base_link");

  if (is_base_link_frame) {
    // Handle base_link frame: transform local map data to map frame
    unsigned int mx, my;
    
    // Get transform from base_link to map frame
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        "map", "base_link", tf2::TimePointZero,
        transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "LocalLayer: %s", ex.what());
      return;
    }
    
    // Copy map data given proper transformations
    tf2::Transform tf2_transform;
    tf2::fromMsg(transform.transform, tf2_transform);
    
    // Iterate through the local map (base_link frame)
    for (unsigned int local_i = 0; local_i < size_x_; ++local_i) {
      for (unsigned int local_j = 0; local_j < size_y_; ++local_j) {
        // Convert local map coordinates to world coordinates in base_link frame
        double local_wx, local_wy;
        mapToWorld(local_i, local_j, local_wx, local_wy);
        
        // Transform from base_link frame to map frame
        tf2::Vector3 p(local_wx, local_wy, 0);
        p = tf2_transform * p;
        
        // Convert map frame world coordinates to master grid coordinates
        if (layered_costmap_->getCostmap()->worldToMap(p.x(), p.y(), mx, my)) {
          // Check if the transformed coordinates are within the update bounds
          if (mx >= static_cast<unsigned int>(min_i) && mx < static_cast<unsigned int>(max_i) &&
              my >= static_cast<unsigned int>(min_j) && my < static_cast<unsigned int>(max_j)) {
            
            unsigned char local_cost = getCost(local_i, local_j);
            
            // Only update when local_layer provides known information (occupied or free)
            if (local_cost == LETHAL_OBSTACLE || local_cost == FREE_SPACE) {
              if (!use_maximum_) {
                master_grid.setCost(mx, my, local_cost);
              } else {
                master_grid.setCost(mx, my, std::max(local_cost, master_grid.getCost(mx, my)));
              }
            }
            // For unknown areas (NO_INFORMATION), keep original values from static_layer
          }
        }
      }
    }
  } else if (!layered_costmap_->isRolling()) {
    // Original logic for map frame when not rolling
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    
    // Override static_layer with local_layer known information
    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        unsigned char local_cost = getCost(i, j);
        
        // Only update when local_layer provides known information (occupied or free)
        if (local_cost == LETHAL_OBSTACLE || local_cost == FREE_SPACE) {
          if (!use_maximum_) {
            master_grid.setCost(i, j, local_cost);
          } else {
            master_grid.setCost(i, j, std::max(local_cost, master_grid.getCost(i, j)));
          }
        }
        // For unknown areas (NO_INFORMATION), keep original values from static_layer
      }
    }
  } else {
    // Original logic for rolling window with map frame
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        map_frame_, global_frame_, tf2::TimePointZero,
        transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "LocalLayer: %s", ex.what());
      return;
    }
    // Copy map data given proper transformations
    tf2::Transform tf2_transform;
    tf2::fromMsg(transform.transform, tf2_transform);
    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        tf2::Vector3 p(wx, wy, 0);
        p = tf2_transform * p;
        // Set master_grid with cell from map
        if (worldToMap(p.x(), p.y(), mx, my)) {
          unsigned char local_cost = getCost(mx, my);
          
          // Only update when local_layer provides known information (occupied or free)
          if (local_cost == LETHAL_OBSTACLE || local_cost == FREE_SPACE) {
            if (!use_maximum_) {
              master_grid.setCost(i, j, local_cost);
            } else {
              master_grid.setCost(i, j, std::max(local_cost, master_grid.getCost(i, j)));
            }
          }
          // For unknown areas (NO_INFORMATION), keep original values from static_layer
        }
      }
    }
  }
  current_ = true;
}

rcl_interfaces::msg::SetParametersResult
LocalLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_name == name_ + "." + "local_map_topic") {
      RCLCPP_WARN(
        logger_, "%s is not a dynamic parameter "
        "cannot be changed while running. Rejecting parameter update.", param_name.c_str());
    } else if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + "." + "transform_tolerance") {
        transform_tolerance_ = tf2::durationFromSec(parameter.as_double());
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        x_ = y_ = 0;
        width_ = size_x_;
        height_ = size_y_;
        has_updated_data_ = true;
        current_ = false;
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "footprint_clearing_enabled") {
        footprint_clearing_enabled_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}
}