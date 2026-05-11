/*
 * ElevationMap.hpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#pragma once

// Grid Map
#include <grid_map_msgs/srv/get_grid_map.hpp>
#include <grid_map_msgs/srv/process_file.hpp>
#include <grid_map_msgs/srv/set_grid_map.hpp>

// ROS
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/string.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"
#include "elevation_mapping/input_sources/InputSourceManager.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

namespace elevation_mapping
{

  /*!
   * The elevation mapping main class. Coordinates the ROS interfaces, the timing,
   * and the data handling between the other classes.
   */
  class ElevationMapping
  {
  public:
    /*!
     * Constructor.
     *
     * @param nodeHandle the ROS node handle.
     */
    explicit ElevationMapping(std::shared_ptr<rclcpp::Node> &nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~ElevationMapping();

    /*!
     * Callback function for new data to be added to the elevation map.
     *
     * @param pointCloudMsg    The point cloud to be fused with the existing data.
     * @param publishPointCloud If true, publishes the pointcloud after updating the map.
     * @param sensorProcessor_ The sensorProcessor to use in this callback.
     */
    void pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointCloudMsg, bool publishPointCloud,
                            const SensorProcessorBase::Ptr &sensorProcessor_);

    /*!
     * Callback function for the update timer. Forces an update of the map from
     * the robot's motion if no new measurements are received for a certain time
     * period.
     *
     */
    void mapUpdateTimerCallback();

    /*!
     * Callback function for cleaning map based on visibility ray tracing.
     *
     */
    void visibilityCleanupCallback();

    /*!
     * ROS service callback function to enable updates of the elevation map.
     *
     * @param request     The ROS service request.
     * @param response    The ROS service response.
     * @return true if successful.
     */
    bool enableUpdatesServiceCallback(const std::shared_ptr<rmw_request_id_t>,
                                      const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                      std::shared_ptr<std_srvs::srv::Empty::Response>);

    /*!
     * ROS service callback function to disable updates of the elevation map.
     *
     * @param request     The ROS service request.
     * @param response    The ROS service response.
     * @return true if successful.
     */
    bool disableUpdatesServiceCallback(const std::shared_ptr<rmw_request_id_t>,
                                       const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                       std::shared_ptr<std_srvs::srv::Empty::Response>);

    /*!
     * ROS service callback function to clear all data of the elevation map.
     *
     * @param request     The ROS service request.
     * @param response    The ROS service response.
     * @return true if successful.
     */
    bool clearMapServiceCallback(const std::shared_ptr<rmw_request_id_t>,
                                 const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                 std::shared_ptr<std_srvs::srv::Empty::Response>);

    /*!
     * ROS service callback function to allow for setting the individual layers of the elevation map through a service call.
     * The layer mask can be used to only set certain cells and not the entire map. Cells
     * containing NAN in the mask are not set, all the others are set. If the layer mask is
     * not supplied, the entire map will be set in the intersection of both maps. The
     * provided map can be of different size and position than the map that will be altered.
     *
     * @param request    The ROS service request.
     * @param response   The ROS service response.
     * @return true if successful.
     */
    bool maskedReplaceServiceCallback(std::shared_ptr<grid_map_msgs::srv::SetGridMap::Request> request, std::shared_ptr<grid_map_msgs::srv::SetGridMap::Response> response);

    /*!
     * ROS service callback function to save the grid map with all layers to a ROS bag file.
     *
     * @param request   The ROS service request.
     * @param response  The ROS service response.
     * @return true if successful.
     */
    bool saveMapServiceCallback(std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request, std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response);

    /*!
     * ROS service callback function to load the grid map with all layers from a ROS bag file.
     *
     * @param request     The ROS service request.
     * @param response    The ROS service response.
     * @return true if successful.
     */
    bool loadMapServiceCallback(std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request, std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response);

  private:
    /*!
     * Reads and verifies the ROS parameters.
     *
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * Performs the initialization procedure.
     *
     * @return true if successful.
     */
    bool initialize();

    /**
     * Sets up the subscribers for both robot poses and input data.
     */
    void setupSubscribers();

    /**
     * Sets up the services.
     */
    void setupServices();

    /**
     * Sets up the timers.
     */
    void setupTimers();

    /*!
     * Update the elevation map from the robot motion up to a certain time.
     *
     * @param time    Time to which the map is updated to.
     * @return true if successful.
     */
    bool updatePrediction(const rclcpp::Time &time);

    /*!
     * Updates the location of the map to follow the tracking point. Takes care
     * of the data handling the goes along with the relocalization.
     *
     * @return true if successful.
     */
    bool updateMapLocation();

    /*!
     * Initializes a submap around the robot of the elevation map with a constant height.
     */
    bool initializeElevationMap();

    //! ROS nodehandle.
    std::shared_ptr<rclcpp::Node> nodeHandle_;

  protected:
    //! Input sources.
    InputSourceManager inputSources_;
    //! ROS subscribers.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSubscriber_; //!< Deprecated, use input_source instead.
    message_filters::Subscriber<nav_msgs::msg::Odometry> robotOdomSubscriber_;

    //! ROS service servers.
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enableUpdatesService_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disableUpdatesService_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clearMapService_;
    rclcpp::Service<grid_map_msgs::srv::SetGridMap>::SharedPtr maskedReplaceService_;
    rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr saveMapService_;
    rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr loadMapService_;

    //! Cache for the robot pose messages.
    message_filters::Cache<nav_msgs::msg::Odometry> robotOdomCache_;

    //! Size of the cache for the robot pose messages.
    int robotOdomCacheSize_;

    //! Frame ID of the elevation map
    std::string mapFrameId_;

    //! TF listener and buffer.
    std::shared_ptr<tf2_ros::Buffer> transformBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> transformListener_;

    //! Point which the elevation map follows.
    kindr::Position3D trackPoint_;
    std::string trackPointFrameId_;

    //! ROS topics for subscriptions.
    std::string pointCloudTopic_; //!< Deprecated, use input_source instead.
    std::string robotOdomTopic_;

    //! Elevation map.
    ElevationMap map_;

    //! Robot motion elevation map updater.
    RobotMotionMapUpdater robotMotionMapUpdater_;

    //! If true, robot motion updates are ignored.
    bool ignoreRobotMotionUpdates_;

    //! If false, elevation mapping stops updating
    bool updatesEnabled_;

    //! Time of the last point cloud update.
    rclcpp::Time lastPointCloudUpdateTime_;

    //! Timer for the robot motion update.
    rclcpp::TimerBase::SharedPtr mapUpdateTimer_;

    //! Maximum time that the map will not be updated.
    rclcpp::Duration maxNoUpdateDuration_;

    //! Time tolerance for updating the map with data before the last update.
    //! This is useful when having multiple sensors adding data to the map.
    rclcpp::Duration timeTolerance_;

    //! Timer for the raytracing cleanup.
    rclcpp::TimerBase::SharedPtr visibilityCleanupTimer_;

    //! Duration for the raytracing cleanup timer.
    rclcpp::Duration visibilityCleanupTimerDuration_;

    //! Callback group for raytracing cleanup thread.
    rclcpp::CallbackGroup::SharedPtr visibilityCleanupGroup_;

    //! Becomes true when corresponding poses and point clouds can be found
    bool receivedFirstMatchingPointcloudAndPose_;

    //! Width and height of submap of the elevation map with a constant height
    double lengthInXInitSubmap_;
    double lengthInYInitSubmap_;

    //! Margin of submap of the elevation map with a constant height
    double marginInitSubmap_;

    //! Target frame to get the init height of the elevation map
    std::string targetFrameInitSubmap_;

    //! Additional offset of the height value
    double initSubmapHeightOffset_;
  };

} // namespace elevation_mapping
