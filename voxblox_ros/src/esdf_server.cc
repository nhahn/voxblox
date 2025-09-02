#include "voxblox_ros/esdf_server.h"

#include "voxblox_ros/node_helper.h"
#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ros_parameters.hpp"

namespace voxblox {

EsdfServer::EsdfServer(rclcpp::Node::SharedPtr node)
    : node_(node),
      TsdfServer(node),
      clear_sphere_for_planning_(false),
      publish_esdf_map_(false),
      publish_traversable_(false),
      traversability_radius_(1.0),
      incremental_update_(true),
      num_subscribers_esdf_map_(0) {
  const EsdfMap::Config esdf_config = getEsdfMapConfigFromRosParam(node.get());
  const EsdfIntegrator::Config esdf_integrator_config =
      getEsdfIntegratorConfigFromRosParam(node.get(), tsdf_integrator_config);

  // Set up map and integrator.
  esdf_map_.reset(new EsdfMap(esdf_config));
  esdf_integrator_.reset(new EsdfIntegrator(esdf_integrator_config,
                                            tsdf_map_->getTsdfLayerPtr(),
                                            esdf_map_->getEsdfLayerPtr()));

  setupRos();
}

std::shared_ptr<EsdfMap> EsdfServer::getEsdfMapPtr() { return esdf_map_; }

std::shared_ptr<const EsdfMap> EsdfServer::getEsdfMapPtr() const {
  return esdf_map_;
}

void EsdfServer::setupRos() {


  // Get namespace and node name
  std::string robot_ns = node_->get_namespace();  // e.g., "/husky"
  std::string node_ns = node_->get_name();     // e.g., "tsdf_server"

  // Ensure namespace ends with no trailing slash
  if (!robot_ns.empty() && robot_ns.back() == '/')
    robot_ns.pop_back();

  // Set up publisher.
  esdf_pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      robot_ns + "/" + node_ns + "/esdf_pointcloud", 1);
  esdf_slice_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(robot_ns + "/" + node_ns + "/esdf_slice", 1);
  traversable_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(robot_ns + "/" + node_ns + "/traversable", 1);

  esdf_map_pub_ =
      node_->create_publisher<voxblox_msgs::msg::Layer>(robot_ns + "/" + node_ns + "/esdf_map_out", 1);

  // Set up subscriber.
  esdf_map_sub_ = node_->create_subscription<voxblox_msgs::msg::Layer>(
      robot_ns + "/" + node_ns + "/esdf_map_in", 1,
      std::bind(&EsdfServer::esdfMapCallback, this, std::placeholders::_1));

  // Whether to clear each new pose as it comes in, and then set a sphere
  // around it to occupied.
  clear_sphere_for_planning_ = node_->declare_parameter(
      "clear_sphere_for_planning", clear_sphere_for_planning_);
  publish_esdf_map_ =
      node_->declare_parameter("publish_esdf_map", publish_esdf_map_);

  // Special output for traversable voxels. Publishes all voxels with distance
  // at least traversibility radius.
  publish_traversable_ =
      node_->declare_parameter("publish_traversable", publish_traversable_);
  traversability_radius_ =
      node_->declare_parameter("traversability_radius", traversability_radius_);

  double update_esdf_every_n_sec = 1.0;
  update_esdf_every_n_sec = node_->declare_parameter("update_esdf_every_n_sec",
                                                    update_esdf_every_n_sec);

  if (update_esdf_every_n_sec > 0.0) {
    update_esdf_timer_ = rclcpp::create_timer(
        node_, node_->get_clock(),
        rclcpp::Duration::from_seconds(update_esdf_every_n_sec),
        std::bind(&EsdfServer::updateEsdfEvent, this));
  }
}

void EsdfServer::publishAllUpdatedEsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;

  sensor_msgs::msg::PointCloud2 pointcloud_message;
  pcl::toROSMsg(pointcloud, pointcloud_message);

  esdf_pointcloud_pub_->publish(pointcloud_message);
}

void EsdfServer::publishSlices() {
  TsdfServer::publishSlices();

  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  constexpr int kZAxisIndex = 2;
  createDistancePointcloudFromEsdfLayerSlice(
      esdf_map_->getEsdfLayer(), kZAxisIndex, slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;

  sensor_msgs::msg::PointCloud2 pointcloud_message;
  pcl::toROSMsg(pointcloud, pointcloud_message);

  esdf_slice_pub_->publish(pointcloud_message);
}

bool EsdfServer::generateEsdfCallback(
    std_srvs::srv::Empty::Request::SharedPtr /*request*/,      // NOLINT
    std_srvs::srv::Empty::Response::SharedPtr /*response*/) {  // NOLINT
  const bool clear_esdf = true;
  if (clear_esdf) {
    esdf_integrator_->updateFromTsdfLayerBatch();
  } else {
    const bool clear_updated_flag = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag);
  }
  publishAllUpdatedEsdfVoxels();
  publishSlices();
  return true;
}

void EsdfServer::updateEsdfEvent() { updateEsdf(); }

void EsdfServer::publishPointclouds() {
  publishAllUpdatedEsdfVoxels();
  if (publish_slices_) {
    publishSlices();
  }

  if (publish_traversable_) {
    publishTraversable();
  }

  TsdfServer::publishPointclouds();
}

void EsdfServer::publishTraversable() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  createFreePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(),
                                    traversability_radius_, &pointcloud);
  pointcloud.header.frame_id = world_frame_;

  sensor_msgs::msg::PointCloud2 pointcloud_message;
  pcl::toROSMsg(pointcloud, pointcloud_message);

  traversable_pub_->publish(pointcloud_message);
}

void EsdfServer::publishMap(bool reset_remote_map) {
  TsdfServer::publishMap();

  if (!publish_esdf_map_) {
    return;
  }

  int subscribers = this->esdf_map_pub_->get_subscription_count();
  if (subscribers > 0) {
    if (num_subscribers_esdf_map_ < subscribers) {
      // Always reset the remote map and send all when a new subscriber
      // subscribes. A bit of overhead for other subscribers, but better than
      // inconsistent map states.
      reset_remote_map = true;
    }
    const bool only_updated = !reset_remote_map;
    timing::Timer publish_map_timer("map/publish_esdf");
    voxblox_msgs::msg::Layer layer_msg;
    serializeLayerAsMsg<EsdfVoxel>(this->esdf_map_->getEsdfLayer(),
                                   only_updated, &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    this->esdf_map_pub_->publish(layer_msg);
    publish_map_timer.Stop();
  }
  num_subscribers_esdf_map_ = subscribers;
}

bool EsdfServer::saveMap(const std::string& file_path) {
  // Output TSDF map first, then ESDF.
  const bool success = TsdfServer::saveMap(file_path);

  constexpr bool kClearFile = false;
  return success &&
         io::SaveLayer(esdf_map_->getEsdfLayer(), file_path, kClearFile);
}

bool EsdfServer::loadMap(const std::string& file_path) {
  // Load in the same order: TSDF first, then ESDF.
  bool success = TsdfServer::loadMap(file_path);

  constexpr bool kMultipleLayerSupport = true;
  return success &&
         io::LoadBlocksFromFile(
             file_path, Layer<EsdfVoxel>::BlockMergingStrategy::kReplace,
             kMultipleLayerSupport, esdf_map_->getEsdfLayerPtr());
}

void EsdfServer::updateEsdf() {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_esdf = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
  }
}

void EsdfServer::updateEsdfBatch(bool full_euclidean) {
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    esdf_integrator_->setFullEuclidean(full_euclidean);
    esdf_integrator_->updateFromTsdfLayerBatch();
  }
}

float EsdfServer::getEsdfMaxDistance() const {
  return esdf_integrator_->getEsdfMaxDistance();
}

void EsdfServer::setEsdfMaxDistance(float max_distance) {
  esdf_integrator_->setEsdfMaxDistance(max_distance);
}

float EsdfServer::getTraversabilityRadius() const {
  return traversability_radius_;
}

void EsdfServer::setTraversabilityRadius(float traversability_radius) {
  traversability_radius_ = traversability_radius;
}

void EsdfServer::newPoseCallback(const Transformation& T_G_C) {
  TsdfServer::newPoseCallback(T_G_C);

  if (clear_sphere_for_planning_) {
    esdf_integrator_->addNewRobotPosition(T_G_C.getPosition());
  }

  timing::Timer block_remove_timer("remove_distant_blocks");
  esdf_map_->getEsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  block_remove_timer.Stop();
}

void EsdfServer::esdfMapCallback(
    const voxblox_msgs::msg::Layer::SharedPtr layer_msg) {
  timing::Timer receive_map_timer("map/receive_esdf");

  bool success =
      deserializeMsgToLayer<EsdfVoxel>(layer_msg.get(), esdf_map_->getEsdfLayerPtr());

  if (!success) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10,
                          "Got an invalid ESDF map message!");
  } else {
    RCLCPP_INFO_ONCE(node_->get_logger(), "Got an ESDF map from ROS topic!");
    if (publish_pointclouds_) {
      publishPointclouds();
    }
  }
}

void EsdfServer::clear() {
  esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
  esdf_integrator_->clear();
  CHECK_EQ(esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks(), 0u);

  TsdfServer::clear();

  // Publish a message to reset the map to all subscribers.
  constexpr bool kResetRemoteMap = true;
  publishMap(kResetRemoteMap);
}

void EsdfServer::pruneMap() {
  TsdfServer::pruneMap();

  size_t num_pruned_blocks = 0u;
  BlockIndexList esdf_blocks_;
  esdf_map_->getEsdfLayerPtr()->getAllAllocatedBlocks(&esdf_blocks_);
  for (const BlockIndex& esdf_block_index : esdf_blocks_) {
    if (!tsdf_map_->getTsdfLayer().hasBlock(esdf_block_index)) {
      // Avoid pruning blocks that are already queued to be updated
      // NOTE: This is mainly important when using clear spheres for planning
      if (!esdf_integrator_->blockQueuedForUpdate(esdf_block_index)) {
        ++num_pruned_blocks;
        esdf_map_->getEsdfLayerPtr()->removeBlock(esdf_block_index);
      }
    }
  }

  RCLCPP_DEBUG_STREAM(node_->get_logger(),
                       "Pruned " << num_pruned_blocks << " ESDF blocks");
}

}  // namespace voxblox
