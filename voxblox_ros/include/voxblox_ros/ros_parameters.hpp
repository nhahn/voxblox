#pragma once

#include <iostream>
#include <sstream>
#include <vector>
#include "minkindr_conversions/kindr_xml.h"

#include <rclcpp/rclcpp.hpp>
#include "voxblox_ros/transformer.h"
#include <voxblox/alignment/icp.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/mesh/mesh_integrator.h>

namespace voxblox {
inline std::vector<std::vector<double>> convert_to_matrix(
    const std::string& multilineString) {
  std::vector<std::vector<double>> result;

  std::istringstream iss(multilineString);
  std::string line;
  while (std::getline(iss, line)) {
    std::istringstream lineStream(line);
    std::vector<double> row;
    std::string valueStr;
    while (std::getline(lineStream, valueStr, ',')) {
      double value = std::stod(valueStr);
      row.push_back(value);
    }
    result.push_back(row);
  }

  return result;
}

inline TsdfMap::Config getTsdfMapConfigFromRosParam(rclcpp::Node * node_) {
  TsdfMap::Config tsdf_config;

  /**
   * Workaround for OS X on mac mini not having specializations for float
   * for some reason.
   */
  double voxel_size = tsdf_config.tsdf_voxel_size;
  int voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  voxel_size = node_->declare_parameter("voxblox.tsdf_voxel_size", voxel_size);
  voxels_per_side =
      node_->declare_parameter("voxblox.tsdf_voxels_per_side", voxels_per_side);
  if (!isPowerOfTwo(voxels_per_side)) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "voxels_per_side must be a power of 2, setting to default value");
    voxels_per_side = tsdf_config.tsdf_voxels_per_side;
  }

  tsdf_config.tsdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
  tsdf_config.tsdf_voxels_per_side = voxels_per_side;

  return tsdf_config;
}

inline voxblox::Transformation get_parameter_as_transformation(
    rclcpp::Node * node, std::string parameter_name) {
  voxblox::Transformation transformation;
  auto matrix_string = node->declare_parameter(parameter_name, "");
  if (!matrix_string.empty()) {
    std::vector<std::vector<double>> matrix =
        voxblox::convert_to_matrix(matrix_string);
    kindr::minimal::vectorOfVectorsToKindr(matrix, &transformation);
  }
  return transformation;
}

inline MeshIntegratorConfig getMeshIntegratorConfigFromRosParam(rclcpp::Node * node_) {
    MeshIntegratorConfig mesh_integrator_config;
    mesh_integrator_config.min_weight = node_->declare_parameter(
        "voxblox.mesh_min_weight", mesh_integrator_config.min_weight);
    mesh_integrator_config.use_color = node_->declare_parameter(
        "voxblox.mesh_use_color", mesh_integrator_config.use_color);
    return mesh_integrator_config;
  }

  inline TsdfIntegratorBase::Config getTsdfIntegratorConfigFromRosParam(
      rclcpp::Node * node_) {
    TsdfIntegratorBase::Config integrator_config;
    TsdfMap::Config tsdf_config;
    double voxel_size = tsdf_config.tsdf_voxel_size;
    node_->get_parameter("voxblox.tsdf_voxel_size", voxel_size);
    integrator_config.voxel_carving_enabled = true;
    integrator_config.default_truncation_distance =
        voxel_size * 4;

    integrator_config.voxel_carving_enabled = node_->declare_parameter(
        "voxblox.voxel_carving_enabled", integrator_config.voxel_carving_enabled);
    integrator_config.max_ray_length_m = node_->declare_parameter(
        "voxblox.max_ray_length_m", integrator_config.max_ray_length_m);
    integrator_config.min_ray_length_m = node_->declare_parameter(
        "voxblox.min_ray_length_m", integrator_config.min_ray_length_m);
    integrator_config.use_const_weight = node_->declare_parameter(
        "voxblox.use_const_weight", integrator_config.use_const_weight);
    integrator_config.use_weight_dropoff = node_->declare_parameter(
        "voxblox.use_weight_dropoff", integrator_config.use_weight_dropoff);
    integrator_config.allow_clear =
        node_->declare_parameter("voxblox.allow_clear", integrator_config.allow_clear);
    integrator_config.start_voxel_subsampling_factor = node_->declare_parameter(
        "voxblox.start_voxel_subsampling_factor",
        integrator_config.start_voxel_subsampling_factor);
    integrator_config.max_consecutive_ray_collisions = node_->declare_parameter(
        "voxblox.max_consecutive_ray_collisions",
        integrator_config.max_consecutive_ray_collisions);
    integrator_config.clear_checks_every_n_frames =
        node_->declare_parameter("voxblox.clear_checks_every_n_frames",
                                 integrator_config.clear_checks_every_n_frames);
    integrator_config.max_integration_time_s = node_->declare_parameter(
        "voxblox.max_integration_time_s", integrator_config.max_integration_time_s);
    integrator_config.enable_anti_grazing = node_->declare_parameter(
        "voxblox.anti_grazing", integrator_config.enable_anti_grazing);
    integrator_config.use_sparsity_compensation_factor =
        node_->declare_parameter(
            "voxblox.use_sparsity_compensation_factor",
            integrator_config.use_sparsity_compensation_factor);
    integrator_config.sparsity_compensation_factor = node_->declare_parameter(
        "voxblox.sparsity_compensation_factor",
        integrator_config.sparsity_compensation_factor);
    integrator_config.integration_order_mode = node_->declare_parameter(
        "voxblox.integration_order_mode", integrator_config.integration_order_mode);

    integrator_config.integrator_threads = node_->declare_parameter(
        "voxblox.integrator_threads",
        static_cast<int>(integrator_config.integrator_threads));
    integrator_config.sensor_horizontal_resolution = node_->declare_parameter(
        "voxblox.sensor_horizontal_resolution",
        integrator_config.sensor_horizontal_resolution);
    integrator_config.sensor_vertical_resolution = node_->declare_parameter(
        "voxblox.sensor_vertical_resolution",
        integrator_config.sensor_vertical_resolution);
    integrator_config.sensor_vertical_field_of_view_degrees = node_->declare_parameter(
        "voxblox.sensor_vertical_field_of_view_degrees",
        integrator_config.sensor_vertical_field_of_view_degrees);        
    integrator_config.use_missing_points_for_clearing = node_->declare_parameter(
        "voxblox.use_missing_points_for_clearing",
        integrator_config.use_missing_points_for_clearing);    

    double truncation_distance = integrator_config.default_truncation_distance;
    truncation_distance =
        node_->declare_parameter("voxblox.truncation_distance", truncation_distance);
    integrator_config.default_truncation_distance =
        static_cast<float>(truncation_distance);

    double max_weight = integrator_config.max_weight;
    max_weight = node_->declare_parameter("voxblox.max_weight", max_weight);
    integrator_config.max_weight = static_cast<float>(max_weight);

    return integrator_config;
  }

  inline ICP::Config getICPConfigFromRosParam(rclcpp::Node * node_) {
    ICP::Config icp_config;

    icp_config.min_match_ratio = node_->declare_parameter(
        "voxblox.icp_min_match_ratio", icp_config.min_match_ratio);
    icp_config.subsample_keep_ratio = node_->declare_parameter(
        "voxblox.icp_subsample_keep_ratio", icp_config.subsample_keep_ratio);
    icp_config.mini_batch_size = node_->declare_parameter(
        "voxblox.icp_mini_batch_size", icp_config.mini_batch_size);
    icp_config.refine_roll_pitch = node_->declare_parameter(
        "voxblox.icp_refine_roll_pitch", icp_config.refine_roll_pitch);
    icp_config.inital_translation_weighting =
        node_->declare_parameter("voxblox.icp_inital_translation_weighting",
                                 icp_config.inital_translation_weighting);
    icp_config.inital_rotation_weighting = node_->declare_parameter(
        "voxblox.icp_inital_rotation_weighting", icp_config.inital_rotation_weighting);

    return icp_config;
  }


  inline EsdfMap::Config getEsdfMapConfigFromRosParam(rclcpp::Node * node_) {
    // EsdfMap::Config esdf_config;
    // esdf_config.esdf_voxel_size = node_->tsdf_config.tsdf_voxel_size;
    // esdf_config.esdf_voxels_per_side =
    // node_->tsdf_config.tsdf_voxels_per_side; return esdf_config;

    /**
     * Workaround for OS X on mac mini not having specializations for float
     * for some reason.
     */
    EsdfMap::Config esdf_config;
    double voxel_size = esdf_config.esdf_voxel_size;
    int voxels_per_side = esdf_config.esdf_voxels_per_side;
    voxel_size = node_->declare_parameter("voxblox.esdf_voxel_size", voxel_size);
    voxels_per_side =
        node_->declare_parameter("voxblox.esdf_voxels_per_side", voxels_per_side);
    if (!isPowerOfTwo(voxels_per_side)) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "voxels_per_side must be a power of 2, setting to default value");
      voxels_per_side = esdf_config.esdf_voxels_per_side;
    }

    esdf_config.esdf_voxel_size = static_cast<FloatingPoint>(voxel_size);
    esdf_config.esdf_voxels_per_side = voxels_per_side;

    return esdf_config;
  }

    inline EsdfIntegrator::Config getEsdfIntegratorConfigFromRosParam(rclcpp::Node * node_,
    const voxblox::TsdfIntegratorBase::Config& tsdf_integrator_config) {
    EsdfIntegrator::Config esdf_integrator_config;
    esdf_integrator_config.min_distance_m =
        tsdf_integrator_config.default_truncation_distance / 2.0;

    esdf_integrator_config.full_euclidean_distance = node_->declare_parameter(
        "voxblox.esdf_euclidean_distance",
        esdf_integrator_config.full_euclidean_distance);
    esdf_integrator_config.max_distance_m = node_->declare_parameter(
        "voxblox.esdf_max_distance_m", esdf_integrator_config.max_distance_m);
    esdf_integrator_config.min_distance_m = node_->declare_parameter(
        "voxblox.esdf_min_distance_m", esdf_integrator_config.min_distance_m);
    esdf_integrator_config.default_distance_m = node_->declare_parameter(
        "voxblox.esdf_default_distance_m", esdf_integrator_config.default_distance_m);
    esdf_integrator_config.min_diff_m = node_->declare_parameter(
        "voxblox.esdf_min_diff_m", esdf_integrator_config.min_diff_m);
    esdf_integrator_config.clear_sphere_radius = node_->declare_parameter(
        "voxblox.clear_sphere_radius", esdf_integrator_config.clear_sphere_radius);
    esdf_integrator_config.occupied_sphere_radius =
        node_->declare_parameter("voxblox.occupied_sphere_radius",
                                 esdf_integrator_config.occupied_sphere_radius);
    esdf_integrator_config.add_occupied_crust = node_->declare_parameter(
        "voxblox.esdf_add_occupied_crust", esdf_integrator_config.add_occupied_crust);

    if (esdf_integrator_config.default_distance_m <
        esdf_integrator_config.max_distance_m) {
      esdf_integrator_config.default_distance_m =
          esdf_integrator_config.max_distance_m;
    }

    return esdf_integrator_config;
  }
}  // namespace ros_parameters
