#include <gtest/gtest.h>
#include <cmath>

#include "../src/nodes/planner_functions.h"

#include "../src/nodes/common.h"

using namespace avoidance;

TEST(PlannerFunctions, generateNewHistogramEmpty) {
  // GIVEN: an empty pointcloud
  pcl::PointCloud<pcl::PointXYZ> empty_cloud;
  Histogram histogram_output = Histogram(ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;

  // WHEN: we build a histogram
  generateNewHistogram(histogram_output, empty_cloud, location);

  // THEN: the histogram should be all zeros
  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      EXPECT_DOUBLE_EQ(0.0, histogram_output.get_bin(e, z));
    }
  }
}

TEST(PlannerFunctions, generateNewHistogramSpecificCells) {
  // GIVEN: a pointcloud with an object of one cell size
  Histogram histogram_output = Histogram(ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;
  double distance = 1.0;

  std::vector<double> e_angle_filled = {-90, -30, 0, 20, 40, 90};
  std::vector<double> z_angle_filled = {-180, -50, 0, 59, 100, 175};
  std::vector<Eigen::Vector3f> middle_of_cell;

  for (int i = 0; i < e_angle_filled.size(); i++) {
    for (int j = 0; j < z_angle_filled.size(); j++) {
      middle_of_cell.push_back(fromPolarToCartesian(e_angle_filled[i],
                                                    z_angle_filled[j], distance,
                                                    location.pose.position));
    }
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < middle_of_cell.size(); i++) {
    for (int j = 0; j < 1; j++) {
      cloud.push_back(toXYZ(middle_of_cell[i]));
    }
  }

  // WHEN: we build a histogram
  generateNewHistogram(histogram_output, cloud, location);

  // THEN: the filled cells in the histogram should be one and the others be
  // zeros

  std::vector<int> e_index;
  std::vector<int> z_index;
  for (int i = 0; i < e_angle_filled.size(); i++) {
    e_index.push_back(elevationAngletoIndex((int)e_angle_filled[i], ALPHA_RES));
    z_index.push_back(azimuthAngletoIndex((int)z_angle_filled[i], ALPHA_RES));
  }

  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      bool e_found =
          std::find(e_index.begin(), e_index.end(), e) != e_index.end();
      bool z_found =
          std::find(z_index.begin(), z_index.end(), z) != z_index.end();
      if (e_found && z_found) {
        EXPECT_DOUBLE_EQ(1.0, histogram_output.get_bin(e, z)) << z << ", " << e;
      } else {
        EXPECT_DOUBLE_EQ(0.0, histogram_output.get_bin(e, z)) << z << ", " << e;
      }
    }
  }
}

TEST(PlannerFunctions, calculateFOV) {
  // GIVEN: the horizontal and vertical Field of View, the vehicle yaw and pitc
  double h_fov = 90.0;
  double v_fov = 45.0;
  double yaw_z_greater_grid_length =
      3.14;  // z_FOV_max >= GRID_LENGTH_Z && z_FOV_min >= GRID_LENGTH_Z
  double yaw_z_max_greater_grid =
      -2.3;  // z_FOV_max >= GRID_LENGTH_Z && z_FOV_min < GRID_LENGTH_Z
  double yaw_z_min_smaller_zero = 3.9;  // z_FOV_min < 0 && z_FOV_max >= 0
  double yaw_z_smaller_zero = 5.6;      // z_FOV_max < 0 && z_FOV_min < 0
  double pitch = 0.0;

  // WHEN: we calculate the Field of View
  std::vector<int> z_FOV_idx_z_greater_grid_length;
  std::vector<int> z_FOV_idx_z_max_greater_grid;
  std::vector<int> z_FOV_idx3_z_min_smaller_zero;
  std::vector<int> z_FOV_idx_z_smaller_zero;
  int e_FOV_min;
  int e_FOV_max;

  calculateFOV(h_fov, v_fov, z_FOV_idx_z_greater_grid_length, e_FOV_min,
               e_FOV_max, yaw_z_greater_grid_length, pitch);
  calculateFOV(h_fov, v_fov, z_FOV_idx_z_max_greater_grid, e_FOV_min, e_FOV_max,
               yaw_z_max_greater_grid, pitch);
  calculateFOV(h_fov, v_fov, z_FOV_idx3_z_min_smaller_zero, e_FOV_min,
               e_FOV_max, yaw_z_min_smaller_zero, pitch);
  calculateFOV(h_fov, v_fov, z_FOV_idx_z_smaller_zero, e_FOV_min, e_FOV_max,
               yaw_z_smaller_zero, pitch);

  // THEN:
  std::vector<int> output_z_greater_grid_length = {
      7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
  std::vector<int> output_z_max_greater_grid = {0, 1, 2,  3,  4,  5,  6, 7,
                                                8, 9, 10, 11, 12, 58, 59};
  std::vector<int> output_z_min_smaller_zero = {0, 1, 2,  3,  4,  5,  6, 7,
                                                8, 9, 10, 11, 12, 13, 59};
  std::vector<int> output_z_smaller_zero = {43, 44, 45, 46, 47, 48, 49, 50,
                                            51, 52, 53, 54, 55, 56, 57, 58};

  EXPECT_EQ(18, e_FOV_max);
  EXPECT_EQ(10, e_FOV_min);
  for (size_t i = 0; i < z_FOV_idx_z_greater_grid_length.size(); i++) {
    EXPECT_EQ(output_z_greater_grid_length.at(i),
              z_FOV_idx_z_greater_grid_length.at(i));
  }

  for (size_t i = 0; i < z_FOV_idx_z_max_greater_grid.size(); i++) {
    EXPECT_EQ(output_z_max_greater_grid.at(i),
              z_FOV_idx_z_max_greater_grid.at(i));
  }

  for (size_t i = 0; i < z_FOV_idx3_z_min_smaller_zero.size(); i++) {
    EXPECT_EQ(output_z_min_smaller_zero.at(i),
              z_FOV_idx3_z_min_smaller_zero.at(i));
  }

  for (size_t i = 0; i < z_FOV_idx_z_smaller_zero.size(); i++) {
    EXPECT_EQ(output_z_smaller_zero.at(i), z_FOV_idx_z_smaller_zero.at(i));
  }
}

TEST(PlannerFunctionsTests, findAllFreeDirections) {
  // GIVEN: empty histogram
  Histogram empty_histogram = Histogram(ALPHA_RES);
  double safety_radius = 15.0;
  int resolution_alpha = ALPHA_RES;
  // all the variables below aren't used by the test
  geometry_msgs::Point goal;
  goal.x = 2.0f;
  goal.y = 0.0f;
  goal.z = 2.0f;
  geometry_msgs::PoseStamped position;
  position.pose.position.x = 0.0f;
  position.pose.position.y = 0.0f;
  position.pose.position.z = 2.0f;
  geometry_msgs::Point position_old;
  position_old.x = -1.0f;
  position_old.y = 0.0;
  position_old.z = 2.0f;
  double goal_cost_param = 2.0;
  double smooth_cost_param = 1.5;
  double height_change_cost_param_adapted = 4.0;
  double height_change_cost_param = 4.0;
  bool only_yawed = false;

  // WHEN: we look for free directions
  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, path_waypoints;
  std::vector<float> cost_path_candidates;  // not needed

  findFreeDirections(empty_histogram, safety_radius, path_candidates,
                     path_selected, path_rejected, path_blocked, path_waypoints,
                     cost_path_candidates, goal, position, position_old,
                     goal_cost_param, smooth_cost_param,
                     height_change_cost_param_adapted, height_change_cost_param,
                     only_yawed, resolution_alpha);

  // THEN: all directions should be classified as path_candidates
  EXPECT_EQ(
      std::floor(360 / resolution_alpha) * std::floor(180 / resolution_alpha),
      path_candidates.cells.size());
  EXPECT_EQ(0, path_rejected.cells.size());
  EXPECT_EQ(0, path_blocked.cells.size());
}

TEST(PlannerFunctionsTests, findFreeDirectionsNoWrap) {
  // GIVEN: an histogram with a rejected cell, no wrapping of blocked cells
  int resolution_alpha = 5;
  double safety_radius = 5.0;
  Histogram histogram = Histogram(resolution_alpha);
  int obstacle_idx_e = 5;
  int obstacle_idx_z = 10;
  histogram.set_bin(obstacle_idx_e, obstacle_idx_z,
                    histogram.get_bin(obstacle_idx_e, obstacle_idx_z) + 1);
  // all the variables below aren't used by the test
  geometry_msgs::Point goal;
  goal.x = 2.0f;
  goal.y = 0.0f;
  goal.z = 2.0f;
  geometry_msgs::PoseStamped position;
  position.pose.position.x = 0.0f;
  position.pose.position.y = 0.0f;
  position.pose.position.z = 2.0f;
  geometry_msgs::Point position_old;
  position_old.x = -1.0f;
  position_old.y = 0.0;
  position_old.z = 2.0f;
  double goal_cost_param = 2.0;
  double smooth_cost_param = 1.5;
  double height_change_cost_param_adapted = 4.0;
  double height_change_cost_param = 4.0;
  bool only_yawed = false;

  // expected output
  std::vector<std::pair<int, int>> blocked;
  std::vector<std::pair<int, int>> free;
  for (int e = 0; e < (180 / resolution_alpha); e++) {
    for (int z = 0; z < (360 / resolution_alpha); z++) {
      if (!(z <= 11 && z >= 9 && e >= 4 && e <= 6)) {
        free.push_back(std::make_pair(e, z));
      } else {
        if (!(e == obstacle_idx_e && z == obstacle_idx_z)) {
          blocked.push_back(std::make_pair(e, z));
        }
      }
    }
  }

  // WHEN: we look for free directions
  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, path_waypoints;
  std::vector<float> cost_path_candidates;  // not needed

  findFreeDirections(histogram, safety_radius, path_candidates, path_selected,
                     path_rejected, path_blocked, path_waypoints,
                     cost_path_candidates, goal, position, position_old,
                     goal_cost_param, smooth_cost_param,
                     height_change_cost_param_adapted, height_change_cost_param,
                     only_yawed, resolution_alpha);

  // THEN: we should have one rejected cell and the 8 neightbooring cells
  // blocked
  for (auto cell_blocked : path_blocked.cells) {
    std::pair<int, int> cell_blocked_idx(
        elevationAngletoIndex(cell_blocked.x, resolution_alpha),
        azimuthAngletoIndex(cell_blocked.y, resolution_alpha));
    EXPECT_EQ(1, std::find(std::begin(blocked), std::end(blocked),
                           cell_blocked_idx) != std::end(blocked));
  }

  for (auto cell_rejected : path_rejected.cells) {
    EXPECT_EQ(1, (elevationAngletoIndex(cell_rejected.x, resolution_alpha) ==
                  obstacle_idx_e) &&
                     (azimuthAngletoIndex(cell_rejected.y, resolution_alpha) ==
                      obstacle_idx_z));
  }

  for (auto cell_candidate : path_candidates.cells) {
    std::pair<int, int> cell_candidate_idx(
        elevationAngletoIndex(cell_candidate.x, resolution_alpha),
        azimuthAngletoIndex(cell_candidate.y, resolution_alpha));
    EXPECT_EQ(1, std::find(std::begin(free), std::end(free),
                           cell_candidate_idx) != std::end(free));
  }

  EXPECT_EQ(1, path_rejected.cells.size());
  EXPECT_EQ(8, path_blocked.cells.size());
  EXPECT_EQ(
      std::floor(360 / resolution_alpha) * std::floor(180 / resolution_alpha) -
          1 - 8,
      path_candidates.cells.size());
}

TEST(PlannerFunctionsTests, findFreeDirectionsWrapLeft) {
  // GIVEN: a histogram with an obstacle at the first azimuth index
  int resolution_alpha = 5;
  double safety_radius = 5.0;
  Histogram histogram = Histogram(resolution_alpha);
  int obstacle_idx_e = 5;
  int obstacle_idx_z = 0;
  histogram.set_bin(obstacle_idx_e, obstacle_idx_z,
                    histogram.get_bin(obstacle_idx_e, obstacle_idx_z) + 1);

  // all the variables below aren't used by the test
  geometry_msgs::Point goal;
  goal.x = 2.0f;
  goal.y = 0.0f;
  goal.z = 2.0f;
  geometry_msgs::PoseStamped position;
  position.pose.position.x = 0.0f;
  position.pose.position.y = 0.0f;
  position.pose.position.z = 2.0f;
  geometry_msgs::Point position_old;
  position_old.x = -1.0f;
  position_old.y = 0.0;
  position_old.z = 2.0f;
  double goal_cost_param = 2.0;
  double smooth_cost_param = 1.5;
  double height_change_cost_param_adapted = 4.0;
  double height_change_cost_param = 4.0;
  bool only_yawed = false;

  // expected output
  std::vector<std::pair<int, int>> blocked;
  std::vector<std::pair<int, int>> free;
  for (int e = 0; e < (180 / resolution_alpha); e++) {
    for (int z = 0; z < (360 / resolution_alpha); z++) {
      if (!((z == 1 || z == 0 || z == 71) && e >= 4 && e <= 6)) {
        free.push_back(std::make_pair(e, z));
      } else {
        if (!(e == obstacle_idx_e && z == obstacle_idx_z)) {
          blocked.push_back(std::make_pair(e, z));
        }
      }
    }
  }

  // WHEN: we look for free directions
  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, path_waypoints;
  std::vector<float> cost_path_candidates;  // not needed

  findFreeDirections(histogram, safety_radius, path_candidates, path_selected,
                     path_rejected, path_blocked, path_waypoints,
                     cost_path_candidates, goal, position, position_old,
                     goal_cost_param, smooth_cost_param,
                     height_change_cost_param_adapted, height_change_cost_param,
                     only_yawed, resolution_alpha);

  // THEN: we should get one rejected cell, the five neighboring cells blocked
  // plus three other blocked cells at the same elevation index and the last
  // azimuth index
  for (auto cell_blocked : path_blocked.cells) {
    std::pair<int, int> cell_blocked_idx(
        elevationAngletoIndex(cell_blocked.x, resolution_alpha),
        azimuthAngletoIndex(cell_blocked.y, resolution_alpha));
    EXPECT_EQ(1, std::find(std::begin(blocked), std::end(blocked),
                           cell_blocked_idx) != std::end(blocked));
  }

  for (auto cell_rejected : path_rejected.cells) {
    EXPECT_EQ(1, (elevationAngletoIndex(cell_rejected.x, resolution_alpha) ==
                  obstacle_idx_e) &&
                     (azimuthAngletoIndex(cell_rejected.y, resolution_alpha) ==
                      obstacle_idx_z));
  }

  for (auto cell_candidate : path_candidates.cells) {
    std::pair<int, int> cell_candidate_idx(
        elevationAngletoIndex(cell_candidate.x, resolution_alpha),
        azimuthAngletoIndex(cell_candidate.y, resolution_alpha));
    EXPECT_EQ(1, std::find(std::begin(free), std::end(free),
                           cell_candidate_idx) != std::end(free));
  }
  EXPECT_EQ(1, path_rejected.cells.size());
  EXPECT_EQ(8, path_blocked.cells.size());
  EXPECT_EQ(
      std::floor(360 / resolution_alpha) * std::floor(180 / resolution_alpha) -
          1 - 8,
      path_candidates.cells.size());
}

TEST(PlannerFunctionsTests, findFreeDirectionsWrapRight) {
  // GIVEN: a histogram with a obstacle at the last azimuth index
  int resolution_alpha = 5;
  double safety_radius = 5.0;
  Histogram histogram = Histogram(resolution_alpha);
  int obstacle_idx_e = 5;
  int obstacle_idx_z = 71;
  histogram.set_bin(obstacle_idx_e, obstacle_idx_z,
                    histogram.get_bin(obstacle_idx_e, obstacle_idx_z) + 1);
  // all the variables below aren't used by the test
  geometry_msgs::Point goal;
  goal.x = 2.0f;
  goal.y = 0.0f;
  goal.z = 2.0f;
  geometry_msgs::PoseStamped position;
  position.pose.position.x = 0.0f;
  position.pose.position.y = 0.0f;
  position.pose.position.z = 2.0f;
  geometry_msgs::Point position_old;
  position_old.x = -1.0f;
  position_old.y = 0.0;
  position_old.z = 2.0f;
  double goal_cost_param = 2.0;
  double smooth_cost_param = 1.5;
  double height_change_cost_param_adapted = 4.0;
  double height_change_cost_param = 4.0;
  bool only_yawed = false;

  // expected output
  std::vector<std::pair<int, int>> blocked;
  std::vector<std::pair<int, int>> free;
  for (int e = 0; e < (180 / resolution_alpha); e++) {
    for (int z = 0; z < (360 / resolution_alpha); z++) {
      if (!((z == 70 || z == 71 || z == 0) && e >= 4 && e <= 6)) {
        free.push_back(std::make_pair(e, z));
      } else {
        if (!(e == obstacle_idx_e && z == obstacle_idx_z)) {
          blocked.push_back(std::make_pair(e, z));
        }
      }
    }
  }

  // WHEN: we look for free directions
  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, path_waypoints;
  std::vector<float> cost_path_candidates;  // not needed

  findFreeDirections(histogram, safety_radius, path_candidates, path_selected,
                     path_rejected, path_blocked, path_waypoints,
                     cost_path_candidates, goal, position, position_old,
                     goal_cost_param, smooth_cost_param,
                     height_change_cost_param_adapted, height_change_cost_param,
                     only_yawed, resolution_alpha);

  // THEN: we should get one rejected cell, the five neighboring cells blocked
  // plus three other blocked cells at the same elevation index and the first
  // azimuth index
  for (auto cell_blocked : path_blocked.cells) {
    std::pair<int, int> cell_blocked_idx(
        elevationAngletoIndex(cell_blocked.x, resolution_alpha),
        azimuthAngletoIndex(cell_blocked.y, resolution_alpha));
    EXPECT_EQ(1, std::find(std::begin(blocked), std::end(blocked),
                           cell_blocked_idx) != std::end(blocked));
  }

  for (auto cell_rejected : path_rejected.cells) {
    EXPECT_EQ(1, (elevationAngletoIndex(cell_rejected.x, resolution_alpha) ==
                  obstacle_idx_e) &&
                     (azimuthAngletoIndex(cell_rejected.y, resolution_alpha) ==
                      obstacle_idx_z));
  }

  for (auto cell_candidate : path_candidates.cells) {
    std::pair<int, int> cell_candidate_idx(
        elevationAngletoIndex(cell_candidate.x, resolution_alpha),
        azimuthAngletoIndex(cell_candidate.y, resolution_alpha));
    EXPECT_EQ(1, std::find(std::begin(free), std::end(free),
                           cell_candidate_idx) != std::end(free));
  }

  EXPECT_EQ(1, path_rejected.cells.size());
  EXPECT_EQ(8, path_blocked.cells.size());
  EXPECT_EQ(
      std::floor(360 / resolution_alpha) * std::floor(180 / resolution_alpha) -
          1 - 8,
      path_candidates.cells.size());
}

TEST(PlannerFunctionsTests, findFreeDirectionsWrapUp) {
  // GIVEN:a histogram with a obstacle at the first elevation index
  int resolution_alpha = 8;
  double safety_radius = 16.0;
  Histogram histogram = Histogram(resolution_alpha);
  int obstacle_idx_e = 0;
  int obstacle_idx_z = 19;
  histogram.set_bin(obstacle_idx_e, obstacle_idx_z,
                    histogram.get_bin(obstacle_idx_e, obstacle_idx_z) + 1);
  // all the variables below aren't used by the test
  geometry_msgs::Point goal;
  goal.x = 2.0f;
  goal.y = 0.0f;
  goal.z = 2.0f;
  geometry_msgs::PoseStamped position;
  position.pose.position.x = 0.0f;
  position.pose.position.y = 0.0f;
  position.pose.position.z = 2.0f;
  geometry_msgs::Point position_old;
  position_old.x = -1.0f;
  position_old.y = 0.0;
  position_old.z = 2.0f;
  double goal_cost_param = 2.0;
  double smooth_cost_param = 1.5;
  double height_change_cost_param_adapted = 4.0;
  double height_change_cost_param = 4.0;
  bool only_yawed = false;

  // expected output
  std::vector<std::pair<int, int>> blocked;
  std::vector<std::pair<int, int>> free;

  for (int e = 0; e < (180 / resolution_alpha); e++) {
    for (int z = 0; z < (360 / resolution_alpha); z++) {
      if (!(e >= 0 && e <= 2 && z >= 17 && z <= 21) &&
          !(e >= 0 && e <= 1 && z >= 40 && z <= 44)) {
        free.push_back(std::make_pair(e, z));
      } else {
        if (!(e == obstacle_idx_e && z == obstacle_idx_z)) {
          blocked.push_back(std::make_pair(e, z));
        }
      }
    }
  }

  // WHEN: we look for free directions
  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, path_waypoints;
  std::vector<float> cost_path_candidates;  // not needed

  findFreeDirections(histogram, safety_radius, path_candidates, path_selected,
                     path_rejected, path_blocked, path_waypoints,
                     cost_path_candidates, goal, position, position_old,
                     goal_cost_param, smooth_cost_param,
                     height_change_cost_param_adapted, height_change_cost_param,
                     only_yawed, resolution_alpha);

  // THEN: we should get one rejected cell, the 14 neighboring cells blocked
  // plus 10 other blocked cells at the same elevation index and azimuth index
  // shifted by 180 degrees
  for (auto cell_blocked : path_blocked.cells) {
    std::pair<int, int> cell_blocked_idx(
        elevationAngletoIndex(cell_blocked.x, resolution_alpha),
        azimuthAngletoIndex(cell_blocked.y, resolution_alpha));
    EXPECT_EQ(1, std::find(std::begin(blocked), std::end(blocked),
                           cell_blocked_idx) != std::end(blocked));
  }

  for (auto cell_rejected : path_rejected.cells) {
    EXPECT_EQ(1, (obstacle_idx_e ==
                  elevationAngletoIndex(cell_rejected.x, resolution_alpha)) &&
                     (obstacle_idx_z ==
                      azimuthAngletoIndex(cell_rejected.y, resolution_alpha)));
  }

  for (auto cell_candidate : path_candidates.cells) {
    std::pair<int, int> cell_candidate_idx(
        elevationAngletoIndex(cell_candidate.x, resolution_alpha),
        azimuthAngletoIndex(cell_candidate.y, resolution_alpha));
    EXPECT_EQ(1, std::find(std::begin(free), std::end(free),
                           cell_candidate_idx) != std::end(free));
  }

  EXPECT_EQ(1, path_rejected.cells.size());
  EXPECT_EQ(24, path_blocked.cells.size());
  EXPECT_EQ(
      std::floor(360 / resolution_alpha) * std::floor(180 / resolution_alpha) -
          1 - 24,
      path_candidates.cells.size());
}

TEST(PlannerFunctionsTests, findFreeDirectionsWrapDown) {
  // GIVEN: a histogram with a obstacle at the last elevation index
  int resolution_alpha = 10;
  double safety_radius = 10.0;
  Histogram histogram = Histogram(resolution_alpha);
  int obstacle_idx_e = 17;
  int obstacle_idx_z = 4;
  histogram.set_bin(obstacle_idx_e, obstacle_idx_z,
                    histogram.get_bin(obstacle_idx_e, obstacle_idx_z) + 1);
  // all the variables below aren't used by the test
  geometry_msgs::Point goal;
  goal.x = 2.0f;
  goal.y = 0.0f;
  goal.z = 2.0f;
  geometry_msgs::PoseStamped position;
  position.pose.position.x = 0.0f;
  position.pose.position.y = 0.0f;
  position.pose.position.z = 2.0f;
  geometry_msgs::Point position_old;
  position_old.x = -1.0f;
  position_old.y = 0.0;
  position_old.z = 2.0f;
  double goal_cost_param = 2.0;
  double smooth_cost_param = 1.5;
  double height_change_cost_param_adapted = 4.0;
  double height_change_cost_param = 4.0;
  bool only_yawed = false;

  // expected output
  std::vector<std::pair<int, int>> blocked;
  std::vector<std::pair<int, int>> free;
  for (int e = 0; e < (180 / resolution_alpha); e++) {
    for (int z = 0; z < (360 / resolution_alpha); z++) {
      if (!((e == 16 || e == 17) && (z >= 3 && z <= 5)) &&
          !(e == 17 && (z >= 21 && z <= 23))) {
        free.push_back(std::make_pair(e, z));
      } else {
        if (!(e == obstacle_idx_e && z == obstacle_idx_z)) {
          blocked.push_back(std::make_pair(e, z));
        }
      }
    }
  }

  // WHEN: we look for free directions
  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, path_waypoints;
  std::vector<float> cost_path_candidates;  // not needed

  findFreeDirections(histogram, safety_radius, path_candidates, path_selected,
                     path_rejected, path_blocked, path_waypoints,
                     cost_path_candidates, goal, position, position_old,
                     goal_cost_param, smooth_cost_param,
                     height_change_cost_param_adapted, height_change_cost_param,
                     only_yawed, resolution_alpha);

  // THEN: we should get one rejected cell, the 5 neighboring cells blocked
  // plus 3 other blocked cells at the same elevation index and azimuth index
  // shifted by 180 degrees
  for (auto cell_blocked : path_blocked.cells) {
    std::pair<int, int> cell_blocked_idx(
        elevationAngletoIndex(cell_blocked.x, resolution_alpha),
        azimuthAngletoIndex(cell_blocked.y, resolution_alpha));
    EXPECT_EQ(1, std::find(std::begin(blocked), std::end(blocked),
                           cell_blocked_idx) != std::end(blocked));
  }

  for (auto cell_rejected : path_rejected.cells) {
    EXPECT_EQ(1, (elevationAngletoIndex(cell_rejected.x, resolution_alpha) ==
                  obstacle_idx_e) &&
                     (azimuthAngletoIndex(cell_rejected.y, resolution_alpha) ==
                      obstacle_idx_z));
  }

  for (auto cell_candidate : path_candidates.cells) {
    std::pair<int, int> cell_candidate_idx(
        elevationAngletoIndex(cell_candidate.x, resolution_alpha),
        azimuthAngletoIndex(cell_candidate.y, resolution_alpha));
    EXPECT_EQ(1, std::find(std::begin(free), std::end(free),
                           cell_candidate_idx) != std::end(free));
  }

  EXPECT_EQ(1, path_rejected.cells.size());
  EXPECT_EQ(8, path_blocked.cells.size());
  EXPECT_EQ(
      std::floor(360 / resolution_alpha) * std::floor(180 / resolution_alpha) -
          1 - 8,
      path_candidates.cells.size());
}
