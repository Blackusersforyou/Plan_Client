#ifndef OPENNESS_PLANNER_H
#define OPENNESS_PLANNER_H

#include "StructData.h"
#include "LaserSLAM.h"
#include "ObstacleAvoidance.h"
#include "MathUtils.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include "MathUtils.h"

class OpennessPlanner {
private:
	LaserSLAM* slam;
	ObstacleAvoidance* obstacle_avoidance;
	
	struct BehaviorWeights {
		double exploration_weight;
		double target_weight;
		double safety_weight;
		double frontier_weight;
		double diversity_weight;
		double lateral_weight;
	};
	
	BehaviorWeights target_known_weights;
	BehaviorWeights target_unknown_weights;
	
	double last_best_angle;
	int stuck_counter;
	POSE last_pose;
	int stationary_counter;
	int frames_in_same_area;

public:
	OpennessPlanner(LaserSLAM* slam_ptr, ObstacleAvoidance* obstacle_ptr) 
		: slam(slam_ptr), obstacle_avoidance(obstacle_ptr) {
		
		target_known_weights.exploration_weight = 0.05;
		target_known_weights.target_weight = 0.70;
		target_known_weights.safety_weight = 0.20;
		target_known_weights.frontier_weight = 0.01;
		target_known_weights.diversity_weight = 0.01;
		target_known_weights.lateral_weight = 0.03;
		
		target_unknown_weights.exploration_weight = 0.18;
		target_unknown_weights.target_weight = 0.25;
		target_unknown_weights.safety_weight = 0.30;
		target_unknown_weights.frontier_weight = 0.23;
		target_unknown_weights.diversity_weight = 0.02;
		target_unknown_weights.lateral_weight = 0.02;
		
		last_best_angle = 0;
		stuck_counter = 0;
		stationary_counter = 0;
		frames_in_same_area = 0;
		last_pose.coor_x = 0;
		last_pose.coor_y = 0;
		last_pose.coor_ori = 0;
		
		std::cout << "OpennessPlanner: Strong target-oriented mode (known:0.70, unknown:0.25)" << std::endl;
	}

	// ✅ 修改: 删除 target_visible 参数，改为 object_detected
	void computeMotionDirection(POSE cur_pose, Point target, INT16* laser_data,
                            double& tra_vel, double& rot_vel, 
                            bool object_detected = false) {
		OccupancyMap& map = slam->getMap();

		double target_dx = target.coor_x - cur_pose.coor_x;
		double target_dy = target.coor_y - cur_pose.coor_y;
		double target_dist = sqrt(target_dx*target_dx + target_dy*target_dy);
		double target_angle = atan2(target_dy, target_dx);

		// 使用更精确的移动检测
		double move_dist = MathUtils::calculateDistance(
			cur_pose.coor_x, cur_pose.coor_y,
			last_pose.coor_x, last_pose.coor_y);
		
		// 提高静止阈值，从5cm提高到10cm
		if (move_dist < 10.0) {
			stationary_counter++;
			frames_in_same_area++;
		} else {
			stationary_counter = 0;
			if (move_dist > 30.0) {
				frames_in_same_area = 0;
			}
		}
		last_pose = cur_pose;
		
		// 选择行为模式
		BehaviorWeights weights;
		std::string mode_name;
		
		// ✅ 修改: 使用 object_detected 代替 target_visible
		if (object_detected && target_dist < 600.0) {
			weights = target_known_weights;
			mode_name = "TARGET_LOCKED";
			
			if (target_dist < 150.0) {
				weights.target_weight = 0.80;
				weights.safety_weight = 0.15;
				weights.exploration_weight = 0.03;
				weights.frontier_weight = 0.01;
				weights.lateral_weight = 0.01;
			} else if (target_dist < 300.0) {
				weights.target_weight = 0.70;
				weights.safety_weight = 0.22;
				weights.exploration_weight = 0.05;
			}
			
			static int mode_log = 0;
			if (++mode_log % 30 == 0) {
				std::cout << ">> [TARGET_LOCKED] dist=" << (int)target_dist 
				          << " weight=" << weights.target_weight << std::endl;
			}
		}
		else {
			weights = target_unknown_weights;
			mode_name = "EXPLORING";
			
			if (frames_in_same_area > 100) {
				weights.frontier_weight = 0.40;
				weights.lateral_weight = 0.15;
				weights.exploration_weight = 0.20;
				std::cout << ">> Long-term stuck! AGGRESSIVE EXPLORATION" << std::endl;
			}
			
			static int explore_log = 0;
			if (++explore_log % 30 == 0) {
				std::cout << ">> [EXPLORING] target_weight=" << weights.target_weight << std::endl;
			}
		}

		double best_angle = cur_pose.coor_ori;
		double best_score = -1e10;

		int num_samples;
		double sample_range;
		
		// ✅ 修改: 使用 object_detected 代替 target_visible
		if (object_detected) {
			num_samples = 30;
			sample_range = PI;
		} else {
			num_samples = 60;
			sample_range = 2 * PI;
		}
		
		double best_direction_score = 0;
		double best_openness_score = 0;
		double best_safety_score = 0;
		
		for (int i = 0; i < num_samples; i++) {
			double test_angle = cur_pose.coor_ori + 
			                    (i - num_samples / 2.0) * sample_range / num_samples;

			double angle_from_forward = test_angle - cur_pose.coor_ori;
			angle_from_forward = MathUtils::normalizeAngle(angle_from_forward);
			
			double check_distance;
			// ✅ 修改: 使用 object_detected 代替 target_visible
			if (object_detected) {
				if (fabs(angle_from_forward) < PI / 6.0) {
					check_distance = (std::min)(80.0, (std::max)(40.0, target_dist * 0.3));
				} else {
					check_distance = 100.0;
				}
			} else {
				check_distance = 120.0;
			}
			
			double check_x = cur_pose.coor_x + check_distance * cos(test_angle);
			double check_y = cur_pose.coor_y + check_distance * sin(test_angle);

			INT16 gx, gy;
			MathUtils::worldToGrid(check_x, check_y, gx, gy);

			if (gx < 0 || gx >= GRID_WIDTH || gy < 0 || gy >= GRID_HEIGHT) continue;
			if (map.grid[gy][gx].occupancy == 100) continue;

			// 1. 空旷度得分
			double openness_score = map.openness[gy][gx];

			// 2. 目标方向得分
			double angle_diff = MathUtils::angleDifference(test_angle, target_angle);
			
			double direction_score;
			// ✅ 修改: 使用 object_detected 代替 target_visible
			if (object_detected) {
				direction_score = pow((cos(angle_diff) + 1.0) / 2.0, 6.0);
			} else {
				direction_score = pow((cos(angle_diff) + 1.0) / 2.0, 2.0);
			}

			// 3. 安全性得分
			double safety_score = obstacle_avoidance->evaluatePathSafety(cur_pose, test_angle, laser_data);
			
			// ✅ 修改: 使用 object_detected 代替 target_visible
			if (object_detected && safety_score < 0.3) {
				safety_score *= 0.7;
			} else if (!object_detected && safety_score < 0.3) {
				safety_score *= 0.4;
			}

			// 4. 侧向空旷度
			double lateral_score = 0;
			if (fabs(angle_from_forward) > PI / 6.0) {
				int check_laser_idx = (int)((angle_from_forward * 180.0 / PI) + 180 + 0.5);
				if (check_laser_idx < 0) check_laser_idx += 360;
				if (check_laser_idx >= 360) check_laser_idx -= 360;
				
				double lateral_distance_sum = 0;
				int lateral_count = 0;
				for (int offset = -20; offset <= 20; offset++) {
					int idx = check_laser_idx + offset;
					if (idx < 0) idx += 360;
					if (idx >= 360) idx -= 360;
					if (laser_data[idx] > 0 && laser_data[idx] < 5000) {
						lateral_distance_sum += laser_data[idx];
						lateral_count++;
					}
				}
				
				if (lateral_count > 0) {
					double avg_lateral_distance = lateral_distance_sum / lateral_count;
					lateral_score = (std::min)(1.0, avg_lateral_distance / 200.0);
					
					if (avg_lateral_distance > 120.0 && openness_score > 0.3) {
						lateral_score *= 1.5;
					}
				}
			}

			// 5. 边界探索得分
			double frontier_score = 0;
			// ✅ 修改: 使用 object_detected 代替 target_visible
			if (!object_detected) {
				int unexplored_nearby = 0;
				int explored_nearby = 0;
				int check_radius = 5;
				
				for (int di = -check_radius; di <= check_radius; di++) {
					for (int dj = -check_radius; dj <= check_radius; dj++) {
						int ni = gy + di;
						int nj = gx + dj;
						if (ni >= 0 && ni < GRID_HEIGHT && nj >= 0 && nj < GRID_WIDTH) {
							if (map.grid[ni][nj].explored == 0 && map.grid[ni][nj].occupancy != 100) {
								unexplored_nearby++;
							} else if (map.grid[ni][nj].explored == 1) {
								explored_nearby++;
							}
						}
					}
				}
				
				frontier_score = (double)unexplored_nearby / 12.0;
				double revisit_penalty = 1.0 - ((double)explored_nearby / 100.0);
				frontier_score *= revisit_penalty;
			}

			// 6. 方向多样性
			double diversity_score = 1.0;
			double angle_diff_from_last = MathUtils::angleDifference(test_angle, last_best_angle);
			
			if (fabs(angle_diff_from_last) < 0.15) {
				diversity_score = 0.6;
			}

			// 综合得分
			double total_score = weights.exploration_weight * openness_score + 
			                     weights.target_weight * direction_score +
			                     weights.safety_weight * safety_score +
			                     weights.frontier_weight * frontier_score +
			                     weights.diversity_weight * diversity_score +
			                     weights.lateral_weight * lateral_score;

			if (total_score > best_score) {
				best_score = total_score;
				best_angle = test_angle;
				best_direction_score = direction_score;
				best_openness_score = openness_score;
				best_safety_score = safety_score;
			}
		}

		// 检测卡住
		double angle_change = fabs(MathUtils::angleDifference(best_angle, last_best_angle));
		
		if (angle_change < 0.1 || stationary_counter > 20) {
			stuck_counter++;
		} else {
			stuck_counter = 0;
		}

		// 脱困策略
		if (stuck_counter > 15 || stationary_counter > 25) {
			std::cout << ">> STUCK! Mode: " << mode_name << " - Force escape" << std::endl;
			std::cout << "   stuck_counter=" << stuck_counter 
			          << " stationary_counter=" << stationary_counter 
			          << " angle_change=" << (angle_change * 180 / PI) << "deg" << std::endl;
			
			// ✅ 修改: 使用 object_detected 代替 target_visible
			if (object_detected) {
				double left_distance = 0, right_distance = 0;
				int left_count = 0, right_count = 0;
				
				for (int i = 30; i <= 90; i++) {
					if (laser_data[i] > 0 && laser_data[i] < 5000) {
						left_distance += laser_data[i];
						left_count++;
					}
				}
				for (int i = 270; i <= 330; i++) {
					if (laser_data[i] > 0 && laser_data[i] < 5000) {
						right_distance += laser_data[i];
						right_count++;
					}
				}
				
				if (left_count > 0) left_distance /= left_count;
				if (right_count > 0) right_distance /= right_count;
				
				double left_angle = cur_pose.coor_ori + PI / 2.0;
				double right_angle = cur_pose.coor_ori - PI / 2.0;
				
				double left_diff = fabs(MathUtils::angleDifference(left_angle, target_angle));
				double right_diff = fabs(MathUtils::angleDifference(right_angle, target_angle));
				
				if (left_distance > 80.0 && (left_diff < right_diff || left_distance > right_distance + 30.0)) {
					best_angle = left_angle;
					std::cout << "   Escape: Turn LEFT (closer to target)" << std::endl;
				} else if (right_distance > 80.0) {
					best_angle = right_angle;
					std::cout << "   Escape: Turn RIGHT (closer to target)" << std::endl;
				} else {
					best_angle = target_angle + ((left_diff < right_diff) ? PI/2.0 : -PI/2.0);
					std::cout << "   Escape: Navigate around obstacle toward target" << std::endl;
				}
			} else {
				double random_offset = (rand() % 120 - 60) * PI / 180.0;
				best_angle = target_angle + random_offset;
				std::cout << "   Escape: Random turn biased toward target area" << std::endl;
			}
			
			stuck_counter = 0;
			stationary_counter = 0;
		}

		last_best_angle = best_angle;
		
		// 计算速度
		double angle_error = MathUtils::angleDifference(best_angle, cur_pose.coor_ori);

		// ✅ 修改: 使用 object_detected 代替 target_visible
		if (object_detected) {
			if (fabs(angle_error) > 0.7) {
				tra_vel = 18.0;
				rot_vel = (angle_error > 0) ? 0.6 : -0.6;
			}
			else if (fabs(angle_error) > 0.3) {
				tra_vel = 32.0;
				rot_vel = angle_error * 1.2;
			}
			else {
				tra_vel = 45.0;
				rot_vel = angle_error * 0.8;
			}
		} else {
			if (fabs(angle_error) > 0.7) {
				tra_vel = 15.0;
				rot_vel = (angle_error > 0) ? 0.5 : -0.5;
			}
			else if (fabs(angle_error) > 0.3) {
				tra_vel = 25.0;
				rot_vel = angle_error * 1.0;
			}
			else {
				tra_vel = 35.0;
				rot_vel = angle_error * 0.6;
			}
		}

		if (stationary_counter > 20) {
			tra_vel = (std::max)(tra_vel, 25.0);
		}

		tra_vel = (std::max)(0.0, (std::min)(50.0, tra_vel));
		rot_vel = (std::max)(-0.6, (std::min)(0.6, rot_vel));
		
		static int detail_debug_counter = 0;
		if (++detail_debug_counter % 20 == 0) {
			double angle_to_target = MathUtils::angleDifference(best_angle, target_angle);
			
			std::cout << "[" << mode_name << "] "
			          << "Score=" << best_score 
			          << " Dir=" << best_direction_score
			          << " Open=" << best_openness_score
			          << " Safe=" << best_safety_score
			          << " Ang=" << (int)(angle_to_target*180/PI) << "deg"
			          << " Dist=" << (int)target_dist
			          << std::endl;
		}
	}
	
	void reset() {
		stuck_counter = 0;
		stationary_counter = 0;
		frames_in_same_area = 0;
		last_best_angle = 0;
		last_pose.coor_x = 0;
		last_pose.coor_y = 0;
		last_pose.coor_ori = 0;
		std::cout << "OpennessPlanner reset" << std::endl;
	}
};

#endif