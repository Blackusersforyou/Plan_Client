#ifndef OPENNESS_PLANNER_H
#define OPENNESS_PLANNER_H

#include "StructData.h"
#include "LaserSLAM.h"
#include "ObstacleAvoidance.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include "MathUtils.h"

class OpennessPlanner {
private:
	LaserSLAM* slam;
	ObstacleAvoidance* obstacle_avoidance;
	
	// 行为模式权重
	struct BehaviorWeights {
		double exploration_weight;
		double target_weight;
		double safety_weight;
		double frontier_weight;
		double diversity_weight;
		double lateral_weight;  // 新增：侧向空旷度权重
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
		
		// ✅ 目标已知模式：极强目标导向
		target_known_weights.exploration_weight = 0.05;  // ✅ 从0.10降低到0.05
		target_known_weights.target_weight = 0.70;       // ✅ 从0.60提高到0.70
		target_known_weights.safety_weight = 0.20;       // ✅ 从0.25降低到0.20
		target_known_weights.frontier_weight = 0.01;
		target_known_weights.diversity_weight = 0.01;
		target_known_weights.lateral_weight = 0.03;      // ✅ 从0.02提高到0.03
		
		// ✅ 目标未知模式：也要倾向目标方向
		target_unknown_weights.exploration_weight = 0.18;
		target_unknown_weights.target_weight = 0.25;     // ✅ 保留目标倾向
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

	void computeMotionDirection(POSE cur_pose, Point target, INT16* laser_data,
                            double& tra_vel, double& rot_vel, 
                            bool target_visible = false) {
    OccupancyMap& map = slam->getMap();

    double target_dx = target.coor_x - cur_pose.coor_x;
    double target_dy = target.coor_y - cur_pose.coor_y;
    double target_dist = sqrt(target_dx*target_dx + target_dy*target_dy);
    double target_angle = atan2(target_dy, target_dx);

    // 检测移动状态
    double move_dist = sqrt(pow(cur_pose.coor_x - last_pose.coor_x, 2) + 
                           pow(cur_pose.coor_y - last_pose.coor_y, 2));
    if (move_dist < 5.0) {
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
    
    // ✅ 关键修复：目标可见时使用强目标导向
    if (target_visible && target_dist < 600.0) {
        weights = target_known_weights;
        mode_name = "TARGET_LOCKED";
        
        // 距离越近，目标权重越高
        if (target_dist < 150.0) {
            weights.target_weight = 0.80;  // ✅ 提高到0.80
            weights.safety_weight = 0.15;
            weights.exploration_weight = 0.03;
            weights.frontier_weight = 0.01;
            weights.lateral_weight = 0.01;
        } else if (target_dist < 300.0) {
            weights.target_weight = 0.70;
            weights.safety_weight = 0.22;
            weights.exploration_weight = 0.05;
        }
        
        // ✅ 添加调试输出
        static int mode_log = 0;
        if (++mode_log % 30 == 0) {
            std::cout << ">> [TARGET_LOCKED] dist=" << (int)target_dist 
                      << " weight=" << weights.target_weight << std::endl;
        }
    }
    else {
        weights = target_unknown_weights;
        mode_name = "EXPLORING";
        
        // ✅ 关键修复：不要强制归零，保留目标倾向
        // 删除以下代码（注释掉）：
        // weights.target_weight = 0.0;
        // weights.exploration_weight = 0.30;
        // weights.frontier_weight = 0.35;
        
        if (frames_in_same_area > 100) {
            weights.frontier_weight = 0.40;
            weights.lateral_weight = 0.15;
            weights.exploration_weight = 0.20;
            std::cout << ">> Long-term stuck! AGGRESSIVE EXPLORATION" << std::endl;
        }
        
        // ✅ 添加调试输出
        static int explore_log = 0;
        if (++explore_log % 30 == 0) {
            std::cout << ">> [EXPLORING] target_weight=" << weights.target_weight << std::endl;
        }
    }

    double best_angle = cur_pose.coor_ori;
    double best_score = -1e10;

    // ✅ 关键修复：目标可见时缩小采样范围，聚焦目标方向
    int num_samples;
    double sample_range;
    
    if (target_visible) {
        num_samples = 30;  // ✅ 从40减少到30
        sample_range = PI;  // ✅ 从2PI缩小到PI（±90度）
    } else {
        num_samples = 60;
        sample_range = 2.5 * PI;
    }
    
    // ✅ 添加调试变量
    double best_direction_score = 0;
    double best_openness_score = 0;
    double best_safety_score = 0;
    
    for (int i = 0; i < num_samples; i++) {
        double test_angle = cur_pose.coor_ori + 
                            (i - num_samples / 2.0) * sample_range / num_samples;

        // 检查距离
        double angle_from_forward = test_angle - cur_pose.coor_ori;
        while (angle_from_forward > PI) angle_from_forward -= 2 * PI;
        while (angle_from_forward < -PI) angle_from_forward += 2 * PI;
        
        // ✅ 关键修复：缩短检查距离，避免采样点落在障碍物上
        double check_distance;
        if (target_visible) {
            if (fabs(angle_from_forward) < PI / 6.0) {
                // 前方±30度：使用较短距离
                check_distance = (std::min)(80.0, (std::max)(40.0, target_dist * 0.3));
            } else {
                check_distance = 100.0;  // ✅ 从140减少到100
            }
        } else {
            check_distance = 120.0;  // ✅ 从160减少到120
        }
        
        double check_x = cur_pose.coor_x + check_distance * cos(test_angle);
        double check_y = cur_pose.coor_y + check_distance * sin(test_angle);

        INT16 gx, gy;
        worldToGrid(check_x, check_y, gx, gy);

        if (gx < 0 || gx >= GRID_WIDTH || gy < 0 || gy >= GRID_HEIGHT) continue;
        if (map.grid[gy][gx].occupancy == 100) continue;

        // ========== 评分计算 ==========
        
        // 1. 空旷度得分
        double openness_score = map.openness[gy][gx];

        // 2. ✅ 目标方向得分 - 使用更陡峭的函数
        double angle_diff = test_angle - target_angle;
        angle_diff = MathUtils::normalizeAngle(angle_diff);
        
        double direction_score;
        if (target_visible) {
            // ✅ 目标可见：使用6次方，极强指向目标
            direction_score = pow((cos(angle_diff) + 1.0) / 2.0, 6.0);
        } else {
            // 目标未知：使用2次方
            direction_score = pow((cos(angle_diff) + 1.0) / 2.0, 2.0);
        }

        // 3. 安全性得分
        double safety_score = obstacle_avoidance->evaluatePathSafety(cur_pose, test_angle, laser_data);
        
        // ✅ 关键修复：目标可见时降低安全性要求
        if (target_visible && safety_score < 0.3) {
            safety_score *= 0.7;  // 从0.4提高到0.7，减小惩罚
        } else if (!target_visible && safety_score < 0.3) {
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
        if (!target_visible) {  // ✅ 仅在探索模式下考虑边界
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
        double angle_diff_from_last = test_angle - last_best_angle;
        while (angle_diff_from_last > PI) angle_diff_from_last -= 2 * PI;
        while (angle_diff_from_last < -PI) angle_diff_from_last += 2 * PI;
        
        if (fabs(angle_diff_from_last) < 0.08) {
            diversity_score = 0.3;
        }

        // ✅ 综合得分
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
    double angle_change = fabs(best_angle - last_best_angle);
    if (angle_change < 0.03 || stationary_counter > 12) {
        stuck_counter++;
    } else {
        stuck_counter = 0;
    }

    // 脱困策略
    if (stuck_counter > 8 || stationary_counter > 15) {
        std::cout << ">> STUCK! Mode: " << mode_name << " - Force escape" << std::endl;
        
        if (target_visible) {
            // ✅ 目标可见但卡住：检查侧面空旷度
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
            
            // ✅ 计算哪个方向更接近目标
            double left_angle = cur_pose.coor_ori + PI / 2.0;
            double right_angle = cur_pose.coor_ori - PI / 2.0;
            
            double left_diff = fabs(left_angle - target_angle);
            double right_diff = fabs(right_angle - target_angle);
            while (left_diff > PI) left_diff = 2*PI - left_diff;
            while (right_diff > PI) right_diff = 2*PI - right_diff;
            
            // 优先选择更接近目标且空旷的方向
            if (left_distance > 80.0 && (left_diff < right_diff || left_distance > right_distance + 30.0)) {
                best_angle = left_angle;
                std::cout << "   Escape: Turn LEFT (closer to target)" << std::endl;
            } else if (right_distance > 80.0) {
                best_angle = right_angle;
                std::cout << "   Escape: Turn RIGHT (closer to target)" << std::endl;
            } else {
                // 两侧都不理想，选择更接近目标的方向
                best_angle = target_angle + ((left_diff < right_diff) ? PI/2.0 : -PI/2.0);
                std::cout << "   Escape: Navigate around obstacle toward target" << std::endl;
            }
        } else {
            // 目标未知：倾向目标方向的随机转向
            double random_offset = (rand() % 120 - 60) * PI / 180.0;
            best_angle = target_angle + random_offset;
            std::cout << "   Escape: Random turn biased toward target area" << std::endl;
        }
        
        stuck_counter = 0;
        stationary_counter = 0;
    }

    last_best_angle = best_angle;
    
    // 计算速度
    double angle_error = best_angle - cur_pose.coor_ori;
    while (angle_error > PI) angle_error -= 2 * PI;
    while (angle_error < -PI) angle_error += 2 * PI;

    // ✅ 速度控制优化
    if (target_visible) {
        if (fabs(angle_error) > 0.7) {
            tra_vel = 18.0;  // 从15提高到18
            rot_vel = (angle_error > 0) ? 0.6 : -0.6;  // 从0.5提高到0.6
        }
        else if (fabs(angle_error) > 0.3) {
            tra_vel = 32.0;  // 从28提高到32
            rot_vel = angle_error * 1.2;  // 从1.0提高到1.2
        }
        else {
            tra_vel = 45.0;  // 从40提高到45
            rot_vel = angle_error * 0.8;  // 从0.6提高到0.8
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

    if (stationary_counter > 15) {
        tra_vel = (std::max)(tra_vel, 20.0);
    }

    tra_vel = (std::max)(0.0, (std::min)(50.0, tra_vel));
    rot_vel = (std::max)(-0.6, (std::min)(0.6, rot_vel));
    
    // ✅ 详细调试输出（使用唯一的计数器变量名）
    static int detail_debug_counter = 0;
    if (++detail_debug_counter % 20 == 0) {
        double angle_to_target = best_angle - target_angle;
        while (angle_to_target > PI) angle_to_target -= 2 * PI;
        while (angle_to_target < -PI) angle_to_target += 2 * PI;
        
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
	
	// ✅ 新增：重置方法（解决问题3）
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

private:
	void worldToGrid(double wx, double wy, INT16& gx, INT16& gy) {
		gx = (INT16)((wx + MAP_WIDTH / 2.0) / MAP_RESOLUTION);
		gy = (INT16)((wy + MAP_HEIGHT / 2.0) / MAP_RESOLUTION);
		gx = (std::max)((INT16)0, (std::min)((INT16)(GRID_WIDTH - 1), gx));
		gy = (std::max)((INT16)0, (std::min)((INT16)(GRID_HEIGHT - 1), gy));
	}
};

#endif