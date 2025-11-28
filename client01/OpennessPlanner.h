#ifndef OPENNESS_PLANNER_H
#define OPENNESS_PLANNER_H

#include "StructData.h"
#include "LaserSLAM.h"
#include "ObstacleAvoidance.h"
#include "MathUtils.h"
#include <cmath>
#include <algorithm>
#include <iostream>

/**
 * @brief 基于空旷度的运动规划器
 *
 * 修复内容 V2：
 * 1. 使用速度命令而非整数位姿变化来判断运动状态
 * 2. 大幅降低stuck判定阈值
 * 3. 移除过度激进的探索模式
 */
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

    // ✅ 新增：使用速度命令来判断运动状态
    double last_cmd_vel;
    double last_cmd_rot;
    double accumulated_distance;  // 累积移动距离
    double accumulated_rotation;  // 累积旋转角度

public:
    OpennessPlanner(LaserSLAM* slam_ptr, ObstacleAvoidance* obstacle_ptr)
        : slam(slam_ptr), obstacle_avoidance(obstacle_ptr) {

        // 目标已知时的权重配置
        target_known_weights.exploration_weight = 0.05;
        target_known_weights.target_weight = 0.70;
        target_known_weights.safety_weight = 0.20;
        target_known_weights.frontier_weight = 0.01;
        target_known_weights.diversity_weight = 0.01;
        target_known_weights.lateral_weight = 0.03;

        // 目标未知时的权重配置
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

        last_cmd_vel = 0;
        last_cmd_rot = 0;
        accumulated_distance = 0;
        accumulated_rotation = 0;

        std::cout << "OpennessPlanner initialized (Fixed Version V2)" << std::endl;
        std::cout << "  Using velocity-based motion detection" << std::endl;
    }

    /**
     * @brief 更新运动状态（基于速度命令而非位姿变化）
     * @param cmd_vel 线速度命令 (cm/s)
     * @param cmd_rot 角速度命令 (rad/s)
     * @param dt 时间步长 (s)
     */
    void updateMotionState(double cmd_vel, double cmd_rot, double dt = 0.03) {
        // ✅ 核心修复：基于速度命令判断运动状态
        double delta_dist = fabs(cmd_vel) * dt;
        double delta_rot = fabs(cmd_rot) * dt;

        accumulated_distance += delta_dist;
        accumulated_rotation += delta_rot;

        // 判断是否在运动：有速度命令就认为在运动
        bool is_moving = (fabs(cmd_vel) > 2.0 || fabs(cmd_rot) > 0.05);

        if (is_moving) {
            stationary_counter = 0;

            // 累积移动超过50cm才重置区域计数器
            if (accumulated_distance > 50.0) {
                frames_in_same_area = 0;
                accumulated_distance = 0;
            }
        }
        else {
            stationary_counter++;
            frames_in_same_area++;
        }

        last_cmd_vel = cmd_vel;
        last_cmd_rot = cmd_rot;
    }

    void computeMotionDirection(POSE cur_pose, Point target, INT16* laser_data,
        double& tra_vel, double& rot_vel,
        bool object_detected = false) {
        OccupancyMap& map = slam->getMap();

        double target_dx = target.coor_x - cur_pose.coor_x;
        double target_dy = target.coor_y - cur_pose.coor_y;
        double target_dist = sqrt(target_dx * target_dx + target_dy * target_dy);
        double target_angle = atan2(target_dy, target_dx);

        // ✅ 修复：使用高精度位姿差异（浮点数）
        // 注意：这里仍然使用整数位姿，但主要依赖updateMotionState
        double move_dist = MathUtils::calculateDistance(
            cur_pose.coor_x, cur_pose.coor_y,
            last_pose.coor_x, last_pose.coor_y);

        double pose_angle_change = fabs(MathUtils::angleDifference(
            cur_pose.coor_ori, last_pose.coor_ori));

        // ✅ 备用检测：位置或角度有变化
        if (move_dist > 1.0 || pose_angle_change > 0.02) {
            // 整数位姿也检测到了移动，更可靠的重置
            if (move_dist > 10.0) {
                frames_in_same_area = (std::max)(0, frames_in_same_area - 10);
            }
        }

        last_pose = cur_pose;

        // 选择行为模式
        BehaviorWeights weights;
        std::string mode_name;

        if (object_detected && target_dist < 600.0) {
            weights = target_known_weights;
            mode_name = "TARGET_LOCKED";

            // 根据距离调整权重
            if (target_dist < 150.0) {
                weights.target_weight = 0.80;
                weights.safety_weight = 0.15;
                weights.exploration_weight = 0.03;
                weights.frontier_weight = 0.01;
                weights.lateral_weight = 0.01;
            }
            else if (target_dist < 300.0) {
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

            // ✅ 修复：大幅提高阈值，减少误判
            // 只有真正长时间停留才触发激进探索
            if (frames_in_same_area > 300) {  // 原来是100，改为300
                weights.frontier_weight = 0.35;
                weights.lateral_weight = 0.10;
                weights.exploration_weight = 0.25;

                static int aggressive_log = 0;
                if (++aggressive_log % 50 == 0) {
                    std::cout << ">> Long-term in same area (" << frames_in_same_area
                        << " frames), increasing exploration" << std::endl;
                }
            }

            static int explore_log = 0;
            if (++explore_log % 30 == 0) {
                std::cout << ">> [EXPLORING] target_weight=" << weights.target_weight
                    << " frames_in_area=" << frames_in_same_area << std::endl;
            }
        }

        double best_angle = cur_pose.coor_ori;
        double best_score = -1e10;

        int num_samples;
        double sample_range;

        if (object_detected) {
            num_samples = 30;
            sample_range = PI;
        }
        else {
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
            if (object_detected) {
                if (fabs(angle_from_forward) < PI / 6.0) {
                    check_distance = MathUtils::clamp(target_dist * 0.3, 40.0, 80.0);
                }
                else {
                    check_distance = 100.0;
                }
            }
            else {
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
            if (object_detected) {
                direction_score = pow((cos(angle_diff) + 1.0) / 2.0, 6.0);
            }
            else {
                direction_score = pow((cos(angle_diff) + 1.0) / 2.0, 2.0);
            }

            // 3. 安全性得分
            double safety_score = obstacle_avoidance->evaluatePathSafety(cur_pose, test_angle, laser_data);

            if (object_detected && safety_score < 0.3) {
                safety_score *= 0.7;
            }
            else if (!object_detected && safety_score < 0.3) {
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
                    lateral_score = MathUtils::clamp(avg_lateral_distance / 200.0, 0.0, 1.0);

                    if (avg_lateral_distance > 120.0 && openness_score > 0.3) {
                        lateral_score *= 1.5;
                    }
                }
            }

            // 5. 边界探索得分
            double frontier_score = 0;
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
                            }
                            else if (map.grid[ni][nj].explored == 1) {
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

        // ✅ 修复：使用速度命令来判断是否卡住
        double angle_change = fabs(MathUtils::angleDifference(best_angle, last_best_angle));

        // 只有当速度命令很小且方向变化很小时才认为卡住
        if ((fabs(last_cmd_vel) < 3.0 && fabs(last_cmd_rot) < 0.1) ||
            (angle_change < 0.1 && stationary_counter > 30)) {
            stuck_counter++;
        }
        else {
            stuck_counter = (std::max)(0, stuck_counter - 2);  // 快速恢复
        }

        // ✅ 修复：提高脱困阈值，减少误触发
        if (stuck_counter > 50 || stationary_counter > 80) {
            std::cout << ">> STUCK! Mode: " << mode_name << " - Force escape" << std::endl;
            std::cout << "   stuck_counter=" << stuck_counter
                << " stationary_counter=" << stationary_counter
                << " cmd_vel=" << last_cmd_vel << std::endl;

            if (object_detected) {
                // 目标已知：寻找绕行路径
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
                    std::cout << "   Escape: Turn LEFT" << std::endl;
                }
                else if (right_distance > 80.0) {
                    best_angle = right_angle;
                    std::cout << "   Escape: Turn RIGHT" << std::endl;
                }
                else {
                    best_angle = target_angle + ((left_diff < right_diff) ? PI / 2.0 : -PI / 2.0);
                    std::cout << "   Escape: Navigate around" << std::endl;
                }
            }
            else {
                // 目标未知：随机探索
                double random_offset = ((rand() % 120) - 60) * PI / 180.0;
                best_angle = target_angle + random_offset;
                std::cout << "   Escape: Random turn" << std::endl;
            }

            stuck_counter = 0;
            stationary_counter = 0;
        }

        last_best_angle = best_angle;

        // 计算速度
        double angle_error = MathUtils::angleDifference(best_angle, cur_pose.coor_ori);

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
        }
        else {
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

        tra_vel = MathUtils::clamp(tra_vel, 0.0, 50.0);
        rot_vel = MathUtils::clamp(rot_vel, -0.6, 0.6);

        // 调试输出
        static int detail_debug_counter = 0;
        if (++detail_debug_counter % 20 == 0) {
            double angle_to_target = MathUtils::angleDifference(best_angle, target_angle);

            std::cout << "[" << mode_name << "] "
                << "Score=" << std::fixed << std::setprecision(2) << best_score
                << " Dir=" << best_direction_score
                << " Open=" << best_openness_score
                << " Safe=" << best_safety_score
                << " Ang=" << (int)MathUtils::radToDeg(angle_to_target) << "deg"
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
        last_cmd_vel = 0;
        last_cmd_rot = 0;
        accumulated_distance = 0;
        accumulated_rotation = 0;
        std::cout << "OpennessPlanner reset" << std::endl;
    }
};

#endif // OPENNESS_PLANNER_H