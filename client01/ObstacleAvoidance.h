#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include "StructData.h"
#include "MathUtils.h"
#include <cmath>
#include <algorithm>
#include <iostream>

struct ObstacleInfo {
    double distance;
    double angle;
    bool is_critical;
};

/**
 * @brief 障碍物避让模块
 *
 * 修复内容：
 * 1. 使用统一的MathUtils
 * 2. 改进脱困响应速度
 * 3. 优化避障逻辑
 */
class ObstacleAvoidance {
private:
    double safe_distance;
    double critical_distance;
    double emergency_distance;
    double side_emergency_distance;
    double front_sector_angle;
    double side_sector_angle;

    double avoidance_gain;
    double max_avoidance_angle;

    ObstacleInfo front_obstacle;
    ObstacleInfo left_obstacle;
    ObstacleInfo right_obstacle;

    int emergency_stop_counter;
    bool in_recovery_mode;
    double recovery_start_angle;
    int recovery_rotation_frames;

public:
    ObstacleAvoidance() {
        safe_distance = 100.0;
        critical_distance = 60.0;
        emergency_distance = 35.0;
        side_emergency_distance = 30.0;
        front_sector_angle = PI / 6.0;
        side_sector_angle = PI / 3.0;

        avoidance_gain = 1.2;
        max_avoidance_angle = PI / 2.0;

        front_obstacle.distance = 9999.0;
        front_obstacle.angle = 0;
        front_obstacle.is_critical = false;

        left_obstacle.distance = 9999.0;
        left_obstacle.angle = 0;
        left_obstacle.is_critical = false;

        right_obstacle.distance = 9999.0;
        right_obstacle.angle = 0;
        right_obstacle.is_critical = false;

        emergency_stop_counter = 0;
        in_recovery_mode = false;
        recovery_start_angle = 0;
        recovery_rotation_frames = 0;

        std::cout << "ObstacleAvoidance initialized (Fixed Version)" << std::endl;
        std::cout << "  safe_dist=" << safe_distance << "cm" << std::endl;
    }

    bool detectObstacles(INT16* laser_data, double robot_orientation) {
        front_obstacle.distance = 9999.0;
        front_obstacle.is_critical = false;
        left_obstacle.distance = 9999.0;
        left_obstacle.is_critical = false;
        right_obstacle.distance = 9999.0;
        right_obstacle.is_critical = false;

        bool has_obstacle = false;

        for (int i = 0; i < 360; i++) {
            if (laser_data[i] <= 0 || laser_data[i] > 5000) continue;

            double angle = (i * PI / 180.0);
            angle = MathUtils::normalizeAngle(angle);

            double distance = laser_data[i];

            // 前方扇区检测
            if (fabs(angle) <= front_sector_angle) {
                if (distance < front_obstacle.distance) {
                    front_obstacle.distance = distance;
                    front_obstacle.angle = angle;
                    front_obstacle.is_critical = (distance < critical_distance);
                    has_obstacle = true;
                }
            }
            // 左侧扇区检测
            else if (angle > 0 && angle <= side_sector_angle) {
                if (distance < left_obstacle.distance) {
                    left_obstacle.distance = distance;
                    left_obstacle.angle = angle;
                    left_obstacle.is_critical = (distance < critical_distance);
                    has_obstacle = true;
                }
            }
            // 右侧扇区检测
            else if (angle < 0 && angle >= -side_sector_angle) {
                if (distance < right_obstacle.distance) {
                    right_obstacle.distance = distance;
                    right_obstacle.angle = angle;
                    right_obstacle.is_critical = (distance < critical_distance);
                    has_obstacle = true;
                }
            }
        }

        return has_obstacle;
    }

    bool hasAnyEmergencyObstacle() {
        return (front_obstacle.distance < emergency_distance ||
            left_obstacle.distance < side_emergency_distance ||
            right_obstacle.distance < side_emergency_distance);
    }

    bool needEmergencyStop() {
        return hasAnyEmergencyObstacle();
    }

    bool hasCriticalObstacle() {
        return (front_obstacle.is_critical ||
            left_obstacle.is_critical ||
            right_obstacle.is_critical);
    }

    // 实时搜寻最佳可行方向
    double findBestEscapeDirection(INT16* laser_data, double current_angle) {
        double best_angle = current_angle;
        double max_clearance = 0;

        // 扫描360度，寻找最空旷的方向
        for (int i = 0; i < 360; i += 10) {
            double test_angle = i * PI / 180.0;
            if (test_angle > PI) test_angle -= 2 * PI;

            // 计算该方向的平均安全距离
            double total_distance = 0;
            int valid_count = 0;

            for (int offset = -15; offset <= 15; offset++) {
                int idx = i + offset;
                if (idx < 0) idx += 360;
                if (idx >= 360) idx -= 360;

                if (laser_data[idx] > 0 && laser_data[idx] < 5000) {
                    total_distance += laser_data[idx];
                    valid_count++;
                }
            }

            if (valid_count > 0) {
                double avg_clearance = total_distance / valid_count;
                if (avg_clearance > max_clearance) {
                    max_clearance = avg_clearance;
                    best_angle = test_angle;
                }
            }
        }

        std::cout << "  Best escape direction: " << (int)MathUtils::radToDeg(best_angle)
            << "deg, clearance: " << (int)max_clearance << "cm" << std::endl;

        return best_angle;
    }

    void computeAvoidanceVelocity(double target_vel, double target_rot,
        double& adjusted_vel, double& adjusted_rot,
        INT16* laser_data, double current_angle) {
        adjusted_vel = target_vel;
        adjusted_rot = target_rot;

        // 紧急情况计数
        if (needEmergencyStop()) {
            emergency_stop_counter++;

            if (emergency_stop_counter > 5 && !in_recovery_mode) {
                std::cout << ">> EMERGENCY! Entering recovery mode (fast)" << std::endl;
                std::cout << "   Front:" << (int)front_obstacle.distance
                    << " Left:" << (int)left_obstacle.distance
                    << " Right:" << (int)right_obstacle.distance << std::endl;
                in_recovery_mode = true;
                recovery_start_angle = current_angle;
                recovery_rotation_frames = 0;
            }
        }
        else {
            if (emergency_stop_counter > 0) {
                std::cout << ">> Cleared emergency zone" << std::endl;
            }
            emergency_stop_counter = 0;
        }

        // 脱困模式
        if (in_recovery_mode) {
            recovery_rotation_frames++;

            // 保持微小前进速度以便位姿更新
            adjusted_vel = 5.0;

            // 实时搜寻最佳可行方向
            double best_direction = findBestEscapeDirection(laser_data, current_angle);
            double angle_to_best = MathUtils::angleDifference(best_direction, current_angle);

            // 检查是否找到足够空旷的路径
            bool path_found = false;
            if (laser_data != nullptr) {
                int best_dir_index = (int)((best_direction * 180.0 / PI) + 180 + 0.5);
                if (best_dir_index < 0) best_dir_index += 360;
                if (best_dir_index >= 360) best_dir_index -= 360;

                double clearance = 0;
                int count = 0;
                for (int offset = -20; offset <= 20; offset++) {
                    int idx = best_dir_index + offset;
                    if (idx < 0) idx += 360;
                    if (idx >= 360) idx -= 360;
                    if (laser_data[idx] > 0 && laser_data[idx] < 5000) {
                        clearance += laser_data[idx];
                        count++;
                    }
                }
                if (count > 0) clearance /= count;

                if (clearance > 70.0 && fabs(angle_to_best) < 0.5) {
                    path_found = true;
                    std::cout << ">> Recovery: Found clear path! Clearance="
                        << (int)clearance << "cm, angle_error="
                        << (int)MathUtils::radToDeg(angle_to_best) << "deg" << std::endl;
                }
            }

            // 超时或找到路径时退出恢复模式
            if (path_found || recovery_rotation_frames > 30) {
                if (path_found) {
                    std::cout << ">> Recovery complete! Clear path found." << std::endl;
                }
                else {
                    std::cout << ">> Recovery timeout (" << recovery_rotation_frames
                        << " frames), forcing resume." << std::endl;
                }
                in_recovery_mode = false;
                recovery_rotation_frames = 0;
                emergency_stop_counter = 0;
                return;
            }

            // 旋转速度
            adjusted_rot = (angle_to_best > 0) ? 0.6 : -0.6;

            if (recovery_rotation_frames % 5 == 1) {
                std::cout << ">> Recovery: Rotating ["
                    << recovery_rotation_frames << "/30] angle_diff="
                    << (int)MathUtils::radToDeg(angle_to_best) << "deg, v="
                    << (int)adjusted_vel << "cm/s" << std::endl;
            }

            return;
        }

        // 正常避障逻辑
        bool need_avoidance = false;
        double avoidance_angle = 0;

        // 前方障碍物处理
        if (front_obstacle.distance < safe_distance) {
            need_avoidance = true;

            double speed_ratio = front_obstacle.distance / safe_distance;
            adjusted_vel *= speed_ratio * speed_ratio;

            if (front_obstacle.distance < 50.0) {
                adjusted_vel = MathUtils::clamp(adjusted_vel, -5.0, 5.0);
            }

            double left_safety = left_obstacle.distance;
            double right_safety = right_obstacle.distance;

            if (left_safety > right_safety + 30.0 && left_safety > 80.0) {
                avoidance_angle = avoidance_gain * (safe_distance - front_obstacle.distance) / safe_distance;
            }
            else if (right_safety > left_safety + 30.0 && right_safety > 80.0) {
                avoidance_angle = -avoidance_gain * (safe_distance - front_obstacle.distance) / safe_distance;
            }
            else {
                if (left_safety > right_safety) {
                    avoidance_angle = avoidance_gain * 0.5;
                    adjusted_vel *= 0.4;
                }
                else {
                    avoidance_angle = -avoidance_gain * 0.5;
                    adjusted_vel *= 0.4;
                }
            }
        }

        // 侧面障碍物处理
        if (left_obstacle.distance < safe_distance * 0.8) {
            need_avoidance = true;
            double side_avoidance = -0.6 * (safe_distance * 0.8 - left_obstacle.distance) / (safe_distance * 0.8);

            if (adjusted_rot > 0.1 && left_obstacle.distance < 45.0) {
                avoidance_angle = -0.7;
                adjusted_vel *= 0.3;
            }
            else {
                avoidance_angle += side_avoidance;
                adjusted_vel *= 0.7;
            }
        }

        if (right_obstacle.distance < safe_distance * 0.8) {
            need_avoidance = true;
            double side_avoidance = 0.6 * (safe_distance * 0.8 - right_obstacle.distance) / (safe_distance * 0.8);

            if (adjusted_rot < -0.1 && right_obstacle.distance < 45.0) {
                avoidance_angle = 0.7;
                adjusted_vel *= 0.3;
            }
            else {
                avoidance_angle += side_avoidance;
                adjusted_vel *= 0.7;
            }
        }

        // 应用避障转向
        if (need_avoidance) {
            avoidance_angle = MathUtils::clamp(avoidance_angle, -max_avoidance_angle, max_avoidance_angle);
            adjusted_rot += avoidance_angle;

            // 确保避障时保持最小前进速度
            if (fabs(adjusted_vel) < 5.0) {
                adjusted_vel = (adjusted_vel >= 0) ? 5.0 : -5.0;
            }
        }

        // 限制速度和转向
        adjusted_vel = MathUtils::clamp(adjusted_vel, -25.0, 50.0);
        adjusted_rot = MathUtils::clamp(adjusted_rot, -0.6, 0.6);
    }

    double evaluatePathSafety(POSE cur_pose, double test_angle, INT16* laser_data) {
        double angle_diff = MathUtils::angleDifference(test_angle, cur_pose.coor_ori);

        int laser_index = (int)((angle_diff * 180.0 / PI) + 180 + 0.5);
        if (laser_index < 0) laser_index += 360;
        if (laser_index >= 360) laser_index -= 360;

        double min_distance = 9999.0;
        int check_range = 25;

        for (int offset = -check_range; offset <= check_range; offset++) {
            int idx = laser_index + offset;
            if (idx < 0) idx += 360;
            if (idx >= 360) idx -= 360;

            if (laser_data[idx] > 0 && laser_data[idx] < min_distance) {
                min_distance = laser_data[idx];
            }
        }

        if (min_distance > safe_distance) {
            return 1.0;
        }
        else if (min_distance < emergency_distance) {
            return 0.0;
        }
        else {
            double ratio = (min_distance - emergency_distance) / (safe_distance - emergency_distance);
            return ratio * ratio;
        }
    }

    void getObstacleInfo(ObstacleInfo& front, ObstacleInfo& left, ObstacleInfo& right) {
        front = front_obstacle;
        left = left_obstacle;
        right = right_obstacle;
    }

    void setSafetyParameters(double safe_dist, double critical_dist, double emergency_dist) {
        safe_distance = safe_dist;
        critical_distance = critical_dist;
        emergency_distance = emergency_dist;
    }

    bool isInRecoveryMode() {
        return in_recovery_mode;
    }

    void reset() {
        emergency_stop_counter = 0;
        recovery_rotation_frames = 0;
        in_recovery_mode = false;
        front_obstacle.distance = 9999.0;
        left_obstacle.distance = 9999.0;
        right_obstacle.distance = 9999.0;
        std::cout << "ObstacleAvoidance reset" << std::endl;
    }

    // ✅ 新增：获取最近障碍物距离
    double getNearestObstacleDistance() const {
        return (std::min)({ front_obstacle.distance,
                          left_obstacle.distance,
                          right_obstacle.distance });
    }
};

#endif // OBSTACLE_AVOIDANCE_H