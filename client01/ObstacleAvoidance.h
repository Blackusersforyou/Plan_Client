#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include "StructData.h"
#include <cmath>
#include <algorithm>
#include <iostream>

struct ObstacleInfo {
	double distance;
	double angle;
	bool is_critical;
};

class ObstacleAvoidance {
private:
	// 安全参数
	double safe_distance;
	double critical_distance;
	double emergency_distance;
	double side_emergency_distance;  // 新增：侧面紧急距离
	double front_sector_angle;
	double side_sector_angle;
	
	// 避障参数
	double avoidance_gain;
	double max_avoidance_angle;
	
	// 障碍物检测历史
	ObstacleInfo front_obstacle;
	ObstacleInfo left_obstacle;
	ObstacleInfo right_obstacle;
	
	// 脱困机制
	int emergency_stop_counter;
	int recovery_mode_counter;
	bool in_recovery_mode;
	double recovery_rotation_dir;
	
public:
	ObstacleAvoidance() {
		// ✅ 问题3解决：增加安全参数，避免贴边行走
		safe_distance = 100.0;           // 从80提高到100cm
		critical_distance = 60.0;        // 从45提高到60cm
		emergency_distance = 35.0;       // 从25提高到35cm
		side_emergency_distance = 30.0;  // 从20提高到30cm
		front_sector_angle = PI / 6.0;
		side_sector_angle = PI / 3.0;
		
		avoidance_gain = 1.2;            // 从1.0提高到1.2，更强避障
		max_avoidance_angle = PI / 2.0;  // 从PI/2.5提高到PI/2
		
		// 初始化障碍物信息
		front_obstacle.distance = 9999.0;
		front_obstacle.angle = 0;
		front_obstacle.is_critical = false;
		
		left_obstacle.distance = 9999.0;
		left_obstacle.angle = 0;
		left_obstacle.is_critical = false;
		
		right_obstacle.distance = 9999.0;
		right_obstacle.angle = 0;
		right_obstacle.is_critical = false;
		
		// 初始化脱困机制
		emergency_stop_counter = 0;
		recovery_mode_counter = 0;
		in_recovery_mode = false;
		recovery_rotation_dir = 1.0;
		
		std::cout << "ObstacleAvoidance initialized (safe_dist=100cm)" << std::endl;
	}
	
	// 检测障碍物
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
			if (angle > PI) angle -= 2 * PI;
			
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
	
	// 新增：检查任意方向是否有紧急障碍物
	bool hasAnyEmergencyObstacle() {
		return (front_obstacle.distance < emergency_distance ||
		        left_obstacle.distance < side_emergency_distance ||
		        right_obstacle.distance < side_emergency_distance);
	}
	
	// 判断是否需要紧急停止
	bool needEmergencyStop() {
		return hasAnyEmergencyObstacle();
	}
	
	// 判断是否有临界障碍物
	bool hasCriticalObstacle() {
		return (front_obstacle.is_critical || 
		        left_obstacle.is_critical || 
		        right_obstacle.is_critical);
	}
	
	// 寻找最佳脱困方向
	double findBestEscapeDirection() {
		// 选择障碍物最少的方向
		if (left_obstacle.distance > right_obstacle.distance + 20.0) {
			std::cout << "  Escape: Turn LEFT (L:" << (int)left_obstacle.distance 
			          << " vs R:" << (int)right_obstacle.distance << ")" << std::endl;
			return 1.0;
		}
		else if (right_obstacle.distance > left_obstacle.distance + 20.0) {
			std::cout << "  Escape: Turn RIGHT (L:" << (int)left_obstacle.distance 
			          << " vs R:" << (int)right_obstacle.distance << ")" << std::endl;
			return -1.0;
		}
		else {
			// 两边差不多，随机选择
			std::cout << "  Escape: Random turn (both sides similar)" << std::endl;
			return (rand() % 2 == 0) ? 1.0 : -1.0;
		}
	}
	
	// 新增：检查后方是否有障碍物
	bool hasRearObstacle(INT16* laser_data, double robot_orientation) {
		// 检查后方±45度范围内的障碍物
		for (int i = 135; i <= 225; i++) {  // 后方90度扇区
			if (laser_data[i] > 0 && laser_data[i] < 40.0) {  // 40cm以内有障碍
				return true;
			}
		}
		return false;
	}
	
	// 修改：计算避障速度调整（改进版） - 更新后退逻辑
	void computeAvoidanceVelocity(double target_vel, double target_rot, 
	                               double& adjusted_vel, double& adjusted_rot,
	                               INT16* laser_data = nullptr) {
		adjusted_vel = target_vel;
		adjusted_rot = target_rot;
		
		// 检查是否需要紧急停止
		if (needEmergencyStop()) {
			emergency_stop_counter++;
			
			if (emergency_stop_counter > 3 && !in_recovery_mode) {
				std::cout << ">> EMERGENCY! Entering recovery mode" << std::endl;
				std::cout << "   Front:" << (int)front_obstacle.distance 
				          << " Left:" << (int)left_obstacle.distance 
				          << " Right:" << (int)right_obstacle.distance << std::endl;
				in_recovery_mode = true;
				recovery_mode_counter = 0;
				recovery_rotation_dir = findBestEscapeDirection();
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
			recovery_mode_counter++;
			
			if (recovery_mode_counter <= 20) {
				bool rear_blocked = false;
				if (laser_data != nullptr) {
					rear_blocked = hasRearObstacle(laser_data, 0);
				}
				
				if (rear_blocked) {
					std::cout << ">> Recovery: Rear blocked, skip backing" << std::endl;
					recovery_mode_counter = 21;
					adjusted_vel = 0;
					adjusted_rot = recovery_rotation_dir * 0.6;
				}
				else {
					adjusted_vel = -12.0;
					adjusted_rot = recovery_rotation_dir * 0.2;
				}
				
				if (recovery_mode_counter % 5 == 1) {
					std::cout << ">> Recovery: Backing up... [" 
					          << recovery_mode_counter << "/20]" 
					          << (rear_blocked ? " BLOCKED" : "") << std::endl;
				}
			}
			else if (recovery_mode_counter <= 50) {
				adjusted_vel = 0;
				adjusted_rot = recovery_rotation_dir * 0.6;
				
				if (recovery_mode_counter % 10 == 1) {
					std::cout << ">> Recovery: Rotating... [" 
					          << (recovery_mode_counter-20) << "/30]" << std::endl;
				}
			}
			else {
				std::cout << ">> Recovery complete!" << std::endl;
				in_recovery_mode = false;
				recovery_mode_counter = 0;
				emergency_stop_counter = 0;
			}
			
			return;
		}
		
		// ✅ 问题3解决：正常避障逻辑 - 更强的离墙倾向
		bool need_avoidance = false;
		double avoidance_angle = 0;
		
		// 前方障碍物处理
		if (front_obstacle.distance < safe_distance) {
			need_avoidance = true;
			
			// ✅ 更激进的减速策略
			double speed_ratio = front_obstacle.distance / safe_distance;
			adjusted_vel *= speed_ratio * speed_ratio;  // 平方关系，更快减速
			
			if (front_obstacle.distance < 50.0) {  // 从40提高到50
				adjusted_vel = (std::min)(adjusted_vel, 5.0);  // 从8降到5
			}
			
			double left_safety = left_obstacle.distance;
			double right_safety = right_obstacle.distance;
			
			// ✅ 更严格的转向判断
			if (left_safety > right_safety + 30.0 && left_safety > 80.0) {  // 从25/60提高到30/80
				avoidance_angle = avoidance_gain * (safe_distance - front_obstacle.distance) / safe_distance;
			}
			else if (right_safety > left_safety + 30.0 && right_safety > 80.0) {
				avoidance_angle = -avoidance_gain * (safe_distance - front_obstacle.distance) / safe_distance;
			}
			else {
				if (left_safety > right_safety) {
					avoidance_angle = avoidance_gain * 0.5;  // 从0.4提高到0.5
					adjusted_vel *= 0.4;  // 从0.5降到0.4
				}
				else {
					avoidance_angle = -avoidance_gain * 0.5;
					adjusted_vel *= 0.4;
				}
				
				std::cout << "-- Caution: Limited turning space (L:" 
			          << (int)left_safety << " R:" << (int)right_safety << ")" << std::endl;
			}
		}
		
		// ✅ 侧面障碍物处理 - 更早避让
		if (left_obstacle.distance < safe_distance * 0.8) {  // 新增：80cm时就开始避让
			need_avoidance = true;
			double side_avoidance = -0.6 * (safe_distance * 0.8 - left_obstacle.distance) / (safe_distance * 0.8);
			
			if (adjusted_rot > 0.1 && left_obstacle.distance < 45.0) {  // 从35提高到45
				avoidance_angle = -0.7;  // 从-0.6提高到-0.7
				adjusted_vel *= 0.3;  // 从0.4降到0.3
				std::cout << "-- Warning: Left turn blocked! Forcing right" << std::endl;
			} else {
				avoidance_angle += side_avoidance;
				adjusted_vel *= 0.7;  // 从0.6提高到0.7
			}
		}
		
		if (right_obstacle.distance < safe_distance * 0.8) {
			need_avoidance = true;
			double side_avoidance = 0.6 * (safe_distance * 0.8 - right_obstacle.distance) / (safe_distance * 0.8);
			
			if (adjusted_rot < -0.1 && right_obstacle.distance < 45.0) {
				avoidance_angle = 0.7;
				adjusted_vel *= 0.3;
				std::cout << "-- Warning: Right turn blocked! Forcing left" << std::endl;
			} else {
				avoidance_angle += side_avoidance;
				adjusted_vel *= 0.7;
			}
		}
		
		// 应用避障转向
		if (need_avoidance) {
			avoidance_angle = (std::max)(-max_avoidance_angle, 
			                           (std::min)(max_avoidance_angle, avoidance_angle));
			adjusted_rot += avoidance_angle;
			
			static int output_counter = 0;
			if (++output_counter % 3 == 0) {
				std::cout << "-- Avoiding: F=" << (int)front_obstacle.distance 
				          << " L=" << (int)left_obstacle.distance 
				          << " R=" << (int)right_obstacle.distance 
				          << " Turn=" << avoidance_angle << std::endl;
			}
		}
		
		// 限制速度和转向
		adjusted_vel = (std::max)(-25.0, (std::min)(50.0, adjusted_vel));
		adjusted_rot = (std::max)(-0.6, (std::min)(0.6, adjusted_rot));
	}
	
	// ✅ 改进路径安全性评估
	double evaluatePathSafety(POSE cur_pose, double test_angle, INT16* laser_data) {
		double angle_diff = test_angle - cur_pose.coor_ori;
		while (angle_diff > PI) angle_diff -= 2 * PI;
		while (angle_diff < -PI) angle_diff += 2 * PI;
		
		int laser_index = (int)((angle_diff * 180.0 / PI) + 180 + 0.5);
		if (laser_index < 0) laser_index += 360;
		if (laser_index >= 360) laser_index -= 360;
		
		double min_distance = 9999.0;
		int check_range = 25;  // 从20提高到25，检查范围更大
		
		for (int offset = -check_range; offset <= check_range; offset++) {
			int idx = laser_index + offset;
			if (idx < 0) idx += 360;
			if (idx >= 360) idx -= 360;
			
			if (laser_data[idx] > 0 && laser_data[idx] < min_distance) {
				min_distance = laser_data[idx];
			}
		}
		
		// ✅ 更严格的安全评分（提高安全距离阈值）
		if (min_distance > safe_distance) {
			return 1.0;
		}
		else if (min_distance < emergency_distance) {
			return 0.0;
		}
		else {
			// 使用平方关系，使得不安全方向得分下降更快
			double ratio = (min_distance - emergency_distance) / (safe_distance - emergency_distance);
			return ratio * ratio;  // 平方关系
		}
	}
	
	// 获取障碍物信息
	void getObstacleInfo(ObstacleInfo& front, ObstacleInfo& left, ObstacleInfo& right) {
		front = front_obstacle;
		left = left_obstacle;
		right = right_obstacle;
	}
	
	// 设置安全参数
	void setSafetyParameters(double safe_dist, double critical_dist, double emergency_dist) {
		safe_distance = safe_dist;
		critical_distance = critical_dist;
		emergency_distance = emergency_dist;
	}
	
	// 检查是否在脱困模式
	bool isInRecoveryMode() {
		return in_recovery_mode;
	}
	
	// 重置状态
	void reset() {
		emergency_stop_counter = 0;
		recovery_mode_counter = 0;
		in_recovery_mode = false;
		front_obstacle.distance = 9999.0;
		left_obstacle.distance = 9999.0;
		right_obstacle.distance = 9999.0;
		std::cout << "ObstacleAvoidance reset" << std::endl;
	}
	
	// 新增：强制进入脱困模式（用于碰撞后）
	void forceRecoveryMode() {
		if (!in_recovery_mode) {
			std::cout << ">> FORCED RECOVERY MODE (collision detected)" << std::endl;
			in_recovery_mode = true;
			recovery_mode_counter = 0;
			emergency_stop_counter = 10;  // 设置高计数触发立即脱困
			recovery_rotation_dir = findBestEscapeDirection();
		}
	}
};

#endif