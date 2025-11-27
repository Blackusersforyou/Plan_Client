#ifndef POSE_TRACKER_H
#define POSE_TRACKER_H

#include "StructData.h"
#include "MathUtils.h"
#include <windows.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <deque>
#include <vector>

/**
 * @brief 高精度位姿跟踪器 - 扩展卡尔曼滤波器 (EKF)
 */
class PoseTracker {
public:
	struct HighPrecisionPose {
		double x;
		double y;
		double ori;
	};
	
	struct AdaptiveCoefficients {
		double vel_scale;
		double angle_scale;
		double confidence;
	};
	
	struct UpdateResult {
		POSE integer_pose;
		double actual_move;
		double actual_rotation;
		bool is_stuck;
		double delta_time;
		AdaptiveCoefficients coefficients;
		double estimated_vel;
		double estimated_rot;
		double position_uncertainty;
		double orientation_uncertainty;
		double innovation;
	};
	
	struct SLAMCorrectionResult {
		bool correction_applied;
		double correction_x;
		double correction_y;
		double correction_angle;
		double correction_distance;
		double confidence;
	};
	
	struct LaserCorrectionResult {
		bool correction_applied;
		int corrected_count;
		double avg_correction;
		double max_correction;
	};

private:
	HighPrecisionPose high_precision_pose_;
	
	LARGE_INTEGER frequency_;
	LARGE_INTEGER last_update_time_;
	LARGE_INTEGER last_valid_time_;
	bool timer_initialized_;
	int abnormal_dt_count_;
	
	double last_vel_;
	double last_rot_;
	int low_speed_frames_;
	int stuck_counter_;
	int frames_no_pose_change_;
	
	double base_vel_scale_;
	double base_angle_scale_;
	const double MIN_VALID_DT = 0.015;
	const double MAX_VALID_DT = 0.100;
	const double DEFAULT_DT = 0.030;
	
	int frames_since_last_slam_correction_;
	const int SLAM_CORRECTION_INTERVAL = 100;
	const double MAX_SLAM_CORRECTION_DISTANCE = 30.0;
	const double SLAM_CORRECTION_BLEND_FACTOR = 0.3;
	double total_distance_traveled_;
	
	// ========== ✅ 简化的 EKF 状态 ==========
	struct EKFState {
		double x, y, theta, v, omega;
		
		double pos_var;
		double angle_var;
		double vel_var;
		double omega_var;
		
		const double PROCESS_NOISE_POS = 0.5;
		const double PROCESS_NOISE_ANGLE = 0.01;
		const double PROCESS_NOISE_VEL = 1.0;
		const double PROCESS_NOISE_OMEGA = 0.05;
		
		const double MEASUREMENT_NOISE_POS = 4.0;
		const double MEASUREMENT_NOISE_ANGLE = 0.05;
		
		EKFState() {
			x = 0.0;
			y = 0.0;
			theta = 0.0;
			v = 0.0;
			omega = 0.0;
			
			pos_var = 10.0;
			angle_var = 0.1;
			vel_var = 5.0;
			omega_var = 0.1;
		}
		
		/**
		 * ✅ 简化的 EKF 预测
		 */
		void predict(double dt, double cmd_v, double cmd_omega, 
		            double vel_scale, double angle_scale) {
			// 速度更新（加衰减）
			const double VEL_DECAY = 0.95;
			v = VEL_DECAY * v + (1 - VEL_DECAY) * cmd_v * vel_scale;
			omega = VEL_DECAY * omega + (1 - VEL_DECAY) * cmd_omega * angle_scale;
			
			v = (std::max)(-50.0, (std::min)(50.0, v));
			omega = (std::max)(-1.0, (std::min)(1.0, omega));
			
			// 状态预测（中点法）
			double theta_mid = theta + 0.5 * omega * dt;
			x += v * cos(theta_mid) * dt;
			y += v * sin(theta_mid) * dt;
			theta = MathUtils::normalizeAngle(theta + omega * dt);
			
			// 协方差预测（简化：仅对角元素）
			pos_var += PROCESS_NOISE_POS * dt;
			angle_var += PROCESS_NOISE_ANGLE * dt;
			vel_var += PROCESS_NOISE_VEL * dt;
			omega_var += PROCESS_NOISE_OMEGA * dt;
			
			// 防止协方差过大
			pos_var = (std::min)(pos_var, 50.0);
			angle_var = (std::min)(angle_var, 0.5);
			vel_var = (std::min)(vel_var, 20.0);
			omega_var = (std::min)(omega_var, 0.5);
		}
		
		/**
		 * ✅ 简化的 EKF 更新
		 */
		double update(double x_obs, double y_obs, double theta_obs, double confidence) {
			// 新息
			double innov_x = x_obs - x;
			double innov_y = y_obs - y;
			double innov_theta = MathUtils::angleDifference(theta_obs, theta);
			
			// 卡尔曼增益（简化）
			double K_pos = pos_var / (pos_var + MEASUREMENT_NOISE_POS / confidence);
			double K_angle = angle_var / (angle_var + MEASUREMENT_NOISE_ANGLE / confidence);
			
			K_pos = (std::max)(0.0, (std::min)(1.0, K_pos));
			K_angle = (std::max)(0.0, (std::min)(1.0, K_angle));
			
			// 状态更新
			x += K_pos * innov_x;
			y += K_pos * innov_y;
			theta = MathUtils::normalizeAngle(theta + K_angle * innov_theta);
			
			// 协方差更新
			pos_var = (1 - K_pos) * pos_var;
			angle_var = (1 - K_angle) * angle_var;
			
			// 确保最小值
			pos_var = (std::max)(0.1, pos_var);
			angle_var = (std::max)(0.001, angle_var);
			
			// 返回新息范数
			return sqrt(innov_x*innov_x + innov_y*innov_y + innov_theta*innov_theta);
		}
		
		double getPositionUncertainty() const {
			return sqrt(2 * pos_var);  // x, y 的组合不确定性
		}
		
		double getOrientationUncertainty() const {
			return sqrt(angle_var);
		}
	};
	
	EKFState ekf_state_;
	
	// ========== ✅ 激光测距误差模型 ==========
	struct LaserErrorModel {
		double distance_scale;
		double distance_offset;
		double angle_noise;
		int calibration_samples;
		
		LaserErrorModel() 
			: distance_scale(1.0)
			, distance_offset(0.0)
			, angle_noise(0.005)
			, calibration_samples(0) {}
		
		double correctDistance(double raw_distance) const {
			if (raw_distance <= 0 || raw_distance > 5000) return raw_distance;
			return raw_distance * distance_scale + distance_offset;
		}
		
		void calibrate(const std::vector<std::pair<double, double>>& samples) {
			if (samples.size() < 10) return;
			
			double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
			int n = 0;
			
			for (const auto& sample : samples) {
				double expected = sample.first;
				double measured = sample.second;
				
				if (expected > 10 && expected < 3000) {
					sum_x += expected;
					sum_y += measured;
					sum_xx += expected * expected;
					sum_xy += expected * measured;
					n++;
				}
			}
			
			if (n > 5) {
				distance_scale = (n * sum_xy - sum_x * sum_y) / 
				                (n * sum_xx - sum_x * sum_x + 1e-6);
				distance_offset = (sum_y - distance_scale * sum_x) / n;
				
				distance_scale = (std::max)(0.9, (std::min)(1.1, distance_scale));
				distance_offset = (std::max)(-10.0, (std::min)(10.0, distance_offset));
				
				calibration_samples += n;
			}
		}
	};
	
	LaserErrorModel laser_error_model_;

public:
	PoseTracker() 
		: timer_initialized_(false)
		, last_vel_(0.0)
		, last_rot_(0.0)
		, low_speed_frames_(0)
		, stuck_counter_(0)
		, frames_no_pose_change_(0)
		, base_vel_scale_(5.0)
		, base_angle_scale_(5.0)
		, frames_since_last_slam_correction_(0)
		, total_distance_traveled_(0.0)
		, abnormal_dt_count_(0)  // ✅ 初始化
	{
		high_precision_pose_.x = 0.0;
		high_precision_pose_.y = 0.0;
		high_precision_pose_.ori = 0.0;
	}
	
	void initialize(const POSE& initial_pose) {
		high_precision_pose_.x = initial_pose.coor_x;
		high_precision_pose_.y = initial_pose.coor_y;
		high_precision_pose_.ori = initial_pose.coor_ori;
		
		ekf_state_.x = initial_pose.coor_x;
		ekf_state_.y = initial_pose.coor_y;
		ekf_state_.theta = initial_pose.coor_ori;
		ekf_state_.v = 0.0;
		ekf_state_.omega = 0.0;
		ekf_state_.pos_var = 10.0;
		ekf_state_.angle_var = 0.1;
		ekf_state_.vel_var = 5.0;
		ekf_state_.omega_var = 0.1;
		
		if (!timer_initialized_) {
			QueryPerformanceFrequency(&frequency_);
			QueryPerformanceCounter(&last_update_time_);
			last_valid_time_ = last_update_time_;
			timer_initialized_ = true;
		}
		
		last_vel_ = 0.0;
		last_rot_ = 0.0;
		low_speed_frames_ = 0;
		stuck_counter_ = 0;
		frames_no_pose_change_ = 0;
		frames_since_last_slam_correction_ = 0;
		total_distance_traveled_ = 0.0;
		abnormal_dt_count_ = 0;
		
		std::cout << "[PoseTracker] Initialized with EKF" << std::endl;
		std::cout << "  Position: (" << high_precision_pose_.x << ", " 
		          << high_precision_pose_.y << "), ori: " 
		          << (high_precision_pose_.ori * 180 / PI) << " deg" << std::endl;
	}
	
	void reset(const POSE& new_pose) {
		high_precision_pose_.x = new_pose.coor_x;
		high_precision_pose_.y = new_pose.coor_y;
		high_precision_pose_.ori = new_pose.coor_ori;
		
		ekf_state_.x = new_pose.coor_x;
		ekf_state_.y = new_pose.coor_y;
		ekf_state_.theta = new_pose.coor_ori;
		ekf_state_.v = 0.0;
		ekf_state_.omega = 0.0;
		ekf_state_.pos_var = 10.0;
		ekf_state_.angle_var = 0.1;
		ekf_state_.vel_var = 5.0;
		ekf_state_.omega_var = 0.1;
		
		stuck_counter_ = 0;
		frames_no_pose_change_ = 0;
		frames_since_last_slam_correction_ = 0;
		total_distance_traveled_ = 0.0;
		abnormal_dt_count_ = 0;
		
		std::cout << "[PoseTracker] Reset with EKF" << std::endl;
	}
	
	UpdateResult update(double cmd_linear_vel, double cmd_angular_vel, 
	                    bool is_avoiding_obstacle, double obstacle_distance) {
		UpdateResult result;
		
		double dt = getActualDeltaTime();
		result.delta_time = dt;
		
		AdaptiveCoefficients coeff = computeAdaptiveCoefficients(
			cmd_linear_vel, cmd_angular_vel, is_avoiding_obstacle, 
			obstacle_distance, stuck_counter_);
		result.coefficients = coeff;
		
		double old_x = ekf_state_.x;
		double old_y = ekf_state_.y;
		double old_theta = ekf_state_.theta;
		
		// ✅ EKF 预测
		ekf_state_.predict(dt, cmd_linear_vel, cmd_angular_vel, 
		                  coeff.vel_scale, coeff.angle_scale);
		
		high_precision_pose_.x = ekf_state_.x;
		high_precision_pose_.y = ekf_state_.y;
		high_precision_pose_.ori = ekf_state_.theta;
		
		result.integer_pose.coor_x = (INT16)(ekf_state_.x + 0.5);
		result.integer_pose.coor_y = (INT16)(ekf_state_.y + 0.5);
		result.integer_pose.coor_ori = ekf_state_.theta;
		
		result.actual_move = MathUtils::calculateDistance(
			old_x, old_y, ekf_state_.x, ekf_state_.y);
		result.actual_rotation = fabs(
			MathUtils::angleDifference(ekf_state_.theta, old_theta));
		
		result.estimated_vel = ekf_state_.v;
		result.estimated_rot = ekf_state_.omega;
		
		result.position_uncertainty = ekf_state_.getPositionUncertainty();
		result.orientation_uncertainty = ekf_state_.getOrientationUncertainty();
		result.innovation = 0.0;
		
		total_distance_traveled_ += result.actual_move;
		frames_since_last_slam_correction_++;
		
		updateStuckDetection(result, cmd_linear_vel, cmd_angular_vel);
		
		last_vel_ = cmd_linear_vel;
		last_rot_ = cmd_angular_vel;
		
		return result;
	}
	
	SLAMCorrectionResult correctWithSLAM(const POSE& slam_pose, double slam_confidence = 0.8) {
		SLAMCorrectionResult result;
		result.correction_applied = false;
		
		double dx = slam_pose.coor_x - ekf_state_.x;
		double dy = slam_pose.coor_y - ekf_state_.y;
		double distance = sqrt(dx * dx + dy * dy);
		double angle_diff = MathUtils::angleDifference(slam_pose.coor_ori, ekf_state_.theta);
		
		result.correction_x = dx;
		result.correction_y = dy;
		result.correction_angle = angle_diff;
		result.correction_distance = distance;
		
		if (distance > MAX_SLAM_CORRECTION_DISTANCE) {
			std::cout << "[SLAM Correction] REJECTED: Distance too large (" 
			          << (int)distance << " cm)" << std::endl;
			return result;
		}
		
		if (distance < 2.0 && fabs(angle_diff) < 0.05) {
			return result;
		}
		
		// ✅ EKF 更新
		double innovation = ekf_state_.update(
			slam_pose.coor_x, slam_pose.coor_y, slam_pose.coor_ori, slam_confidence);
		
		high_precision_pose_.x = ekf_state_.x;
		high_precision_pose_.y = ekf_state_.y;
		high_precision_pose_.ori = ekf_state_.theta;
		
		result.correction_applied = true;
		result.confidence = slam_confidence;
		
		total_distance_traveled_ = 0.0;
		frames_since_last_slam_correction_ = 0;
		
		std::cout << "[SLAM Correction] APPLIED with EKF:" << std::endl;
		std::cout << "  Position: dx=" << std::fixed << std::setprecision(1) 
		          << dx << " dy=" << dy << " (dist=" << (int)distance << " cm)" << std::endl;
		std::cout << "  Innovation: " << std::setprecision(2) << innovation << std::endl;
		std::cout << "  Uncertainty: pos=" << ekf_state_.getPositionUncertainty() 
		          << " cm, ori=" << (ekf_state_.getOrientationUncertainty() * 180 / PI) << " deg" << std::endl;
		
		return result;
	}
	
	LaserCorrectionResult correctLaserData(INT16* laser_data) {
		LaserCorrectionResult result;
		result.correction_applied = false;
		result.corrected_count = 0;
		result.avg_correction = 0.0;
		result.max_correction = 0.0;
		
		double total_correction = 0.0;
		
		for (int i = 0; i < 360; i++) {
			if (laser_data[i] > 0 && laser_data[i] < 5000) {
				double raw_distance = laser_data[i];
				double corrected_distance = laser_error_model_.correctDistance(raw_distance);
				
				double correction = fabs(corrected_distance - raw_distance);
				if (correction > 0.1) {
					laser_data[i] = (INT16)(corrected_distance + 0.5);
					result.corrected_count++;
					total_correction += correction;
					
					if (correction > result.max_correction) {
						result.max_correction = correction;
					}
				}
			}
		}
		
		if (result.corrected_count > 0) {
			result.correction_applied = true;
			result.avg_correction = total_correction / result.corrected_count;
		}
		
		return result;
	}
	
	void calibrateLaserModel(const std::vector<std::pair<double, double>>& calibration_samples) {
		laser_error_model_.calibrate(calibration_samples);
		
		if (laser_error_model_.calibration_samples > 0) {
			std::cout << "[Laser Calibration] Updated:" << std::endl;
			std::cout << "  Scale: " << std::fixed << std::setprecision(4) 
			          << laser_error_model_.distance_scale << std::endl;
			std::cout << "  Offset: " << laser_error_model_.distance_offset << " cm" << std::endl;
		}
	}
	
	bool shouldPerformSLAMCorrection() const {
		if (abnormal_dt_count_ >= 5) return true;
		if (frames_since_last_slam_correction_ >= SLAM_CORRECTION_INTERVAL && 
		    total_distance_traveled_ >= 100.0) return true;
		if (frames_since_last_slam_correction_ >= 50 && 
		    total_distance_traveled_ >= 30.0) return true;
		if (ekf_state_.getPositionUncertainty() > 20.0) return true;
		return false;
	}
	
	double getTotalDistanceTraveled() const { return total_distance_traveled_; }
	int getFramesSinceLastCorrection() const { return frames_since_last_slam_correction_; }
	HighPrecisionPose getHighPrecisionPose() const { return high_precision_pose_; }
	int getStuckCounter() const { return stuck_counter_; }
	
	void printStatus(const UpdateResult& result, 
	                 const AdaptiveCoefficients& coeff,
	                 double obstacle_dist) const {
		std::cout << "[POSE] x=" << result.integer_pose.coor_x 
		          << " y=" << result.integer_pose.coor_y 
		          << " ori=" << (int)(result.integer_pose.coor_ori * 180 / PI) << " deg" << std::endl;
		
		std::cout << "      EKF: x=" << std::fixed << std::setprecision(2) 
		          << ekf_state_.x << " y=" << ekf_state_.y
		          << " | dt=" << (result.delta_time * 1000) << " ms" << std::endl;
		
		std::cout << "      Coeff: v=" << coeff.vel_scale
		          << " angle=" << coeff.angle_scale
		          << " | stuck=" << stuck_counter_ << std::endl;
		
		std::cout << "      Delta: move=" << std::setprecision(3) << result.actual_move << " cm"
		          << " rot=" << (int)(result.actual_rotation * 180 / PI) << " deg" << std::endl;
		
		std::cout << "      Uncertainty: pos=" << std::setprecision(2) << result.position_uncertainty 
		          << " cm ori=" << (result.orientation_uncertainty * 180 / PI) << " deg" << std::endl;
	}

private:
	AdaptiveCoefficients computeAdaptiveCoefficients(
		double velocity, double angular_velocity, 
		bool is_avoiding_obstacle, double obstacle_distance,
		int stuck_counter) {
		
		AdaptiveCoefficients coeff;
		
		if (abnormal_dt_count_ > 0 && abnormal_dt_count_ < 20) {
			coeff.vel_scale = base_vel_scale_ * 0.6;
			coeff.angle_scale = base_angle_scale_ * 0.6;
			coeff.confidence = 0.4;
			return coeff;
		}
		
		if (stuck_counter > 10) {
			coeff.vel_scale = base_vel_scale_ * 1.5;
			coeff.angle_scale = base_angle_scale_ * 1.5;
			coeff.confidence = 0.5;
			return coeff;
		}
		
		if (velocity > 0 && velocity < 10.0) {
			coeff.vel_scale = base_vel_scale_ * 1.4;
			coeff.angle_scale = base_angle_scale_ * 1.1;
			coeff.confidence = 0.85;
			return coeff;
		}
		
		if (is_avoiding_obstacle && obstacle_distance < 30.0) {
			coeff.vel_scale = base_vel_scale_ * 0.5;
			coeff.angle_scale = base_angle_scale_ * 0.5;
			coeff.confidence = 0.4;
			return coeff;
		}
		
		if (is_avoiding_obstacle && obstacle_distance < 50.0) {
			coeff.vel_scale = base_vel_scale_ * 0.70;
			coeff.angle_scale = base_angle_scale_ * 0.65;
			coeff.confidence = 0.6;
			return coeff;
		}
		
		if (fabs(angular_velocity) > 0.6) {
			coeff.vel_scale = base_vel_scale_ * 0.65;
			coeff.angle_scale = base_angle_scale_ * 0.50;
			coeff.confidence = 0.65;
			return coeff;
		}
		
		if (fabs(angular_velocity) > 0.4) {
			coeff.vel_scale = base_vel_scale_ * 0.80;
			coeff.angle_scale = base_angle_scale_ * 0.65;
			coeff.confidence = 0.7;
			return coeff;
		}
		
		coeff.vel_scale = base_vel_scale_;
		coeff.angle_scale = base_angle_scale_;
		coeff.confidence = 0.9;
		
		return coeff;
	}
	
	void updateStuckDetection(UpdateResult& result, double cmd_vel, double cmd_rot) {
		bool pose_changed = (result.actual_move > 0.5 || result.actual_rotation > 0.01);
		
		if (!pose_changed && (fabs(cmd_vel) > 2.0 || fabs(cmd_rot) > 0.05)) {
			frames_no_pose_change_++;
			stuck_counter_++;
		} else {
			frames_no_pose_change_ = 0;
			if (stuck_counter_ > 0) stuck_counter_--;
		}
		
		result.is_stuck = (stuck_counter_ > 5);
	}
	
	double getActualDeltaTime() {
		if (!timer_initialized_) {
			QueryPerformanceFrequency(&frequency_);
			QueryPerformanceCounter(&last_update_time_);
			last_valid_time_ = last_update_time_;
			timer_initialized_ = true;
			return DEFAULT_DT;
		}
		
		LARGE_INTEGER current_time;
		QueryPerformanceCounter(&current_time);
		
		double dt = (double)(current_time.QuadPart - last_update_time_.QuadPart) 
		            / (double)frequency_.QuadPart;
		
		bool is_abnormal = false;
		
		if (dt < MIN_VALID_DT) {
			is_abnormal = true;
			dt = DEFAULT_DT * 0.5;
		} 
		else if (dt > MAX_VALID_DT) {
			is_abnormal = true;
			
			LARGE_INTEGER time_since_last_valid;
			time_since_last_valid.QuadPart = current_time.QuadPart - last_valid_time_.QuadPart;
			double time_since_valid = (double)time_since_last_valid.QuadPart / (double)frequency_.QuadPart;
			
			if (time_since_valid < MAX_VALID_DT * 2) {
				dt = time_since_valid;
			} else {
				dt = DEFAULT_DT;
			}
			
			abnormal_dt_count_++;
			if (abnormal_dt_count_ % 10 == 1) {
				std::cout << "[WARN] Abnormal dt (count=" << abnormal_dt_count_ 
				          << "), smoothed to " << (dt * 1000) << " ms" << std::endl;
			}
		}
		
		last_update_time_ = current_time;
		
		if (!is_abnormal) {
			last_valid_time_ = current_time;
			if (abnormal_dt_count_ > 0) {
				abnormal_dt_count_ = 0;
			}
		}
		
		return dt;
	}
};

#endif // POSE_TRACKER_H