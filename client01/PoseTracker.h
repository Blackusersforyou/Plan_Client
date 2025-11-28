#ifndef POSE_TRACKER_H
#define POSE_TRACKER_H

#include "StructData.h"
#include "MathUtils.h"
#include <windows.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <deque>

/**
 * @brief 简化版位姿跟踪器 - 基于命令速度积分
 *
 * 设计理念：
 * 1. 命令速度是最可靠的位移来源（虽然需要缩放）
 * 2. ICP 在低速场景下不稳定，暂时禁用
 * 3. 提供清晰的调试输出，便于调参
 *
 * 关于速度缩放：
 * 根据之前的分析，命令速度需要乘以 ~3.5 才能匹配实际移动
 * 这可能是因为仿真器输出的速度单位或采样问题
 */
class PoseTracker {
public:
    struct HighPrecisionPose {
        double x;      // cm
        double y;      // cm  
        double ori;    // rad
    };

    struct UpdateResult {
        POSE integer_pose;
        double actual_move;
        double actual_rotation;
        bool is_stuck;
        double delta_time;
        double estimated_vel;
        double estimated_omega;
        double position_uncertainty;
        double orientation_uncertainty;
        double laser_confidence;      // 保留接口兼容
        int laser_matched_points;     // 保留接口兼容
        bool laser_valid;             // 保留接口兼容
    };

private:
    // 高精度位姿
    HighPrecisionPose high_precision_pose_;

    // 时间管理
    LARGE_INTEGER frequency_;
    LARGE_INTEGER last_update_time_;
    bool timer_initialized_;

    // 命令速度历史（用于平滑）
    double vel_history_[5];
    double omega_history_[5];
    int history_idx_;

    // 状态跟踪
    int stuck_counter_;
    int frames_no_pose_change_;
    double total_distance_traveled_;
    int frame_count_;

    // 不确定度估计
    double pos_uncertainty_;
    double ori_uncertainty_;

    // ========== 关键参数 ==========
    // 速度缩放系数：根据实际测试调整
    // 如果位姿滞后，增加这个值
    // 如果位姿超前，减小这个值
    double vel_scale_;      // 默认 3.5
    double omega_scale_;    // 默认 1.2

    // 速度平滑系数
    double vel_alpha_;      // 默认 0.3（新值权重）

    // 时间常量
    const double MIN_VALID_DT = 0.010;
    const double MAX_VALID_DT = 0.150;
    const double DEFAULT_DT = 0.033;

public:
    PoseTracker()
        : timer_initialized_(false)
        , history_idx_(0)
        , stuck_counter_(0)
        , frames_no_pose_change_(0)
        , total_distance_traveled_(0.0)
        , frame_count_(0)
        , pos_uncertainty_(10.0)
        , ori_uncertainty_(0.1)
        , vel_scale_(3.5)      // 核心参数！
        , omega_scale_(1.2)
        , vel_alpha_(0.3)
    {
        high_precision_pose_.x = 0;
        high_precision_pose_.y = 0;
        high_precision_pose_.ori = 0;

        for (int i = 0; i < 5; i++) {
            vel_history_[i] = 0;
            omega_history_[i] = 0;
        }

        std::cout << "[PoseTracker] Initialized (Command Velocity Based)" << std::endl;
        std::cout << "  vel_scale: " << vel_scale_ << std::endl;
        std::cout << "  omega_scale: " << omega_scale_ << std::endl;
        std::cout << "  vel_alpha: " << vel_alpha_ << std::endl;
    }

    /**
     * @brief 设置速度缩放系数
     * @param vel_scale 线速度缩放（如果位姿滞后就增大）
     * @param omega_scale 角速度缩放
     */
    void setScaleFactors(double vel_scale, double omega_scale) {
        vel_scale_ = vel_scale;
        omega_scale_ = omega_scale;
        std::cout << "[PoseTracker] Scale factors updated: vel=" << vel_scale
            << " omega=" << omega_scale << std::endl;
    }

    void setFusionWeights(double laser_weight, double cmd_weight) {
        // 保留接口兼容，但不使用
        (void)laser_weight;
        (void)cmd_weight;
    }

    void initialize(const POSE& initial_pose) {
        high_precision_pose_.x = initial_pose.coor_x;
        high_precision_pose_.y = initial_pose.coor_y;
        high_precision_pose_.ori = initial_pose.coor_ori;

        if (!timer_initialized_) {
            QueryPerformanceFrequency(&frequency_);
            QueryPerformanceCounter(&last_update_time_);
            timer_initialized_ = true;
        }

        stuck_counter_ = 0;
        frames_no_pose_change_ = 0;
        total_distance_traveled_ = 0;
        frame_count_ = 0;
        pos_uncertainty_ = 10.0;
        ori_uncertainty_ = 0.1;
        history_idx_ = 0;

        for (int i = 0; i < 5; i++) {
            vel_history_[i] = 0;
            omega_history_[i] = 0;
        }

        std::cout << "[PoseTracker] Initialized at ("
            << initial_pose.coor_x << ", " << initial_pose.coor_y
            << "), ori=" << MathUtils::radToDeg(initial_pose.coor_ori)
            << " deg" << std::endl;
    }

    void reset(const POSE& new_pose) {
        high_precision_pose_.x = new_pose.coor_x;
        high_precision_pose_.y = new_pose.coor_y;
        high_precision_pose_.ori = new_pose.coor_ori;

        stuck_counter_ = 0;
        frames_no_pose_change_ = 0;
        total_distance_traveled_ = 0;
        frame_count_ = 0;
        pos_uncertainty_ = 10.0;
        ori_uncertainty_ = 0.1;

        for (int i = 0; i < 5; i++) {
            vel_history_[i] = 0;
            omega_history_[i] = 0;
        }
    }

    /**
     * @brief 核心更新方法 - 基于命令速度积分
     *
     * @param laser_data 激光数据（暂不使用，保留接口）
     * @param cmd_vel 命令线速度 (cm/s)
     * @param cmd_omega 命令角速度 (rad/s)
     * @return 更新结果
     */
    UpdateResult update(INT16* laser_data, double cmd_vel, double cmd_omega) {
        UpdateResult result;
        frame_count_++;

        // 忽略激光数据，避免编译警告
        (void)laser_data;

        // 获取时间步长
        double dt = getActualDeltaTime();
        result.delta_time = dt;

        double old_x = high_precision_pose_.x;
        double old_y = high_precision_pose_.y;
        double old_theta = high_precision_pose_.ori;

        // ========== 速度平滑 ==========
        vel_history_[history_idx_] = cmd_vel;
        omega_history_[history_idx_] = cmd_omega;
        history_idx_ = (history_idx_ + 1) % 5;

        // 加权平均（最近的权重更高）
        double smoothed_vel = 0;
        double smoothed_omega = 0;
        double weights[5] = { 0.35, 0.25, 0.20, 0.12, 0.08 };
        for (int i = 0; i < 5; i++) {
            int idx = (history_idx_ - 1 - i + 5) % 5;
            smoothed_vel += vel_history_[idx] * weights[i];
            smoothed_omega += omega_history_[idx] * weights[i];
        }

        // ========== 缩放速度 ==========
        double scaled_v = smoothed_vel * vel_scale_;
        double scaled_omega = smoothed_omega * omega_scale_;

        // ========== 中点法积分 ==========
        // 先计算中点角度，提高精度
        double theta_mid = high_precision_pose_.ori + 0.5 * scaled_omega * dt;

        double dx = scaled_v * cos(theta_mid) * dt;
        double dy = scaled_v * sin(theta_mid) * dt;
        double dtheta = scaled_omega * dt;

        // ========== 更新位姿 ==========
        high_precision_pose_.x += dx;
        high_precision_pose_.y += dy;
        high_precision_pose_.ori = MathUtils::normalizeAngle(high_precision_pose_.ori + dtheta);

        // 边界检查
        high_precision_pose_.x = MathUtils::clamp(high_precision_pose_.x, 0.0, static_cast<double>(MAP_WIDTH));
        high_precision_pose_.y = MathUtils::clamp(high_precision_pose_.y, 0.0, static_cast<double>(MAP_HEIGHT));

        // ========== 不确定度更新 ==========
        // 速度越大，不确定度增长越快
        pos_uncertainty_ += fabs(scaled_v) * dt * 0.05;
        ori_uncertainty_ += fabs(scaled_omega) * dt * 0.1;

        // 限制不确定度
        pos_uncertainty_ = MathUtils::clamp(pos_uncertainty_, 1.0, 100.0);
        ori_uncertainty_ = MathUtils::clamp(ori_uncertainty_, 0.01, 1.0);

        // ========== 填充结果 ==========
        result.integer_pose.coor_x = static_cast<INT16>(high_precision_pose_.x + 0.5);
        result.integer_pose.coor_y = static_cast<INT16>(high_precision_pose_.y + 0.5);
        result.integer_pose.coor_ori = high_precision_pose_.ori;

        result.actual_move = MathUtils::calculateDistance(old_x, old_y,
            high_precision_pose_.x,
            high_precision_pose_.y);
        result.actual_rotation = fabs(MathUtils::angleDifference(high_precision_pose_.ori, old_theta));

        result.estimated_vel = result.actual_move / dt;
        result.estimated_omega = result.actual_rotation / dt;

        result.position_uncertainty = pos_uncertainty_;
        result.orientation_uncertainty = ori_uncertainty_;

        // ICP 相关字段（保持接口兼容）
        result.laser_valid = false;
        result.laser_confidence = 0;
        result.laser_matched_points = 0;

        // 更新统计
        total_distance_traveled_ += result.actual_move;

        // 卡住检测
        updateStuckDetection(result, cmd_vel, cmd_omega);

        return result;
    }

    void printStatus(const UpdateResult& result) const {
        std::cout << "[POSE_] x=" << result.integer_pose.coor_x
            << " y=" << result.integer_pose.coor_y
            << " ori=" << static_cast<int>(MathUtils::radToDeg(result.integer_pose.coor_ori))
            << " deg" << std::endl;

        std::cout << "         HP: x=" << std::fixed << std::setprecision(1)
            << high_precision_pose_.x << " y=" << high_precision_pose_.y
            << " | dt=" << std::setprecision(1) << (result.delta_time * 1000) << " ms" << std::endl;

        std::cout << "         Move: " << std::setprecision(2) << result.actual_move << " cm"
            << " rot=" << static_cast<int>(MathUtils::radToDeg(result.actual_rotation)) << " deg"
            << " | v_est=" << std::setprecision(1) << result.estimated_vel
            << " w_est=" << result.estimated_omega << std::endl;

        std::cout << "         Uncertainty: pos=" << std::setprecision(1) << pos_uncertainty_
            << " cm ori=" << MathUtils::radToDeg(ori_uncertainty_) << " deg"
            << " | stuck=" << stuck_counter_
            << " | scale=" << vel_scale_ << std::endl;
    }

    // ========== Getter 方法 ==========
    HighPrecisionPose getHighPrecisionPose() const { return high_precision_pose_; }
    POSE getPose() const {
        POSE p;
        p.coor_x = static_cast<INT16>(high_precision_pose_.x + 0.5);
        p.coor_y = static_cast<INT16>(high_precision_pose_.y + 0.5);
        p.coor_ori = high_precision_pose_.ori;
        return p;
    }

    double getTotalDistanceTraveled() const { return total_distance_traveled_; }
    int getStuckCounter() const { return stuck_counter_; }
    double getPositionUncertainty() const { return pos_uncertainty_; }
    double getOrientationUncertainty() const { return ori_uncertainty_; }

    bool shouldPerformSLAMCorrection() const {
        return false;
    }

private:
    void updateStuckDetection(UpdateResult& result, double cmd_vel, double cmd_omega) {
        double expected_move = fabs(cmd_vel * vel_scale_) * result.delta_time * 0.5;
        double move_threshold = (std::max)(0.2, expected_move * 0.2);

        bool pose_changed = (result.actual_move > move_threshold || result.actual_rotation > 0.01);
        bool command_active = (fabs(cmd_vel) > 2.0 || fabs(cmd_omega) > 0.03);

        if (!pose_changed && command_active) {
            frames_no_pose_change_++;
            stuck_counter_++;
        }
        else if (pose_changed) {
            frames_no_pose_change_ = 0;
            stuck_counter_ = (std::max)(0, stuck_counter_ - 2);
        }

        result.is_stuck = (stuck_counter_ > 20);
    }

    double getActualDeltaTime() {
        if (!timer_initialized_) {
            QueryPerformanceFrequency(&frequency_);
            QueryPerformanceCounter(&last_update_time_);
            timer_initialized_ = true;
            return DEFAULT_DT;
        }

        LARGE_INTEGER current_time;
        QueryPerformanceCounter(&current_time);

        double dt = static_cast<double>(current_time.QuadPart - last_update_time_.QuadPart)
            / static_cast<double>(frequency_.QuadPart);

        last_update_time_ = current_time;

        if (dt < MIN_VALID_DT) dt = MIN_VALID_DT;
        if (dt > MAX_VALID_DT) dt = MAX_VALID_DT;

        return dt;
    }
};

#endif // POSE_TRACKER_H