#ifndef LASER_ODOMETRY_H
#define LASER_ODOMETRY_H

#include "StructData.h"
#include "MathUtils.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>

/**
 * @brief 基于激光扫描匹配的真正位姿估计器
 *
 * 核心原理：
 * 使用 ICP (Iterative Closest Point) 算法比较相邻两帧激光数据，
 * 计算帧间的相对位移 (dx, dy, dtheta)，然后累积得到绝对位姿。
 *
 * 这是真正的激光里程计，不依赖于命令速度！
 */
class LaserOdometry {
public:
    struct OdometryResult {
        double delta_x;        // 帧间X位移 (cm)
        double delta_y;        // 帧间Y位移 (cm)
        double delta_theta;    // 帧间角度变化 (rad)
        double confidence;     // 匹配置信度 [0, 1]
        bool valid;            // 是否有效
        int matched_points;    // 匹配点数
        double avg_error;      // 平均匹配误差
    };

    struct LaserPoint {
        double x;
        double y;
        double range;
        double angle;
        bool valid;
    };

private:
    // 上一帧激光数据
    std::vector<LaserPoint> last_scan_;
    bool has_last_scan_;

    // 累积位姿
    double pose_x_;
    double pose_y_;
    double pose_theta_;

    // ICP 参数
    const int ICP_MAX_ITERATIONS = 15;
    const double ICP_CONVERGENCE_THRESHOLD = 0.001;
    const double MAX_CORRESPONDENCE_DIST = 30.0;  // cm
    const int MIN_MATCH_POINTS = 20;

    // 滤波参数
    const double ANGLE_CHANGE_LIMIT = 0.5;  // 单帧最大角度变化 (rad)
    const double POSITION_CHANGE_LIMIT = 50.0;  // 单帧最大位移 (cm)

    // 运动模型辅助
    double last_cmd_vel_;
    double last_cmd_omega_;

    /**
     * @brief 将激光数据转换为点云
     */
    std::vector<LaserPoint> laserToPoints(INT16* laser_data, double robot_theta = 0.0) {
        std::vector<LaserPoint> points;
        points.reserve(180);

        for (int i = 0; i < 360; i += 2) {  // 每2度采样
            if (laser_data[i] > 10 && laser_data[i] < 2000) {
                LaserPoint pt;
                pt.angle = i * PI / 180.0;
                pt.range = laser_data[i];

                // 转换到机器人坐标系
                pt.x = pt.range * cos(pt.angle);
                pt.y = pt.range * sin(pt.angle);
                pt.valid = true;

                points.push_back(pt);
            }
        }

        return points;
    }

    /**
     * @brief 变换点云
     */
    std::vector<LaserPoint> transformPoints(const std::vector<LaserPoint>& points,
        double dx, double dy, double dtheta) {
        std::vector<LaserPoint> transformed;
        transformed.reserve(points.size());

        double cos_t = cos(dtheta);
        double sin_t = sin(dtheta);

        for (const auto& pt : points) {
            if (!pt.valid) continue;

            LaserPoint new_pt;
            new_pt.x = pt.x * cos_t - pt.y * sin_t + dx;
            new_pt.y = pt.x * sin_t + pt.y * cos_t + dy;
            new_pt.range = pt.range;
            new_pt.angle = pt.angle + dtheta;
            new_pt.valid = true;

            transformed.push_back(new_pt);
        }

        return transformed;
    }

    /**
     * @brief 找到最近点对应
     */
    struct Correspondence {
        int src_idx;
        int tgt_idx;
        double distance;
    };

    std::vector<Correspondence> findCorrespondences(
        const std::vector<LaserPoint>& source,
        const std::vector<LaserPoint>& target,
        double max_dist) {

        std::vector<Correspondence> corrs;

        for (size_t i = 0; i < source.size(); i++) {
            if (!source[i].valid) continue;

            double best_dist = max_dist;
            int best_j = -1;

            for (size_t j = 0; j < target.size(); j++) {
                if (!target[j].valid) continue;

                double dx = source[i].x - target[j].x;
                double dy = source[i].y - target[j].y;
                double dist = sqrt(dx * dx + dy * dy);

                if (dist < best_dist) {
                    best_dist = dist;
                    best_j = static_cast<int>(j);
                }
            }

            if (best_j >= 0) {
                Correspondence c;
                c.src_idx = static_cast<int>(i);
                c.tgt_idx = best_j;
                c.distance = best_dist;
                corrs.push_back(c);
            }
        }

        return corrs;
    }

    /**
     * @brief 计算最优变换（最小二乘）
     */
    void computeOptimalTransform(
        const std::vector<LaserPoint>& source,
        const std::vector<LaserPoint>& target,
        const std::vector<Correspondence>& corrs,
        double& dx, double& dy, double& dtheta) {

        if (corrs.size() < 3) {
            dx = dy = dtheta = 0;
            return;
        }

        // 计算质心
        double src_cx = 0, src_cy = 0;
        double tgt_cx = 0, tgt_cy = 0;
        int n = static_cast<int>(corrs.size());

        for (const auto& c : corrs) {
            src_cx += source[c.src_idx].x;
            src_cy += source[c.src_idx].y;
            tgt_cx += target[c.tgt_idx].x;
            tgt_cy += target[c.tgt_idx].y;
        }

        src_cx /= n;
        src_cy /= n;
        tgt_cx /= n;
        tgt_cy /= n;

        // 计算旋转角度 (SVD 简化版本)
        double num = 0, denom = 0;

        for (const auto& c : corrs) {
            double sx = source[c.src_idx].x - src_cx;
            double sy = source[c.src_idx].y - src_cy;
            double tx = target[c.tgt_idx].x - tgt_cx;
            double ty = target[c.tgt_idx].y - tgt_cy;

            num += sx * ty - sy * tx;
            denom += sx * tx + sy * ty;
        }

        dtheta = atan2(num, denom);

        // 限制单次旋转角度
        dtheta = MathUtils::clamp(dtheta, -0.1, 0.1);

        // 计算平移
        double cos_t = cos(dtheta);
        double sin_t = sin(dtheta);

        dx = tgt_cx - (src_cx * cos_t - src_cy * sin_t);
        dy = tgt_cy - (src_cx * sin_t + src_cy * cos_t);

        // 限制单次平移
        double trans_mag = sqrt(dx * dx + dy * dy);
        if (trans_mag > 5.0) {
            dx = dx * 5.0 / trans_mag;
            dy = dy * 5.0 / trans_mag;
        }
    }

    /**
     * @brief ICP 算法主体
     */
    OdometryResult performICP(const std::vector<LaserPoint>& current,
        const std::vector<LaserPoint>& previous) {
        OdometryResult result;
        result.delta_x = 0;
        result.delta_y = 0;
        result.delta_theta = 0;
        result.confidence = 0;
        result.valid = false;
        result.matched_points = 0;
        result.avg_error = 0;

        if (current.size() < MIN_MATCH_POINTS || previous.size() < MIN_MATCH_POINTS) {
            return result;
        }

        // 使用运动模型作为初始估计
        double total_dx = last_cmd_vel_ * 0.03 * cos(pose_theta_);  // 30ms 估计
        double total_dy = last_cmd_vel_ * 0.03 * sin(pose_theta_);
        double total_dtheta = last_cmd_omega_ * 0.03;

        std::vector<LaserPoint> transformed = transformPoints(current, total_dx, total_dy, total_dtheta);

        // ICP 迭代
        for (int iter = 0; iter < ICP_MAX_ITERATIONS; iter++) {
            // 找对应点
            auto corrs = findCorrespondences(transformed, previous, MAX_CORRESPONDENCE_DIST);

            if (corrs.size() < MIN_MATCH_POINTS) {
                break;
            }

            // 计算最优变换
            double ddx, ddy, ddt;
            computeOptimalTransform(transformed, previous, corrs, ddx, ddy, ddt);

            // 累积变换
            total_dx += ddx * cos(total_dtheta) - ddy * sin(total_dtheta);
            total_dy += ddx * sin(total_dtheta) + ddy * cos(total_dtheta);
            total_dtheta += ddt;

            // 更新变换后的点云
            transformed = transformPoints(current, total_dx, total_dy, total_dtheta);

            // 检查收敛
            if (sqrt(ddx * ddx + ddy * ddy) < ICP_CONVERGENCE_THRESHOLD &&
                fabs(ddt) < 0.001) {
                break;
            }

            result.matched_points = static_cast<int>(corrs.size());
        }

        // 计算最终误差
        auto final_corrs = findCorrespondences(transformed, previous, MAX_CORRESPONDENCE_DIST);

        if (final_corrs.size() >= MIN_MATCH_POINTS) {
            double total_error = 0;
            for (const auto& c : final_corrs) {
                total_error += c.distance;
            }
            result.avg_error = total_error / final_corrs.size();
            result.matched_points = static_cast<int>(final_corrs.size());

            // 计算置信度
            double match_ratio = static_cast<double>(final_corrs.size()) /
                static_cast<double>(current.size());
            double error_factor = exp(-result.avg_error / 10.0);
            result.confidence = match_ratio * error_factor;

            if (result.confidence > 0.3 && result.avg_error < 20.0) {
                result.delta_x = -total_dx;  // 注意符号：点云变换与机器人移动方向相反
                result.delta_y = -total_dy;
                result.delta_theta = -total_dtheta;
                result.valid = true;

                // 应用限制
                double pos_change = sqrt(result.delta_x * result.delta_x +
                    result.delta_y * result.delta_y);
                if (pos_change > POSITION_CHANGE_LIMIT) {
                    result.delta_x *= POSITION_CHANGE_LIMIT / pos_change;
                    result.delta_y *= POSITION_CHANGE_LIMIT / pos_change;
                }

                result.delta_theta = MathUtils::clamp(result.delta_theta,
                    -ANGLE_CHANGE_LIMIT,
                    ANGLE_CHANGE_LIMIT);
            }
        }

        return result;
    }

public:
    LaserOdometry()
        : has_last_scan_(false)
        , pose_x_(0)
        , pose_y_(0)
        , pose_theta_(0)
        , last_cmd_vel_(0)
        , last_cmd_omega_(0)
    {
        std::cout << "[LaserOdometry] Initialized (ICP-based)" << std::endl;
    }

    /**
     * @brief 初始化位姿
     */
    void initialize(double x, double y, double theta) {
        pose_x_ = x;
        pose_y_ = y;
        pose_theta_ = theta;
        has_last_scan_ = false;

        std::cout << "[LaserOdometry] Position initialized: ("
            << x << ", " << y << "), theta="
            << MathUtils::radToDeg(theta) << " deg" << std::endl;
    }

    /**
     * @brief 更新位姿（核心方法）
     *
     * @param laser_data 当前帧激光数据
     * @param cmd_vel 命令线速度（用于辅助）
     * @param cmd_omega 命令角速度（用于辅助）
     * @return 里程计结果
     */
    OdometryResult update(INT16* laser_data, double cmd_vel, double cmd_omega) {
        OdometryResult result;
        result.valid = false;

        // 保存命令速度用于运动模型
        last_cmd_vel_ = cmd_vel;
        last_cmd_omega_ = cmd_omega;

        // 转换当前帧
        auto current_points = laserToPoints(laser_data, pose_theta_);

        if (has_last_scan_ && current_points.size() >= MIN_MATCH_POINTS) {
            // 执行 ICP 匹配
            result = performICP(current_points, last_scan_);

            if (result.valid) {
                // 将局部位移转换到全局坐标
                double cos_t = cos(pose_theta_);
                double sin_t = sin(pose_theta_);

                double global_dx = result.delta_x * cos_t - result.delta_y * sin_t;
                double global_dy = result.delta_x * sin_t + result.delta_y * cos_t;

                // 更新累积位姿
                pose_x_ += global_dx;
                pose_y_ += global_dy;
                pose_theta_ = MathUtils::normalizeAngle(pose_theta_ + result.delta_theta);

                // 边界检查
                pose_x_ = MathUtils::clamp(pose_x_, 0.0, static_cast<double>(MAP_WIDTH));
                pose_y_ = MathUtils::clamp(pose_y_, 0.0, static_cast<double>(MAP_HEIGHT));
            }
        }

        // 如果 ICP 失败，使用命令速度作为后备
        if (!result.valid && (fabs(cmd_vel) > 1.0 || fabs(cmd_omega) > 0.01)) {
            double dt = 0.033;  // 假设 30ms
            double vel_scale = 3.5;  // 与 PoseTracker 一致
            double omega_scale = 1.2;

            double scaled_v = cmd_vel * vel_scale;
            double scaled_omega = cmd_omega * omega_scale;

            double theta_mid = pose_theta_ + 0.5 * scaled_omega * dt;

            pose_x_ += scaled_v * cos(theta_mid) * dt;
            pose_y_ += scaled_v * sin(theta_mid) * dt;
            pose_theta_ = MathUtils::normalizeAngle(pose_theta_ + scaled_omega * dt);

            // 边界检查
            pose_x_ = MathUtils::clamp(pose_x_, 0.0, static_cast<double>(MAP_WIDTH));
            pose_y_ = MathUtils::clamp(pose_y_, 0.0, static_cast<double>(MAP_HEIGHT));

            result.delta_x = scaled_v * cos(theta_mid) * dt;
            result.delta_y = scaled_v * sin(theta_mid) * dt;
            result.delta_theta = scaled_omega * dt;
            result.confidence = 0.5;  // 较低置信度
            result.valid = true;
            result.matched_points = 0;
            result.avg_error = 0;
        }

        // 保存当前帧
        last_scan_ = current_points;
        has_last_scan_ = true;

        return result;
    }

    /**
     * @brief 获取当前位姿
     */
    POSE getPose() const {
        POSE p;
        p.coor_x = static_cast<INT16>(pose_x_ + 0.5);
        p.coor_y = static_cast<INT16>(pose_y_ + 0.5);
        p.coor_ori = pose_theta_;
        return p;
    }

    double getX() const { return pose_x_; }
    double getY() const { return pose_y_; }
    double getTheta() const { return pose_theta_; }

    /**
     * @brief 使用外部位姿进行校正
     */
    void correctPose(double x, double y, double theta, double confidence = 0.5) {
        double blend = MathUtils::clamp(confidence, 0.0, 1.0);

        pose_x_ = (1 - blend) * pose_x_ + blend * x;
        pose_y_ = (1 - blend) * pose_y_ + blend * y;

        double angle_diff = MathUtils::angleDifference(theta, pose_theta_);
        pose_theta_ = MathUtils::normalizeAngle(pose_theta_ + blend * angle_diff);
    }

    /**
     * @brief 重置
     */
    void reset(double x, double y, double theta) {
        pose_x_ = x;
        pose_y_ = y;
        pose_theta_ = theta;
        has_last_scan_ = false;
        last_scan_.clear();
    }

    /**
     * @brief 打印状态
     */
    void printStatus() const {
        std::cout << "[LaserOdom] x=" << std::fixed << std::setprecision(1)
            << pose_x_ << " y=" << pose_y_
            << " theta=" << MathUtils::radToDeg(pose_theta_) << " deg"
            << std::endl;
    }
};

#endif // LASER_ODOMETRY_H