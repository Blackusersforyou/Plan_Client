#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "StructData.h"
#include <cmath>
#include <iostream>

// ============================================================================
// 坐标系定义说明（统一版本）：
// ============================================================================
// 世界坐标系: 
//   - 原点在地图左上角 (0, 0)
//   - X轴向右为正, Y轴向下为正
//   - 范围 [0, MAP_WIDTH] x [0, MAP_HEIGHT]，即 [0, 1000] x [0, 1000] cm
//   - 地图中心点为 (500, 500)
//
// 栅格坐标系:
//   - 原点在地图左上角 (0, 0)  
//   - X轴向右为正, Y轴向下为正
//   - 范围 [0, GRID_WIDTH-1] x [0, GRID_HEIGHT-1]，即 [0, 199] x [0, 199]
//   - 每个栅格代表 MAP_RESOLUTION x MAP_RESOLUTION cm (5x5 cm)
//
// 机器人朝向:
//   - 0 弧度指向 +X 方向（右）
//   - 逆时针为正角度
//   - 范围 [-PI, PI]
// ============================================================================

class MathUtils {
public:
    // ========================================================================
    // 角度处理函数
    // ========================================================================

    /**
     * @brief 将角度归一化到 [-PI, PI] 范围
     * @param angle 输入角度（弧度）
     * @return 归一化后的角度
     */
    static double normalizeAngle(double angle) {
        while (angle > PI) angle -= 2.0 * PI;
        while (angle < -PI) angle += 2.0 * PI;
        return angle;
    }

    /**
     * @brief 计算两个角度之间的差值（归一化到 [-PI, PI]）
     * @param target_angle 目标角度
     * @param current_angle 当前角度
     * @return 角度差 (target - current)，正值表示需要逆时针旋转
     */
    static double angleDifference(double target_angle, double current_angle) {
        double diff = target_angle - current_angle;
        return normalizeAngle(diff);
    }

    /**
     * @brief 角度转换：度 -> 弧度
     */
    static double degToRad(double degrees) {
        return degrees * PI / 180.0;
    }

    /**
     * @brief 角度转换：弧度 -> 度
     */
    static double radToDeg(double radians) {
        return radians * 180.0 / PI;
    }

    // ========================================================================
    // 距离和方向计算
    // ========================================================================

    /**
     * @brief 计算两点之间的欧氏距离
     * @return 距离（与输入坐标单位相同，通常为 cm）
     */
    static double calculateDistance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return sqrt(dx * dx + dy * dy);
    }

    /**
     * @brief 计算从点1指向点2的方向角
     * @return 方向角（弧度），范围 [-PI, PI]
     */
    static double calculateBearing(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return atan2(dy, dx);
    }

    // ========================================================================
    // 坐标转换函数（核心修复）
    // ========================================================================

    /**
     * @brief 世界坐标转栅格坐标
     *
     * 转换公式: grid = floor(world / resolution)
     *
     * @param wx 世界坐标 X (cm)，范围 [0, MAP_WIDTH]
     * @param wy 世界坐标 Y (cm)，范围 [0, MAP_HEIGHT]
     * @param gx 输出栅格坐标 X，范围 [0, GRID_WIDTH-1]
     * @param gy 输出栅格坐标 Y，范围 [0, GRID_HEIGHT-1]
     */
    static void worldToGrid(double wx, double wy, INT16& gx, INT16& gy) {
        // 直接除以分辨率并取整
        gx = (INT16)(wx / MAP_RESOLUTION);
        gy = (INT16)(wy / MAP_RESOLUTION);

        // 边界保护
        gx = clamp(gx, (INT16)0, (INT16)(GRID_WIDTH - 1));
        gy = clamp(gy, (INT16)0, (INT16)(GRID_HEIGHT - 1));
    }

    /**
     * @brief 栅格坐标转世界坐标（返回栅格中心点）
     *
     * 转换公式: world = grid * resolution + resolution/2
     *
     * @param gx 栅格坐标 X
     * @param gy 栅格坐标 Y
     * @param wx 输出世界坐标 X (cm)
     * @param wy 输出世界坐标 Y (cm)
     */
    static void gridToWorld(INT16 gx, INT16 gy, double& wx, double& wy) {
        // 返回栅格中心点的世界坐标
        wx = gx * MAP_RESOLUTION + MAP_RESOLUTION / 2.0;
        wy = gy * MAP_RESOLUTION + MAP_RESOLUTION / 2.0;
    }

    /**
     * @brief 栅格坐标转世界坐标（返回栅格左上角）
     */
    static void gridToWorldCorner(INT16 gx, INT16 gy, double& wx, double& wy) {
        wx = gx * MAP_RESOLUTION;
        wy = gy * MAP_RESOLUTION;
    }

    // ========================================================================
    // 位姿相关工具
    // ========================================================================

    /**
     * @brief 检查两个位姿是否显著不同
     * @param distance_threshold 位置差阈值 (cm)
     * @param angle_threshold 角度差阈值 (rad)
     * @return true 如果位姿差异超过阈值
     */
    static bool isPoseSignificantlyDifferent(const POSE& pose1, const POSE& pose2,
        double distance_threshold = 50.0,
        double angle_threshold = 0.5) {
        double dist = calculateDistance(pose1.coor_x, pose1.coor_y,
            pose2.coor_x, pose2.coor_y);
        double angle_diff = fabs(angleDifference(pose1.coor_ori, pose2.coor_ori));

        return (dist > distance_threshold || angle_diff > angle_threshold);
    }

    /**
     * @brief 根据当前位姿和运动命令预测下一位姿
     * @param current 当前位姿
     * @param linear_vel 线速度 (cm/s)
     * @param angular_vel 角速度 (rad/s)
     * @param dt 时间步长 (s)
     * @return 预测的下一位姿
     */
    static POSE predictPose(const POSE& current, double linear_vel,
        double angular_vel, double dt) {
        POSE predicted;

        // 使用中点法进行更精确的预测
        double theta_mid = current.coor_ori + 0.5 * angular_vel * dt;

        predicted.coor_x = (INT16)(current.coor_x + linear_vel * cos(theta_mid) * dt);
        predicted.coor_y = (INT16)(current.coor_y + linear_vel * sin(theta_mid) * dt);
        predicted.coor_ori = normalizeAngle(current.coor_ori + angular_vel * dt);

        return predicted;
    }

    // ========================================================================
    // 辅助函数
    // ========================================================================

    /**
     * @brief 限制值在指定范围内
     */
    template<typename T>
    static T clamp(T value, T min_val, T max_val) {
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }

    /**
     * @brief 线性插值
     */
    static double lerp(double a, double b, double t) {
        return a + t * (b - a);
    }

    /**
     * @brief 检查点是否在地图范围内（世界坐标）
     */
    static bool isInMapBounds(double wx, double wy) {
        return wx >= 0 && wx < MAP_WIDTH && wy >= 0 && wy < MAP_HEIGHT;
    }

    /**
     * @brief 检查点是否在栅格范围内
     */
    static bool isInGridBounds(INT16 gx, INT16 gy) {
        return gx >= 0 && gx < GRID_WIDTH && gy >= 0 && gy < GRID_HEIGHT;
    }

    // ========================================================================
    // 调试工具
    // ========================================================================

    /**
     * @brief 验证坐标转换的往返误差
     */
    static void debugCoordinateConversion(double wx, double wy, const char* label = "") {
        INT16 gx, gy;
        worldToGrid(wx, wy, gx, gy);

        double wx2, wy2;
        gridToWorld(gx, gy, wx2, wy2);

        double error_x = fabs(wx - wx2);
        double error_y = fabs(wy - wy2);

        std::cout << "[COORD_DEBUG] " << label
            << " World(" << wx << "," << wy << ")"
            << " -> Grid(" << gx << "," << gy << ")"
            << " -> World(" << wx2 << "," << wy2 << ")"
            << " Error=(" << error_x << "," << error_y << ")" << std::endl;

        // 误差应该不超过半个栅格
        if (error_x > MAP_RESOLUTION || error_y > MAP_RESOLUTION) {
            std::cout << "[COORD_WARN] Large conversion error detected!" << std::endl;
        }
    }

    /**
     * @brief 打印位姿信息
     */
    static void printPose(const POSE& pose, const char* label = "") {
        std::cout << "[POSE] " << label
            << " x=" << pose.coor_x
            << " y=" << pose.coor_y
            << " ori=" << radToDeg(pose.coor_ori) << " deg" << std::endl;
    }
};

#endif // MATH_UTILS_H