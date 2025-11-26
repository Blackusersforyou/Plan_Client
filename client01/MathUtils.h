#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "StructData.h"
#include <cmath>
#include <iostream>

// ✅ 坐标系定义说明：
// 世界坐标系: 中心(500,500)为地图中心, X向右, Y向下, 范围[0,1000]
// 栅格坐标系: 左上角(0,0), X向右, Y向下, 范围[0,199]
// 机器人朝向: 0度向右(+X方向), 逆时针为正

class MathUtils {
public:
    // 角度差归一化到 [-PI, PI]
    static double normalizeAngle(double angle) {
        while (angle > PI) angle -= 2 * PI;
        while (angle < -PI) angle += 2 * PI;
        return angle;
    }
    
    // 计算两点之间的距离
    static double calculateDistance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return sqrt(dx * dx + dy * dy);
    }
    
    // 计算从点1到点2的方向角
    static double calculateBearing(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return atan2(dy, dx);
    }
    
    // 计算两个角度之间的差值(归一化)
    static double angleDifference(double angle1, double angle2) {
        double diff = angle1 - angle2;
        return normalizeAngle(diff);
    }
    
    // ✅ 核心修复: 世界坐标转栅格坐标
    // 世界坐标: 范围[0, 1000], 中心点(500, 500)
    // 栅格坐标: 范围[0, 199], 中心点(100, 100), 分辨率5cm
    static void worldToGrid(double wx, double wy, INT16& gx, INT16& gy) {
        // 世界坐标原点在左上角(0,0), 单位cm
        // 栅格坐标原点也在左上角(0,0), 每格5cm
        
        gx = (INT16)(wx / MAP_RESOLUTION);
        gy = (INT16)(wy / MAP_RESOLUTION);
        
        // 边界限制
        gx = (std::max)((INT16)0, (std::min)((INT16)(GRID_WIDTH - 1), gx));
        gy = (std::max)((INT16)0, (std::min)((INT16)(GRID_HEIGHT - 1), gy));
    }
    
    // ✅ 核心修复: 栅格坐标转世界坐标
    static void gridToWorld(INT16 gx, INT16 gy, double& wx, double& wy) {
        // 栅格中心点对应的世界坐标
        wx = gx * MAP_RESOLUTION + MAP_RESOLUTION / 2.0;
        wy = gy * MAP_RESOLUTION + MAP_RESOLUTION / 2.0;
    }
    
    // 检查两个位姿是否显著不同
    static bool isPoseSignificantlyDifferent(const POSE& pose1, const POSE& pose2, 
                                              double distance_threshold = 50.0,
                                              double angle_threshold = 0.5) {
        double dist = calculateDistance(pose1.coor_x, pose1.coor_y, 
                                       pose2.coor_x, pose2.coor_y);
        double angle_diff = fabs(normalizeAngle(pose1.coor_ori - pose2.coor_ori));
        
        return (dist > distance_threshold || angle_diff > angle_threshold);
    }
    
    // ✅ 验证坐标转换 - 仅用于调试
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
    }
};

#endif