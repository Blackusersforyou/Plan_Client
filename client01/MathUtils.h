#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "StructData.h"
#include <cmath>

// 集成的数学工具函数类
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
    
    // 世界坐标转栅格坐标
    static void worldToGrid(double wx, double wy, INT16& gx, INT16& gy) {
        gx = (INT16)((wx + MAP_WIDTH / 2.0) / MAP_RESOLUTION);
        gy = (INT16)((wy + MAP_HEIGHT / 2.0) / MAP_RESOLUTION);
        gx = (std::max)((INT16)0, (std::min)((INT16)(GRID_WIDTH - 1), gx));
        gy = (std::max)((INT16)0, (std::min)((INT16)(GRID_HEIGHT - 1), gy));
    }
    
    // 栅格坐标转世界坐标
    static void gridToWorld(INT16 gx, INT16 gy, double& wx, double& wy) {
        wx = gx * MAP_RESOLUTION - MAP_WIDTH / 2.0 + MAP_RESOLUTION / 2.0;
        wy = gy * MAP_RESOLUTION - MAP_HEIGHT / 2.0 + MAP_RESOLUTION / 2.0;
    }
};

#endif