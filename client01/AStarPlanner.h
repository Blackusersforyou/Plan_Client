#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include "StructData.h"
#include "LaserSLAM.h"
#include "MathUtils.h"
#include "ObstacleAvoidance.h"
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <map>
#include <iostream>

/**
 * @brief A*路径规划器
 *
 * 修复内容：
 * 1. 使用统一的MathUtils坐标转换（已在原代码中使用）
 * 2. 改进路径跟踪逻辑
 * 3. 增加调试信息
 */
class AStarPlanner {
private:
    LaserSLAM* slam;
    ObstacleAvoidance* obstacle_avoidance;

    INT16 current_goal_x;
    INT16 current_goal_y;
    INT16 current_start_x;
    INT16 current_start_y;

    struct CompareNode {
        bool operator()(const AStarNode& a, const AStarNode& b) const {
            return a.f > b.f;
        }
    };

    double heuristic(INT16 x1, INT16 y1, INT16 x2, INT16 y2) {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return sqrt(dx * dx + dy * dy);
    }

    bool isWalkable(INT16 x, INT16 y, OccupancyMap& map) {
        if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT) return false;

        // 起点和目标点始终允许通行
        if ((x == current_goal_x && y == current_goal_y) ||
            (x == current_start_x && y == current_start_y)) {
            return true;
        }

        // ✅ 修复：起点周围 7×7 区域放宽阈值（原来是5×5）
        int dx_start = abs(x - current_start_x);
        int dy_start = abs(y - current_start_y);
        if (dx_start <= 3 && dy_start <= 3) {
            return map.grid[y][x].occupancy < 98;  // 几乎只排除确定的墙壁
        }

        // ✅ 修复：目标周围 7×7 区域放宽阈值
        int dx_goal = abs(x - current_goal_x);
        int dy_goal = abs(y - current_goal_y);
        if (dx_goal <= 3 && dy_goal <= 3) {
            return map.grid[y][x].occupancy < 98;
        }

        // ✅ 修复：其他位置使用更宽松的阈值（原来是70）
        return map.grid[y][x].occupancy < 80;
    }

    int getKey(INT16 x, INT16 y) {
        return y * GRID_WIDTH + x;
    }

public:
    AStarPlanner(LaserSLAM* slam_ptr, ObstacleAvoidance* obstacle_ptr)
        : slam(slam_ptr), obstacle_avoidance(obstacle_ptr) {
        current_goal_x = -1;
        current_goal_y = -1;
        current_start_x = -1;
        current_start_y = -1;
        std::cout << "AStarPlanner initialized (Fixed Version)" << std::endl;
    }

    bool planPath(POSE start_pose, Point goal, Point* path, int& path_length) {
        if (slam == nullptr) {
            path_length = 0;
            return false;
        }

        OccupancyMap& map = slam->getMap();

        // ✅ 使用统一的MathUtils进行坐标转换
        INT16 start_gx, start_gy, goal_gx, goal_gy;
        MathUtils::worldToGrid(start_pose.coor_x, start_pose.coor_y, start_gx, start_gy);
        MathUtils::worldToGrid(goal.coor_x, goal.coor_y, goal_gx, goal_gy);

        current_start_x = start_gx;
        current_start_y = start_gy;
        current_goal_x = goal_gx;
        current_goal_y = goal_gy;

        std::cout << "A* Planning:" << std::endl;
        std::cout << "  World Start: (" << start_pose.coor_x << "," << start_pose.coor_y
            << ") -> Grid: (" << start_gx << "," << start_gy << ")" << std::endl;
        std::cout << "  World Goal:  (" << goal.coor_x << "," << goal.coor_y
            << ") -> Grid: (" << goal_gx << "," << goal_gy << ")" << std::endl;

        // ✅ 验证坐标转换
        double verify_wx, verify_wy;
        MathUtils::gridToWorld(start_gx, start_gy, verify_wx, verify_wy);
        double start_error = MathUtils::calculateDistance(
            start_pose.coor_x, start_pose.coor_y, verify_wx, verify_wy);
        if (start_error > MAP_RESOLUTION) {
            std::cout << "  [WARN] Start coord conversion error: " << start_error << " cm" << std::endl;
        }

        std::cout << "  Start occupancy: " << map.grid[start_gy][start_gx].occupancy
            << ", Goal occupancy: " << map.grid[goal_gy][goal_gx].occupancy << std::endl;

        // Bresenham清除视线上的临时障碍
        std::vector<std::pair<INT16, INT16>> cleared_cells;
        std::vector<INT16> original_occupancy;

        int dx = abs(goal_gx - start_gx);
        int dy = abs(goal_gy - start_gy);
        int sx = (start_gx < goal_gx) ? 1 : -1;
        int sy = (start_gy < goal_gy) ? 1 : -1;
        int err = dx - dy;

        INT16 x = start_gx;
        INT16 y = start_gy;

        int steps = 0;
        const int max_steps = 500;

        while (steps < max_steps) {
            steps++;

            if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
                // 仅清除中等占用值的格子（避免清除真实墙壁）
                if (map.grid[y][x].occupancy >= 60 && map.grid[y][x].occupancy < 100) {
                    cleared_cells.push_back(std::make_pair(x, y));
                    original_occupancy.push_back(map.grid[y][x].occupancy);
                    map.grid[y][x].occupancy = 30;
                }

                // 清除左右各1格
                for (int offset = -1; offset <= 1; offset++) {
                    INT16 nx = x + offset;
                    if (nx >= 0 && nx < GRID_WIDTH) {
                        if (map.grid[y][nx].occupancy >= 60 && map.grid[y][nx].occupancy < 100) {
                            cleared_cells.push_back(std::make_pair(nx, y));
                            original_occupancy.push_back(map.grid[y][nx].occupancy);
                            map.grid[y][nx].occupancy = 30;
                        }
                    }
                }
            }

            if (x == goal_gx && y == goal_gy) break;

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }

        if (!cleared_cells.empty()) {
            std::cout << "  Cleared " << cleared_cells.size() << " cells on sight line" << std::endl;
        }

        // A* 搜索
        std::priority_queue<AStarNode, std::vector<AStarNode>, CompareNode> open_list;
        std::map<int, bool> closed_set;
        std::map<int, AStarNode> parent_map;
        std::map<int, double> g_score;

        AStarNode start_node;
        start_node.x = start_gx;
        start_node.y = start_gy;
        start_node.g = 0;
        start_node.h = heuristic(start_gx, start_gy, goal_gx, goal_gy);
        start_node.f = start_node.g + start_node.h;
        start_node.parent_x = -1;
        start_node.parent_y = -1;

        open_list.push(start_node);
        g_score[getKey(start_gx, start_gy)] = 0;

        int dx_search[] = { -1, 0, 1, -1, 1, -1, 0, 1 };
        int dy_search[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
        double cost[] = { 1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414 };

        int iterations = 0;
        const int max_iterations = 5000;  // ✅ 修复：增加迭代限制（原来是2000）
        bool path_found = false;

        while (!open_list.empty() && iterations < max_iterations) {
            iterations++;

            AStarNode current = open_list.top();
            open_list.pop();

            int current_key = getKey(current.x, current.y);

            if (current.x == goal_gx && current.y == goal_gy) {
                std::cout << "  Path found! Iterations: " << iterations << std::endl;

                // 回溯路径
                std::vector<Point> reverse_path;
                INT16 cx = goal_gx, cy = goal_gy;

                int backtrack_count = 0;
                while (cx != -1 && cy != -1 && backtrack_count < 500) {
                    backtrack_count++;
                    Point p;
                    double wx, wy;
                    MathUtils::gridToWorld(cx, cy, wx, wy);
                    p.coor_x = (INT16)wx;
                    p.coor_y = (INT16)wy;
                    reverse_path.push_back(p);

                    int key = getKey(cx, cy);
                    if (parent_map.find(key) == parent_map.end()) break;

                    INT16 px = parent_map[key].parent_x;
                    INT16 py = parent_map[key].parent_y;
                    cx = px;
                    cy = py;
                }

                // 路径简化 - 自适应采样
                path_length = 0;

                double total_path_length = 0;
                for (size_t i = 1; i < reverse_path.size(); i++) {
                    total_path_length += MathUtils::calculateDistance(
                        reverse_path[i - 1].coor_x, reverse_path[i - 1].coor_y,
                        reverse_path[i].coor_x, reverse_path[i].coor_y);
                }

                // 每20cm保留一个路径点
                int target_waypoints = (int)(total_path_length / 20.0);
                target_waypoints = MathUtils::clamp(target_waypoints, 5, 30);
                int step = (std::max)(1, (int)reverse_path.size() / target_waypoints);

                std::cout << "  Path simplification: " << reverse_path.size()
                    << " nodes -> ~" << target_waypoints << " waypoints (step=" << step << ")" << std::endl;

                for (int i = (int)reverse_path.size() - 1; i >= 0 && path_length < 100; i -= step) {
                    path[path_length++] = reverse_path[i];
                }

                // 确保终点在路径中
                if (path_length > 0) {
                    Point& last = path[path_length - 1];
                    double dist_to_goal = MathUtils::calculateDistance(
                        last.coor_x, last.coor_y, goal.coor_x, goal.coor_y);

                    if (dist_to_goal > 10.0 && path_length < 100) {
                        path[path_length++] = goal;
                    }
                }

                std::cout << "  Final path: " << path_length << " waypoints" << std::endl;

                // 打印路径摘要
                if (path_length > 0) {
                    std::cout << "  Path: (" << path[0].coor_x << "," << path[0].coor_y << ")";
                    if (path_length > 2) std::cout << " -> ... -> ";
                    else if (path_length > 1) std::cout << " -> ";
                    if (path_length > 1) {
                        std::cout << "(" << path[path_length - 1].coor_x << ","
                            << path[path_length - 1].coor_y << ")";
                    }
                    std::cout << std::endl;
                }

                path_found = true;
                break;
            }

            if (closed_set[current_key]) continue;
            closed_set[current_key] = true;

            for (int i = 0; i < 8; i++) {
                INT16 nx = current.x + dx_search[i];
                INT16 ny = current.y + dy_search[i];

                if (!isWalkable(nx, ny, map)) continue;

                int neighbor_key = getKey(nx, ny);
                if (closed_set[neighbor_key]) continue;

                double new_g = current.g + cost[i];

                if (g_score.find(neighbor_key) != g_score.end() &&
                    g_score[neighbor_key] <= new_g) {
                    continue;
                }

                double new_h = heuristic(nx, ny, goal_gx, goal_gy);
                double new_f = new_g + new_h;

                AStarNode neighbor;
                neighbor.x = nx;
                neighbor.y = ny;
                neighbor.g = new_g;
                neighbor.h = new_h;
                neighbor.f = new_f;
                neighbor.parent_x = current.x;
                neighbor.parent_y = current.y;

                parent_map[neighbor_key] = neighbor;
                g_score[neighbor_key] = new_g;
                open_list.push(neighbor);
            }
        }

        // 恢复临时清除的格子
        for (size_t i = 0; i < cleared_cells.size(); i++) {
            map.grid[cleared_cells[i].second][cleared_cells[i].first].occupancy = original_occupancy[i];
        }

        if (!path_found) {
            std::cout << "  No path found (" << iterations << " iterations)" << std::endl;
            path_length = 0;
        }

        return path_found;
    }

    void followPath(POSE cur_pose, Point* path, int path_length, INT16* laser_data,
        double& tra_vel, double& rot_vel) {
        if (path_length < 1) {
            tra_vel = 0;
            rot_vel = 0;
            return;
        }

        // 找到最近的路径点
        int nearest_idx = 0;
        double min_dist = 1e10;

        for (int i = 0; i < path_length; i++) {
            double dist = MathUtils::calculateDistance(
                cur_pose.coor_x, cur_pose.coor_y,
                path[i].coor_x, path[i].coor_y);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }

        // 动态前瞻距离
        int lookahead;
        if (min_dist < 30.0) {
            lookahead = 1;
        }
        else if (min_dist < 60.0) {
            lookahead = 2;
        }
        else {
            lookahead = 3;
        }

        int target_idx = MathUtils::clamp(nearest_idx + lookahead, 0, path_length - 1);
        Point target_point = path[target_idx];

        double distance = MathUtils::calculateDistance(
            cur_pose.coor_x, cur_pose.coor_y,
            target_point.coor_x, target_point.coor_y);

        // 到达最终目标判断
        if (distance < 5.0 && target_idx == path_length - 1) {
            tra_vel = 0;
            rot_vel = 0;
            std::cout << "   [PATH_COMPLETE] Reached final target!" << std::endl;
            return;
        }

        // 计算目标方向
        double target_angle = MathUtils::calculateBearing(
            cur_pose.coor_x, cur_pose.coor_y,
            target_point.coor_x, target_point.coor_y);

        double angle_error = MathUtils::angleDifference(target_angle, cur_pose.coor_ori);

        // 调试输出
        static int debug_counter = 0;
        if (++debug_counter % 30 == 0) {
            std::cout << "   [PATH_FOLLOW] idx=" << nearest_idx << "/" << path_length
                << " target_idx=" << target_idx
                << " cur=(" << cur_pose.coor_x << "," << cur_pose.coor_y << ")"
                << " target=(" << target_point.coor_x << "," << target_point.coor_y << ")"
                << " dist=" << (int)distance
                << " angle_err=" << (int)MathUtils::radToDeg(angle_error) << "deg" << std::endl;
        }

        // 速度控制
        if (fabs(angle_error) > 1.2) {
            // 角度误差非常大（>69度）：原地转向 + 微小前进
            tra_vel = 5.0;
            rot_vel = (angle_error > 0) ? 0.6 : -0.6;
        }
        else if (fabs(angle_error) > 0.7) {
            // 角度误差大（>40度）：慢速前进 + 快速转向
            tra_vel = 15.0;
            rot_vel = (angle_error > 0) ? 0.5 : -0.5;
        }
        else if (fabs(angle_error) > 0.4) {
            // 角度误差中等（>23度）
            tra_vel = 25.0;
            rot_vel = angle_error * 1.5;
        }
        else {
            // 角度误差小：正常前进
            tra_vel = (distance > 50.0) ? 35.0 :
                (distance > 20.0) ? 25.0 : 15.0;
            rot_vel = angle_error * 1.0;
        }

        // 限制速度范围
        tra_vel = MathUtils::clamp(tra_vel, 5.0, 40.0);
        rot_vel = MathUtils::clamp(rot_vel, -0.6, 0.6);
    }

    // ✅ 新增：重置规划器状态
    void reset() {
        current_goal_x = -1;
        current_goal_y = -1;
        current_start_x = -1;
        current_start_y = -1;
        std::cout << "AStarPlanner reset" << std::endl;
    }
};

#endif // ASTAR_PLANNER_H