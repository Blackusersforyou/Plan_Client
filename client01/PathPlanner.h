#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <Windows.h>
#include "StructData.h"

// A* 路径规划节点
struct AStarNode {
    int x, y;
    double g, h, f;  // g: 起点到当前, h: 当前到终点, f: g+h
    AStarNode* parent;
    
    AStarNode(int _x, int _y, double _g, double _h, AStarNode* _parent = nullptr)
        : x(_x), y(_y), g(_g), h(_h), f(_g + _h), parent(_parent) {}
    
    bool operator>(const AStarNode& other) const {
        return f > other.f;
    }
};

// 路径规划器类
class PathPlanner {
private:
    static const int GRID_SIZE = 20;           // 栅格大小 (cm)
    static const int MAP_WIDTH = 300;          // 地图宽度 (格子数)
    static const int MAP_HEIGHT = 300;         // 地图高度
    static const int SAFE_DISTANCE = 400;      // 安全距离 (mm)
    
    std::vector<std::vector<int>> occupancyGrid; // 栅格地图
    
    // 计算欧几里得距离
    double euclideanDistance(int x1, int y1, int x2, int y2) {
        return sqrt((double)((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
    }
    
    // 世界坐标转栅格坐标
    void worldToGrid(INT16 wx, INT16 wy, int& gx, int& gy) {
        gx = wx / GRID_SIZE + MAP_WIDTH / 2;
        gy = wy / GRID_SIZE + MAP_HEIGHT / 2;
    }
    
    // 栅格坐标转世界坐标
    void gridToWorld(int gx, int gy, INT16& wx, INT16& wy) {
        wx = (INT16)((gx - MAP_WIDTH / 2) * GRID_SIZE);
        wy = (INT16)((gy - MAP_HEIGHT / 2) * GRID_SIZE);
    }

public:
    PathPlanner() {
        occupancyGrid.resize(MAP_WIDTH, std::vector<int>(MAP_HEIGHT, 0));
    }
    
    // 更新栅格地图 (从激光雷达数据)
    void updateOccupancyGrid(const INT16 obstacle[360], const POSE& robotPos) {
        // 清空旧数据
        for (size_t i = 0; i < occupancyGrid.size(); i++) {
            std::fill(occupancyGrid[i].begin(), occupancyGrid[i].end(), 0);
        }
        
        // 更新障碍物
        for (int angle = 0; angle < 360; angle++) {
            if (obstacle[angle] > 0 && obstacle[angle] < 5000) {
                double rad = (angle * PI / 180.0) + robotPos.coor_ori;
                double obst_x = robotPos.coor_x + obstacle[angle] * cos(rad);
                double obst_y = robotPos.coor_y + obstacle[angle] * sin(rad);
                
                int gx, gy;
                worldToGrid((INT16)obst_x, (INT16)obst_y, gx, gy);
                
                // 膨胀障碍物 (安全边界)
                int inflate = 2;
                for (int dx = -inflate; dx <= inflate; dx++) {
                    for (int dy = -inflate; dy <= inflate; dy++) {
                        int nx = gx + dx;
                        int ny = gy + dy;
                        if (nx >= 0 && nx < MAP_WIDTH && ny >= 0 && ny < MAP_HEIGHT) {
                            occupancyGrid[nx][ny] = 1;
                        }
                    }
                }
            }
        }
    }
    
    // A* 路径规划
    std::vector<Point> planPath(const POSE& start, const Point& goal) {
        std::vector<Point> path;
        
        int start_gx, start_gy, goal_gx, goal_gy;
        worldToGrid(start.coor_x, start.coor_y, start_gx, start_gy);
        worldToGrid(goal.coor_x, goal.coor_y, goal_gx, goal_gy);
        
        // 边界检查
        if (goal_gx < 0 || goal_gx >= MAP_WIDTH || goal_gy < 0 || goal_gy >= MAP_HEIGHT) {
            return path;
        }
        
        // 优先队列 (小顶堆)
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openList;
        std::vector<std::vector<bool>> closedList(MAP_WIDTH, std::vector<bool>(MAP_HEIGHT, false));
        
        // 起始节点
        openList.push(AStarNode(start_gx, start_gy, 0, 
                                euclideanDistance(start_gx, start_gy, goal_gx, goal_gy)));
        
        // 8方向搜索
        int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        double cost[] = {1.414, 1, 1.414, 1, 1, 1.414, 1, 1.414};
        
        AStarNode* goalNode = nullptr;
        std::vector<AStarNode*> allocatedNodes;  // 用于内存管理
        
        while (!openList.empty()) {
            AStarNode current = openList.top();
            openList.pop();
            
            // 到达目标
            if (current.x == goal_gx && current.y == goal_gy) {
                goalNode = new AStarNode(current);
                allocatedNodes.push_back(goalNode);
                break;
            }
            
            if (closedList[current.x][current.y]) continue;
            closedList[current.x][current.y] = true;
            
            // 扩展邻居节点
            for (int i = 0; i < 8; i++) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];
                
                // 边界和障碍物检查
                if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT ||
                    occupancyGrid[nx][ny] == 1 || closedList[nx][ny]) {
                    continue;
                }
                
                double g = current.g + cost[i];
                double h = euclideanDistance(nx, ny, goal_gx, goal_gy);
                
                AStarNode* parentNode = new AStarNode(current);
                allocatedNodes.push_back(parentNode);
                openList.push(AStarNode(nx, ny, g, h, parentNode));
            }
        }
        
        // 回溯路径
        if (goalNode != nullptr) {
            AStarNode* node = goalNode;
            while (node != nullptr) {
                Point p;
                gridToWorld(node->x, node->y, p.coor_x, p.coor_y);
                path.insert(path.begin(), p);
                node = node->parent;
            }
        }
        
        // 清理内存
        for (size_t i = 0; i < allocatedNodes.size(); i++) {
            delete allocatedNodes[i];
        }
        
        return path;
    }
    
    // 简化路径 (每隔 N 个点采样)
    std::vector<Point> simplifyPath(const std::vector<Point>& path, int step = 5) {
        std::vector<Point> simplified;
        for (size_t i = 0; i < path.size(); i += step) {
            simplified.push_back(path[i]);
        }
        if (!path.empty() && 
            (simplified.empty() || 
             simplified.back().coor_x != path.back().coor_x || 
             simplified.back().coor_y != path.back().coor_y)) {
            simplified.push_back(path.back());
        }
        return simplified;
    }
};

// 局部路径跟踪 (Pure Pursuit)
class PurePursuitController {
private:
    static const int LOOKAHEAD_DISTANCE = 500; // 前瞻距离 (mm)
    
public:
    // 寻找前瞻点
    Point findLookaheadPoint(const std::vector<Point>& path, const POSE& robotPos) {
        Point lookahead;
        lookahead.coor_x = 0;
        lookahead.coor_y = 0;
        
        if (path.empty()) return lookahead;
        
        double minDist = 1e9;
        for (size_t i = 0; i < path.size(); i++) {
            const Point& p = path[i];
            double dist = sqrt((double)((p.coor_x - robotPos.coor_x) * (p.coor_x - robotPos.coor_x) + 
                                       (p.coor_y - robotPos.coor_y) * (p.coor_y - robotPos.coor_y)));
            
            if (dist >= LOOKAHEAD_DISTANCE && dist < minDist) {
                lookahead = p;
                minDist = dist;
            }
        }
        
        // 如果没找到,返回最后一个点
        if (lookahead.coor_x == 0 && lookahead.coor_y == 0) {
            return path.back();
        }
        return lookahead;
    }
    
    // 计算转向角速度
    double calculateRotVel(const POSE& robotPos, const Point& lookahead) {
        double dx = lookahead.coor_x - robotPos.coor_x;
        double dy = lookahead.coor_y - robotPos.coor_y;
        double targetAngle = atan2(dy, dx);
        
        double angleError = targetAngle - robotPos.coor_ori;
        
        // 归一化到 [-PI, PI]
        while (angleError > PI) angleError -= 2 * PI;
        while (angleError < -PI) angleError += 2 * PI;
        
        // 比例控制
        return angleError * 1.0; // Kp = 1.0
    }
};

// 动态窗口避障 (DWA)
class DynamicWindowApproach {
private:
    static const int MAX_SPEED = 50;          // 最大线速度 (cm/s)
    static const int MAX_ROT_SPEED = 1;       // 最大角速度 (rad/s)
    static const int ACC = 20;                // 加速度 (cm/s²)
    
public:
    // 检测前方障碍物
    bool checkFrontObstacle(const INT16 obstacle[360], int angleRange = 45) {
        for (int angle = -angleRange; angle <= angleRange; angle++) {
            int idx = (360 + angle) % 360;
            if (obstacle[idx] > 0 && obstacle[idx] < 600) {
                return true; // 检测到近距离障碍物
            }
        }
        return false;
    }
    
    // 寻找最安全方向
    double findSafeDirection(const INT16 obstacle[360]) {
        INT16 maxDist = 0;
        int safeAngle = 0;
        
        for (int angle = -90; angle <= 90; angle += 10) {
            int idx = (360 + angle) % 360;
            if (obstacle[idx] > maxDist) {
                maxDist = obstacle[idx];
                safeAngle = angle;
            }
        }
        
        return safeAngle * PI / 180.0; // 转换为弧度
    }
};

#endif // PATH_PLANNER_H