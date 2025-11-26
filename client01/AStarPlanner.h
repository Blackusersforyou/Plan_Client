#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include "StructData.h"
#include "LaserSLAM.h"
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <map>
#include <iostream>
#include "MathUtils.h"

class AStarPlanner {
private:
	LaserSLAM* slam;
	ObstacleAvoidance* obstacle_avoidance;
	
	INT16 current_goal_x;
	INT16 current_goal_y;
	INT16 current_start_x;  // ✅ 新增
	INT16 current_start_y;  // ✅ 新增

	struct CompareNode {
		bool operator()(const AStarNode& a, const AStarNode& b) const {
			return a.f > b.f;
		}
	};

	double heuristic(INT16 x1, INT16 y1, INT16 x2, INT16 y2) {
		double dx = x1 - x2;
		double dy = y1 - y2;
		return sqrt(dx*dx + dy*dy);
	}

	// ✅ 修改：允许起点和目标点通行
	bool isWalkable(INT16 x, INT16 y, OccupancyMap& map) {
		if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT) return false;
		
		// ✅ 如果是起点或目标点本身，始终允许通行
		if ((x == current_goal_x && y == current_goal_y) ||
			(x == current_start_x && y == current_start_y)) {
			return true;
		}
		
		// ✅ 扩大到 5×5 区域：起点周围
		int dx_start = abs(x - current_start_x);
		int dy_start = abs(y - current_start_y);
		if (dx_start <= 2 && dy_start <= 2) {
			// 起点周围 5×5 区域：只要不是 100% 障碍就允许通行
			return map.grid[y][x].occupancy < 95;
		}
		
		// ✅ 扩大到 5×5 区域：目标周围
		int dx_goal = abs(x - current_goal_x);
		int dy_goal = abs(y - current_goal_y);
		if (dx_goal <= 2 && dy_goal <= 2) {
			// 目标周围 5×5 区域：只要不是 100% 障碍就允许通行
			return map.grid[y][x].occupancy < 95;
		}
		
		// ✅ 其他位置：标准阈值
		return map.grid[y][x].occupancy < 70;
	}

	void worldToGrid(double wx, double wy, INT16& gx, INT16& gy) {
		gx = (INT16)((wx + MAP_WIDTH / 2.0) / MAP_RESOLUTION);
		gy = (INT16)((wy + MAP_HEIGHT / 2.0) / MAP_RESOLUTION);
		gx = (std::max)((INT16)0, (std::min)((INT16)(GRID_WIDTH - 1), gx));
		gy = (std::max)((INT16)0, (std::min)((INT16)(GRID_HEIGHT - 1), gy));
	}

	void gridToWorld(INT16 gx, INT16 gy, double& wx, double& wy) {
		wx = gx * MAP_RESOLUTION - MAP_WIDTH / 2.0 + MAP_RESOLUTION / 2.0;
		wy = gy * MAP_RESOLUTION - MAP_HEIGHT / 2.0 + MAP_RESOLUTION / 2.0;
	}

	int getKey(INT16 x, INT16 y) {
		return y * GRID_WIDTH + x;
	}

public:
	AStarPlanner(LaserSLAM* slam_ptr, ObstacleAvoidance* obstacle_ptr) 
		: slam(slam_ptr), obstacle_avoidance(obstacle_ptr) {
		current_goal_x = -1;
		current_goal_y = -1;
		current_start_x = -1;  // ✅ 初始化
		current_start_y = -1;  // ✅ 初始化
		std::cout << "AStarPlanner initialized" << std::endl;
	}

	bool planPath(POSE start_pose, Point goal, Point* path, int& path_length) {
		if (slam == nullptr) {
			path_length = 0;
			return false;
		}

		OccupancyMap& map = slam->getMap();

		INT16 start_gx, start_gy, goal_gx, goal_gy;
		worldToGrid(start_pose.coor_x, start_pose.coor_y, start_gx, start_gy);
		worldToGrid(goal.coor_x, goal.coor_y, goal_gx, goal_gy);
		
		current_start_x = start_gx;
		current_start_y = start_gy;
		current_goal_x = goal_gx;
		current_goal_y = goal_gy;

		std::cout << "A* Planning: Start(" << start_gx << "," << start_gy 
		          << ") -> Goal(" << goal_gx << "," << goal_gy << ")" << std::endl;
		std::cout << "  Start occupancy: " << map.grid[start_gy][start_gx].occupancy 
		          << ", Goal occupancy: " << map.grid[goal_gy][goal_gx].occupancy << std::endl;

		// ✅ 核心修复：Bresenham算法清除直线路径上的障碍
		std::vector<std::pair<INT16, INT16>> cleared_cells;
		std::vector<INT16> original_occupancy;
		
		// Bresenham 直线算法
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
				// ✅ 临时清除直线路径上占用值 >= 50 的格子
				if (map.grid[y][x].occupancy >= 50) {
					cleared_cells.push_back(std::make_pair(x, y));
					original_occupancy.push_back(map.grid[y][x].occupancy);
					map.grid[y][x].occupancy = 20;  // 临时降低到可通行
				}
				
				// ✅ 同时清除左右各1格（扩大通道宽度）
				for (int offset = -1; offset <= 1; offset++) {
					INT16 nx = x + offset;
					INT16 ny = y;
					if (nx >= 0 && nx < GRID_WIDTH && ny >= 0 && ny < GRID_HEIGHT) {
						if (map.grid[ny][nx].occupancy >= 50) {
							cleared_cells.push_back(std::make_pair(nx, ny));
							original_occupancy.push_back(map.grid[ny][nx].occupancy);
							map.grid[ny][nx].occupancy = 20;
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
			std::cout << "  Temporarily cleared " << cleared_cells.size() 
					  << " cells on direct path (3 cells wide)" << std::endl;
		}

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
		const int max_iterations = 2000;
		bool path_found = false;

		while (!open_list.empty() && iterations < max_iterations) {
			iterations++;

			AStarNode current = open_list.top();
			open_list.pop();

			int current_key = getKey(current.x, current.y);

			if (current.x == goal_gx && current.y == goal_gy) {
				std::cout << "  Path found! Iterations: " << iterations << std::endl;
				
				std::vector<Point> reverse_path;
				INT16 cx = goal_gx, cy = goal_gy;

				int backtrack_count = 0;
				while (cx != -1 && cy != -1 && backtrack_count < 500) {
					backtrack_count++;
					Point p;
					double wx, wy;
					gridToWorld(cx, cy, wx, wy);
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

				path_length = 0;
				int step = (std::max)(1, (int)reverse_path.size() / 15);
				
				for (int i = (int)reverse_path.size() - 1; i >= 0 && path_length < 100; i -= step) {
					path[path_length++] = reverse_path[i];
				}
				
				if (path_length > 0) {
					Point& last = path[path_length - 1];
					if (last.coor_x != goal.coor_x || last.coor_y != goal.coor_y) {
						if (path_length < 100) {
							path[path_length++] = goal;
						}
					}
				}

				std::cout << "  Final path: " << path_length << " waypoints" << std::endl;
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

		if (!path_found) {
			std::cout << "  No path found (" << iterations << " iterations)" << std::endl;
			path_length = 0;
		}
		
		// ✅ 可选：恢复原始占用值（或保持清除状态以便后续使用）
		// 这里选择不恢复，因为这些格子应该是可通行的
		/*
		for (size_t i = 0; i < cleared_cells.size(); i++) {
			INT16 cx = cleared_cells[i].first;
			INT16 cy = cleared_cells[i].second;
			map.grid[cy][cx].occupancy = original_occupancy[i];
		}
		*/

		return path_found;
	}

	void followPath(POSE cur_pose, Point* path, int path_length, INT16* laser_data,
                double& tra_vel, double& rot_vel) {
    if (path_length < 1) {
        tra_vel = 0;
        rot_vel = 0;
        return;
    }

    int nearest_idx = 0;
    double min_dist = 1e10;
    
    for (int i = 0; i < path_length; i++) {
        double dx = path[i].coor_x - cur_pose.coor_x;
        double dy = path[i].coor_y - cur_pose.coor_y;
        double dist = sqrt(dx*dx + dy*dy);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }

    int lookahead;
    if (min_dist < 50.0) {
        lookahead = 2;
    } else {
        lookahead = 4;
    }
    
    int target_idx = (std::min)(nearest_idx + lookahead, path_length - 1);
    Point target_point = path[target_idx];

    double dx = target_point.coor_x - cur_pose.coor_x;
    double dy = target_point.coor_y - cur_pose.coor_y;
    double distance = sqrt(dx*dx + dy*dy);
    
    if (distance < 5.0) {
        tra_vel = 0;
        rot_vel = 0;
        return;
    }

    double target_angle = atan2(dy, dx);
    double angle_error = target_angle - cur_pose.coor_ori;
    
    while (angle_error > PI) angle_error -= 2 * PI;
    while (angle_error < -PI) angle_error += 2 * PI;

    // ✅ 移除 safety 评分，使用固定速度
    if (fabs(angle_error) > 0.6) {
        tra_vel = 12.0;  // ✅ 降低从 15 到 12
        rot_vel = (angle_error > 0) ? 0.5 : -0.5;
    }
    else if (fabs(angle_error) > 0.3) {
        tra_vel = 25.0;  // ✅ 降低从 30 到 25
        rot_vel = angle_error * 1.2;
    }
    else {
        tra_vel = 35.0;  // ✅ 降低从 42 到 35
        rot_vel = angle_error * 0.8;
    }

    tra_vel = (std::max)(0.0, (std::min)(40.0, tra_vel));  // ✅ 上限从 50 降到 40
    rot_vel = (std::max)(-0.6, (std::min)(0.6, rot_vel));
	}
};

#endif