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

class AStarPlanner {
private:
	LaserSLAM* slam;
	ObstacleAvoidance* obstacle_avoidance;  // 新增

	struct CompareNode {
		bool operator()(const AStarNode& a, const AStarNode& b) const {
			return a.f > b.f;  // 小顶堆
		}
	};

	// 计算启发函数（欧几里得距离）
	double heuristic(INT16 x1, INT16 y1, INT16 x2, INT16 y2) {
		double dx = x1 - x2;
		double dy = y1 - y2;
		return sqrt(dx*dx + dy*dy);
	}

	// 检查位置是否可通行
	bool isWalkable(INT16 x, INT16 y, OccupancyMap& map) {
		if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT) return false;
		// 可以通过未知区域或空闲区域
		return map.grid[y][x].occupancy != 100;
	}

	// 世界坐标转栅格坐标
	void worldToGrid(double wx, double wy, INT16& gx, INT16& gy) {
		gx = (INT16)((wx + MAP_WIDTH / 2.0) / MAP_RESOLUTION);
		gy = (INT16)((wy + MAP_HEIGHT / 2.0) / MAP_RESOLUTION);
		gx = (std::max)((INT16)0, (std::min)((INT16)(GRID_WIDTH - 1), gx));
		gy = (std::max)((INT16)0, (std::min)((INT16)(GRID_HEIGHT - 1), gy));
	}

	// 栅格坐标转世界坐标
	void gridToWorld(INT16 gx, INT16 gy, double& wx, double& wy) {
		wx = gx * MAP_RESOLUTION - MAP_WIDTH / 2.0 + MAP_RESOLUTION / 2.0;
		wy = gy * MAP_RESOLUTION - MAP_HEIGHT / 2.0 + MAP_RESOLUTION / 2.0;
	}

	// 生成唯一键
	int getKey(INT16 x, INT16 y) {
		return y * GRID_WIDTH + x;
	}

public:
	AStarPlanner(LaserSLAM* slam_ptr, ObstacleAvoidance* obstacle_ptr) 
		: slam(slam_ptr), obstacle_avoidance(obstacle_ptr) {
		std::cout << "AStarPlanner initialized" << std::endl;
	}

	// A*路径规划
	bool planPath(POSE start_pose, Point goal, Point* path, int& path_length) {
		if (slam == nullptr) {
			path_length = 0;
			return false;
		}

		OccupancyMap& map = slam->getMap();

		INT16 start_gx, start_gy, goal_gx, goal_gy;
		worldToGrid(start_pose.coor_x, start_pose.coor_y, start_gx, start_gy);
		worldToGrid(goal.coor_x, goal.coor_y, goal_gx, goal_gy);

		std::cout << "A* Planning: Start(" << start_gx << "," << start_gy 
		          << ") -> Goal(" << goal_gx << "," << goal_gy << ")" << std::endl;

		// 检查起点和终点是否有效
		if (!isWalkable(start_gx, start_gy, map)) {
			std::cout << "  Start position blocked" << std::endl;
			path_length = 0;
			return false;
		}
		
		if (!isWalkable(goal_gx, goal_gy, map)) {
			std::cout << "  Goal position blocked" << std::endl;
			path_length = 0;
			return false;
		}

		// 使用 map 避免大数组
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

		// 8方向搜索
		int dx[] = { -1, 0, 1, -1, 1, -1, 0, 1 };
		int dy[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
		double cost[] = { 1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414 };

		int iterations = 0;
		const int max_iterations = 2000;

		while (!open_list.empty() && iterations < max_iterations) {
			iterations++;

			AStarNode current = open_list.top();
			open_list.pop();

			int current_key = getKey(current.x, current.y);

			// 到达目标
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

				// 反转并简化路径（每隔几个点取一个）
				path_length = 0;
				int step = (std::max)(1, (int)reverse_path.size() / 15);  // 最多15个路径点
				
				for (int i = (int)reverse_path.size() - 1; i >= 0 && path_length < 100; i -= step) {
					path[path_length++] = reverse_path[i];
				}
				
				// 确保终点在路径中
				if (path_length > 0) {
					Point& last = path[path_length - 1];
					if (last.coor_x != goal.coor_x || last.coor_y != goal.coor_y) {
						if (path_length < 100) {
							path[path_length++] = goal;
						}
					}
				}

				std::cout << "  Final path: " << path_length << " waypoints" << std::endl;
				return true;
			}

			if (closed_set[current_key]) continue;
			closed_set[current_key] = true;

			// 扩展邻居
			for (int i = 0; i < 8; i++) {
				INT16 nx = current.x + dx[i];
				INT16 ny = current.y + dy[i];

				if (!isWalkable(nx, ny, map)) continue;

				int neighbor_key = getKey(nx, ny);
				if (closed_set[neighbor_key]) continue;

				double new_g = current.g + cost[i];
				
				// 如果已经有更好的路径，跳过
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

		std::cout << "  No path found (" << iterations << " iterations)" << std::endl;
		path_length = 0;
		return false;
	}

	// 沿路径计算速度指令
	void followPath(POSE cur_pose, Point* path, int path_length, INT16* laser_data,
	                double& tra_vel, double& rot_vel) {
		if (path_length < 1) {
			tra_vel = 0;
			rot_vel = 0;
			return;
		}

		// 找到当前位置最近的路径点
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

		// ✅ 修改：动态前视距离
		int lookahead;
		if (min_dist < 50.0) {
			lookahead = 2;  // 接近路径点时看近一点
		} else {
			lookahead = 4;  // ✅ 从3改为4，看得更远
		}
		
		int target_idx = (std::min)(nearest_idx + lookahead, path_length - 1);
		Point target_point = path[target_idx];

		// 计算目标方向
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

		// 检查路径方向的安全性
		double safety = obstacle_avoidance->evaluatePathSafety(cur_pose, target_angle, laser_data);

		// ✅ 修改：更激进的速度控制
		if (fabs(angle_error) > 0.6) {
			tra_vel = 12.0 * safety;  // 从10提高到12
			rot_vel = (angle_error > 0) ? 0.5 : -0.5;
		}
		else if (fabs(angle_error) > 0.3) {
			tra_vel = 28.0 * safety;  // 从20提高到28
			rot_vel = angle_error * 1.2;  // 从1.0提高到1.2
		}
		else {
			tra_vel = 42.0 * safety;  // ✅ 从35提高到42
			rot_vel = angle_error * 0.8;  // 从0.6提高到0.8
		}

		// 速度限制
		tra_vel = (std::max)(0.0, (std::min)(50.0, tra_vel));
		rot_vel = (std::max)(-0.6, (std::min)(0.6, rot_vel));
	}
};

#endif