#ifndef LASER_SLAM_H
#define LASER_SLAM_H

#include "StructData.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>

class LaserSLAM {
private:
	OccupancyMap* map;
	POSE robot_pose;
	Point target_pose;
	bool target_visible;
	
	// 访问历史记录
	struct VisitRecord {
		INT16 x, y;
		int visit_count;
		int last_visit_frame;
	};
	std::vector<VisitRecord> visit_history;
	int current_frame;
	const int VISIT_DECAY_FRAMES = 100;
	
	// 概率占用参数
	const double LOG_ODD_OCCUPIED = 0.85;
	const double LOG_ODD_FREE = -0.4;
	const double LOG_ODD_MIN = -2.0;
	const double LOG_ODD_MAX = 2.0;

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

	void rayTraceWithProbability(INT16 x0, INT16 y0, INT16 x1, INT16 y1) {
		int dx = abs(x1 - x0);
		int dy = abs(y1 - y0);
		int sx = (x0 < x1) ? 1 : -1;
		int sy = (y0 < y1) ? 1 : -1;
		int err = dx - dy;

		int step_count = 0;
		const int max_steps = 600;

		while (step_count < max_steps) {
			step_count++;
			
			if (x0 >= 0 && x0 < GRID_WIDTH && y0 >= 0 && y0 < GRID_HEIGHT) {
				if (x0 != x1 || y0 != y1) {
					map->grid[y0][x0].miss_count = 
						(std::min)(255, (int)map->grid[y0][x0].miss_count + 1);
					updateCellProbability(x0, y0, false);
				}
				map->grid[y0][x0].explored = 
					(std::min)(255, (int)map->grid[y0][x0].explored + 1);
			}

			if (x0 == x1 && y0 == y1) break;

			int e2 = 2 * err;
			if (e2 > -dy) {
				err -= dy;
				x0 += sx;
			}
			if (e2 < dx) {
				err += dx;
				y0 += sy;
			}
		}
		
		if (x1 >= 0 && x1 < GRID_WIDTH && y1 >= 0 && y1 < GRID_HEIGHT) {
			map->grid[y1][x1].hit_count = 
				(std::min)(255, (int)map->grid[y1][x1].hit_count + 1);
			updateCellProbability(x1, y1, true);
			map->grid[y1][x1].explored = 
				(std::min)(255, (int)map->grid[y1][x1].explored + 1);
			
			for (int di = -1; di <= 1; di++) {
				for (int dj = -1; dj <= 1; dj++) {
					int ni = y1 + di;
					int nj = x1 + dj;
					if (ni >= 0 && ni < GRID_HEIGHT && nj >= 0 && nj < GRID_WIDTH) {
						if (di != 0 || dj != 0) {
							map->grid[ni][nj].hit_count = 
								(std::min)(255, (int)map->grid[ni][nj].hit_count + 1);
							updateCellProbability(nj, ni, true);
						}
					}
				}
			}
		}
	}
	
	void updateCellProbability(INT16 gx, INT16 gy, bool is_obstacle) {
		if (gx < 0 || gx >= GRID_WIDTH || gy < 0 || gy >= GRID_HEIGHT) return;
		
		int hit = map->grid[gy][gx].hit_count;
		int miss = map->grid[gy][gx].miss_count;
		int total = hit + miss;
		
		if (total == 0) {
			map->grid[gy][gx].occupancy = -1;
			return;
		}
		
		double log_odd = 0;
		if (is_obstacle) {
			log_odd += LOG_ODD_OCCUPIED;
		} else {
			log_odd += LOG_ODD_FREE;
		}
		
		log_odd = (std::max)(LOG_ODD_MIN, (std::min)(LOG_ODD_MAX, log_odd));
		double probability = 1.0 / (1.0 + exp(-log_odd));
		map->grid[gy][gx].occupancy = (INT16)(probability * 100.0);
		
		if (hit > miss + 3) {
			map->grid[gy][gx].occupancy = 100;
		} else if (miss > hit + 5) {
			map->grid[gy][gx].occupancy = 0;
		}
	}

	void smoothMap() {
		std::vector<std::vector<INT16>> temp_occupancy(GRID_HEIGHT, 
			std::vector<INT16>(GRID_WIDTH));
		
		for (int i = 1; i < GRID_HEIGHT - 1; i++) {
			for (int j = 1; j < GRID_WIDTH - 1; j++) {
				int obstacle_count = 0;
				int free_count = 0;
				
				for (int di = -1; di <= 1; di++) {
					for (int dj = -1; dj <= 1; dj++) {
						int ni = i + di;
						int nj = j + dj;
						if (map->grid[ni][nj].occupancy == 100) obstacle_count++;
						else if (map->grid[ni][nj].occupancy == 0) free_count++;
					}
				}
				
				if (map->grid[i][j].occupancy == 100 && obstacle_count <= 2) {
					temp_occupancy[i][j] = 50;
				} else if (map->grid[i][j].occupancy == 0 && free_count <= 2) {
					temp_occupancy[i][j] = 50;
				} else {
					temp_occupancy[i][j] = map->grid[i][j].occupancy;
				}
			}
		}
			
		for (int i = 1; i < GRID_HEIGHT - 1; i++) {
			for (int j = 1; j < GRID_WIDTH - 1; j++) {
				if (temp_occupancy[i][j] == 50 && map->grid[i][j].explored > 0) {
					int sum_occupancy = 0;
					int count = 0;
					for (int di = -1; di <= 1; di++) {
						for (int dj = -1; dj <= 1; dj++) {
							if (di == 0 && dj == 0) continue;
							sum_occupancy += map->grid[i+di][j+dj].occupancy;
							count++;
						}
					}
					map->grid[i][j].occupancy = (INT16)(sum_occupancy / count);
				}
			}
		}
	}
	
	void recordVisit(INT16 gx, INT16 gy) {
		for (auto& record : visit_history) {
			if (record.x == gx && record.y == gy) {
				record.visit_count++;
				record.last_visit_frame = current_frame;
				return;
			}
		}
		
		VisitRecord new_record;
		new_record.x = gx;
		new_record.y = gy;
		new_record.visit_count = 1;
		new_record.last_visit_frame = current_frame;
		visit_history.push_back(new_record);
		
		if (visit_history.size() > 500) {
			visit_history.erase(visit_history.begin());
		}
	}
	
	double getVisitPenalty(INT16 gx, INT16 gy) {
		for (const auto& record : visit_history) {
			if (record.x == gx && record.y == gy) {
				int frame_diff = current_frame - record.last_visit_frame;
				double time_decay = (std::min)(1.0, (double)frame_diff / VISIT_DECAY_FRAMES);
				double visit_penalty = 1.0 / (1.0 + record.visit_count * (1.0 - time_decay));
				return visit_penalty;
			}
		}
		return 1.0;
	}

	// ✅ 新增:候选目标列表
	std::vector<TargetCandidate> target_candidates;
	const int MIN_OBJECT_SIZE = 1;   // 最小1个栅格 (5cm)
	const int MAX_OBJECT_SIZE = 4;   // 最大4个栅格 (20cm)
	const int LARGE_OBSTACLE_SIZE = 8; // 大于此尺寸为真正障碍物

	// ✅ 新增:检测候选目标
	void detectCandidateTargets() {
		std::vector<std::vector<bool>> visited(GRID_HEIGHT, 
			std::vector<bool>(GRID_WIDTH, false));
		
		for (int i = 0; i < GRID_HEIGHT; i++) {
			for (int j = 0; j < GRID_WIDTH; j++) {
				if (map->grid[i][j].occupancy >= 70 && !visited[i][j]) {
					// 使用BFS找连通的障碍物区域
					std::vector<std::pair<int, int>> cluster;
					std::vector<std::pair<int, int>> queue;
					queue.push_back(std::make_pair(i, j));
					visited[i][j] = true;
					
					while (!queue.empty() && cluster.size() < 100) {
						std::pair<int, int> current = queue[0];
						queue.erase(queue.begin());
						cluster.push_back(current);
						
						int cy = current.first;
						int cx = current.second;
						
						// 4邻域
						int dirs[4][2] = {{-1,0}, {1,0}, {0,-1}, {0,1}};
						for (int d = 0; d < 4; d++) {
							int ny = cy + dirs[d][0];
							int nx = cx + dirs[d][1];
							
							if (ny >= 0 && ny < GRID_HEIGHT && 
							    nx >= 0 && nx < GRID_WIDTH &&
							    !visited[ny][nx] &&
							    map->grid[ny][nx].occupancy >= 70) {
								visited[ny][nx] = true;
								queue.push_back(std::make_pair(ny, nx));
							}
						}
					}
					
					int cluster_size = static_cast<int>(cluster.size());
					
					// ✅ 判断是否为候选目标(小物体)
					if (cluster_size >= MIN_OBJECT_SIZE && 
					    cluster_size <= MAX_OBJECT_SIZE) {
						// 计算中心
						double center_y = 0, center_x = 0;
						for (const auto& cell : cluster) {
							center_y += cell.first;
							center_x += cell.second;
						}
						center_y /= cluster_size;
						center_x /= cluster_size;
						
						INT16 center_gx = (INT16)(center_x + 0.5);
						INT16 center_gy = (INT16)(center_y + 0.5);
						
						// 转世界坐标
						double world_x, world_y;
						gridToWorld(center_gx, center_gy, world_x, world_y);
						
						// 计算特征得分
						double confidence = evaluateObjectAsTarget(
							center_gx, center_gy, cluster_size);
						
						// 更新或添加候选
						updateTargetCandidate(world_x, world_y, 
							center_gx, center_gy, cluster_size, confidence);
					}
					// ✅ 大物体保持为障碍物
					else if (cluster_size > LARGE_OBSTACLE_SIZE) {
						// 不做处理,保持为障碍物
					}
				}
			}
		}
		
		// 清理过期候选
		cleanupOldCandidates();
	}
	
	// ✅ 评估物体是否可能是目标
	double evaluateObjectAsTarget(INT16 gx, INT16 gy, int size) {
		double score = 0.0;
		
		// 特征1:尺寸评分(越小越可能是目标)
		double size_score = 1.0 - (double)(size - MIN_OBJECT_SIZE) / 
		                           (double)(MAX_OBJECT_SIZE - MIN_OBJECT_SIZE);
		size_score = (std::max)(0.0, (std::min)(1.0, size_score));
		
		// 特征2:孤立性(周围是否空旷)
		int free_count = 0;
		int total_count = 0;
		for (int di = -5; di <= 5; di++) {
			for (int dj = -5; dj <= 5; dj++) {
				int ni = gy + di;
				int nj = gx + dj;
				if (ni >= 0 && ni < GRID_HEIGHT && nj >= 0 && nj < GRID_WIDTH) {
					total_count++;
					if (map->grid[ni][nj].occupancy < 50) {
						free_count++;
					}
				}
			}
		}
		double isolation_score = (total_count > 0) ? 
			(double)free_count / total_count : 0.0;
		
		// 特征3:距离机器人(不要太近,避免噪声)
		INT16 robot_gx, robot_gy;
		worldToGrid(robot_pose.coor_x, robot_pose.coor_y, robot_gx, robot_gy);
		double dist = sqrt(pow(gx - robot_gx, 2) + pow(gy - robot_gy, 2));
		double distance_score = (dist > 5.0) ? 1.0 : 0.5;
		
		// 综合评分
		score = 0.5 * size_score + 0.3 * isolation_score + 0.2 * distance_score;
		
		return score;
	}
	
	// ✅ 更新候选目标列表
	void updateTargetCandidate(double world_x, double world_y,
	                           INT16 gx, INT16 gy, int size, double confidence) {
		// 查找是否已存在
		bool found = false;
		for (auto& candidate : target_candidates) {
			double dx = candidate.position.coor_x - world_x;
			double dy = candidate.position.coor_y - world_y;
			double distance = sqrt(dx*dx + dy*dy);
			
			if (distance < 20.0) {  // 20cm内认为是同一个
				candidate.last_seen_frame = current_frame;
				candidate.confidence = 0.7 * candidate.confidence + 0.3 * confidence;
				candidate.size_cells = size;
				found = true;
				break;
			}
		}
		
		if (!found && confidence > 0.3) {
			TargetCandidate new_candidate;
			new_candidate.position.coor_x = (INT16)world_x;
			new_candidate.position.coor_y = (INT16)world_y;
			new_candidate.grid_x = gx;
			new_candidate.grid_y = gy;
			new_candidate.size_cells = size;
			new_candidate.confidence = confidence;
			new_candidate.first_seen_frame = current_frame;
			new_candidate.last_seen_frame = current_frame;
			new_candidate.is_small_object = true;
			new_candidate.physically_confirmed = false;
			
			target_candidates.push_back(new_candidate);
		}
	}
	
	// ✅ 清理过期候选
	void cleanupOldCandidates() {
		auto it = target_candidates.begin();
		while (it != target_candidates.end()) {
			if (current_frame - it->last_seen_frame > 100) {
				it = target_candidates.erase(it);
			} else {
				++it;
			}
		}
	}
	
	// ✅ 将候选目标区域标记为可通行
	void markCandidatesAsNavigable() {
		for (const auto& candidate : target_candidates) {
			if (candidate.confidence > 0.5) {
				INT16 gx = candidate.grid_x;
				INT16 gy = candidate.grid_y;
				
				// 降低占用值,允许接近
				for (int di = -2; di <= 2; di++) {
					for (int dj = -2; dj <= 2; dj++) {
						int ni = gy + di;
						int nj = gx + dj;
						if (ni >= 0 && ni < GRID_HEIGHT && 
						    nj >= 0 && nj < GRID_WIDTH) {
							if (map->grid[ni][nj].occupancy >= 70) {
								// 保留一定占用值(30-50),不完全清空
								map->grid[ni][nj].occupancy = 40;
							}
						}
					}
				}
			}
		}
	}

public:
	LaserSLAM() {
		std::cout << "Creating LaserSLAM with map size: " << GRID_WIDTH << "x" << GRID_HEIGHT << std::endl;
		try {
			map = new OccupancyMap();
			target_visible = false;
			current_frame = 0;
			visit_history.reserve(500);
			std::cout << "LaserSLAM initialized successfully" << std::endl;
		}
		catch (const std::exception& e) {
			std::cout << "Error initializing LaserSLAM: " << e.what() << std::endl;
			throw;
		}
	}

	~LaserSLAM() {
		if (map) {
			delete map;
		}
	}

	void updateMap(POSE cur_pose, INT16* laser_data) {
		robot_pose = cur_pose;
		current_frame++;

		INT16 robot_gx, robot_gy;
		worldToGrid(cur_pose.coor_x, cur_pose.coor_y, robot_gx, robot_gy);
		recordVisit(robot_gx, robot_gy);

		for (int i = 0; i < 360; i += 2) {
			if (laser_data[i] > 0 && laser_data[i] < 3000) {
				double angle = cur_pose.coor_ori + (i * PI / 180.0);
				double obs_x = cur_pose.coor_x + laser_data[i] * cos(angle);
				double obs_y = cur_pose.coor_y + laser_data[i] * sin(angle);

				INT16 obs_gx, obs_gy;
				worldToGrid(obs_x, obs_y, obs_gx, obs_gy);
				rayTraceWithProbability(robot_gx, robot_gy, obs_gx, obs_gy);
			}
		}
		
		if (current_frame % 20 == 0) {
			smoothMap();
		}
		
		// 每5帧检测一次候选目标
		if (current_frame % 5 == 0) {
			detectCandidateTargets();
			markCandidatesAsNavigable();
		}
	}

	// ✅ 添加 updateOpenness 方法
	void updateOpenness(Point target) {
		int radius = 5;
		
		for (int i = 0; i < GRID_HEIGHT; i++) {
			for (int j = 0; j < GRID_WIDTH; j++) {
				if (map->grid[i][j].occupancy == 100) {
					map->openness[i][j] = 0;
					continue;
				}

				int free_count = 0;
				int unexplored_count = 0;
				int total_count = 0;

				for (int di = -radius; di <= radius; di++) {
					for (int dj = -radius; dj <= radius; dj++) {
						int ni = i + di;
						int nj = j + dj;
						if (ni >= 0 && ni < GRID_HEIGHT && nj >= 0 && nj < GRID_WIDTH) {
							total_count++;
							if (map->grid[ni][nj].occupancy == 0) {
								free_count++;
							}
							if (map->grid[ni][nj].explored == 0) {
								unexplored_count++;
							}
						}
					}
				}

				double free_ratio = (total_count > 0) ? (double)free_count / total_count : 0;
				double unexplored_ratio = (total_count > 0) ? (double)unexplored_count / total_count : 0;
				double visit_penalty = getVisitPenalty(j, i);
				
				double wx, wy;
				gridToWorld(j, i, wx, wy);
				double dx = target.coor_x - wx;
				double dy = target.coor_y - wy;
				double dist_to_target = sqrt(dx*dx + dy*dy);
				
				double target_bonus = 0;
				if (unexplored_ratio > 0.3 && dist_to_target < 300) {
					target_bonus = 0.2 * (1.0 - dist_to_target / 300.0);
				}
				
				map->openness[i][j] = (free_ratio + 0.5 * unexplored_ratio + target_bonus) * visit_penalty;
			}
		}
	}

	// ✅ 添加 isDirectPathClear 方法
	bool isDirectPathClear(Point target, INT16* laser_data, double safe_margin = 80.0) {
		double dx = target.coor_x - robot_pose.coor_x;
		double dy = target.coor_y - robot_pose.coor_y;
		double distance = sqrt(dx*dx + dy*dy);
		
		if (distance < 30.0) return true;
		
		double target_angle = atan2(dy, dx) - robot_pose.coor_ori;
		while (target_angle > PI) target_angle -= 2 * PI;
		while (target_angle < -PI) target_angle += 2 * PI;
		
		if (fabs(target_angle) > PI / 3.0) return false;
		
		int laser_index = (int)(target_angle * 180.0 / PI + 0.5);
		if (laser_index < 0) laser_index += 360;
		if (laser_index >= 360) laser_index -= 360;
		
		int obstacles_count = 0;
		int total_samples = 0;
		
		for (int offset = -15; offset <= 15; offset++) {
			int idx = laser_index + offset;
			if (idx < 0) idx += 360;
			if (idx >= 360) idx -= 360;
			
			if (laser_data[idx] > 0 && laser_data[idx] < 5000) {
				total_samples++;
				if (laser_data[idx] < distance - safe_margin) {
					obstacles_count++;
				}
			}
		}
		
		if (total_samples > 0 && (double)obstacles_count / total_samples > 0.3) {
			return false;
		}
		
		return true;
	}

	// ✅ 添加 isTargetVisible 方法
	bool isTargetVisible(Point target, INT16* laser_data) {
		target_pose = target;

		double dx = target.coor_x - robot_pose.coor_x;
		double dy = target.coor_y - robot_pose.coor_y;
		double distance = sqrt(dx*dx + dy*dy);
		
		if (distance > 1000.0) {
			target_visible = false;
			return false;
		}
		
		if (distance < 30.0) {
			target_visible = true;
			return true;
		}

		double angle = atan2(dy, dx) - robot_pose.coor_ori;
		while (angle > PI) angle -= 2 * PI;
		while (angle < -PI) angle += 2 * PI;

		if (fabs(angle) > PI / 3.0) {
			target_visible = false;
			return false;
		}

		int laser_index = (int)(angle * 180.0 / PI + 0.5);
		if (laser_index < 0) laser_index += 360;
		if (laser_index >= 360) laser_index -= 360;

		bool visible = false;
		int votes = 0;
		int total_checks = 0;
		for (int offset = -8; offset <= 8; offset++) {
			int idx = laser_index + offset;
			if (idx < 0) idx += 360;
			if (idx >= 360) idx -= 360;
			
			if (laser_data[idx] > 0 && laser_data[idx] < 5000) {
				total_checks++;
				double diff = fabs(laser_data[idx] - distance);
				if (diff < 25.0) {
					votes++;
				}
			}
		}
		
		visible = (total_checks > 0) && ((double)votes / total_checks >= 0.6);
		target_visible = visible;
		return visible;
	}

	OccupancyMap& getMap() {
		return *map;
	}

	bool getTargetVisible() {
		return target_visible;
	}
	
	double getExplorationProgress() {
		int explored_cells = 0;
		int total_cells = GRID_WIDTH * GRID_HEIGHT;
		
		for (int i = 0; i < GRID_HEIGHT; i++) {
			for (int j = 0; j < GRID_WIDTH; j++) {
				if (map->grid[i][j].explored > 0) {
					explored_cells++;
				}
			}
		}
		
		return (double)explored_cells / total_cells;
	}
	
	void resetVisitHistory() {
		visit_history.clear();
		current_frame = 0;
		std::cout << "Visit history reset" << std::endl;
	}
	
	// ✅ 新增:获取候选目标列表
	std::vector<TargetCandidate> getTargetCandidates() const {
		return target_candidates;
	}
	
	// ✅ 新增:获取最佳候选目标
	bool getBestCandidate(Point& candidate_pos, double& confidence) {
		double best_score = 0;
		bool found = false;
		
		INT16 robot_gx, robot_gy;
		worldToGrid(robot_pose.coor_x, robot_pose.coor_y, robot_gx, robot_gy);
		
		for (const auto& candidate : target_candidates) {
			// 持续性要求:至少被看到10帧
			int persistence = candidate.last_seen_frame - candidate.first_seen_frame;
			if (persistence < 10) continue;
			
			// 计算综合得分
			double dx = candidate.grid_x - robot_gx;
			double dy = candidate.grid_y - robot_gy;
			double distance = sqrt(dx*dx + dy*dy);
			
			// 得分 = 置信度 × 距离因子
			double distance_factor = 1.0 / (1.0 + distance / 50.0);
			double score = candidate.confidence * distance_factor;
			
			if (score > best_score) {
				best_score = score;
				candidate_pos = candidate.position;
				confidence = candidate.confidence;
				found = true;
			}
		}
		
		return found;
	}
	
	// ✅ 新增:物理确认目标
	void confirmTargetPhysically(double world_x, double world_y) {
		for (auto& candidate : target_candidates) {
			double dx = candidate.position.coor_x - world_x;
			double dy = candidate.position.coor_y - world_y;
			double distance = sqrt(dx*dx + dy*dy);
			
			if (distance < 30.0) {  // 30cm内
				candidate.physically_confirmed = true;
				candidate.confidence = 1.0;  // 置信度提升到100%
				std::cout << ">>> Target physically confirmed at (" 
				          << candidate.position.coor_x << "," 
				          << candidate.position.coor_y << ")" << std::endl;
				break;
			}
		}
	}
};

#endif