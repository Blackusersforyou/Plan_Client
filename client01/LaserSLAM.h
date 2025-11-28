#ifndef LASER_SLAM_H
#define LASER_SLAM_H

#include "StructData.h"
#include "MathUtils.h"  // ✅ 使用统一的坐标转换
#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>

/**
 * @brief 激光SLAM系统
 *
 * 修复内容：
 * 1. 使用统一的MathUtils坐标转换
 * 2. 实现基于扫描匹配的位姿估计
 * 3. 改进地图更新逻辑
 */
class LaserSLAM {
private:
    OccupancyMap* map;
    POSE robot_pose;
    Point target_pose;

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

    // ✅ 新增：扫描匹配相关
    struct ScanMatchResult {
        double dx;
        double dy;
        double dtheta;
        double score;
        bool valid;
    };

    // 保存上一帧激光数据用于扫描匹配
    INT16 last_laser_data[360];
    bool has_last_scan;

    // ✅ 修复：删除重复的坐标转换函数，使用MathUtils
    // （原来的worldToGrid和gridToWorld已删除）

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

            // 障碍物膨胀
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
        }
        else {
            log_odd += LOG_ODD_FREE;
        }

        log_odd = MathUtils::clamp(log_odd, LOG_ODD_MIN, LOG_ODD_MAX);
        double probability = 1.0 / (1.0 + exp(-log_odd));
        map->grid[gy][gx].occupancy = (INT16)(probability * 100.0);

        // ✅ 修复：提高障碍物判定阈值，减少过度标记
        // 原来是 hit > miss + 3，现在改为 hit > miss + 6
        if (hit > miss + 6 && hit >= 4) {
            map->grid[gy][gx].occupancy = 100;
        }
        else if (miss > hit + 3) {  // 放宽自由空间判定
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
                }
                else if (map->grid[i][j].occupancy == 0 && free_count <= 2) {
                    temp_occupancy[i][j] = 50;
                }
                else {
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
                            sum_occupancy += map->grid[i + di][j + dj].occupancy;
                            count++;
                        }
                    }
                    if (count > 0) {
                        map->grid[i][j].occupancy = (INT16)(sum_occupancy / count);
                    }
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
                double time_decay = MathUtils::clamp((double)frame_diff / VISIT_DECAY_FRAMES, 0.0, 1.0);
                double visit_penalty = 1.0 / (1.0 + record.visit_count * (1.0 - time_decay));
                return visit_penalty;
            }
        }
        return 1.0;
    }

    // ✅ 候选目标列表
    std::vector<TargetCandidate> target_candidates;
    const int MIN_OBJECT_SIZE = 1;
    const int MAX_OBJECT_SIZE = 4;
    const int LARGE_OBSTACLE_SIZE = 8;

    void detectCandidateTargets() {
        std::vector<std::vector<bool>> visited(GRID_HEIGHT,
            std::vector<bool>(GRID_WIDTH, false));

        for (int i = 0; i < GRID_HEIGHT; i++) {
            for (int j = 0; j < GRID_WIDTH; j++) {
                if (map->grid[i][j].occupancy >= 70 && !visited[i][j]) {
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

                        int dirs[4][2] = { {-1,0}, {1,0}, {0,-1}, {0,1} };
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

                    if (cluster_size >= MIN_OBJECT_SIZE &&
                        cluster_size <= MAX_OBJECT_SIZE) {
                        double center_y = 0, center_x = 0;
                        for (const auto& cell : cluster) {
                            center_y += cell.first;
                            center_x += cell.second;
                        }
                        center_y /= cluster_size;
                        center_x /= cluster_size;

                        INT16 center_gx = (INT16)(center_x + 0.5);
                        INT16 center_gy = (INT16)(center_y + 0.5);

                        double world_x, world_y;
                        MathUtils::gridToWorld(center_gx, center_gy, world_x, world_y);

                        double confidence = evaluateObjectAsTarget(
                            center_gx, center_gy, cluster_size);

                        updateTargetCandidate(world_x, world_y,
                            center_gx, center_gy, cluster_size, confidence);
                    }
                }
            }
        }

        cleanupOldCandidates();
    }

    double evaluateObjectAsTarget(INT16 gx, INT16 gy, int size) {
        double score = 0.0;

        double size_score = 1.0 - (double)(size - MIN_OBJECT_SIZE) /
            (double)(MAX_OBJECT_SIZE - MIN_OBJECT_SIZE);
        size_score = MathUtils::clamp(size_score, 0.0, 1.0);

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

        INT16 robot_gx, robot_gy;
        MathUtils::worldToGrid(robot_pose.coor_x, robot_pose.coor_y, robot_gx, robot_gy);
        double dist = sqrt(pow(gx - robot_gx, 2) + pow(gy - robot_gy, 2));
        double distance_score = (dist > 5.0) ? 1.0 : 0.5;

        score = 0.5 * size_score + 0.3 * isolation_score + 0.2 * distance_score;

        return score;
    }

    void updateTargetCandidate(double world_x, double world_y,
        INT16 gx, INT16 gy, int size, double confidence) {
        bool found = false;
        for (auto& candidate : target_candidates) {
            double dx = candidate.position.coor_x - world_x;
            double dy = candidate.position.coor_y - world_y;
            double distance = sqrt(dx * dx + dy * dy);

            if (distance < 20.0) {
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

    void cleanupOldCandidates() {
        auto it = target_candidates.begin();
        while (it != target_candidates.end()) {
            if (current_frame - it->last_seen_frame > 100) {
                it = target_candidates.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    void markCandidatesAsNavigable() {
        for (const auto& candidate : target_candidates) {
            if (candidate.confidence > 0.5) {
                INT16 gx = candidate.grid_x;
                INT16 gy = candidate.grid_y;

                for (int di = -2; di <= 2; di++) {
                    for (int dj = -2; dj <= 2; dj++) {
                        int ni = gy + di;
                        int nj = gx + dj;
                        if (ni >= 0 && ni < GRID_HEIGHT &&
                            nj >= 0 && nj < GRID_WIDTH) {
                            if (map->grid[ni][nj].occupancy >= 70) {
                                map->grid[ni][nj].occupancy = 40;
                            }
                        }
                    }
                }
            }
        }
    }

    // ✅ 新增：简化的扫描匹配算法
    ScanMatchResult performScanMatching(INT16* current_scan, POSE current_pose) {
        ScanMatchResult result;
        result.dx = 0;
        result.dy = 0;
        result.dtheta = 0;
        result.score = 0;
        result.valid = false;

        if (!has_last_scan) {
            return result;
        }

        // 简化的基于地图的扫描匹配
        // 计算当前扫描与地图的匹配度
        double best_score = -1e10;
        double best_dx = 0, best_dy = 0, best_dtheta = 0;

        // 搜索范围
        const double SEARCH_RANGE = 10.0;  // cm
        const double SEARCH_ANGLE = 0.1;   // rad (~5.7度)
        const int SEARCH_STEPS = 5;

        for (int ix = -SEARCH_STEPS; ix <= SEARCH_STEPS; ix++) {
            for (int iy = -SEARCH_STEPS; iy <= SEARCH_STEPS; iy++) {
                for (int ia = -SEARCH_STEPS; ia <= SEARCH_STEPS; ia++) {
                    double test_dx = ix * SEARCH_RANGE / SEARCH_STEPS;
                    double test_dy = iy * SEARCH_RANGE / SEARCH_STEPS;
                    double test_dtheta = ia * SEARCH_ANGLE / SEARCH_STEPS;

                    double score = evaluateScanPose(current_scan,
                        current_pose.coor_x + test_dx,
                        current_pose.coor_y + test_dy,
                        current_pose.coor_ori + test_dtheta);

                    if (score > best_score) {
                        best_score = score;
                        best_dx = test_dx;
                        best_dy = test_dy;
                        best_dtheta = test_dtheta;
                    }
                }
            }
        }

        // 只有当匹配分数足够高时才认为有效
        if (best_score > 0.5) {
            result.dx = best_dx;
            result.dy = best_dy;
            result.dtheta = best_dtheta;
            result.score = best_score;
            result.valid = true;
        }

        return result;
    }

    // 评估扫描在给定位姿下与地图的匹配程度
    double evaluateScanPose(INT16* scan, double x, double y, double theta) {
        int match_count = 0;
        int total_count = 0;

        for (int i = 0; i < 360; i += 5) {  // 每5度采样一次
            if (scan[i] <= 0 || scan[i] > 3000) continue;

            double angle = theta + i * PI / 180.0;
            double obs_x = x + scan[i] * cos(angle);
            double obs_y = y + scan[i] * sin(angle);

            INT16 obs_gx, obs_gy;
            MathUtils::worldToGrid(obs_x, obs_y, obs_gx, obs_gy);

            if (obs_gx >= 0 && obs_gx < GRID_WIDTH &&
                obs_gy >= 0 && obs_gy < GRID_HEIGHT) {
                total_count++;
                // 检查该点附近是否有障碍物
                for (int di = -1; di <= 1; di++) {
                    for (int dj = -1; dj <= 1; dj++) {
                        int ni = obs_gy + di;
                        int nj = obs_gx + dj;
                        if (ni >= 0 && ni < GRID_HEIGHT &&
                            nj >= 0 && nj < GRID_WIDTH) {
                            if (map->grid[ni][nj].occupancy >= 70) {
                                match_count++;
                                goto next_ray;
                            }
                        }
                    }
                }
            }
        next_ray:;
        }

        return (total_count > 0) ? (double)match_count / total_count : 0.0;
    }

public:
    LaserSLAM() {
        std::cout << "Creating LaserSLAM with map size: " << GRID_WIDTH << "x" << GRID_HEIGHT << std::endl;
        try {
            map = new OccupancyMap();
            current_frame = 0;
            visit_history.reserve(500);
            has_last_scan = false;
            memset(last_laser_data, 0, sizeof(last_laser_data));
            std::cout << "LaserSLAM initialized successfully (Fixed Version)" << std::endl;
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
        MathUtils::worldToGrid(cur_pose.coor_x, cur_pose.coor_y, robot_gx, robot_gy);
        recordVisit(robot_gx, robot_gy);

        // 清除机器人当前位置周围 7×7 区域
        for (int di = -3; di <= 3; di++) {
            for (int dj = -3; dj <= 3; dj++) {
                int ni = robot_gy + di;
                int nj = robot_gx + dj;
                if (ni >= 0 && ni < GRID_HEIGHT && nj >= 0 && nj < GRID_WIDTH) {
                    map->grid[ni][nj].occupancy = 0;
                    map->grid[ni][nj].hit_count = 0;
                    map->grid[ni][nj].miss_count = 10;
                }
            }
        }

        for (int i = 0; i < 360; i += 2) {
            if (laser_data[i] > 0 && laser_data[i] < 3000) {
                double angle = cur_pose.coor_ori + (i * PI / 180.0);
                double obs_x = cur_pose.coor_x + laser_data[i] * cos(angle);
                double obs_y = cur_pose.coor_y + laser_data[i] * sin(angle);

                INT16 obs_gx, obs_gy;
                MathUtils::worldToGrid(obs_x, obs_y, obs_gx, obs_gy);

                int dx = abs(obs_gx - robot_gx);
                int dy = abs(obs_gy - robot_gy);
                if (dx <= 3 && dy <= 3) {
                    continue;
                }

                rayTraceWithProbability(robot_gx, robot_gy, obs_gx, obs_gy);
            }
        }

        if (current_frame % 20 == 0) {
            smoothMap();
        }

        if (current_frame % 5 == 0) {
            detectCandidateTargets();
            markCandidatesAsNavigable();
        }

        // 保存当前扫描用于下一次匹配
        memcpy(last_laser_data, laser_data, 360 * sizeof(INT16));
        has_last_scan = true;
    }

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
                MathUtils::gridToWorld(j, i, wx, wy);
                double dx = target.coor_x - wx;
                double dy = target.coor_y - wy;
                double dist_to_target = sqrt(dx * dx + dy * dy);

                double target_bonus = 0;
                if (unexplored_ratio > 0.3 && dist_to_target < 300) {
                    target_bonus = 0.2 * (1.0 - dist_to_target / 300.0);
                }

                map->openness[i][j] = (free_ratio + 0.5 * unexplored_ratio + target_bonus) * visit_penalty;
            }
        }
    }

    bool isDirectPathClear(Point target, INT16* laser_data, double safe_margin = 80.0) {
        double dx = target.coor_x - robot_pose.coor_x;
        double dy = target.coor_y - robot_pose.coor_y;
        double distance = sqrt(dx * dx + dy * dy);

        if (distance < 30.0) return true;

        double target_angle = atan2(dy, dx) - robot_pose.coor_ori;
        target_angle = MathUtils::normalizeAngle(target_angle);

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

    bool isTargetVisible(Point target, INT16* laser_data) {
        target_pose = target;

        double dx = target.coor_x - robot_pose.coor_x;
        double dy = target.coor_y - robot_pose.coor_y;
        double distance = sqrt(dx * dx + dy * dy);

        if (distance > 1000.0) return false;
        if (distance < 30.0) return true;

        double angle = atan2(dy, dx) - robot_pose.coor_ori;
        angle = MathUtils::normalizeAngle(angle);

        if (fabs(angle) > PI / 3.0) return false;

        int laser_index = (int)(angle * 180.0 / PI + 0.5);
        if (laser_index < 0) laser_index += 360;
        if (laser_index >= 360) laser_index -= 360;

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

        return (total_checks > 0) && ((double)votes / total_checks >= 0.6);
    }

    OccupancyMap& getMap() {
        return *map;
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
        has_last_scan = false;
        std::cout << "Visit history reset" << std::endl;
    }

    std::vector<TargetCandidate> getTargetCandidates() const {
        return target_candidates;
    }

    bool getBestCandidate(Point& candidate_pos, double& confidence) {
        double best_score = 0;
        bool found = false;

        INT16 robot_gx, robot_gy;
        MathUtils::worldToGrid(robot_pose.coor_x, robot_pose.coor_y, robot_gx, robot_gy);

        for (const auto& candidate : target_candidates) {
            int persistence = candidate.last_seen_frame - candidate.first_seen_frame;
            if (persistence < 10) continue;

            double dx = candidate.grid_x - robot_gx;
            double dy = candidate.grid_y - robot_gy;
            double distance = sqrt(dx * dx + dy * dy);

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

    void confirmTargetPhysically(double world_x, double world_y) {
        INT16 detection_gx, detection_gy;
        MathUtils::worldToGrid(world_x, world_y, detection_gx, detection_gy);

        INT16 robot_gx, robot_gy;
        MathUtils::worldToGrid(robot_pose.coor_x, robot_pose.coor_y, robot_gx, robot_gy);

        double dist_to_robot = sqrt(pow(detection_gx - robot_gx, 2) +
            pow(detection_gy - robot_gy, 2));

        if (dist_to_robot < 5.0) {
            std::cout << ">>> Ignored self-detection (too close to robot, dist="
                << (int)(dist_to_robot * MAP_RESOLUTION) << "cm)" << std::endl;
            return;
        }

        static INT16 last_confirmed_gx = -1;
        static INT16 last_confirmed_gy = -1;

        if (abs(detection_gx - last_confirmed_gx) < 3 &&
            abs(detection_gy - last_confirmed_gy) < 3) {
            return;
        }

        for (auto& candidate : target_candidates) {
            double dx = candidate.position.coor_x - world_x;
            double dy = candidate.position.coor_y - world_y;
            double distance = sqrt(dx * dx + dy * dy);

            if (distance < 30.0) {
                candidate.physically_confirmed = true;
                candidate.confidence = 1.0;

                if (last_confirmed_gx != candidate.grid_x ||
                    last_confirmed_gy != candidate.grid_y) {

                    for (int di = -4; di <= 4; di++) {
                        for (int dj = -4; dj <= 4; dj++) {
                            int ni = candidate.grid_y + di;
                            int nj = candidate.grid_x + dj;
                            if (ni >= 0 && ni < GRID_HEIGHT &&
                                nj >= 0 && nj < GRID_WIDTH) {
                                map->grid[ni][nj].occupancy = 0;
                                map->grid[ni][nj].hit_count = 0;
                                map->grid[ni][nj].miss_count = 30;
                            }
                        }
                    }

                    last_confirmed_gx = candidate.grid_x;
                    last_confirmed_gy = candidate.grid_y;

                    std::cout << ">>> Target confirmed at ("
                        << candidate.position.coor_x << ","
                        << candidate.position.coor_y
                        << "), cleared 9x9 region" << std::endl;
                }
                break;
            }
        }
    }

    /**
     * ✅ 修复后的位姿估计 - 实现真正的扫描匹配
     */
    bool getPoseEstimate(POSE& estimated_pose, double& confidence) {
        // 使用扫描匹配来估计位姿
        if (has_last_scan && current_frame > 50) {
            ScanMatchResult match = performScanMatching(last_laser_data, robot_pose);

            if (match.valid) {
                // 应用扫描匹配的校正
                estimated_pose.coor_x = (INT16)(robot_pose.coor_x + match.dx);
                estimated_pose.coor_y = (INT16)(robot_pose.coor_y + match.dy);
                estimated_pose.coor_ori = MathUtils::normalizeAngle(robot_pose.coor_ori + match.dtheta);
                confidence = match.score;

                if (match.score > 0.7) {
                    std::cout << "[SLAM] Scan match: dx=" << match.dx
                        << " dy=" << match.dy
                        << " dtheta=" << MathUtils::radToDeg(match.dtheta)
                        << " score=" << match.score << std::endl;
                }

                return true;
            }
        }

        // 回退：基于地图质量的置信度估计
        estimated_pose = robot_pose;

        INT16 robot_gx, robot_gy;
        MathUtils::worldToGrid(robot_pose.coor_x, robot_pose.coor_y, robot_gx, robot_gy);

        int explored_count = 0;
        int total_count = 0;
        int radius = 20;

        for (int di = -radius; di <= radius; di++) {
            for (int dj = -radius; dj <= radius; dj++) {
                int ni = robot_gy + di;
                int nj = robot_gx + dj;
                if (ni >= 0 && ni < GRID_HEIGHT && nj >= 0 && nj < GRID_WIDTH) {
                    total_count++;
                    if (map->grid[ni][nj].explored > 0) {
                        explored_count++;
                    }
                }
            }
        }

        double exploration_ratio = (total_count > 0) ?
            (double)explored_count / total_count : 0.0;

        confidence = 0.3 + 0.4 * exploration_ratio;

        if (exploration_ratio < 0.2) {
            confidence = 0.2;
        }

        if (current_frame < 50) {
            return false;
        }

        return true;
    }

    int getCurrentFrame() const {
        return current_frame;
    }

    POSE getRobotPose() const {
        return robot_pose;
    }
};

#endif // LASER_SLAM_H