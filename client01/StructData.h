#ifndef STRUCTDATA_H
#define STRUCTDATA_H

#include <vector>

#define PI 3.1415926

// 地图配置
#define MAP_WIDTH 1000      // 地图宽度 (cm)
#define MAP_HEIGHT 1000     // 地图高度 (cm)
#define MAP_RESOLUTION 5    // 地图分辨率 (cm/grid)
#define GRID_WIDTH (MAP_WIDTH / MAP_RESOLUTION)   // 200x200
#define GRID_HEIGHT (MAP_HEIGHT / MAP_RESOLUTION)

typedef struct{
	INT16 coor_x;
	INT16 coor_y;
	double coor_ori;
}POSE;

typedef struct{
	INT16 coor_x;
	INT16 coor_y;
}Point;

// 栅格地图单元 - 保持原有结构
typedef struct{
	INT16 occupancy;  // 占用概率: -1未知, 0空闲, 100占用
	BYTE explored;    // 是否已探索
	BYTE hit_count;   // 激光命中次数
	BYTE miss_count;  // 激光穿过次数
}GridCell;

// 使用动态分配的地图结构
class OccupancyMap {
public:
	std::vector<std::vector<GridCell>> grid;
	std::vector<std::vector<double>> openness;
	
	OccupancyMap() {
		grid.resize(GRID_HEIGHT, std::vector<GridCell>(GRID_WIDTH));
		openness.resize(GRID_HEIGHT, std::vector<double>(GRID_WIDTH, 0.0));
		
		for (int i = 0; i < GRID_HEIGHT; i++) {
			for (int j = 0; j < GRID_WIDTH; j++) {
				grid[i][j].occupancy = -1;
				grid[i][j].explored = 0;
				grid[i][j].hit_count = 0;
				grid[i][j].miss_count = 0;
			}
		}
	}
};

// A*节点
typedef struct{
	INT16 x;
	INT16 y;
	double f;
	double g;
	double h;
	INT16 parent_x;
	INT16 parent_y;
}AStarNode;

typedef struct{
	INT16 Timestamp;
	INT16 Runstatus;
	BYTE task_finish;
	BYTE detect_object;
	BYTE collision;
	INT16 obstacle[360];
	POSE initial_rpose;
	Point initial_dpose;
	double target_angle;
}S2CINFO;

// ✅ 恢复原有的 C2SINFO 结构,不添加新字段
typedef struct{
	INT16 Timestamp;
	INT16 Runstatus;
	double tra_vel;
	double rot_vel;
	POSE cur_rpose;
	Point Traj[100];
}C2SINFO;

// ✅ 候选目标点结构
typedef struct {
	Point position;          // 世界坐标
	INT16 grid_x, grid_y;   // 栅格坐标
	int size_cells;         // 占据的栅格数
	double confidence;      // 置信度 [0, 1]
	int first_seen_frame;   // 首次检测帧
	int last_seen_frame;    // 最后检测帧
	bool is_small_object;   // 是否为小物体
	bool physically_confirmed; // 是否物理确认
} TargetCandidate;

#endif