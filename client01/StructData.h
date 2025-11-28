#ifndef STRUCTDATA_H
#define STRUCTDATA_H

#include <vector>

// ============================================================================
// 常量定义
// ============================================================================

#define PI 3.1415926

// 地图配置
#define MAP_WIDTH 1000      // 地图宽度 (cm)
#define MAP_HEIGHT 1000     // 地图高度 (cm)
#define MAP_RESOLUTION 5    // 地图分辨率 (cm/grid)
#define GRID_WIDTH (MAP_WIDTH / MAP_RESOLUTION)   // 200
#define GRID_HEIGHT (MAP_HEIGHT / MAP_RESOLUTION) // 200

// ✅ 数据类型定义（确保跨平台兼容）
#ifndef INT16
typedef short INT16;
#endif

#ifndef BYTE
typedef unsigned char BYTE;
#endif

// ============================================================================
// 基础数据结构
// ============================================================================

/**
 * @brief 机器人位姿
 */
typedef struct {
    INT16 coor_x;      // X坐标 (cm)
    INT16 coor_y;      // Y坐标 (cm)
    double coor_ori;   // 朝向 (rad)，0指向+X，逆时针为正
} POSE;

/**
 * @brief 二维点
 */
typedef struct {
    INT16 coor_x;      // X坐标 (cm)
    INT16 coor_y;      // Y坐标 (cm)
} Point;

// ============================================================================
// 栅格地图相关
// ============================================================================

/**
 * @brief 栅格地图单元
 */
typedef struct {
    INT16 occupancy;   // 占用概率: -1未知, 0空闲, 100占用
    BYTE explored;     // 是否已探索
    BYTE hit_count;    // 激光命中次数
    BYTE miss_count;   // 激光穿过次数
} GridCell;

/**
 * @brief 占用栅格地图
 */
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

    /**
     * @brief 重置地图
     */
    void reset() {
        for (int i = 0; i < GRID_HEIGHT; i++) {
            for (int j = 0; j < GRID_WIDTH; j++) {
                grid[i][j].occupancy = -1;
                grid[i][j].explored = 0;
                grid[i][j].hit_count = 0;
                grid[i][j].miss_count = 0;
                openness[i][j] = 0.0;
            }
        }
    }
};

// ============================================================================
// A*路径规划相关
// ============================================================================

/**
 * @brief A*搜索节点
 */
typedef struct {
    INT16 x;
    INT16 y;
    double f;          // f = g + h
    double g;          // 从起点到当前节点的代价
    double h;          // 启发式估计（到目标的代价）
    INT16 parent_x;
    INT16 parent_y;
} AStarNode;

// ============================================================================
// 通信数据结构
// ============================================================================

/**
 * @brief 服务器到客户端的数据包
 */
typedef struct {
    INT16 Timestamp;           // 时间戳
    INT16 Runstatus;           // 运行状态
    BYTE task_finish;          // 任务是否完成
    BYTE detect_object;        // 是否检测到目标
    BYTE collision;            // 是否发生碰撞
    INT16 obstacle[360];       // 360度激光数据 (cm)
    POSE initial_rpose;        // 机器人初始位姿
    Point initial_dpose;       // 目标初始位置
    double target_angle;       // 目标角度
} S2CINFO;

/**
 * @brief 客户端到服务器的数据包
 */
typedef struct {
    INT16 Timestamp;           // 时间戳
    INT16 Runstatus;           // 运行状态
    double tra_vel;            // 线速度 (cm/s)
    double rot_vel;            // 角速度 (rad/s)
    POSE cur_rpose;            // 当前机器人位姿
    Point Traj[100];           // 规划路径
} C2SINFO;

// ============================================================================
// 目标检测相关
// ============================================================================

/**
 * @brief 候选目标点
 */
typedef struct {
    Point position;            // 世界坐标
    INT16 grid_x, grid_y;      // 栅格坐标
    int size_cells;            // 占据的栅格数
    double confidence;         // 置信度 [0, 1]
    int first_seen_frame;      // 首次检测帧
    int last_seen_frame;       // 最后检测帧
    bool is_small_object;      // 是否为小物体
    bool physically_confirmed; // 是否物理确认
} TargetCandidate;

#endif // STRUCTDATA_H