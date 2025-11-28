/**
 * @file Plan_Client.cpp
 * @brief 机器人导航主控制程序 - 简化版
 *
 * 使用基于命令速度的位姿跟踪（不使用 ICP）
 * ICP 在低速场景下不稳定，暂时禁用
 */

#include <iostream>
#include <iomanip> 
#include <stdlib.h>
#include <stdio.h>
#include <atlstr.h>
#include "list"
#include "vector"
#include "winsock2.h"
#include "StructData.h"
#include "LaserSLAM.h"
#include "ObstacleAvoidance.h"
#include "OpennessPlanner.h"
#include "AStarPlanner.h"
#include "MathUtils.h"
#include "PoseTracker.h"
 // 注意：不需要包含 LaserOdometry.h

#pragma comment(lib,"ws2_32.lib")
using namespace std;

// ============================================================================
// 全局变量
// ============================================================================

SOCKET sockClient;
S2CINFO S2Cdata;
C2SINFO C2Sdata;
POSE Initial_rPos;
POSE Cur_rPos;
POSE Cur_dPos;
INT16 obstacle[360];
int Runstatus = 1;
double cur_tra_vel = 0;
double cur_rot_vel = 0;

// 系统组件
LaserSLAM* slam_system = nullptr;
ObstacleAvoidance* obstacle_avoidance = nullptr;
OpennessPlanner* openness_planner = nullptr;
AStarPlanner* astar_planner = nullptr;
PoseTracker* pose_tracker = nullptr;
bool slam_ready = false;

// ============================================================================
// 初始化函数
// ============================================================================

void Initialition() {
    cout << "====== Initializing System (Command Velocity Version) ======" << endl;
    memset(&S2Cdata, 0, sizeof(S2CINFO));
    memset(&C2Sdata, 0, sizeof(C2SINFO));
    memset(&Initial_rPos, 0, sizeof(POSE));
    memset(&Cur_rPos, 0, sizeof(POSE));
    memset(&Cur_dPos, 0, sizeof(POSE));
    memset(obstacle, 0, 360 * sizeof(INT16));

    try {
        cout << "Creating SLAM system (for mapping)..." << endl;
        slam_system = new LaserSLAM();

        cout << "Creating Obstacle Avoidance..." << endl;
        obstacle_avoidance = new ObstacleAvoidance();

        cout << "Creating Openness Planner..." << endl;
        openness_planner = new OpennessPlanner(slam_system, obstacle_avoidance);

        cout << "Creating A* Planner..." << endl;
        astar_planner = new AStarPlanner(slam_system, obstacle_avoidance);

        cout << "Creating Pose Tracker (Command Velocity Based)..." << endl;
        pose_tracker = new PoseTracker();

        // ========== 关键参数调整 ==========
        // vel_scale: 如果位姿滞后（绿色轨迹太短），增大这个值
        // omega_scale: 如果角度估计错误，调整这个值
        pose_tracker->setScaleFactors(3.5, 1.2);

        slam_ready = true;
        cout << "====== All systems initialized ======" << endl;
    }
    catch (const std::exception& e) {
        cout << "Failed to initialize: " << e.what() << endl;
        slam_ready = false;
    }
}

CString GetExePath() {
    CString strExePath;
    CString strPath;
    GetModuleFileName(NULL, strPath.GetBufferSetLength(MAX_PATH + 1), MAX_PATH + 1);
    int nPos = strPath.ReverseFind(_T('\\'));
    strExePath = strPath.Left(nPos + 1);
    return strExePath;
}

// ============================================================================
// 接收线程
// ============================================================================

DWORD WINAPI Recv_Thre(LPVOID lpParameter) {
    cout << "Receive thread started" << endl;
    char recvBuf[800];
    char Sendbuff[600];
    int frame_count = 0;
    Point astar_path[100];
    int path_length = 0;
    bool using_astar = false;
    int frames_since_replan = 0;
    bool first_frame_done = false;
    bool last_collision_state = false;

    POSE Last_Initial_rPos;
    memset(&Last_Initial_rPos, 0, sizeof(POSE));
    bool pose_initialized = false;

    double nearest_obstacle_dist = 9999.0;
    bool is_avoiding_obstacle = false;

    int pose_update_count = 0;
    double total_pose_drift = 0;

    int astar_fail_count = 0;
    const int ASTAR_FAIL_THRESHOLD = 5;

    while (Runstatus > 0) {
        try {
            memset(recvBuf, 0, sizeof(recvBuf));
            int recv_len = recv(sockClient, recvBuf, sizeof(recvBuf), 0);

            if (recv_len > 0) {
                memset(&S2Cdata, 0, sizeof(S2CINFO));
                memset(&C2Sdata, 0, sizeof(C2SINFO));
                memcpy(&S2Cdata, recvBuf, sizeof(S2CINFO));

                if (S2Cdata.Runstatus == 0) {
                    cout << "Shutdown signal received" << endl;
                    Runstatus = 0;
                    break;
                }

                // 检测场景切换
                bool pose_changed = false;
                if (pose_initialized && first_frame_done) {
                    POSE current_initial_pose = S2Cdata.initial_rpose;

                    if (MathUtils::isPoseSignificantlyDifferent(
                        current_initial_pose, Last_Initial_rPos, 30.0, 0.3)) {
                        pose_changed = true;
                        cout << "====== POSE CHANGE DETECTED! ======" << endl;

                        first_frame_done = false;
                        S2Cdata.Timestamp = 0;

                        memcpy(&Cur_rPos, &current_initial_pose, sizeof(POSE));
                        memcpy(&Initial_rPos, &current_initial_pose, sizeof(POSE));
                        memcpy(&Last_Initial_rPos, &current_initial_pose, sizeof(POSE));

                        if (pose_tracker) {
                            pose_tracker->reset(current_initial_pose);
                        }
                    }
                }

                // 碰撞重置
                bool current_collision = (S2Cdata.collision > 0);
                if (current_collision && !last_collision_state) {
                    cout << "====== COLLISION DETECTED - FULL RESET ======" << endl;
                    cur_tra_vel = 0;
                    cur_rot_vel = 0;

                    if (obstacle_avoidance) obstacle_avoidance->reset();
                    if (openness_planner) openness_planner->reset();
                    if (astar_planner) astar_planner->reset();

                    using_astar = false;
                    path_length = 0;
                    frames_since_replan = 0;
                }
                last_collision_state = current_collision;

                // 第一帧初始化
                if (S2Cdata.Timestamp == 0 && !first_frame_done) {
                    cout << "====== NEW SCENE ======" << endl;

                    memcpy(&Initial_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
                    memcpy(&Cur_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
                    memcpy(&Last_Initial_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
                    pose_initialized = true;

                    if (pose_tracker) {
                        pose_tracker->initialize(Cur_rPos);
                    }

                    Cur_dPos.coor_x = S2Cdata.initial_dpose.coor_x;
                    Cur_dPos.coor_y = S2Cdata.initial_dpose.coor_y;
                    Cur_dPos.coor_ori = 0;

                    cout << "Robot pos: (" << Cur_rPos.coor_x << ", "
                        << Cur_rPos.coor_y << "), ori: "
                        << MathUtils::radToDeg(Cur_rPos.coor_ori) << " deg" << endl;
                    cout << "Target pos: (" << Cur_dPos.coor_x << ", "
                        << Cur_dPos.coor_y << ")" << endl;

                    double dist_to_target = MathUtils::calculateDistance(
                        Cur_rPos.coor_x, Cur_rPos.coor_y,
                        Cur_dPos.coor_x, Cur_dPos.coor_y);
                    cout << "Distance to target: " << (int)dist_to_target << " cm" << endl;

                    using_astar = false;
                    path_length = 0;
                    frames_since_replan = 0;
                    frame_count = 0;
                    first_frame_done = true;
                    pose_update_count = 0;
                    total_pose_drift = 0;

                    if (obstacle_avoidance) obstacle_avoidance->reset();
                    if (openness_planner) openness_planner->reset();
                    if (astar_planner) astar_planner->reset();
                    if (slam_system) slam_system->resetVisitHistory();

                    cur_tra_vel = 0;
                    cur_rot_vel = 0;
                }

                C2Sdata.Timestamp = S2Cdata.Timestamp;
                C2Sdata.Runstatus = S2Cdata.Runstatus;
                memcpy(obstacle, S2Cdata.obstacle, 360 * sizeof(INT16));

                // 障碍物状态
                nearest_obstacle_dist = 9999.0;
                for (int i = 0; i < 360; i++) {
                    if (obstacle[i] > 0 && obstacle[i] < 5000) {
                        if (obstacle[i] < nearest_obstacle_dist) {
                            nearest_obstacle_dist = obstacle[i];
                        }
                    }
                }
                is_avoiding_obstacle = (nearest_obstacle_dist < 100.0);

                // ============================================================
                // 位姿更新（基于命令速度）
                // ============================================================
                if (frame_count > 0 && first_frame_done && pose_tracker) {
                    POSE prev_pose = Cur_rPos;

                    // 使用命令速度更新位姿
                    PoseTracker::UpdateResult result = pose_tracker->update(
                        obstacle,       // 保留接口，但不使用
                        cur_tra_vel,    // 命令线速度
                        cur_rot_vel);   // 命令角速度

                    Cur_rPos = result.integer_pose;

                    // 边界检查
                    Cur_rPos.coor_x = MathUtils::clamp(Cur_rPos.coor_x, (INT16)0, (INT16)(MAP_WIDTH - 1));
                    Cur_rPos.coor_y = MathUtils::clamp(Cur_rPos.coor_y, (INT16)0, (INT16)(MAP_HEIGHT - 1));

                    // 统计
                    double pose_change = MathUtils::calculateDistance(
                        prev_pose.coor_x, prev_pose.coor_y,
                        Cur_rPos.coor_x, Cur_rPos.coor_y);
                    total_pose_drift += pose_change;
                    pose_update_count++;

                    // 每10帧打印状态
                    if (frame_count % 10 == 0) {
                        pose_tracker->printStatus(result);

                        cout << "      Cmd: v=" << fixed << setprecision(1) << cur_tra_vel
                            << " w=" << cur_rot_vel << endl;

                        cout << "      Total: traveled=" << (int)pose_tracker->getTotalDistanceTraveled()
                            << " cm" << endl;
                    }

                    if (result.is_stuck) {
                        static int stuck_warn_counter = 0;
                        if (++stuck_warn_counter % 20 == 1) {
                            cout << "[WARN] Pose tracker detects stuck!" << endl;
                        }
                    }
                }

                // ============================================================
                // 导航决策
                // ============================================================
                if (slam_ready && slam_system && obstacle_avoidance && openness_planner && astar_planner) {
                    obstacle_avoidance->detectObstacles(obstacle, Cur_rPos.coor_ori);

                    // SLAM地图更新
                    if (frame_count % 10 == 0) {
                        slam_system->updateMap(Cur_rPos, obstacle);

                        if (frame_count % 30 == 0) {
                            Point target_point;
                            target_point.coor_x = Cur_dPos.coor_x;
                            target_point.coor_y = Cur_dPos.coor_y;
                            slam_system->updateOpenness(target_point);
                        }
                    }

                    Point target_point;
                    target_point.coor_x = Cur_dPos.coor_x;
                    target_point.coor_y = Cur_dPos.coor_y;

                    double dist_to_target = MathUtils::calculateDistance(
                        Cur_rPos.coor_x, Cur_rPos.coor_y,
                        target_point.coor_x, target_point.coor_y);

                    double target_angle = MathUtils::calculateBearing(
                        Cur_rPos.coor_x, Cur_rPos.coor_y,
                        target_point.coor_x, target_point.coor_y);

                    bool object_detected = (S2Cdata.detect_object > 0);
                    bool task_finished = (S2Cdata.task_finish > 0);

                    double planned_vel = 0;
                    double planned_rot = 0;

                    frames_since_replan++;

                    if (task_finished) {
                        cout << "====== TASK FINISHED! ======" << endl;
                        cur_tra_vel = 0;
                        cur_rot_vel = 0;
                        first_frame_done = false;
                    }
                    else if (object_detected) {
                        bool should_try_astar = (astar_fail_count < ASTAR_FAIL_THRESHOLD);

                        if (should_try_astar && (!using_astar || frames_since_replan > 25 || pose_changed)) {
                            cout << ">> [A* MODE] Target detected, dist=" << (int)dist_to_target << "cm" << endl;

                            if (pose_changed) {
                                astar_fail_count = 0;
                            }

                            if (astar_planner->planPath(Cur_rPos, target_point, astar_path, path_length)) {
                                using_astar = true;
                                frames_since_replan = 0;
                                astar_fail_count = 0;

                                for (int i = 0; i < (std::min)(path_length, 100); i++) {
                                    C2Sdata.Traj[i] = astar_path[i];
                                }

                                cout << "   [SUCCESS] Path planned: " << path_length << " waypoints" << endl;
                            }
                            else {
                                using_astar = false;
                                astar_fail_count++;
                            }
                        }
                        else if (!should_try_astar) {
                            using_astar = false;
                            frames_since_replan++;

                            if (frames_since_replan > 50) {
                                astar_fail_count = 0;
                                frames_since_replan = 0;
                            }
                        }

                        if (using_astar && path_length > 0) {
                            astar_planner->followPath(Cur_rPos, astar_path, path_length, obstacle,
                                planned_vel, planned_rot);

                            obstacle_avoidance->computeAvoidanceVelocity(planned_vel, planned_rot,
                                cur_tra_vel, cur_rot_vel, obstacle, Cur_rPos.coor_ori);
                        }
                        else {
                            double angle_error = MathUtils::angleDifference(target_angle, Cur_rPos.coor_ori);

                            if (fabs(angle_error) > 0.6) {
                                planned_vel = 18.0;
                                planned_rot = (angle_error > 0) ? 0.5 : -0.5;
                            }
                            else if (fabs(angle_error) > 0.3) {
                                planned_vel = 30.0;
                                planned_rot = angle_error * 1.2;
                            }
                            else {
                                planned_vel = (dist_to_target > 100.0) ? 45.0 :
                                    (dist_to_target > 50.0) ? 35.0 : 25.0;
                                planned_rot = angle_error * 0.8;
                            }

                            obstacle_avoidance->computeAvoidanceVelocity(planned_vel, planned_rot,
                                cur_tra_vel, cur_rot_vel, obstacle, Cur_rPos.coor_ori);
                        }
                    }
                    else {
                        if (using_astar) {
                            cout << ">> [EXPLORE] Target lost, switching to exploration" << endl;
                            using_astar = false;
                            path_length = 0;
                        }

                        openness_planner->computeMotionDirection(Cur_rPos, target_point, obstacle,
                            planned_vel, planned_rot, object_detected);

                        obstacle_avoidance->computeAvoidanceVelocity(planned_vel, planned_rot,
                            cur_tra_vel, cur_rot_vel, obstacle, Cur_rPos.coor_ori);
                    }

                    openness_planner->updateMotionState(cur_tra_vel, cur_rot_vel, 0.03);
                }
                else {
                    cur_tra_vel = 0;
                    cur_rot_vel = 0;
                }

                frame_count++;

                C2Sdata.cur_rpose = Cur_rPos;
                C2Sdata.tra_vel = cur_tra_vel;
                C2Sdata.rot_vel = cur_rot_vel;

                memset(Sendbuff, 0, sizeof(Sendbuff));
                memcpy(Sendbuff, &C2Sdata, sizeof(C2SINFO));
                send(sockClient, Sendbuff, sizeof(Sendbuff), 0);
                Sleep(30);
            }
            else if (recv_len == 0) {
                cout << "Server closed connection" << endl;
                Runstatus = 0;
                break;
            }
            else {
                Sleep(30);
            }
        }
        catch (const std::exception& e) {
            cout << "Error: " << e.what() << endl;
            Sleep(100);
        }
    }

    cout << "Receive thread exiting" << endl;
    closesocket(sockClient);
    return 0;
}

// ============================================================================
// 主函数
// ============================================================================

int main() {
    system("chcp 65001");

    cout << "=== SLAM Navigation System (Command Velocity Version) ===" << endl;
    cout << "Press Ctrl+C to exit" << endl;
    cout << endl;
    cout << "Pose Tracking: Command velocity integration" << endl;
    cout << "  - vel_scale=3.5 (adjust if pose lags)" << endl;
    cout << "  - omega_scale=1.2" << endl;
    cout << endl;

    try {
        cout << "Starting server..." << endl;
        CString path = GetExePath() + "Plan_Server.exe";
        ShellExecute(NULL, NULL, path, NULL, NULL, SW_SHOW);
        Sleep(1000);

        Initialition();

        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            cout << "Winsock failed" << endl;
            system("pause");
            return 0;
        }

        SOCKADDR_IN addrSrv;
        addrSrv.sin_family = AF_INET;
        addrSrv.sin_port = htons(8888);
        addrSrv.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
        sockClient = socket(AF_INET, SOCK_STREAM, 0);

        if (SOCKET_ERROR == sockClient) {
            cout << "Socket error" << endl;
            system("pause");
            WSACleanup();
            return 0;
        }

        int retry = 0;
        while (connect(sockClient, (struct sockaddr*)&addrSrv, sizeof(addrSrv)) == INVALID_SOCKET) {
            if (++retry >= 5) {
                cout << "Connection failed" << endl;
                system("pause");
                closesocket(sockClient);
                WSACleanup();
                return 0;
            }
            Sleep(1000);
        }
        cout << "Connected to server" << endl;

        HANDLE hThreadRecv = CreateThread(NULL, 0, Recv_Thre, 0, 0, NULL);
        if (NULL == hThreadRecv) {
            cout << "Thread creation failed" << endl;
            system("pause");
            return 0;
        }

        cout << "=== System Running ===" << endl;

        while (Runstatus > 0) {
            Sleep(100);
        }

        WaitForSingleObject(hThreadRecv, 2000);
        CloseHandle(hThreadRecv);

        if (pose_tracker) delete pose_tracker;
        if (astar_planner) delete astar_planner;
        if (openness_planner) delete openness_planner;
        if (obstacle_avoidance) delete obstacle_avoidance;
        if (slam_system) delete slam_system;

        WSACleanup();
        cout << "=== Exiting ===" << endl;
        system("pause");
    }
    catch (...) {
        cout << "Fatal error" << endl;
        system("pause");
    }

    return 0;
}