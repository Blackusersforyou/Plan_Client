#include <iostream>
#include <iomanip> 
#include <stdlib.h>
#include <stdio.h>
#include <atlstr.h>
#include "list"
#include "vector"
#include "winsock2.h"
#include "StructData.h"`
#include "LaserSLAM.h"
#include "ObstacleAvoidance.h"
#include "OpennessPlanner.h"
#include "AStarPlanner.h"
#include "MathUtils.h"
#include "PoseTracker.h"  // ✅ 添加位姿跟踪器头文件

#pragma comment(lib,"ws2_32.lib")
using namespace std;

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
PoseTracker* pose_tracker = nullptr;  // ✅ 位姿跟踪器
bool slam_ready = false;

void Initialition(){
	cout << "Initializing..." << endl;
	memset(&S2Cdata, 0, sizeof(S2CINFO));
	memset(&C2Sdata, 0, sizeof(C2SINFO));
	memset(&Initial_rPos, 0, sizeof(POSE));
	memset(&Cur_rPos, 0, sizeof(POSE));
	memset(&Cur_dPos, 0, sizeof(POSE));
	memset(obstacle, 0, 360 * sizeof(INT16));
	
	try {
		cout << "Creating SLAM system..." << endl;
		slam_system = new LaserSLAM();
		
		cout << "Creating Obstacle Avoidance..." << endl;
		obstacle_avoidance = new ObstacleAvoidance();
		
		cout << "Creating Openness Planner..." << endl;
		openness_planner = new OpennessPlanner(slam_system, obstacle_avoidance);
		
		cout << "Creating A* Planner..." << endl;
		astar_planner = new AStarPlanner(slam_system, obstacle_avoidance);
		
		cout << "Creating Pose Tracker..." << endl;
		pose_tracker = new PoseTracker();
		
		slam_ready = true;
		cout << "All systems initialized" << endl;
	}
	catch (const std::exception& e) {
		cout << "Failed to initialize: " << e.what() << endl;
		slam_ready = false;
	}
	
	cout << "Initialization complete" << endl;
}

CString GetExePath()
{
	CString strExePath;
	CString strPath;
	GetModuleFileName(NULL, strPath.GetBufferSetLength(MAX_PATH + 1), MAX_PATH + 1);
	int nPos = strPath.ReverseFind(_T('\\'));
	strExePath = strPath.Left(nPos + 1);
	return strExePath;
}

// ... [接收线程和主函数保持不变] ...

// 在 Recv_Thre 函数中修改位姿更新部分
DWORD WINAPI Recv_Thre(LPVOID lpParameter)
{
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
	
	// ✅ 障碍物状态跟踪
	double nearest_obstacle_dist = 9999.0;
	bool is_avoiding_obstacle = false;
	
	while (Runstatus > 0){
		try {
			memset(recvBuf, 0, sizeof(recvBuf));
			int recv_len = recv(sockClient, recvBuf, sizeof(recvBuf), 0);
			
			if (recv_len > 0){
				memset(&S2Cdata, 0, sizeof(S2CINFO));
				memset(&C2Sdata, 0, sizeof(C2SINFO));
				memcpy(&S2Cdata, recvBuf, sizeof(S2CINFO));
				
				if (S2Cdata.Runstatus == 0){
					cout << "Shutdown signal received" << endl;
					Runstatus = 0;
					break;
				}
				
				// ✅ 检测机器人初始位姿是否发生变化
				bool pose_changed = false;
				if (pose_initialized && first_frame_done) {
					POSE current_initial_pose = S2Cdata.initial_rpose;
					
					if (MathUtils::isPoseSignificantlyDifferent(
						current_initial_pose, Last_Initial_rPos, 30.0, 0.3)) {
						pose_changed = true;
						cout << "====== POSE CHANGE DETECTED! ======" << endl;
						cout << "   Previous: (" << Last_Initial_rPos.coor_x << ", " 
						     << Last_Initial_rPos.coor_y << ", " 
						     << Last_Initial_rPos.coor_ori << ")" << endl;
						cout << "   Current:  (" << current_initial_pose.coor_x << ", " 
						     << current_initial_pose.coor_y << ", " 
						     << current_initial_pose.coor_ori << ")" << endl;
						
						// 触发重新初始化
						first_frame_done = false;
						S2Cdata.Timestamp = 0;
						
						// 立即更新位姿
						memcpy(&Cur_rPos, &current_initial_pose, sizeof(POSE));
						memcpy(&Initial_rPos, &current_initial_pose, sizeof(POSE));
						memcpy(&Last_Initial_rPos, &current_initial_pose, sizeof(POSE));
						
						// ✅ 重置位姿跟踪器
						if (pose_tracker) {
							pose_tracker->reset(current_initial_pose);
						}
						
						cout << "   Position reset to: (" << Cur_rPos.coor_x << ", " 
						     << Cur_rPos.coor_y << ", " << Cur_rPos.coor_ori << ")" << endl;
					}
				}
				
				// 碰撞检测与重置
				bool current_collision = (S2Cdata.collision > 0);
				if (current_collision && !last_collision_state) {
					cout << "====== COLLISION DETECTED - FULL RESET ======" << endl;
					cur_tra_vel = 0;
					cur_rot_vel = 0;
					
					if (obstacle_avoidance) obstacle_avoidance->reset();
					if (openness_planner) openness_planner->reset();
					
					using_astar = false;
					path_length = 0;
					frames_since_replan = 0;
				}
				last_collision_state = current_collision;
				
				// ✅ 第一帧初始化
				if(S2Cdata.Timestamp == 0 && !first_frame_done){
					cout << "====== NEW SCENE ======" << endl;
					cout << "First frame received" << endl;
					
					memcpy(&Initial_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
					memcpy(&Cur_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
					memcpy(&Last_Initial_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
					pose_initialized = true;
					
					// ✅ 初始化位姿跟踪器
					if (pose_tracker) {
						pose_tracker->initialize(Cur_rPos);
					}
					
					Cur_dPos.coor_x = S2Cdata.initial_dpose.coor_x;
					Cur_dPos.coor_y = S2Cdata.initial_dpose.coor_y;
					Cur_dPos.coor_ori = 0;
					
					cout << "Robot pos: (" << Cur_rPos.coor_x << ", " 
					     << Cur_rPos.coor_y << "), ori: " 
					     << (Cur_rPos.coor_ori * 180 / PI) << " deg" << endl;
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
					
					if (obstacle_avoidance) obstacle_avoidance->reset();
					if (openness_planner) openness_planner->reset();
					if (slam_system) slam_system->resetVisitHistory();
					
					cur_tra_vel = 0;
					cur_rot_vel = 0;
				}
				
				C2Sdata.Timestamp = S2Cdata.Timestamp;
				C2Sdata.Runstatus = S2Cdata.Runstatus;
				memcpy(obstacle, S2Cdata.obstacle, 360 * sizeof(INT16));
				
				// ✅ 计算障碍物状态
				nearest_obstacle_dist = 9999.0;
				for (int i = 0; i < 360; i++) {
					if (obstacle[i] > 0 && obstacle[i] < 5000) {
						if (obstacle[i] < nearest_obstacle_dist) {
							nearest_obstacle_dist = obstacle[i];
						}
					}
				}
				is_avoiding_obstacle = (nearest_obstacle_dist < 100.0);
				
				// ✅✅✅ 核心：使用 EKF 位姿跟踪器更新位姿 ✅✅✅
				if (frame_count > 0 && first_frame_done && pose_tracker) {
					POSE prev_pose = Cur_rPos;
					
					// ✅ 激光数据校正（提高SLAM精度）
					PoseTracker::LaserCorrectionResult laser_correction = 
						pose_tracker->correctLaserData(obstacle);
					
					if (laser_correction.correction_applied && frame_count % 100 == 0) {
						cout << "[Laser Correction] " << laser_correction.corrected_count 
						     << " points, avg=" << std::setprecision(2) << laser_correction.avg_correction 
						     << " cm, max=" << laser_correction.max_correction << " cm" << endl;
					}
					
					// 更新位姿
					PoseTracker::UpdateResult result = pose_tracker->update(
						cur_tra_vel, cur_rot_vel,
						is_avoiding_obstacle, nearest_obstacle_dist);
					
					Cur_rPos = result.integer_pose;
					
					// 检测异常
					double pose_change = MathUtils::calculateDistance(
						prev_pose.coor_x, prev_pose.coor_y,
						Cur_rPos.coor_x, Cur_rPos.coor_y);
					double angle_change = fabs(MathUtils::angleDifference(
						Cur_rPos.coor_ori, prev_pose.coor_ori));
					
					if (pose_change > 15.0) {
						static int large_jump_counter = 0;
						if (++large_jump_counter % 5 == 1) {
							cout << "[WARN] Large pose jump: " << (int)pose_change 
							     << " cm | v=" << cur_tra_vel << " w=" << cur_rot_vel << endl;
						}
					}
					
					// 每50帧打印状态
					static int pose_log_counter = 0;
					if (++pose_log_counter % 50 == 0) {
						pose_tracker->printStatus(result, result.coefficients, nearest_obstacle_dist);
						
						cout << "      Motion: move=" << std::fixed << std::setprecision(2) 
						     << pose_change << " cm rot=" << (int)(angle_change * 180 / PI) 
						     << " deg | v=" << cur_tra_vel << " w=" << cur_rot_vel << endl;
						
						cout << "      SLAM: traveled=" 
						     << (int)pose_tracker->getTotalDistanceTraveled() << " cm"
						     << " frames_since_correction=" 
						     << pose_tracker->getFramesSinceLastCorrection() << endl;
					}
					
					if (result.is_stuck) {
						static int stuck_warn_counter = 0;
						if (++stuck_warn_counter % 20 == 1) {
							cout << "[WARN] EKF detects stuck! move=" 
							     << std::fixed << std::setprecision(2) << result.actual_move 
							     << " cm" << endl;
						}
					}
				}
				
				// 障碍物检测和避让
				if (slam_ready && slam_system && obstacle_avoidance && openness_planner && astar_planner) {
					obstacle_avoidance->detectObstacles(obstacle, Cur_rPos.coor_ori);
					
					// ✅✅✅ SLAM 位姿校正（使用 EKF）✅✅✅
					if (pose_tracker && pose_tracker->shouldPerformSLAMCorrection()) {
						POSE slam_estimated_pose;
						double slam_confidence = 0.0;
						
						if (slam_system->getPoseEstimate(slam_estimated_pose, slam_confidence)) {
							PoseTracker::SLAMCorrectionResult correction = 
								pose_tracker->correctWithSLAM(slam_estimated_pose, slam_confidence);
							
							if (correction.correction_applied) {
								PoseTracker::HighPrecisionPose hp = pose_tracker->getHighPrecisionPose();
								Cur_rPos.coor_x = (INT16)(hp.x + 0.5);
								Cur_rPos.coor_y = (INT16)(hp.y + 0.5);
								Cur_rPos.coor_ori = hp.ori;
								
								static int correction_count = 0;
								correction_count++;
								cout << ">>> SLAM Correction #" << correction_count 
								     << " | EKF fusion applied" << endl;
							}
						}
					}
					
					// SLAM地图更新（使用校正后的激光数据）
					if (frame_count % 10 == 0) {
						slam_system->updateMap(Cur_rPos, obstacle);
						
						if (frame_count % 30 == 0) {
							Point target_point;
							target_point.coor_x = Cur_dPos.coor_x;
							target_point.coor_y = Cur_dPos.coor_y;
							slam_system->updateOpenness(target_point);
						}
					}
					
					// 目标点
					Point target_point;
					target_point.coor_x = Cur_dPos.coor_x;
					target_point.coor_y = Cur_dPos.coor_y;
					
					// 计算距离和角度
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

					// 决策1: 任务完成
					if (task_finished) {
						cout << "====== TASK FINISHED! ======" << endl;
						cout << "Target contacted successfully!" << endl;
						cur_tra_vel = 0;
						cur_rot_vel = 0;
						first_frame_done = false;
					}
					// 决策2: 目标已检测 (S2Cdata.detect_object > 0)
					else if (object_detected) {
						if (!using_astar || frames_since_replan > 25 || pose_changed) {
							cout << ">> [A* MODE] Target detected, dist=" << (int)dist_to_target << "cm" << endl;
							
							if (pose_changed) {
								cout << "   [REPLAN] Robot pose changed, replanning..." << endl;
							}
							
							if (astar_planner->planPath(Cur_rPos, target_point, astar_path, path_length)) {
								using_astar = true;
								frames_since_replan = 0;
								
								for (int i = 0; i < (std::min)(path_length, 100); i++) {
									C2Sdata.Traj[i] = astar_path[i];
								}
								
								cout << "   [SUCCESS] Path planned: " << path_length << " waypoints" << endl;
							}
							else {
								using_astar = false;
								cout << "   [FAILED] A* failed, using direct approach" << endl;
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
							} else if (fabs(angle_error) > 0.3) {
								planned_vel = 30.0;
								planned_rot = angle_error * 1.2;
							} else {
								planned_vel = (dist_to_target > 100.0) ? 45.0 : 
								              (dist_to_target > 50.0) ? 35.0 : 25.0;
								planned_rot = angle_error * 0.8;
							}
							
							obstacle_avoidance->computeAvoidanceVelocity(planned_vel, planned_rot,
							                            cur_tra_vel, cur_rot_vel, obstacle, Cur_rPos.coor_ori);
						}
					}
					// 决策3: 未检测到目标 - 使用探索模式
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
				}
				else {
					// ⚠️ 系统未就绪时停止运动，避免持续前进
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

int main()
{
	system("chcp 65001");
	
	cout << "=== SLAM with Obstacle Avoidance ===" << endl;
	cout << "Press Ctrl+C to exit" << endl;
	
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
		
		if (SOCKET_ERROR == sockClient){
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
		cout << "Connected" << endl;
		
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
		
		// 清理
		if (pose_tracker) delete pose_tracker;  // ✅ 添加
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
