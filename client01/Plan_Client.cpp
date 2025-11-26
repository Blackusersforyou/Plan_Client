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
	
	// ✅ 改进2: 机器人实时角度已知/未知标志
	bool robot_angle_known = false;
	
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
				
				// 第一帧初始化
				if(S2Cdata.Timestamp == 0 && !first_frame_done){
					cout << "====== NEW SCENE ======" << endl;
					memcpy(&Initial_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
					memcpy(&Cur_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
					
					// ✅ 改进2: 判断机器人实时角度是否已知
					if (fabs(S2Cdata.target_angle) > PI) {
						robot_angle_known = false;
						cout << "Robot angle: UNKNOWN (will estimate from motion)" << endl;
					} else {
						robot_angle_known = true;
						Cur_rPos.coor_ori = S2Cdata.target_angle;
						cout << "Robot angle: KNOWN (" << S2Cdata.target_angle << " rad)" << endl;
					}
					
					// ✅ 改进2: 目标点坐标必须已知,否则无法工作
					Cur_dPos.coor_x = S2Cdata.initial_dpose.coor_x;
					Cur_dPos.coor_y = S2Cdata.initial_dpose.coor_y;
					Cur_dPos.coor_ori = 0;
					
					cout << "Robot pos: (" << Initial_rPos.coor_x << ", " 
					     << Initial_rPos.coor_y << ")" << endl;
					cout << "Target pos: (" << Cur_dPos.coor_x << ", " 
					     << Cur_dPos.coor_y << ")" << endl;
					
					double dist_to_target = MathUtils::calculateDistance(
						Initial_rPos.coor_x, Initial_rPos.coor_y,
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
				
				// ✅ 改进2: 位姿更新 - 根据角度已知/未知选择不同策略
				if (frame_count > 0 && first_frame_done) {
					if (robot_angle_known) {
						// 角度已知:正常更新
						double delta_ori = cur_rot_vel * 0.2;
						Cur_rPos.coor_ori = MathUtils::normalizeAngle(Cur_rPos.coor_ori + delta_ori);
						
						double delta_x = cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori);
						double delta_y = cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori);
						
						Cur_rPos.coor_x = (INT16)(Cur_rPos.coor_x + delta_x + 0.5);
						Cur_rPos.coor_y = (INT16)(Cur_rPos.coor_y + delta_y + 0.5);
					} else {
						// 角度未知:仅更新位置,角度由目标方向估算
						double target_angle = MathUtils::calculateBearing(
							Cur_rPos.coor_x, Cur_rPos.coor_y,
							Cur_dPos.coor_x, Cur_dPos.coor_y);
						Cur_rPos.coor_ori = target_angle;
						
						double delta_x = cur_tra_vel * 0.2 * cos(Cur_rPos.coor_ori);
						double delta_y = cur_tra_vel * 0.2 * sin(Cur_rPos.coor_ori);
						
						Cur_rPos.coor_x = (INT16)(Cur_rPos.coor_x + delta_x + 0.5);
						Cur_rPos.coor_y = (INT16)(Cur_rPos.coor_y + delta_y + 0.5);
					}
				}
				
				// 障碍物检测和避让
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
					
					// ✅ 改进2: 目标点必须已知
					Point target_point;
					target_point.coor_x = Cur_dPos.coor_x;
					target_point.coor_y = Cur_dPos.coor_y;
					
					// 计算距离和角度
					double dist_to_target = MathUtils::calculateDistance(
						target_point.coor_x, target_point.coor_y,
						Cur_rPos.coor_x, Cur_rPos.coor_y);
					
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
						cur_tra_vel = 0;
						cur_rot_vel = 0;
						first_frame_done = false;
					}
					// 决策2: 激光检测到目标 - 使用 A*
					else if (object_detected) {
						if (!using_astar || frames_since_replan > 25) {
							cout << ">> [A* MODE] Target detected, dist=" << (int)dist_to_target << "cm" << endl;
							
							if (astar_planner->planPath(Cur_rPos, target_point, astar_path, path_length)) {
								using_astar = true;
								frames_since_replan = 0;
								
								for (int i = 0; i < (std::min)(path_length, 100); i++) {
									C2Sdata.Traj[i] = astar_path[i];
								}
							}
							else {
								using_astar = false;
							}
						}
						
						if (using_astar && path_length > 0) {
							astar_planner->followPath(Cur_rPos, astar_path, path_length, obstacle,
												  planned_vel, planned_rot);
							
							// ✅ 改进1: 调用改进后的避障函数
							obstacle_avoidance->computeAvoidanceVelocity(planned_vel, planned_rot,
														cur_tra_vel, cur_rot_vel, obstacle, Cur_rPos.coor_ori);
						}
						else {
							// 直接朝向目标
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
					// 决策3: 未检测到目标 - 探索模式
					else {
						if (using_astar) {
							cout << ">> [EXPLORE] Target lost, switching to exploration" << endl;
							using_astar = false;
							path_length = 0;
						}
						
						openness_planner->computeMotionDirection(Cur_rPos, target_point, obstacle,
																 planned_vel, planned_rot, true);
						
						obstacle_avoidance->computeAvoidanceVelocity(planned_vel, planned_rot,
														cur_tra_vel, cur_rot_vel, obstacle, Cur_rPos.coor_ori);
					}
				}
				else {
					cur_tra_vel = 30;
					cur_rot_vel = -0.15;
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
