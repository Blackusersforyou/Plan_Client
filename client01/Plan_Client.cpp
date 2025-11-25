#include <iostream>
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
	bool target_visible = false;
	bool last_collision_state = false;
	bool actually_reached = false;  // ✅ 添加这一行,在函数开始处声明
	
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
				
				// ✅ 问题2解决：碰撞后立即重置环境
				bool current_collision = (S2Cdata.collision > 0);
				if (current_collision && !last_collision_state) {
					// 检测到碰撞的上升沿（从无碰撞到碰撞）
					cout << "====== COLLISION DETECTED - FULL RESET ======" << endl;
					
					// 立即停止
					cur_tra_vel = 0;
					cur_rot_vel = 0;
					
					// 完全重置所有状态
					if (obstacle_avoidance) {
						obstacle_avoidance->reset();
						cout << "   Obstacle avoidance reset" << endl;
					}
					
					if (openness_planner) {
						openness_planner->reset();
						cout << "   Openness planner reset" << endl;
					}
					
					// 重置路径规划状态
					using_astar = false;
					path_length = 0;
					frames_since_replan = 0;
					
					cout << "Waiting for manual repositioning..." << endl;
				}
				last_collision_state = current_collision;
				
				// 第一帧初始化
				if(S2Cdata.Timestamp == 0 && !first_frame_done){
					cout << "====== NEW SCENE ======" << endl;
					cout << "First frame received" << endl;
					memcpy(&Initial_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
					memcpy(&Cur_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
					Cur_dPos.coor_x = S2Cdata.initial_dpose.coor_x;
					Cur_dPos.coor_y = S2Cdata.initial_dpose.coor_y;
					Cur_dPos.coor_ori = 0;
					
					cout << "Robot pos: (" << Initial_rPos.coor_x << ", " 
					     << Initial_rPos.coor_y << "), ori: " << Initial_rPos.coor_ori << endl;
					cout << "Target pos: (" << Cur_dPos.coor_x << ", " << Cur_dPos.coor_y << ")" << endl;
					
					double dist_to_target = sqrt(pow(Cur_dPos.coor_x - Initial_rPos.coor_x, 2) +
					                             pow(Cur_dPos.coor_y - Initial_rPos.coor_y, 2));
					cout << "Distance to target: " << (int)dist_to_target << " cm" << endl;
					
					using_astar = false;
					path_length = 0;
					frames_since_replan = 0;
					frame_count = 0;
					first_frame_done = true;
					target_visible = false;
					
					// ✅ 修复问题3：完全重置所有状态
					if (obstacle_avoidance) {
						obstacle_avoidance->reset();
						cout << "   Obstacle avoidance reset" << endl;
					}
					
					if (openness_planner) {
						openness_planner->reset();  // 使用新增的reset方法
						cout << "   Openness planner reset" << endl;
					}
					
					if (slam_system) {
						slam_system->resetVisitHistory();
						cout << "   SLAM visit history reset" << endl;
					}
					
					cur_tra_vel = 0;
					cur_rot_vel = 0;
					
					cout << "All systems reset for new scene" << endl;
				}
				
				C2Sdata.Timestamp = S2Cdata.Timestamp;
				C2Sdata.Runstatus = S2Cdata.Runstatus;
				memcpy(obstacle, S2Cdata.obstacle, 360 * sizeof(INT16));
				
				// 修复问题2：位姿更新 - 仅在有控制指令时才更新
				// 避免累积误差和鼠标拖动时的异常行为
				if (frame_count > 0 && first_frame_done) {
					double delta_ori = cur_rot_vel * 0.2;
					double new_ori = Cur_rPos.coor_ori + delta_ori;
					
					double delta_x = cur_tra_vel * 0.2 * cos(new_ori);
					double delta_y = cur_tra_vel * 0.2 * sin(new_ori);
					
					Cur_rPos.coor_ori = new_ori;
					Cur_rPos.coor_x = (INT16)(Cur_rPos.coor_x + delta_x + 0.5);
					Cur_rPos.coor_y = (INT16)(Cur_rPos.coor_y + delta_y + 0.5);
				}
				
				// 障碍物检测和避让
				if (slam_ready && slam_system && obstacle_avoidance && openness_planner && astar_planner) {
					// 1. 障碍物检测
					obstacle_avoidance->detectObstacles(obstacle, Cur_rPos.coor_ori);
					
					// 2. SLAM地图更新
					if (frame_count % 10 == 0) {
						slam_system->updateMap(Cur_rPos, obstacle);
						
						if (frame_count % 30 == 0) {
							// 修改：传入目标点进行目标导向的空旷度计算
							Point target_point;
							target_point.coor_x = Cur_dPos.coor_x;
							target_point.coor_y = Cur_dPos.coor_y;
							slam_system->updateOpenness(target_point);
						}
					}
					
					// 3. 确定目标点
					Point target_point;
					bool using_candidate_target = false;

					// ✅ 优先级1:使用已知目标点
					if (Cur_dPos.coor_x != 0 || Cur_dPos.coor_y != 0) {
						target_point.coor_x = Cur_dPos.coor_x;
						target_point.coor_y = Cur_dPos.coor_y;
					} 
					// ✅ 优先级2:目标未知,尝试从候选中选择
					else if (slam_system) {
						Point candidate_pos;
						double candidate_conf;
						
						if (slam_system->getBestCandidate(candidate_pos, candidate_conf)) {
							target_point = candidate_pos;
							using_candidate_target = true;
							
							static int log_counter = 0;
							if (++log_counter % 30 == 0) {
								cout << ">> Using CANDIDATE target at (" 
								     << candidate_pos.coor_x << "," << candidate_pos.coor_y << ")"
								     << " confidence=" << candidate_conf << endl;
							}
						} else {
							// 没有候选,继续探索
							target_point.coor_x = 0;
							target_point.coor_y = 0;
						}
					}

					// 计算距离
					double dist_to_target = sqrt(pow(target_point.coor_x - Cur_rPos.coor_x, 2) +
					                             pow(target_point.coor_y - Cur_rPos.coor_y, 2));

					// ✅ 物理确认检测
					bool target_physically_detected = (S2Cdata.detect_object > 0);
					if (target_physically_detected && slam_system) {
						// 物理传感器检测到物体,确认为真实目标
						slam_system->confirmTargetPhysically(Cur_rPos.coor_x, Cur_rPos.coor_y);
						
						static int confirm_log = 0;
						if (++confirm_log % 10 == 0) {
							cout << ">>> Physical sensor confirms target!" << endl;
						}
					}

					// ✅ 到达判断
					bool actually_reached = (dist_to_target < 10.0) && target_physically_detected;

					if (actually_reached) {
						cout << "====== TARGET REACHED! ======" << endl;
						if (using_candidate_target) {
							cout << "Reached CANDIDATE target (unknown location)" << endl;
						} else {
							cout << "Reached KNOWN target" << endl;
						}
						cout << "Final distance: " << dist_to_target << " cm" << endl;
						cur_tra_vel = 0;
						cur_rot_vel = 0;
						first_frame_done = false;
						target_visible = false;
					}
					else {
							// 声明变量
							double planned_vel = 0;
							double planned_rot = 0;
							
							frames_since_replan++;
							double exploration_progress = slam_system->getExplorationProgress();
							
							// ✅ 目标检测
				bool target_position_known = (target_point.coor_x != 0 || target_point.coor_y != 0);
				bool target_laser_visible = slam_system->isTargetVisible(target_point, obstacle);
				bool target_physically_detected = (S2Cdata.detect_object > 0);
				bool direct_path_clear = slam_system->isDirectPathClear(target_point, obstacle, 80.0);

				// ✅ 关键修复：目标位置已知就应该使用目标导向模式
				if (target_position_known) {
				    // 位置已知，即使暂时看不见也应该朝目标方向前进
				    target_visible = true;
				    
				    // ✅ 添加详细调试输出
				    static int visibility_log = 0;
				    if (++visibility_log % 50 == 0) {
				        cout << ">> Target Status: position_known=YES"
				             << ", laser_visible=" << (target_laser_visible ? "YES" : "NO")
				             << ", physical=" << (target_physically_detected ? "YES" : "NO")
				             << ", path_clear=" << (direct_path_clear ? "YES" : "NO")
				             << ", dist=" << (int)dist_to_target << "cm" << endl;
				    }
				} else {
				    target_visible = false;
				}

							// 综合判断：用于显示状态
							if (target_physically_detected) {
									target_visible = true;
							}
							else if (target_laser_visible && dist_to_target < 600.0) {
									target_visible = true;
							}
							else {
									target_visible = false;
							}

							// ✅ 核心修改：Object Find 状态时优先使用 A*
							// 优先级：
							// 1. A* 模式：Object Find (target_visible=true) + 距离合适 + 已探索
							// 2. 直达模式：距离很近 (<=50cm)
							// 3. 探索模式：目标不可见
							
							if (target_visible && dist_to_target > 50.0 && dist_to_target < 600.0 && 
							    exploration_progress > 0.03) {
									// ✅ 优先级1：A* 模式 - Object Find 状态，使用最优路径规划
									if (!using_astar || frames_since_replan > 25) {
											cout << ">> [A* MODE] Object detected, planning optimal path" << endl;
											cout << "   Target: visible=" << target_visible 
											     << ", laser=" << target_laser_visible
											     << ", physical=" << target_physically_detected
											     << ", dist=" << (int)dist_to_target << "cm" << endl;
											cout << "   Path: " << (direct_path_clear ? "CLEAR" : "BLOCKED") << endl;
											
											if (astar_planner->planPath(Cur_rPos, target_point, astar_path, path_length)) {
													using_astar = true;
													frames_since_replan = 0;
													
													for (int i = 0; i < (std::min)(path_length, 100); i++) {
														C2Sdata.Traj[i] = astar_path[i];
													}
													
													cout << "   [OK] A* path: " << path_length << " waypoints" << endl;
											}
											else {
													using_astar = false;
													cout << "   [FAIL] A* failed, fallback to direct mode" << endl;
											}
									}
									
									if (using_astar && path_length > 0) {
											// 跟随 A* 路径
											astar_planner->followPath(Cur_rPos, astar_path, path_length, obstacle,
																  planned_vel, planned_rot);
									}
									else {
											// A* 失败，直达模式
											if (direct_path_clear) {
													// 路径畅通，直接冲刺
													double target_dx = target_point.coor_x - Cur_rPos.coor_x;
													double target_dy = target_point.coor_y - Cur_rPos.coor_y;
													double target_angle = atan2(target_dy, target_dx);
													double angle_error = target_angle - Cur_rPos.coor_ori;
													
													while (angle_error > PI) angle_error -= 2 * PI;
													while (angle_error < -PI) angle_error += 2 * PI;
													
													if (fabs(angle_error) > 0.6) {
															planned_vel = 18.0;
															planned_rot = (angle_error > 0) ? 0.5 : -0.5;
													} else if (fabs(angle_error) > 0.3) {
															planned_vel = 30.0;
															planned_rot = angle_error * 1.2;
													} else {
															if (dist_to_target > 100.0) {
																	planned_vel = 45.0;
															} else if (dist_to_target > 50.0) {
																	planned_vel = 35.0;
															} else {
																	planned_vel = 25.0;
															}
															planned_rot = angle_error * 0.8;
													}
											}
											else {
													// 路径有障碍，使用导向模式
													openness_planner->computeMotionDirection(Cur_rPos, target_point, obstacle,
																					   planned_vel, planned_rot, true);
											}
									}
							}
							else if (target_visible && dist_to_target <= 50.0) {
									// ✅ 优先级2：近距离接近模式
									cout << ">> [APPROACH MODE] Final approach (dist: " << (int)dist_to_target << "cm)" << endl;
									using_astar = false;
									
									double target_dx = target_point.coor_x - Cur_rPos.coor_x;
									double target_dy = target_point.coor_y - Cur_rPos.coor_y;
									double target_angle = atan2(target_dy, target_dx);
									double angle_error = target_angle - Cur_rPos.coor_ori;
									
									while (angle_error > PI) angle_error -= 2 * PI;
									while (angle_error < -PI) angle_error += 2 * PI;
									
									if (fabs(angle_error) > 0.5) {
											planned_vel = 12.0;
											planned_rot = (angle_error > 0) ? 0.5 : -0.5;
									} else {
											planned_vel = 20.0;
											planned_rot = angle_error * 1.5;
									}
							}
							else {
									// ✅ 优先级3：探索模式（目标不可见或距离/探索度不满足）
									if (using_astar) {
											cout << ">> [EXPLORE MODE] Target lost or conditions not met" << endl;
											cout << "   visible=" << target_visible 
											     << ", dist=" << (int)dist_to_target
											     << ", explored=" << (int)(exploration_progress*100) << "%" << endl;
											using_astar = false;
											path_length = 0;
									}
									
									openness_planner->computeMotionDirection(Cur_rPos, target_point, obstacle,
																		   planned_vel, planned_rot, false);
							}
							
							// 5. 障碍物避让调整
							obstacle_avoidance->computeAvoidanceVelocity(planned_vel, planned_rot,
																	 cur_tra_vel, cur_rot_vel, obstacle);
							
							// 碰撞检测
							if (current_collision) {
								cur_tra_vel = 0;
								cur_rot_vel = 0;
							}
					}
					
					// 每30帧输出一次信息
					if (frame_count % 30 == 0 && frame_count > 0) {
						ObstacleInfo front, left, right;
						obstacle_avoidance->getObstacleInfo(front, left, right);
						double exploration_progress = slam_system->getExplorationProgress();
						
						// ✅ 新增：显示路径状态
						bool direct_clear = slam_system->isDirectPathClear(target_point, obstacle, 80.0);
						
						cout << "F:" << frame_count 
						     << " Pos:(" << Cur_rPos.coor_x << "," << Cur_rPos.coor_y << ")"
						     << " Dist:" << (int)dist_to_target
						     << " Mode:" << (using_astar ? "[A*]" : (target_visible ? (direct_clear ? "[DIRECT]" : "[GUIDED]") : "[EXPLORE]"))
						     << (obstacle_avoidance->isInRecoveryMode() ? " [RECOVERY]" : "")
						     << (current_collision ? " [COLLISION!]" : "")
						     << " PathClear:" << (direct_clear ? "YES" : "NO")
						     << " Explored:" << (int)(exploration_progress*100) << "%"
						     << " Obs:[F:" << (int)front.distance 
						     << " L:" << (int)left.distance 
						     << " R:" << (int)right.distance << "]"
						     << " V:" << (int)cur_tra_vel << " W:" << cur_rot_vel << endl;
					}
				}
				else {
					cur_tra_vel = 30;
					cur_rot_vel = -0.15;
				}
				
				frame_count++;
				
				// 直接继续发送数据
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
