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
						
						// 计算距离
						double dist_to_target = sqrt(pow(target_point.coor_x - Cur_rPos.coor_x, 2) +
													 pow(target_point.coor_y - Cur_rPos.coor_y, 2));

    // ✅ 简化的决策逻辑：仅依赖 detect_object 和 task_finish
    bool object_detected = (S2Cdata.detect_object > 0);
    bool task_finished = (S2Cdata.task_finish > 0);
    bool target_known = (target_point.coor_x != 0 || target_point.coor_y != 0);

    // 检查任务完成状态
    if (task_finished) {
        cout << "====== TASK FINISHED! ======" << endl;
        cout << "Target contacted successfully!" << endl;
        cur_tra_vel = 0;
        cur_rot_vel = 0;
        first_frame_done = false;
    }
    else if (object_detected) {
        // ✅ 激光传感器检测到目标：使用 A* 规划
        
        // 每25帧或首次检测到时重新规划路径
        if (!using_astar || frames_since_replan > 25) {
            cout << ">> [A* MODE] Laser sensor detected target" << endl;
            cout << "   Distance: " << (int)dist_to_target << "cm" << endl;
            
            if (astar_planner->planPath(Cur_rPos, target_point, astar_path, path_length)) {
                using_astar = true;
                frames_since_replan = 0;
                
                // 将路径存入通信数据（用于可视化）
                for (int i = 0; i < (std::min)(path_length, 100); i++) {
                    C2Sdata.Traj[i] = astar_path[i];
                }
                
                cout << "   [OK] A* planned: " << path_length << " waypoints" << endl;
            }
            else {
                using_astar = false;
                cout << "   [FAIL] A* failed, using direct approach" << endl;
            }
        }
        
        if (using_astar && path_length > 0) {
            // ✅ 跟随 A* 路径，并应用避障
            astar_planner->followPath(Cur_rPos, astar_path, path_length, obstacle,
                                  planned_vel, planned_rot);
            
            // ✅ 使用避障调整（恢复原始行为）
            obstacle_avoidance->computeAvoidanceVelocity(planned_vel, planned_rot,
                                                     cur_tra_vel, cur_rot_vel, obstacle);
        }
        else {
            // A* 失败或路径为空，直接朝向目标
            double target_dx = target_point.coor_x - Cur_rPos.coor_x;
            double target_dy = target_point.coor_y - Cur_rPos.coor_y;
            double target_angle = atan2(target_dy, target_dx);
            double angle_error = target_angle - Cur_rPos.coor_ori;
            
            while (angle_error > PI) angle_error -= 2 * PI;
            while (angle_error < -PI) angle_error += 2 * PI;
            
            // 根据角度误差调整速度
            if (fabs(angle_error) > 0.6) {
                planned_vel = 18.0;
                planned_rot = (angle_error > 0) ? 0.5 : -0.5;
            } else if (fabs(angle_error) > 0.3) {
                planned_vel = 30.0;
                planned_rot = angle_error * 1.2;
            } else {
                // 根据距离调整速度
                if (dist_to_target > 100.0) {
                    planned_vel = 45.0;
                } else if (dist_to_target > 50.0) {
                    planned_vel = 35.0;
                } else {
                    planned_vel = 25.0;
                }
                planned_rot = angle_error * 0.8;
            }
            
            // ✅ 使用避障调整
            obstacle_avoidance->computeAvoidanceVelocity(planned_vel, planned_rot,
                                                     cur_tra_vel, cur_rot_vel, obstacle);
        }
    }
    else {
        // ✅ 未检测到目标：使用探索模式
        if (using_astar) {
            cout << ">> [EXPLORE] Target lost, switching to exploration" << endl;
            using_astar = false;
            path_length = 0;
        }
        
        // 目标位置已知时，传入目标点引导探索方向
        openness_planner->computeMotionDirection(Cur_rPos, target_point, obstacle,
                                                 planned_vel, planned_rot, target_known);
        
        // ✅ 使用避障调整
        obstacle_avoidance->computeAvoidanceVelocity(planned_vel, planned_rot,
                                                     cur_tra_vel, cur_rot_vel, obstacle);
    }
    
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
    
    // ✅ 简化的状态输出
    bool object_detected = (S2Cdata.detect_object > 0);
    bool task_finished = (S2Cdata.task_finish > 0);
    
    cout << "F:" << frame_count 
         << " Pos:(" << Cur_rPos.coor_x << "," << Cur_rPos.coor_y << ")"
         << " Dist:" << (int)dist_to_target;
    
    // 显示当前模式
    if (task_finished) {
        cout << " [FINISHED]";
    } else if (using_astar) {
        cout << " [A*]";
    } else {
        cout << " [EXPLORE]";
    }
    
    // 显示传感器状态
    cout << " Sensor:" << (object_detected ? "DETECT" : "NONE");
    
    // 显示其他状态
    if (obstacle_avoidance->isInRecoveryMode()) {
        cout << " [RECOVERY]";
    }
    if (current_collision) {
        cout << " [COLLISION!]";
    }
    
    // 显示障碍物信息
    cout << " Obs:[F:" << (int)front.distance 
         << " L:" << (int)left.distance 
         << " R:" << (int)right.distance << "]"
         << " V:" << (int)cur_tra_vel 
         << " W:" << std::fixed << std::setprecision(2) << cur_rot_vel 
         << endl;
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
