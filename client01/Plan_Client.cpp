#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <atlstr.h>
#include "list"
#include "vector"
#include "winsock2.h"
#include "StructData.h"

#pragma comment(lib,"ws2_32.lib")
using namespace std;
SOCKET sockClient;
S2CINFO S2Cdata;
C2SINFO C2Sdata;
POSE Initial_rPos;
POSE Cur_rPos;
POSE Cur_dPos;
INT16 obstacle[360]; //Laser data, 360 degree
int Runstatus = 1;   //0:Program shutdown 2: running
double cur_tra_vel = 0;
double cur_rot_vel = 0;
void Initialition(){
	memset(&S2Cdata, 0, sizeof(S2CINFO));
	memset(&C2Sdata, 0, sizeof(C2SINFO));
	memset(&Initial_rPos, 0, sizeof(POSE));
	memset(&Cur_rPos, 0, sizeof(POSE));
	memset(&Cur_dPos, 0, sizeof(POSE));
	memset(obstacle, 0, 360 * sizeof(INT16));
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
	char recvBuf[800]; // Data received
	char Sendbuff[600];// Data sent
	while (true){
		memset(recvBuf, 0, sizeof(recvBuf));
		if (recv(sockClient, recvBuf, sizeof(recvBuf), 0) > 0){
			memset(&S2Cdata, 0, sizeof(S2CINFO));
			memset(&C2Sdata, 0, sizeof(C2SINFO));
			memcpy(&S2Cdata, recvBuf, sizeof(S2CINFO));
			{
				if (S2Cdata.Runstatus == 0){
					Runstatus = 0;
					return 0;
				}
				if(S2Cdata.Timestamp == 0){//Get the poses of robot and target at the first frame;
					memcpy(&Initial_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
					memcpy(&Cur_rPos, &S2Cdata.initial_rpose, sizeof(POSE));
					memcpy(&Cur_dPos, &S2Cdata.initial_dpose, sizeof(POSE));
				}
				C2Sdata.Timestamp = S2Cdata.Timestamp;
				C2Sdata.Runstatus = S2Cdata.Runstatus;
				memcpy(obstacle, S2Cdata.obstacle, 360 * sizeof(INT16));
				//Add global path here
				

				//C2Sdata.Traj[0].coor_x = Initial_rPos.coor_x;
				//C2Sdata.Traj[0].coor_y = Initial_rPos.coor_y;
				//C2Sdata.Traj[1].coor_x = Cur_dPos.coor_x;
				//C2Sdata.Traj[1].coor_y = Cur_dPos.coor_y;
				//Add localization of robot here

				if (S2Cdata.detect_object > 0)
					S2Cdata.detect_object = S2Cdata.detect_object;
				Cur_rPos.coor_ori = Cur_rPos.coor_ori + cur_rot_vel*0.2;
				Cur_rPos.coor_x = INT16(Cur_rPos.coor_x + cur_tra_vel * 0.2*cos(Cur_rPos.coor_ori) + 0.5);
				Cur_rPos.coor_y = INT16(Cur_rPos.coor_y + cur_tra_vel * 0.2*sin(Cur_rPos.coor_ori) + 0.5);
				C2Sdata.cur_rpose.coor_x = Cur_rPos.coor_x;
				C2Sdata.cur_rpose.coor_y = Cur_rPos.coor_y;
				C2Sdata.cur_rpose.coor_ori = Cur_rPos.coor_ori;
				//Add plan code here


				C2Sdata.tra_vel = 30;
				C2Sdata.rot_vel = -0.15;
				cur_tra_vel = 30;
				cur_rot_vel = -0.15;
				//Send data
				cout << "TimeStamp: " << C2Sdata.Timestamp << endl;
				memset(Sendbuff, 0, sizeof(Sendbuff));
				memcpy(Sendbuff, &C2Sdata, sizeof(C2SINFO));
				send(sockClient, Sendbuff, sizeof(Sendbuff), 0);
				Sleep(30);
			}
		}
		else
			Sleep(30);
	}
	closesocket(sockClient);
	return 0;
}
int main()
{
	CString path = GetExePath() + "Plan_Server.exe";
	ShellExecute(NULL, NULL, path, NULL, NULL, SW_SHOW);
	Sleep(1000);
	Initialition();
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		printf("Winsock initialization failure");
		return 0;
	}
	SOCKADDR_IN addrSrv;
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(8888);
	addrSrv.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	sockClient = socket(AF_INET, SOCK_STREAM, 0);
	if (SOCKET_ERROR == sockClient){
		printf("Socket() error:%d", WSAGetLastError());
		return 0;
	}
	if (connect(sockClient, (struct  sockaddr*)&addrSrv, sizeof(addrSrv)) == INVALID_SOCKET){
		printf("Connection failure:%d", WSAGetLastError());
		return 0;
	}
	HANDLE hThreadRecv = CreateThread(NULL, 0, Recv_Thre, 0, 0, NULL);
	if (NULL == hThreadRecv)
		CloseHandle(hThreadRecv);
	while (Runstatus)
	{
		Sleep(50);
	}
	WSACleanup();
	return 0;
}
