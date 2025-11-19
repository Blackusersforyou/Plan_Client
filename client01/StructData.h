
#define PI 3.1415926
typedef struct{
	INT16 coor_x;
	INT16 coor_y;
	double coor_ori;
}POSE;
typedef struct{
	INT16 coor_x;
	INT16 coor_y;
}Point;
typedef struct{
	INT16 Timestamp;           //Timestamp
	INT16 Runstatus;           //Runstatus£º0£ºShutdown£¬1£ºdefauted 2£ºrunning
	BYTE task_finish;          //task_finish
	BYTE detect_object;        //detect target
	BYTE collision;            //collision label
	INT16 obstacle[360];       //Laser information, 0 degree for forward direction
	POSE initial_rpose;        //initial pose of robot,0 means invalid
	Point initial_dpose;       //initial pose of target,0 means invalid
	double target_angle;       //Target direction in robot coordinates frame£¬clockwise for [0,PI],anticlockwise[0, -PI];
}S2CINFO;
typedef struct{
	INT16 Timestamp;           //Timestamp 1 for 0.2 seconds
	INT16 Runstatus;
	double tra_vel;            //Linear speed cm/s
	double rot_vel;            //Angular speed Rad/s
	POSE cur_rpose;            //Current pose of robot, 0-invalid
	Point Traj[100];           //Global path from map[][] based pathplanning 
}C2SINFO;