#include "ros/ros.h"
#include "geometry_msgs/Twist.h"			//for command velocity
#include "geometry_msgs/Vector3.h"			//for command velocity
#include "std_msgs/Empty.h"	 				//For take off and landing
#include "ardrone_autonomy/CamSelect.h"    	// For toggling camera
#include "ardrone_autonomy/Navdata.h"    //Accessing ardrone's published data
#include "ardrone_autonomy/vector31.h"
#include "ardrone_autonomy/vector21.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "ardrone_autonomy/navdata_gps.h"    //Accessing ardrone's published data
#include <sensor_msgs/Joy.h>
#include <stdio.h>
#include <math.h>
#include <termios.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ardrone_test/Drone_odo.h"
#include "ardrone_test/est_co.h"
//for gazebo sim
#include "nav_msgs/Odometry.h"

// class variables
std_msgs::Empty emp_msg;				// variable in the take off and landing class
geometry_msgs::Twist vel_msg;			// variable in the command velocity class

// Variables for Publishing
ros::Publisher T_pub_empty;				//take off publisher
ros::Publisher L_pub_empty;				//landing publisher
ros::Publisher velocity_publisher;		// velocity publisher

// Variables for Subscribing
ros::Subscriber pose_subscriber;		// ardrone navdata subsciber
ros::Subscriber gps_subscriber;		// ardrone gps data subsciber
ros::Subscriber imu_subscriber;		// ardrone imu data subsciber
ros::Subscriber joy_sub_;


// Variables for Service
ros::ServiceClient client1;		// ardrone camera service

using namespace std;

float lx, ly, lz, ax, ay, az,foc = 685, angx,angy,angz, est_x, est_y, est_z,K_est_x, K_est_y, K_est_z,test_x,test_y,test_z;
int to, la, f = 200 , redX, redY, res =0, H=0,I=0,J=0;
double k = 0.51, z, tf, ang_time,hem; // k value for tau
double lat, lon, ele;
double xD=7,yD=4,zD=1;
//gps variables///
double new_lat, new_lon;
bool gps_cntr = 0;
//static float pos_x = 0, pos_y = 0, pos_z = 0;
const double PI = 3.14159265359;
double Kp = 0.2, Kd = 1,Kpz;// Gain for the controller
double k1 = 2.438, k2 = 0.779, k3 = 1.234, k4 = 0.221;// Gain for the controller
double tol = 0.4;// error tolerance
int kinect_ready=0;

//class instances
ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata
sensor_msgs::Imu ang;
nav_msgs::Odometry odo;
nav_msgs::Odometry sim_statedata;  	// Using this class variable for storing state data for Gazebo simulator

//callback function declarations
void NumCallback(const ardrone_test::Drone_odo::ConstPtr& val);
void EstCallback(const ardrone_test::est_co::ConstPtr& est);
void INS_K_Callback(const ardrone_test::est_co::ConstPtr& esti);
void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message);		// drone actual data
void gpsCallback(const ardrone_autonomy::navdata_gps::ConstPtr & gps_message);		// drone actual data
void new_gpsCallback(const std_msgs::String::ConstPtr& gps1);
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message);		// drone actual data
void joyCallback(const sensor_msgs::Joy::ConstPtr & joy);		// Joy data
void record();
int getch();
void testdataCallback(const ardrone_autonomy::vector31::ConstPtr & test );
void SimStateCallback(const nav_msgs::Odometry::ConstPtr & sim_state_message); //gazebo sim 

// basic func
void hover(int timee);		// hovering
void takeoff();		// take off
void land();		//landing
void move(float lx, float ly, float lz, float ax, float ay, float az ); // publishing command velocity
// Basic Navigation func
void basic_movement();
// advanced control functions
void traj_track();

int main(int argc, char **argv)
{
	//Initiate the ROS

	ros::init(argc, argv, "waypoint_nav");
	ros::NodeHandle n; 			// Nodehandle is like a class where you are operating

	// Publishing the data
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100); // initialize to send the Control Input

	T_pub_empty = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);  //initialize to send the take off command  /* Message queue length is just 1 */

	L_pub_empty = n.advertise<std_msgs::Empty>("/ardrone/land", 1); //initialize to send the land command /* Message queue length is just 1 */

	// Subscribing the data
	pose_subscriber = n.subscribe("/ardrone/navdata", 200, poseCallback);	//initialize to receive processed sensor data
	imu_subscriber = n.subscribe("/ardrone/imu", 200, imuCallback); //initialize to receive raw sensor data
	gps_subscriber = n.subscribe("/ardrone/navdata_gps", 10, gpsCallback); //initialize to receive gps data
	est_sub = n.subscribe("Coordinate", 100, EstCallback); //initialize to receive ground cam target data
	INS_K_est = n.subscribe("/estimation_data", 200, INS_K_Callback); //initialize to receive kinect+ uav sensor estimation sensor data
	test_sub = n.subscribe("/test", 100, testdataCallback);
	//gazebo simulation subscribers
	sim_state_sub = n.subscribe("/ground_truth/state",10,SimStateCallback);
	num_sub = n.subscribe("/chatter1", 100, NumCallback);
	joy_sub_ = n.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);

while(ros::ok())
{

cout<<"Press key"<<endl;
cout<<"w-takeoff | s-land | m-basic movements"<<endl;
cout<<"n-record | h-hover for input time | t - traj track"<<endl;

    int c = getch();   // call your non-blocking input function
    int timee;
    double x, y, z;			// for reference value

    switch (c)
    {
    case 't':
			traj_track();
			break; 
    case 'm':
			cout<<"\n basic movement menu"<<endl;
			basic_movement();
    case 'h':
			cout<<"\n enter hover time"<<endl;
			cin>>timee;
    			cout<<"\n Hovering in a place"<<endl;
    			hover(timee);
    			break;
    case 's':
    			cout<<endl<<"\n Landing initiated"<<endl;
    			land();
    			break;
    case 'w':
    			cout<<endl<<"\n Take off initiated"<<endl;
    			takeoff();
    			hover(2);
    			break;
	case 'n':
    			record();
    			break;
    //case 'y':
    			//cout<<endl<<"Rotation in degree :"<<endl;
    			//cin>>x;
    			//yawing(x);
    			//break;
    default:
    			land();
    			break;

    }
//ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and will not be able to publish
}
land();
cout<<"landed";

}
void traj_track(){
ofstream myfile;
myfile.open("/home/kaustubh/data/dataf.dat", ios::out | ios::app );
ofstream myfile2;
myfile2.open("/home/kaustubh/data/recorded_data_0.txt", ios::out | ios::app );
myfile<<"xd"<<setw(20)<<"x"<<setw(20)<<"yd"<<setw(20)<<"y"<<setw(20)<<"zd"<<setw(20)<<"z"<<endl;
myfile2<<"term_1_in_lx. "<<"term_2_in_lx. "<<"term_3_in_lx."<<setw(20)<<"k1"<<setw(20)<<"k1"<<setw(20)<<"k2"<<setw(20)<<"k2"<<setw(20)<<"k3"<<setw(20)<<"k3"<<setw(20)<<"k4"<<setw(20)<<"k4"<<endl;
double d2r = 1*3.14159265/180.0, pi = 3.14159265;
double lx=0,ly=0,lz=0,ax=0,ay=0,az=0;
double a1=3.613,a3=3.384,b1=6.542,b2=25.5,b3=-67.3,c1=4.302,c2=28.24,c3=60.4,d1=4.225,d3=3.828;
double Kpz=1,Kdz=1,Kp_psi=1,Kd_psi=1;
double k1=1.75,k2=1.75,kp=1.5,kd=1.5;//what worked - k1=1.5,k2=1.5,kp=1.25,kd=1.25; loop rate - 10 hz, increment every 32 times
double t=0, T= 60, g=9.8 , g_dot=0;
double b = 0.5, c= 0.5;
//double gz,gz_dot;
//double kp,kd,k1,k2;
double x0 = sim_statedata.pose.pose.position.x , y0 = sim_statedata.pose.pose.position.y , z0 = sim_statedata.pose.pose.position.z; 
double phi0=(drone_navdata.rotX)*d2r,theta0=(drone_navdata.rotY)*d2r,psi0 = (drone_navdata.rotZ)*d2r;
ros::Rate loop_rate(100);
int ct=0;
while(ros::ok()){
    if(ct%250==0)
    	t = t + 1;
    ct++;
    //actual vals
    double x = sim_statedata.pose.pose.position.x , y = sim_statedata.pose.pose.position.y , z = sim_statedata.pose.pose.position.z; 
    double phi=(drone_navdata.rotX)*d2r,theta=(drone_navdata.rotY)*d2r, psi=0;//psi=(drone_navdata.rotZ)*d2r;
    double x_dot = sim_statedata.twist.twist.linear.x , y_dot = sim_statedata.twist.twist.linear.y , z_dot = sim_statedata.twist.twist.linear.z;
    double phi_dot=sim_statedata.twist.twist.angular.x , theta_dot=sim_statedata.twist.twist.angular.y, psi_dot=0, psi_ddot = 0;//psi_dot=sim_statedata.twist.twist.angular.z

    /* //desired vals for hover/st.line/
    double zd=t,psid=0,xd=0,yd=0,xd_dot=0,yd_dot=0,zd_dot=1,psid_dot=0,xd_ddot=0,yd_ddot=0,zd_ddot=0,phid_ddot=0,thetad_ddot=0,psid_ddot=0;
    double phid=0,thetad=0,phid_dot=0,thetad_dot=0;
*/
//8 shape desired position
    double zd=1,psid=0,xd=5-5*cos(2*pi*t/T),yd=2.5*sin(4*pi*t/T);//zd=1.3+0.3*sin(2*pi*t/T + 3*pi/2),psid=sin(2*pi*t/T);
    //desired derivatives
    double xd_dot=(+10*pi/T)*sin(2*pi*t/T) ,yd_dot=(10*pi/T)*cos(4*pi*t/T) ,zd_dot=0;//zd_dot=(0.3*2*pi/T)*cos(2*pi*t/T + 3*pi/2);
    double xd_ddot=(+20*pi*pi/(T*T))*cos(2*pi*t/T) ,yd_ddot=-(40*pi*pi/(T*T))*sin(4*pi*t/T) ,zd_ddot=0;//zd_ddot=-(0.3*4*pi*pi/(T*T))*sin(2*pi*t/T + 3*pi/2); 

    //saving data
    myfile<<xd<<setw(20)<<x<<setw(20)<<yd<<setw(20)<<y<<setw(20)<<zd<<setw(20)<<z<<endl;

    //control
    lz = (1/a3)*(a1*z_dot-Kpz*(z-zd)-Kdz*(z_dot-zd_dot)+zd_ddot);
    g = 9.8 - a1*z_dot - a3*lz;
    g_dot = -a1*(g-9.8);//-a1*z_ddot;

    //desired phi, theta, psi and derivatives (first and second)
    double phid = (1/g)*(sin(psid)*xd_ddot-cos(psid)*yd_ddot), thetad = (1/g)*(cos(psid)*xd_ddot+sin(psid)*yd_ddot);
    double psid_dot=0, psid_ddot = 0;//double psid_dot=(2*pi/T)*cos(2*pi*t/T), psid_ddot=-(4*pi*pi/(T*T))*sin(2*pi*t/T);
    
    double xd_dddot = (-40*pi*pi*pi/(T*T*T))*sin(2*pi*t/T), yd_dddot = -(160*pi*pi*pi/(T*T*T))*cos(4*pi*t/T);
    double xd_ddddot = (-80*pi*pi*pi*pi/(T*T*T*T))*cos(2*pi*t/T), yd_ddddot = (640*pi*pi*pi*pi/(T*T*T*T))*sin(4*pi*t/T);

    double phid_dot= (1/g)*(sin(psid)*xd_dddot-cos(psid)*yd_dddot), thetad_dot= (1/g)*(cos(psid)*xd_dddot+sin(psid)*yd_dddot);

    double phid_ddot = (1/g)*(sin(psid)*xd_ddddot-cos(psid)*yd_ddddot), thetad_ddot = (1/g)*(cos(psid)*xd_ddddot+sin(psid)*yd_ddddot);    

    //cont. control
    double K1[]={ (1/g)*(cos(psi)*(kp+k2*b*g*g+k2*k1*kp)-sin(psi)*psi_dot*(k2*kp+b*g*g+k1*kp)+kp*(-cos(psi)*psi_dot-sin(psi)*psi_ddot)) + (1/g_dot)*(cos(psi)*(k2*kp+b*g_dot*g_dot+k1*kp)-sin(psi)*psi_dot*(2*kp))              ,           (1/g)*(+sin(psi)*(kp+k2*b*g*g+k2*k1*kp)+cos(psi)*psi_dot*(k2*kp+b*g*g+k1*kp)+kp*(-sin(psi)*psi_dot+cos(psi)*psi_ddot) ) + (1/g_dot)*(sin(psi)*(k2*kp+b*g_dot*g_dot+k1*kp)+cos(psi)*psi_dot*(2*kp))} ;
    // FIRST ROW OF K1 MATRIX, (THIRD = - FIRST AND FOURTH = SECOND)
    double K2[]={ (1/g)*(cos(psi)*(b*g*g+k1*kp+kd+k2*kp+k2*c*g*g+k2*k1*kd)-sin(psi)*psi_dot*(k2*kd+c*g*g+k1*kd+2*kp)+(-cos(psi)*psi_dot-sin(psi)*psi_ddot)*(kd)) + (1/g_dot)*(cos(psi)*(k2*kd+c*g_dot*g_dot+k1*kd+2*kp)-sin(psi)*psi_dot*(2*kd))              ,           (1/g)*(+sin(psi)*(b*g*g+k1*kp+kd+k2*kp+k2*c*g*g+k2*k1*kd)+cos(psi)*(k2*kd+c*g*g+k1*kd+2*kp)+kd*(-sin(psi)*psi_dot+cos(psi)*psi_ddot)) + (1/g_dot)*(sin(psi)*(k2*kd+c*g_dot*g_dot+k1*kd+2*kp)+cos(psi)*(2*kd))} ;
    //second term in K1[1] is negligible . it is the first term that is causing anuisance being -42 in value. IN the first term, it is the cos(psi) multiplied term which is large and causing problems.
    // FIRST ROW OF K2 MATRIX, (THIRD = - FIRST AND FOURTH = SECOND)
    double K3[]={ kp+k1*kd+c*g*g+ 1 +k2*kd+k2*k1+2*kd*(  0  )  +(g/g_dot),   kp+k1*kd+c*g*g+ 0 +k2*kd+k2*k1+2*kd*(  1  )  +g/g_dot,   kp+k1*kd+c*g*g+ 1 +k2*kd+k2*k1+2*kd*(  -1  )  +(g/g_dot)}; 
    // FIRST 3 OUT OF 4 ELEMENTS OF K3 MATRIX... 4TH IS SAME AS FIRST
    double K4[]={k2+k1+kd,k2+kd+k1}; // K4 IS SACALAR

    //saving data
    myfile2<<c1*theta_dot<<" "<<c2*theta<<" "<<thetad_ddot<<setw(20)<<K1[0]<<setw(20)<<K1[1]<<setw(20)<<K2[0]<<setw(20)<<K2[1]<<setw(20)<<K3[0]<<setw(20)<<K3[1]<<setw(20)<<K4[0]<<setw(20)<<K4[1]<<endl;
    
    //az = (1/d3)*(d1*psi_dot-Kp_psi*(psi-psid)-Kd_psi*(psi_dot-psid_dot)+psid_ddot);
    lx = (1/c3)*(c1*theta_dot+c2*theta+thetad_ddot-K1[0]*(x-xd)-K1[1]*(y-yd)-K2[0]*(x_dot-xd_dot)- K2[1]*(y_dot-yd_dot) -K3[0]*(theta-thetad)-K3[1]*(phi-phid)-K4[0]*(theta_dot-thetad_dot)-K4[1]*(phi_dot-phid_dot));

    ly = (1/b3)*(b1*phi_dot+b2*phi+phid_ddot-K1[1]*(x-xd)-(-K1[0])*(y-yd)-K2[1]*(x_dot-xd_dot)-(-K2[0])*(y_dot-yd_dot) -K3[2]*(theta-thetad)-K3[0]*(phi-phid)-K4[1]*(theta_dot-thetad_dot)-(-K4[0])*(phi_dot-phid_dot) );

    move(lx,ly,lz,0,0,0);
    
    //cout<<g<<" "<<g_dot<<endl; // g takes values in (9.6,9.9) and g_dot in (-1.5, 1.6)
    //cout<<g<<" "<<K1[0]<<","<<K1[1]<<","<<K2[0]<<","<<K2[1]<<","<<endl;
    cout<<lx<<" "<<ly<<" "<<lz<<endl;
    ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and wont be able to publish
    loop_rate.sleep();
    }
myfile.close();
}


void basic_movement(){
ros::Rate loop_rate(10);
int exit_val=0;

while(exit_val==0){
cout<<"Press any key"<<endl;
cout<<"q |up|       i        |forward|             e |exit to prev menu|"<<endl;
cout<<"a |down|    jkl  |left|  back |right|       s |land| other |hover|"<<endl;
int l = getch(); //calling non blocking input fucntion
int sval=0.5;

if(l=='q')
move(0,0,1,0,0,0);
else if(l=='a')
move(0,0,-1,0,0,0);
else if(l=='i')
move(1,0,0,0,0,0);
else if(l=='k')
move(-1,0,0,0,0,0);
else if(l=='j')
move(0,1,0,0,0,0);
else if(l=='l')
move(0,-1,0,0,0,0);
else if(l=='e')
exit_val = 1;
else if(l=='s')
land();
else
hover(2);

ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
loop_rate.sleep();
}
}

void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message )
{
	drone_navdata.vx	= 	pose_message->vx;
	drone_navdata.vy 	= 	pose_message->vy;
	drone_navdata.vz 	= 	pose_message->vz;
	drone_navdata.ax	= 	pose_message->ax;
	drone_navdata.ay 	= 	pose_message->ay;
	drone_navdata.az 	= 	pose_message->az;
	drone_navdata.rotX	= 	pose_message->rotX;
	drone_navdata.rotY	= 	pose_message->rotY;
	drone_navdata.rotZ	= 	pose_message->rotZ;
	drone_navdata.magX	= 	pose_message->magX;
	drone_navdata.magY	= 	pose_message->magY;
	drone_navdata.magZ	= 	pose_message->magZ;
	drone_navdata.altd 	= 	pose_message->altd;
	drone_navdata.tm	= 	pose_message->tm;
	drone_navdata.header	= 	pose_message->header;
	drone_navdata.batteryPercent= pose_message->batteryPercent;
}
void SimStateCallback(const nav_msgs::Odometry::ConstPtr & sim_state_message) //gazebo sim callback fn
{
	/*sim_statedata.x	= 	sim_state_message->pose.pose.position.x;
	sim_statedata.y	= 	sim_state_message->pose.pose.position.y;
	sim_statedata.z	= 	sim_state_message->pose.pose.position.z;
	sim_statedata.vx	= 	sim_state_message->twist.twist.linear.x;
	sim_statedata.vy	= 	sim_state_message->twist.twist.linear.y;
	sim_statedata.vz	= 	sim_state_message->twist.twist.linear.z;
	
	sim_statedata.ax	= 	sim_state_message->twist.twist.angular.x;
	sim_statedata.ay	= 	sim_state_message->twist.twist.angular.y;
	sim_statedata.az	= 	sim_state_message->twist.twist.angular.z;
	sim_statedata.qx	= 	sim_state_message->pose.pose.orientation.x;//quaternion orientation x,y,z,w
	sim_statedata.qy	= 	sim_state_message->pose.pose.orientation.y;
	sim_statedata.qz	= 	sim_state_message->pose.pose.orientation.z;
	sim_statedata.qw	= 	sim_state_message->pose.pose.orientation.w;*/
	sim_statedata.pose.pose.position.x	= 	sim_state_message->pose.pose.position.x;
	sim_statedata.pose.pose.position.y	= 	sim_state_message->pose.pose.position.y;
	sim_statedata.pose.pose.position.z	= 	sim_state_message->pose.pose.position.z;
	sim_statedata.twist.twist.linear.x	= 	sim_state_message->twist.twist.linear.x;
	sim_statedata.twist.twist.linear.y	= 	sim_state_message->twist.twist.linear.y;
	sim_statedata.twist.twist.linear.z	= 	sim_state_message->twist.twist.linear.z;
	sim_statedata.twist.twist.angular.x	= 	sim_state_message->twist.twist.angular.x;
	sim_statedata.twist.twist.angular.y	= 	sim_state_message->twist.twist.angular.y;
	sim_statedata.twist.twist.angular.z	= 	sim_state_message->twist.twist.angular.z;
	sim_statedata.pose.pose.orientation.x	= 	sim_state_message->pose.pose.orientation.x;//quaternion orientation x,y,z,w
	sim_statedata.pose.pose.orientation.y	= 	sim_state_message->pose.pose.orientation.y;
	sim_statedata.pose.pose.orientation.z	= 	sim_state_message->pose.pose.orientation.z;
	sim_statedata.pose.pose.orientation.w	= 	sim_state_message->pose.pose.orientation.w;
	
}
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message)
{
	ang.header	= 	imu_message->header;
	ang.angular_velocity	= 	imu_message->angular_velocity;
	ang.linear_acceleration =  imu_message->linear_acceleration;
}

void gpsCallback(const ardrone_autonomy::navdata_gps::ConstPtr & gps_message)
{
	lat	= 	gps_message->lat_fused;
	lon 	= 	gps_message->long_fused;
	ele 	= 	gps_message->elevation;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr & joy )
{
	lx = joy->axes[0];
	ly = joy->axes[1];
	az = joy->axes[2];
	lz = joy->axes[3];
	to = joy->buttons[8];
	la = joy->buttons[9];
}

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

double deg2rad(double angle_in_degrees)
{
	return angle_in_degrees*PI/180.0;
}

double getDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
	return sqrt(pow(x1-x2, 2)+pow(y1-y2, 2)+pow(z1-z2, 2));
}

void takeoff()
{
	ros::Rate loop_rate(10);
	while (ros::ok())
		{
			double init_time=ros::Time::now().toSec();  // epoch time
			double time;
			while (time < (init_time+5.0)) /* Send command for five seconds*/
			{
				T_pub_empty.publish(emp_msg); /* launches the drone */
				ros::spinOnce(); // feedback
				loop_rate.sleep();
				time = ros::Time::now().toSec();
			}//time loop
		ROS_INFO("ARdrone launched");
		exit(0);
		}//ros::ok loop
	hover(2);

}

void land()
{
	ros::Rate loop_rate(10);
	while (ros::ok())
		{
			double init_time=ros::Time::now().toSec();
			double time;
			while (time < (init_time+5.0)) /* Send command for five seconds*/
			{
				L_pub_empty.publish(emp_msg); /* lands the drone */
				ros::spinOnce();
				loop_rate.sleep();
				time = ros::Time::now().toSec();
			}//time loop
		ROS_INFO("ARdrone landed");
		exit(0);
		}//ros::ok loop

}

void hover(int timee)
{

	double t0 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'
	double t1;

	ros::Rate loop_rate(200);

	do{

		t1 = ros::Time::now().toSec(); //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

		vel_msg.linear.x = 0;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 0;

	velocity_publisher.publish(vel_msg);

	ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
	loop_rate.sleep();

	}while(t1 <= (t0+timee));

	//ros::spinOnce();
}

void move(float lx, float ly, float lz, float ax, float ay, float az )
{

	//defining the linear velocity
	vel_msg.linear.x = lx;
	vel_msg.linear.y = ly;
	vel_msg.linear.z = lz;

	//defining the linear velocity
	vel_msg.angular.x = ax;
	vel_msg.angular.y = ay;
	vel_msg.angular.z = az;

	velocity_publisher.publish(vel_msg);

}

void record()
{
	double t0=ros::Time::now().toSec();
	double i =0, f= 200.0, d2r = 3.14159265/180,dt;
	ofstream myfile;
	myfile.open("/home/icgel/icgel7/recorded_data.txt", ios::out | ios::app );
	myfile<<"gx gy gz ax ay az vxo vyo vzo theta phi psi lat lon ele T"<<endl;
	ros::Rate loop_rate(200);
	do{		//if(i<=500)
			move(0.1,0,0,0,0,0);
			/*else if (i>500 && i<1000){
				move(0,0,0,0,0,0);}
			else{
				move(0.1,0,0,0,0,0);}
			*/
			myfile<<ang.angular_velocity.x<<setw(20);
			myfile<<ang.angular_velocity.y<<setw(20);
			myfile<<ang.angular_velocity.z<<setw(20);

			//myfile<<drone_navdata.ax*9.81<<setw(20);
			//myfile<<drone_navdata.ay*9.81<<setw(20);
			//myfile<<drone_navdata.az*9.81<<setw(20);
			myfile<<ang.linear_acceleration.x<<setw(20);
			myfile<<ang.linear_acceleration.y<<setw(20);
			myfile<<ang.linear_acceleration.z<<setw(20);

			myfile<<drone_navdata.vx*0.001<<setw(20);
			myfile<<drone_navdata.vy*0.001<<setw(20);
			myfile<<drone_navdata.vz*0.001<<setw(20);

			myfile<<(drone_navdata.rotY)*d2r<<setw(20);//theta-pitch
			myfile<<(drone_navdata.rotX)*d2r<<setw(20);//phi-roll
			myfile<<(drone_navdata.rotZ)*d2r<<setw(20);
			
			myfile<<setprecision(12)<<lat<<setw(20);
			myfile<<setprecision(12)<<lon<<setw(20);
			myfile<<setprecision(12)<<ele<<setw(20);

			double t1=ros::Time::now().toSec();
			dt = t1-t0;
			myfile<<dt<<endl;

			i = i+1;

			ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
			loop_rate.sleep();
		}while(i<10000);
		myfile.close();
		land();
}
