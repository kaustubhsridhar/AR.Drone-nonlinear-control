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
// including vicons message
#include "geometry_msgs/TransformStamped.h"

// class variables
std_msgs::Empty emp_msg;				// variable in the take off and landing class
geometry_msgs::Twist vel_msg;			// variable in the command velocity class

// Variables for Publishing
ros::Publisher T_pub_empty;				//take off publisher
ros::Publisher L_pub_empty;				//landing publisher
ros::Publisher velocity_publisher;		// velocity publisher

// Variables for Subscribing
ros::Subscriber pose_subscriber;		// ardrone navdata subsciber
ros::Subscriber gps_subscriber;		// ardrone navdata subsciber
ros::Subscriber imu_subscriber;		// ardrone navdata subsciber
ros::Subscriber joy_sub_;
ros::Subscriber sim_state_sub;          //getting state from gazebo simulator
ros::Subscriber vicon_sub;		// vicon data topic subscriber for ardrone1 object
ros::Subscriber vicon_sub_2;		// vicon data topic subscriber for lambo object
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
geometry_msgs::TransformStamped vicon_statedata;  	// Using this class variable for storing state data for ardrone1 from Vicon 
geometry_msgs::TransformStamped vicon_statedata_2;  	// Using this class variable for storing state data for lambo (RC CAR) from Vicon 

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
void ViconStateCallback(const geometry_msgs::TransformStamped::ConstPtr & vicon_state_message); //callback function for vicon data of ardrone1
void ViconStateCallback_2(const geometry_msgs::TransformStamped::ConstPtr & vicon_state_message); //callback function for vicon data of lambo (RC CAR)

// Basic func
void hover(int timee);		// hovering
void takeoff();		// take off
void land();		//landing
void move(float lx, float ly, float lz, float ax, float ay, float az ); // publishing command velocity
// Basic Navigation func
void basic_movement();
// Advanced control functions
void traj_track(int choice);
void vicon_traj_track();
void vicon_follow();
// Misc. functions
//void quickSort(double arr[], int left, int right);
//int binarySearch(double arr[], int l, int r, int x);

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
	//gazebo simulation subscribers
	sim_state_sub = n.subscribe("/ground_truth/state",10,SimStateCallback);
        //joy sub
	joy_sub_ = n.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);
	//vicon data subscriber
	vicon_sub = n.subscribe("/vicon/ardrone1/ardrone1",200,ViconStateCallback);
	vicon_sub_2 = n.subscribe("/vicon/lambo/lambo",200,ViconStateCallback_2);
while(ros::ok())
{

cout<<"Press key"<<endl;
cout<<"w-takeoff | s-land | m-basic movements"<<endl;
cout<<"n-record | h-hover for input time | t - traj track (simulation) | v - traj track (using vicon)"<<endl;
cout<<"p - pid control based traj track (simulation) | f - follow human (using vicon)"<<endl;
//cout<<"t - 8-shape trajectory track (with nonlinear control) | t - 8-shape trajectory track (with PD)"<<endl;

    int c = getch();   // call your non-blocking input function
    int timee;
    double x, y, z;			// for reference value

    switch (c)
    {
    case 't':
			traj_track(0);
			break; 
    case 'p':
			traj_track(1);
			break; 
    case 'v':
			vicon_traj_track();
			break; 
    case 'f':
			vicon_follow();
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
    default:
    			land();
    			break;

    }
//ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and will not be able to publish
}
land();
cout<<"landed";

}

void vicon_follow(){
ofstream myfile;
myfile.open("/home/kaustubh/data/dataf.dat", ios::out | ios::app );
ofstream myfile2;
myfile2.open("/home/kaustubh/data/vicon_2.txt", ios::out | ios::app );
myfile<<"xd"<<setw(20)<<"x"<<setw(20)<<"yd"<<setw(20)<<"y"<<setw(20)<<"zd"<<setw(20)<<"z"<<endl;
myfile2<<"term_1_in_lx. "<<"term_2_in_lx. "<<"term_3_in_lx."<<setw(20)<<"k1"<<setw(20)<<"k1"<<setw(20)<<"k2"<<setw(20)<<"k2"<<setw(20)<<"k3"<<setw(20)<<"k3"<<setw(20)<<"k4"<<setw(20)<<"k4"<<endl;
double d2r = 1*3.14159265/180.0, pi = 3.14159265;
double lx=0,ly=0,lz=0,ax=0,ay=0,az=0;
double a1=3.613,a3=3.384,b1=6.542,b2=25.5,b3=-67.3,c1=4.302,c2=28.24,c3=60.4,d1=4.225,d3=3.828;
double Kpz=1,Kdz=1,Kp_psi=1,Kd_psi=1;
double k1=1.5,k2=1.5,kp=1.25,kd=1.25;//what worked - k1=1.5,k2=1.5,kp=1.25,kd=1.25; loop rate - 10 hz, increment every 32 times
double t=0, T= 60, g=9.8 , g_dot=0;
double b = 0.5, c= 0.5;
double A = 3, A1 = 1.75;
double f=10;
ros::Rate loop_rate(f);

double x0,y0,z0,qx0,qy0,qz0,qw0,theta0,phi0,psi0;
int ctr = 0;
while(ctr<3){
	x0 = vicon_statedata.transform.translation.x, y0 = vicon_statedata.transform.translation.y, z0 = vicon_statedata.transform.translation.z;
	qx0=vicon_statedata.transform.rotation.x, qy0=vicon_statedata.transform.rotation.y, qz0=vicon_statedata.transform.rotation.z, qw0=vicon_statedata.transform.rotation.w;
	theta0 = atan2(+2.0 * (qw0*qx0 + qy0*qz0) , +1.0 - 2.0 * (qx0*qx0 + qy0*qy0));
	phi0 = asin(+2.0 * (qw0*qy0 - qz0*qx0)); 
	psi0 = 0;
	ctr = ctr+1;

	ros::spinOnce(); 
	loop_rate.sleep();
}

double xp=x0,yp=y0,zp=z0,phip=phi0,thetap=theta0,psip=psi0;
double xpd=x0,ypd=x0,xpd_dot=0,xpd_ddot=0,xpd_dddot=0,ypd_dot=0,ypd_ddot=0,ypd_dddot=0;

int ct=0;
while(ros::ok()){
   //actual vals for vicon
    double x = vicon_statedata.transform.translation.x - x0, y = vicon_statedata.transform.translation.y - y0, z = vicon_statedata.transform.translation.z;
    double qx=vicon_statedata.transform.rotation.x, qy=vicon_statedata.transform.rotation.y, qz=vicon_statedata.transform.rotation.z, qw=vicon_statedata.transform.rotation.w;
    double theta = atan2(+2.0 * (qw*qx + qy*qz) , +1.0 - 2.0 * (qx*qx + qy*qy));
    double phi = asin(+2.0 * (qw*qy - qz*qx)); 
    double psi = 0; //psi = atan2(+2.0 * (qw*qz + qx*qy) , +1.0 - 2.0 * (qy*qy + qz*qz));
    double x_dot = f*(x-xp) , y_dot = f*(y-yp) , z_dot = f*(z-zp);
    double phi_dot = f*(phi-phip) , theta_dot= f*(theta-thetap) , psi_dot=0, psi_ddot = 0;//psi_dot=sim_statedata.twist.twist.angular.z
    
    // desired position (i.e current human position)
    double zd=1,psid=0,xd=vicon_statedata_2.transform.translation.x - x0,yd=vicon_statedata_2.transform.translation.y - y0;
    //desired derivatives
    double xd_dot= f*(xd-xpd) , yd_dot = f*(yd-ypd),zd_dot=0;
    double xd_ddot= f*(xd_dot-xpd_dot),yd_ddot= f*(yd_dot-ypd_dot),zd_ddot=0;

    //changing prev pos
    xp=x;yp=y;zp=z;phip=phi;thetap=theta;

    //saving data
    myfile<<xd<<setw(20)<<x<<setw(20)<<yd<<setw(20)<<y<<setw(20)<<zd<<setw(20)<<z<<endl;

    //control in z axis
    lz = (1/a3)*(a1*z_dot-Kpz*(z-zd)-Kdz*(z_dot-zd_dot)+zd_ddot);
    g = 9.8 - a1*z_dot - a3*lz;
    g_dot = -a1*(g-9.8);//-a1*z_ddot;

    //desired phi, theta, psi and derivatives (first and second)
    double phid = (1/g)*(sin(psid)*xd_ddot-cos(psid)*yd_ddot), thetad = (1/g)*(cos(psid)*xd_ddot+sin(psid)*yd_ddot);
    double psid_dot=0, psid_ddot = 0;

    double xd_dddot = f*(xd_ddot-xpd_ddot), yd_dddot = f*(yd_ddot-ypd_ddot);
    double xd_ddddot = f*(xd_dddot-xpd_dddot), yd_ddddot = f*(yd_dddot-ypd_dddot);
    double phid_dot= (1/g)*(sin(psid)*xd_dddot-cos(psid)*yd_dddot), thetad_dot= (1/g)*(cos(psid)*xd_dddot+sin(psid)*yd_dddot);
    double phid_ddot = (1/g)*(sin(psid)*xd_ddddot-cos(psid)*yd_ddddot), thetad_ddot = (1/g)*(cos(psid)*xd_ddddot+sin(psid)*yd_ddddot);

    xpd = xd;ypd=yd;xpd_dot=xd_dot;ypd_dot=yd_dot;
    ypd_ddot=yd_ddot;ypd_dddot=yd_dddot;
    xpd_ddot=xd_ddot;xpd_dddot=xd_dddot;    

    //cont. -- control in x,y plane
    double K1[]={ (1/g)*(cos(psi)*(kp+k2*b*g*g+k2*k1*kp)-sin(psi)*psi_dot*(k2*kp+b*g*g+k1*kp)+kp*(-cos(psi)*psi_dot-sin(psi)*psi_ddot)) + (1/g_dot)*(cos(psi)*(k2*kp+b*g_dot*g_dot+k1*kp)-sin(psi)*psi_dot*(2*kp))              ,           (1/g)*(+sin(psi)*(kp+k2*b*g*g+k2*k1*kp)+cos(psi)*psi_dot*(k2*kp+b*g*g+k1*kp)+kp*(-sin(psi)*psi_dot+cos(psi)*psi_ddot) ) + (1/g_dot)*(sin(psi)*(k2*kp+b*g_dot*g_dot+k1*kp)+cos(psi)*psi_dot*(2*kp))} ;
    // FIRST ROW OF K1 MATRIX, (THIRD = - FIRST AND FOURTH = SECOND)
    double K2[]={ (1/g)*(cos(psi)*(b*g*g+k1*kp+kd+k2*kp+k2*c*g*g+k2*k1*kd)-sin(psi)*psi_dot*(k2*kd+c*g*g+k1*kd+2*kp)+(-cos(psi)*psi_dot-sin(psi)*psi_ddot)*(kd)) + (1/g_dot)*(cos(psi)*(k2*kd+c*g_dot*g_dot+k1*kd+2*kp)-sin(psi)*psi_dot*(2*kd))              ,           (1/g)*(+sin(psi)*(b*g*g+k1*kp+kd+k2*kp+k2*c*g*g+k2*k1*kd)+cos(psi)*(k2*kd+c*g*g+k1*kd+2*kp)+kd*(-sin(psi)*psi_dot+cos(psi)*psi_ddot)) + (1/g_dot)*(sin(psi)*(k2*kd+c*g_dot*g_dot+k1*kd+2*kp)+cos(psi)*(2*kd))} ;
    //second term in K1[1] is negligible . it is the first term that is causing anuisance being -42 in value. IN the first term, it is the cos(psi) multiplied term which is large and causing problems.
    // FIRST ROW OF K2 MATRIX, (THIRD = - FIRST AND FOURTH = SECOND)
    double K3[]={ kp+k1*kd+c*g*g+ 1 +k2*kd+k2*k1+2*kd*(  0  )  +(g/g_dot),   kp+k1*kd+c*g*g+ 0 +k2*kd+k2*k1+2*kd*(  1  )  +g/g_dot,   kp+k1*kd+c*g*g+ 1 +k2*kd+k2*k1+2*kd*(  -1  )  +(g/g_dot)}; 
    // FIRST 3 OUT OF 4 ELEMENTS OF K3 MATRIX... 4TH IS SAME AS FIRST
    double K4[]={k2+k1+kd,k2+kd+k1}; // K4 IS SACALAR

    //saving some more data
    myfile2<<c1*theta_dot<<" "<<c2*theta<<" "<<thetad_ddot<<setw(20)<<K1[0]<<setw(20)<<K1[1]<<setw(20)<<K2[0]<<setw(20)<<K2[1]<<setw(20)<<K3[0]<<setw(20)<<K3[1]<<setw(20)<<K4[0]<<setw(20)<<K4[1]<<endl;
	    
    //az = (1/d3)*(d1*psi_dot-Kp_psi*(psi-psid)-Kd_psi*(psi_dot-psid_dot)+psid_ddot);
    lx = (1/c3)*(c1*theta_dot+c2*theta+thetad_ddot-K1[0]*(x-xd)-K1[1]*(y-yd)-K2[0]*(x_dot-xd_dot)- K2[1]*(y_dot-yd_dot) -K3[0]*(theta-thetad)-K3[1]*(phi-phid)-K4[0]*(theta_dot-thetad_dot)-K4[1]*(phi_dot-phid_dot));

    ly = (1/b3)*(b1*phi_dot+b2*phi+phid_ddot-K1[1]*(x-xd)-(-K1[0])*(y-yd)-K2[1]*(x_dot-xd_dot)-(-K2[0])*(y_dot-yd_dot) -K3[2]*(theta-thetad)-K3[0]*(phi-phid)-K4[1]*(theta_dot-thetad_dot)-(-K4[0])*(phi_dot-phid_dot) );

    move(lx,ly,lz,0,0,0);
    cout<<lx<<" "<<ly<<" "<<lz<<endl;

    ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and wont be able to publish
    loop_rate.sleep();
    }
myfile.close();
}


void vicon_traj_track(){
ofstream myfile;
myfile.open("/home/kaustubh/data/dataf.dat", ios::out | ios::app );
ofstream myfile2;
myfile2.open("/home/kaustubh/data/vicon_2.txt", ios::out | ios::app );
myfile<<"xd"<<setw(20)<<"x"<<setw(20)<<"yd"<<setw(20)<<"y"<<setw(20)<<"zd"<<setw(20)<<"z"<<endl;
myfile2<<"term_1_in_lx. "<<"term_2_in_lx. "<<"term_3_in_lx."<<setw(20)<<"k1"<<setw(20)<<"k1"<<setw(20)<<"k2"<<setw(20)<<"k2"<<setw(20)<<"k3"<<setw(20)<<"k3"<<setw(20)<<"k4"<<setw(20)<<"k4"<<endl;
double d2r = 1*3.14159265/180.0, pi = 3.14159265;
double lx=0,ly=0,lz=0,ax=0,ay=0,az=0;
double a1=3.613,a3=3.384,b1=6.542,b2=25.5,b3=-67.3,c1=4.302,c2=28.24,c3=60.4,d1=4.225,d3=3.828;
double Kpz=1,Kdz=1,Kp_psi=1,Kd_psi=1;
double k1=1.5,k2=1.5,kp=1.25,kd=1.25;//what worked - k1=1.5,k2=1.5,kp=1.25,kd=1.25; loop rate - 10 hz, increment every 32 times
double t=0, T= 60, g=9.8 , g_dot=0;
double b = 0.5, c= 0.5;
double A = 3, A1 = 1.75;

ros::Rate loop_rate(10);

double x0,y0,z0,qx0,qy0,qz0,qw0,theta0,phi0,psi0;
int ctr = 0;
while(ctr<3){
	x0 = vicon_statedata.transform.translation.x, y0 = vicon_statedata.transform.translation.y, z0 = vicon_statedata.transform.translation.z;
	qx0=vicon_statedata.transform.rotation.x, qy0=vicon_statedata.transform.rotation.y, qz0=vicon_statedata.transform.rotation.z, qw0=vicon_statedata.transform.rotation.w;
	theta0 = atan2(+2.0 * (qw0*qx0 + qy0*qz0) , +1.0 - 2.0 * (qx0*qx0 + qy0*qy0));
	phi0 = asin(+2.0 * (qw0*qy0 - qz0*qx0)); 
	psi0 = 0;
	ctr = ctr+1;

	ros::spinOnce(); 
	loop_rate.sleep();
}

double xp=x0,yp=y0,zp=z0,phip=phi0,thetap=theta0,psip=psi0;

int ct=0;
while(ros::ok()){
   //actual vals for vicon
    double x = vicon_statedata
.transform.translation.x - x0, y = vicon_statedata.transform.translation.y - y0, z = vicon_statedata.transform.translation.z;
    
    double qx=vicon_statedata.transform.rotation.x, qy=vicon_statedata.transform.rotation.y, qz=vicon_statedata.transform.rotation.z, qw=vicon_statedata.transform.rotation.w;
    double theta = atan2(+2.0 * (qw*qx + qy*qz) , +1.0 - 2.0 * (qx*qx + qy*qy));
    double phi = asin(+2.0 * (qw*qy - qz*qx)); 
    double psi = 0; //psi = atan2(+2.0 * (qw*qz + qx*qy) , +1.0 - 2.0 * (qy*qy + qz*qz));
    double x_dot = 10*(x-xp) , y_dot = 10*(y-yp) , z_dot = 10*(z-zp);
    double phi_dot = 10*(phi-phip) , theta_dot= 10*(theta-thetap) , psi_dot=0, psi_ddot = 0;//psi_dot=sim_statedata.twist.twist.angular.z
    
    //8 shape desired position
    double zd=1,psid=0,xd=A-A*cos(2*pi*t/T),yd=A1*sin(4*pi*t/T);//zd=1.3+0.3*sin(2*pi*t/T + 3*pi/2),psid=sin(2*pi*t/T);
    //desired derivatives
    double xd_dot=(A*2*pi/T)*sin(2*pi*t/T) ,yd_dot=(A1*4*pi/T)*cos(4*pi*t/T) ,zd_dot=0;//zd_dot=(0.3*2*pi/T)*cos(2*pi*t/T + 3*pi/2);
    double xd_ddot=(A*2*2*pi*pi/(T*T))*cos(2*pi*t/T) ,yd_ddot=-(A1*4*4*pi*pi/(T*T))*sin(4*pi*t/T) ,zd_ddot=0;//zd_ddot=-(0.3*4*pi*pi/(T*T))*sin(2*pi*t/T + 3*pi/2); 

    //updating desired value
    if(ct%10==0 && ((x-xd)*(x-xd) + (y-yd)*(y-yd))<0.4){
    	t = t + 1;
	ct++;}
    else if(ct%10!=0)
	ct++;
    //changing prev pos
    xp=x;yp=y;zp=z;phip=phi;thetap=theta;

    //saving data
    myfile<<xd<<setw(20)<<x<<setw(20)<<yd<<setw(20)<<y<<setw(20)<<zd<<setw(20)<<z<<endl;

    //control
    lz = (1/a3)*(a1*z_dot-Kpz*(z-zd)-Kdz*(z_dot-zd_dot)+zd_ddot);
    g = 9.8 - a1*z_dot - a3*lz;
    g_dot = -a1*(g-9.8);//-a1*z_ddot;

    //desired phi, theta, psi and derivatives (first and second)
    double phid = (1/g)*(sin(psid)*xd_ddot-cos(psid)*yd_ddot), thetad = (1/g)*(cos(psid)*xd_ddot+sin(psid)*yd_ddot);
    double psid_dot=0, psid_ddot = 0;//double psid_dot=(2*pi/T)*cos(2*pi*t/T), psid_ddot=-(4*pi*pi/(T*T))*sin(2*pi*t/T);
    
    double xd_dddot = (-A*2*2*2*pi*pi*pi/(T*T*T))*sin(2*pi*t/T), yd_dddot = -(A1*4*4*4*pi*pi*pi/(T*T*T))*cos(4*pi*t/T);
    double xd_ddddot = (-A*2*2*2*2*pi*pi*pi*pi/(T*T*T*T))*cos(2*pi*t/T), yd_ddddot = (A1*4*4*4*4*pi*pi*pi*pi/(T*T*T*T))*sin(4*pi*t/T);

    double phid_dot= (1/g)*(sin(psid)*xd_dddot-cos(psid)*yd_dddot), thetad_dot= (1/g)*(cos(psid)*xd_dddot+sin(psid)*yd_dddot);

    double phid_ddot = (1/g)*(sin(psid)*xd_ddddot-cos(psid)*yd_ddddot), thetad_ddot = (1/g)*(cos(psid)*xd_ddddot+sin(psid)*yd_ddddot);    

    //cont. -- control in x,y plane
    double K1[]={ (1/g)*(cos(psi)*(kp+k2*b*g*g+k2*k1*kp)-sin(psi)*psi_dot*(k2*kp+b*g*g+k1*kp)+kp*(-cos(psi)*psi_dot-sin(psi)*psi_ddot)) + (1/g_dot)*(cos(psi)*(k2*kp+b*g_dot*g_dot+k1*kp)-sin(psi)*psi_dot*(2*kp))              ,           (1/g)*(+sin(psi)*(kp+k2*b*g*g+k2*k1*kp)+cos(psi)*psi_dot*(k2*kp+b*g*g+k1*kp)+kp*(-sin(psi)*psi_dot+cos(psi)*psi_ddot) ) + (1/g_dot)*(sin(psi)*(k2*kp+b*g_dot*g_dot+k1*kp)+cos(psi)*psi_dot*(2*kp))} ;
    // FIRST ROW OF K1 MATRIX, (THIRD = - FIRST AND FOURTH = SECOND)
    double K2[]={ (1/g)*(cos(psi)*(b*g*g+k1*kp+kd+k2*kp+k2*c*g*g+k2*k1*kd)-sin(psi)*psi_dot*(k2*kd+c*g*g+k1*kd+2*kp)+(-cos(psi)*psi_dot-sin(psi)*psi_ddot)*(kd)) + (1/g_dot)*(cos(psi)*(k2*kd+c*g_dot*g_dot+k1*kd+2*kp)-sin(psi)*psi_dot*(2*kd))              ,           (1/g)*(+sin(psi)*(b*g*g+k1*kp+kd+k2*kp+k2*c*g*g+k2*k1*kd)+cos(psi)*(k2*kd+c*g*g+k1*kd+2*kp)+kd*(-sin(psi)*psi_dot+cos(psi)*psi_ddot)) + (1/g_dot)*(sin(psi)*(k2*kd+c*g_dot*g_dot+k1*kd+2*kp)+cos(psi)*(2*kd))} ;
    //second term in K1[1] is negligible . it is the first term that is causing anuisance being -42 in value. IN the first term, it is the cos(psi) multiplied term which is large and causing problems.
    // FIRST ROW OF K2 MATRIX, (THIRD = - FIRST AND FOURTH = SECOND)
    double K3[]={ kp+k1*kd+c*g*g+ 1 +k2*kd+k2*k1+2*kd*(  0  )  +(g/g_dot),   kp+k1*kd+c*g*g+ 0 +k2*kd+k2*k1+2*kd*(  1  )  +g/g_dot,   kp+k1*kd+c*g*g+ 1 +k2*kd+k2*k1+2*kd*(  -1  )  +(g/g_dot)}; 
    // FIRST 3 OUT OF 4 ELEMENTS OF K3 MATRIX... 4TH IS SAME AS FIRST
    double K4[]={k2+k1+kd,k2+kd+k1}; // K4 IS SACALAR

    //saving some more data
    myfile2<<c1*theta_dot<<" "<<c2*theta<<" "<<thetad_ddot<<setw(20)<<K1[0]<<setw(20)<<K1[1]<<setw(20)<<K2[0]<<setw(20)<<K2[1]<<setw(20)<<K3[0]<<setw(20)<<K3[1]<<setw(20)<<K4[0]<<setw(20)<<K4[1]<<endl;
	    
    //az = (1/d3)*(d1*psi_dot-Kp_psi*(psi-psid)-Kd_psi*(psi_dot-psid_dot)+psid_ddot);
    lx = (1/c3)*(c1*theta_dot+c2*theta+thetad_ddot-K1[0]*(x-xd)-K1[1]*(y-yd)-K2[0]*(x_dot-xd_dot)- K2[1]*(y_dot-yd_dot) -K3[0]*(theta-thetad)-K3[1]*(phi-phid)-K4[0]*(theta_dot-thetad_dot)-K4[1]*(phi_dot-phid_dot));

    ly = (1/b3)*(b1*phi_dot+b2*phi+phid_ddot-K1[1]*(x-xd)-(-K1[0])*(y-yd)-K2[1]*(x_dot-xd_dot)-(-K2[0])*(y_dot-yd_dot) -K3[2]*(theta-thetad)-K3[0]*(phi-phid)-K4[1]*(theta_dot-thetad_dot)-(-K4[0])*(phi_dot-phid_dot) );

    move(lx,ly,lz,0,0,0);
    cout<<lx<<" "<<ly<<" "<<lz<<endl;

    ros::spinOnce(); //if this function is not mentioned the function will stay in the buffer and wont be able to publish
    loop_rate.sleep();
    }
myfile.close();
}



void traj_track(int choice){
ofstream myfile;
myfile.open("/home/kaustubh/data/dataf.dat", ios::out | ios::app );
ofstream myfile2;
myfile2.open("/home/kaustubh/data/vicon_2.txt", ios::out | ios::app );
myfile<<"xd"<<setw(20)<<"x"<<setw(20)<<"yd"<<setw(20)<<"y"<<setw(20)<<"zd"<<setw(20)<<"z"<<endl;
myfile2<<"term_1_in_lx. "<<"term_2_in_lx. "<<"term_3_in_lx."<<setw(20)<<"k1"<<setw(20)<<"k1"<<setw(20)<<"k2"<<setw(20)<<"k2"<<setw(20)<<"k3"<<setw(20)<<"k3"<<setw(20)<<"k4"<<setw(20)<<"k4"<<endl;
double d2r = 1*3.14159265/180.0, pi = 3.14159265;
double lx=0,ly=0,lz=0,ax=0,ay=0,az=0;
double a1=3.613,a3=3.384,b1=6.542,b2=25.5,b3=-67.3,c1=4.302,c2=28.24,c3=60.4,d1=4.225,d3=3.828;
double Kpz=1,Kdz=1,Kp_psi=1,Kd_psi=1;
//backstepping control constants
double k1=1.5,k2=1.5,kp=1.25,kd=1.25;//what worked - k1=1.5,k2=1.5,kp=1.25,kd=1.25; loop rate - 10 hz, increment every 32 times
double t=0, T= 60, g=9.8 , g_dot=0;
double b = 0.5, c= 0.5;
//PID control constants
double ktheta_p = 50,ktheta_d = 50,kphi_p = 50,kphi_d = 50;


double xp = sim_statedata.pose.pose.position.x , yp = sim_statedata.pose.pose.position.y , zp = sim_statedata.pose.pose.position.z; 
double phip=(drone_navdata.rotX)*d2r,thetap=(drone_navdata.rotY)*d2r,psip = (drone_navdata.rotZ)*d2r;

ros::Rate loop_rate(10);
int ct=0;
while(ros::ok()){

    //actual vals for gazebo sim
    double x = sim_statedata.pose.pose.position.x , y = sim_statedata.pose.pose.position.y , z = sim_statedata.pose.pose.position.z; 
    //double phi=(drone_navdata.rotX)*d2r,theta=(drone_navdata.rotY)*d2r, psi=0;//psi=(drone_navdata.rotZ)*d2r;
    double qx=sim_statedata.pose.pose.orientation.x, qy=sim_statedata.pose.pose.orientation.y, qz=sim_statedata.pose.pose.orientation.z, qw=sim_statedata.pose.pose.orientation.w;
    double theta = atan2(+2.0 * (qw*qx + qy*qz) , +1.0 - 2.0 * (qx*qx + qy*qy));
    double phi = asin(+2.0 * (qw*qy - qz*qx)); 
    double psi = 0; //psi = atan2(+2.0 * (qw*qz + qx*qy) , +1.0 - 2.0 * (qy*qy + qz*qz));
    double x_dot = sim_statedata.twist.twist.linear.x , y_dot = sim_statedata.twist.twist.linear.y , z_dot = sim_statedata.twist.twist.linear.z;
    double phi_dot=sim_statedata.twist.twist.angular.x , theta_dot=sim_statedata.twist.twist.angular.y, psi_dot=0, psi_ddot = 0;//psi_dot=sim_statedata.twist.twist.angular.z
    
    //8 shape desired position
    double zd=1,psid=0,xd=5-5*cos(2*pi*t/T),yd=2.5*sin(4*pi*t/T);//xd=5-5*cos(2*pi*t/T), zd=1.3+0.3*sin(2*pi*t/T + 3*pi/2),psid=sin(2*pi*t/T);
    //desired derivatives
    double xd_dot=(+10*pi/T)*sin(2*pi*t/T) ,yd_dot=(10*pi/T)*cos(4*pi*t/T) ,zd_dot=0;//zd_dot=(0.3*2*pi/T)*cos(2*pi*t/T + 3*pi/2);
    double xd_ddot=(+20*pi*pi/(T*T))*cos(2*pi*t/T) ,yd_ddot=-(40*pi*pi/(T*T))*sin(4*pi*t/T) ,zd_ddot=0;//zd_ddot=-(0.3*4*pi*pi/(T*T))*sin(2*pi*t/T + 3*pi/2); 

    cout<<((x-xd)*(x-xd) + (y-yd)*(y-yd))<<endl;
    /*if(ct%32==0)
	t=t+1;
    ct++;*/
    if(ct%32==0 && ((x-xd)*(x-xd) + (y-yd)*(y-yd))<0.4){
    	t = t + 1;
	ct++;}
    else if(ct%32!=0)
	ct++;
    //changing prev pos
    xp=x;yp=y;zp=z;phip=phi;thetap=theta;


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

    //cont. -- control in x,y plane
    // choice = 0 -> Backstepping Control
    if(choice == 0)
    {

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

    }

    // choice = 1 -> PID Control
    if(choice == 1)
    {	    
	    double theta_ddot = -ktheta_p*(theta- thetad) - ktheta_d*(theta_dot - thetad_dot) + thetad_ddot;
	    double phi_ddot = -kphi_p*(phi- phid) - kphi_d*(phi_dot - phid_dot) + phid_ddot;	 		
	    lx = (1/c3)*(c1*theta_dot+c2*theta+theta_ddot);
	    ly = (1/b3)*(b1*phi_dot+b2*phi+phi_ddot);
    }

    
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
cout<<"Press any key"<<endl;
cout<<"q |up|       i        |forward|             e |exit to prev menu|"<<endl;
cout<<"a |down|    jkl  |left|  back |right|       s |land| other |hover|"<<endl;

while(exit_val==0){
int l = getch(); //calling non blocking input fucntion
int sval=0.5;
double t0 = 0,t1 = 0;
if(l=='q'){
	t0 = ros::Time::now().toSec();
	do{
	t1 = ros::Time::now().toSec();
	move(0,0,1,0,0,0);
	}while(t1 <= (t0+0.5));}
else if(l=='a'){
	t0 = ros::Time::now().toSec();
	do{
	t1 = ros::Time::now().toSec();
	move(0,0,-1,0,0,0);
	}while(t1 <= (t0+0.5));}
else if(l=='i'){
	t0 = ros::Time::now().toSec();
	do{
	t1 = ros::Time::now().toSec();
	move(1,0,0,0,0,0);
	}while(t1 <= (t0+0.5));}
else if(l=='k'){
	t0 = ros::Time::now().toSec();
	do{
	t1 = ros::Time::now().toSec();
	move(-1,0,0,0,0,0);
	}while(t1 <= (t0+0.5));}
else if(l=='j'){
	t0 = ros::Time::now().toSec();
	do{
	t1 = ros::Time::now().toSec();
	move(0,1,0,0,0,0);
	}while(t1 <= (t0+0.5));}
else if(l=='l'){
	t0 = ros::Time::now().toSec();
	do{
	t1 = ros::Time::now().toSec();
	move(0,-1,0,0,0,0);
	}while(t1 <= (t0+0.5));}
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
void ViconStateCallback(const geometry_msgs::TransformStamped::ConstPtr & vicon_state_message)
{
	vicon_statedata.transform.translation.x	= 	vicon_state_message->transform.translation.x;
	vicon_statedata.transform.translation.y	= 	vicon_state_message->transform.translation.y;
	vicon_statedata.transform.translation.z	= 	vicon_state_message->transform.translation.z;
	
	vicon_statedata.transform.rotation.x	= 	vicon_state_message->transform.rotation.x;//quaternion orientation x,y,z,w
	vicon_statedata.transform.rotation.y	= 	vicon_state_message->transform.rotation.y;
	vicon_statedata.transform.rotation.z	= 	vicon_state_message->transform.rotation.z;
	vicon_statedata.transform.rotation.w	= 	vicon_state_message->transform.rotation.w;
	
}

void ViconStateCallback_2(const geometry_msgs::TransformStamped::ConstPtr & vicon_state_message) // for lambo (RC CAR)
{
	vicon_statedata_2.transform.translation.x	= 	vicon_state_message->transform.translation.x;
	vicon_statedata_2.transform.translation.y	= 	vicon_state_message->transform.translation.y;
	//vicon_statedata_2.transform.translation.z	= 	vicon_state_message->transform.translation.z;
	
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

/*void quickSort(double arr[], int left, int right) {
      int i = left, j = right;
      int tmp;
      int pivot = arr[(left + right) / 2];
 
      //partition
      while (i <= j) {
            while (arr[i] < pivot)
                  i++;
            while (arr[j] > pivot)
                  j--;
            if (i <= j) {
                  tmp = arr[i];
                  arr[i] = arr[j];
                  arr[j] = tmp;
                  i++;
                  j--;
            }
      };
 
      //recursion
      if (left < j)
            quickSort(arr, left, j);
      if (i < right)
            quickSort(arr, i, right);
}

int binarySearch(double arr[], int l, int r, int x) //l-left, r-right, x-element searched for
{
   if (r >= l)
   {
        int mid = l + (r - l)/2;
        if (arr[mid] == x)  return mid;
        if (arr[mid] > x) return binarySearch(arr, l, mid-1, x);
        return binarySearch(arr, mid+1, r, x);
   }
   return -1; // if element not present in array
}
*/
