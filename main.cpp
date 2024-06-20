/** 
* This code doesnt consider the time delay caused by the communication and the solving process;
* The failure to solve is considered but it is assumed the time it costs is ignorable even if it fails to solve;
**/



#include "batch_solver.h"
#include <iostream>
#include "ros/ros.h"
#include "utility.h"
#include <visualization_msgs/Marker.h>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include "mymsg/neighborpos.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h" //added new data type to send
#include "std_msgs/Bool.h"
#include "geometry_msgs/Vector3.h"
#include <Eigen/Dense>
#include <signal.h>
#include <vector>
#include <cmath>
#include <iomanip>
#include <cstdint>

# define PI 3.14159265358979323846

size_t N = 25;
size_t m = 1;// represent the num of neighbors for this vehicle
double Hz = 2.0;
size_t update_shift = 0;

double xr;
double yr;
double thetar ;
 
int bcwd = 0; //backward initialize
int co = 0; //time counter for backward
int delay = 10; //time delay for backward
double backspeed = -0.3;
int pnum = 8;
int objnum = 4;

//defining 4 objtect's position variables
double xobj[5]={0,0,0,0,0}; 
double yobj[5]={0,0,0,0,0};
double thetaobj[5] = {0,0,0,0,0};

std::mutex neig_mtx;
std::mutex init_mtx;
//std::mutex obj_mtx;

double xinit;
double yinit;
double thetainit;

//Define the Tolerance (the region where robot is considered to be within an object's range)
double tolerance = 0.2; 
double tolerance1 = 0.13;  //for object catching
double tolerance2 = 0.13;  //for backward



//Dummy Variable
int gope = 0;
int distfunc = 0;
int datacounter = 0; //for robot 2's rec data flag

//robot1 current position
double xrob1 = 0, yrob1 = 0;
int task = 0;

//defining 4 objtect's position variables
// double xobj1, xobj2, xobj3, xobj4; 
// double yobj1, yobj2, yobj3, yobj4;
//double thetaobj1, thetaobj2, thetaobj3, thetaobj4;

//define minimum object and place number

int min_object_1 = -1, min_place_1 = -8, min_object_2 = -1, min_place_2 = -8;
//int min_object_1, min_place_1, min_object_2, min_place_2;

//Temporary Flags
bool rec_data[4] = {false, false, false, false};


//Global Variables associated with the BestPathSelector Function (Incomplete)

    std_msgs::UInt16 command; //Starting from LSB, first 8 bits for position, the next 4 for objects
    //Sender Data for robot1
	std_msgs::Bool robot1_busy; //a flag to tell the other robot that it has completed its task

	//robot task flags
	bool object_taken = false;
	bool place_reached = false;

	//object filled variable
	bool obj_filled[4] = {false, false, false, false}; 

	//Place filled Variable
	bool place_filled[8] = {false, false, false, false, false, false, false, false};

	//define object coordinates
	double object[4][2] =          //define position of the object with array
		{
			{0, 0},            //object1
			{0, 0},            //object2
			{0, 0},            //object3
			{0, 0},             //object4
		};

		double theta[4] = {0,0,0,0};
//callback robot_03
	double place[8][2] =           //define position of place with vector
		{  
            {-0.6, 0.66},			   //p1
			{-0.6, 0.32},              //p2
			{-0.6, 0.02},              //p3
			{-0.6, -0.36},             //p4
			{0.9, 0.66},               //p5
			{0.9, 0.32},               //p6
			{0.9, 0.02},               //p7
			{0.9, -0.36},               //p8
		};

		//double place[8][2];





// ******protected by neig_mtx
std::vector<std::vector<std::vector<double>>> neig;
 

std::vector<std::vector<double>> pre_states(N+1,std::vector<double>(3.0,0.0));
std::vector<std::vector<double>> pre_inputs(N+1,std::vector<double>(2.0,0.0));

std::vector<std::pair<double, double>> vec;

bool solve_success = false;

// each vehicle may have different solving time for the intial solution;
// some of them might be quite long; So these flags are used for synchronous purpose;
bool first_solution_v1 = false;
bool first_solution_v2 = false;

double d = 0.11; //0.11
double ts = 0.5;//0.5
double safety_dist = 0.15; //0.15
	
Eigen::Matrix4d Twc; // camera frame in world frame

std::vector<std::vector<double>> obst;

std::shared_ptr<BatchSolver> bs(new BatchSolver(N,xr, yr, thetar, d, xinit,yinit, thetainit, ts, safety_dist,obst,neig));

ros::Publisher vehicle_pub;// for visulization
ros::Publisher markerArray;
ros::Publisher neig_pub;// tells other robots my pos
ros::Publisher control_pub;
ros::Publisher central_command; //gives command to the other robot
ros::Subscriber sub1;
ros::Subscriber sub2;
ros::Subscriber localization_sub;
ros::Subscriber localization_obj1_sub;
ros::Subscriber localization_obj2_sub;
ros::Subscriber localization_obj3_sub;
ros::Subscriber localization_obj4_sub;
ros::Subscriber robot1_busy_sub;


void UpdateVisualize();
void UpdateNeighborsPos();
void Initialize(ros::NodeHandle& n);
void NeighborCallback1(const mymsg::neighborpos& msg);
void NeighborCallback2(const mymsg::neighborpos& msg);
void LocalizationCallback(const apriltag_ros::AprilTagDetectionArray& msg);
void LocalizationCallback1(const apriltag_ros::AprilTagDetectionArray& msg);
void LocalizationCallback2(const apriltag_ros::AprilTagDetectionArray& msg);
void LocalizationCallback3(const apriltag_ros::AprilTagDetectionArray& msg);
void LocalizationCallback4(const apriltag_ros::AprilTagDetectionArray& msg);
// void BestPathSelector();
void SendData();
void Robot1_Busy(const std_msgs::Bool::ConstPtr& msg);
bool DistanceCalcualtor();
void Retreat();
void ObstacleAvoidance();
//void dectobinary(int n);

void mySigintHandler(int sig)
{
	std::cout << "Experiment stopped by user. Stopping Robot..." << std::endl;
	geometry_msgs::Vector3 msg_con;	
	msg_con.x = 0;
	msg_con.y = 0;
	msg_con.z = 0.0;
	control_pub.publish(msg_con);
	std::cout << "THE END." << std::endl;
	ros::shutdown();
}

int main(int argc,char* argv[]){
	ros::init(argc,argv,"vehicle03");
	ros::NodeHandle n;
	signal(SIGINT,mySigintHandler);
   	ros::Rate loop_rate(2);
	Initialize(n);
	std::thread sim_thread(&UpdateVisualize);
	sim_thread.detach();
	
	robot1_busy.data = false; //robot 2  is not busy



	while(ros::ok())
	{
    	/* Optimal Path Selection Function To Be Called Here (hopelessly incomplete)
		//decide the best place
		BestPathSelector();
    	*/
		ObstacleAvoidance();
		if(rec_data[0] == true || rec_data[1] == true || rec_data[2] == true || rec_data[3] == true){
			
		//std::cout <<"AAAAAAAAAAA" << min_object_1 << std::endl << min_place_1 << std::endl; //print obj and place num
			if(task == 0){
				while(DistanceCalcualtor() && ros::ok()){ros::spinOnce();}//wait until object is detected
				std::cout << "dsdsd"<< command.data << std::endl; 
				ObstacleAvoidance();
				//DistanceCalcualtor();
				xr = object[min_object_1][0]; yr = object[min_object_1][1];
				bs->set_ref_states(xr,yr, thetainit);
				task++;
			}
			// && (abs(thetainit - theta[min_object_1]) < 0.5)
			if(task == 1 && ((xinit<xr+tolerance1 && xinit>xr-tolerance1) && (yinit<yr+tolerance1 && yinit>yr-tolerance1))){
				co = 1;
				ObstacleAvoidance();
				obj_filled[min_object_1] = true;
				xr = place[min_place_1][0]; yr = place[min_place_1][1];
				bs->set_ref_states(xr,yr,thetainit+0.1);
				task++;
			} 
             if(task == 2 && ((xinit<xr+tolerance && xinit>xr-tolerance) && (yinit<yr+tolerance && yinit>yr-tolerance))){
				ObstacleAvoidance();
				if(min_place_1>3){
					if(thetainit<-0.5&&thetainit>-1)
					{
						Retreat(); //backward moving
						ObstacleAvoidance();
						place_filled[min_place_1] = true; //set this place as filled			
						bs->set_ref_states(xinit,yinit,thetainit);
						task = 0;
						gope = 999; //random (to test whether this if condition runs simultanously with robot's reversing)
					}
				}
				else{
					if(thetainit>2.6||thetainit<-2.6)
					{
						Retreat(); //backward moving
						ObstacleAvoidance();
						place_filled[min_place_1] = true; //set this place as filled			
						bs->set_ref_states(xinit,yinit,thetainit);
						task = 0;
						gope = 999; //random (to test whether this if condition runs simultanously with robot's reversing)
					}
				}
			}

		} 
		ros::spinOnce();
        loop_rate.sleep();
	}

    geometry_msgs::Vector3 msg_con;
	msg_con.x = 0.0;
	msg_con.y = 0.0;
	msg_con.z = 0.0;
	control_pub.publish(msg_con);
	return 0;

};

void Initialize(ros::NodeHandle& n){

	xinit=0;yinit=1;thetainit=-PI/2;
	xr=0;yr=1;thetar=-PI/2;             //Initialization!!
	std::vector<double> obst1 = {0.0,0.0};	
	std::vector<std::vector<double>> neig1(N+1,std::vector<double>{0.5,0.25});//v1
	std::vector<std::vector<double>> neig2(N+1,std::vector<double>{-0.5,0.35});//v2
	//obst.push_back(obst1);
	neig.push_back(neig1);	
	neig.push_back(neig2);	
	bs->set_obst_(obst);
	//bs->set_ref_states(xr,yr,thetar);
	//bs->set_initial_states(xinit,yinit,thetainit);
	bs->set_neighbors(neig,neig_mtx);



	vehicle_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    markerArray = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
	neig_pub= n.advertise<mymsg::neighborpos>("neig3pos", 10);
	control_pub = n.advertise<geometry_msgs::Vector3>("control_signal3",10);
	central_command = n.advertise<std_msgs::UInt16>("send_command", 10);
        sub1 = n.subscribe("neig1pos", 10, NeighborCallback1);
        sub2 = n.subscribe("neig2pos", 10, NeighborCallback2);
        localization_sub = n.subscribe("tag_detections", 10, LocalizationCallback);
		localization_obj1_sub = n.subscribe("tag_detections", 10, LocalizationCallback1);
		localization_obj2_sub = n.subscribe("tag_detections", 10, LocalizationCallback2);
		localization_obj3_sub = n.subscribe("tag_detections", 10, LocalizationCallback3);
		localization_obj4_sub = n.subscribe("tag_detections", 10, LocalizationCallback4);
		robot1_busy_sub = n.subscribe("robot_busy_topic", 10, Robot1_Busy);



	// x 180
	Twc << 1.0 , 0.0 , 0.0, 0.0, 
	      0.0 , cos(PI) , -sin(PI),0.0,
	      0.0 , sin(PI) , cos(PI),1.27,
	      0.0,0.0,0.0,1.0;
};
void UpdateVisualize(){

	ros::Rate loop_rate_sim(Hz);

	ros::spinOnce();
	loop_rate_sim.sleep();

	while(ros::ok()){
		ObstRviz(obst,safety_dist,markerArray);
		TrajRviz(pre_states, safety_dist,markerArray);
		if(1){	
			std::lock_guard<std::mutex> lk(init_mtx);
			VehicleRviz(xinit,yinit,thetainit,safety_dist,vehicle_pub);
			HeadingRviz(xinit,yinit,thetainit,safety_dist,vehicle_pub);
			bs->set_initial_states(xinit,yinit,thetainit);
		}

			
		bs->set_neighbors(neig,neig_mtx);
		bs->Solve(pre_states,pre_inputs,solve_success);

		//while(first_solution_v1 == false){ std::cout<<"ddddd"<<std::endl;};

		if(solve_success){
			update_shift = 0;

			std::vector<double> pre_x;
			std::vector<double> pre_y;
			for(int i = 0 ; i < pre_states.size(); i++){
				pre_x.push_back(pre_states[i][0]);
				pre_y.push_back(pre_states[i][1]);
			}
   			mymsg::neighborpos msg;
   			msg.xpos = pre_x;
   			msg.ypos = pre_y;
    			msg.time_stamp = ros::Time::now().toSec();
			neig_pub.publish(msg);

            if(bcwd==0)
			{
				geometry_msgs::Vector3 msg_con;  // forward moving signal publish
				msg_con.x = pre_inputs[0][0];
				msg_con.y = pre_inputs[0][1];
				msg_con.z = 0.0;
				control_pub.publish(msg_con);
			}
			if(bcwd==1)
			{
				geometry_msgs::Vector3 msg_con;  // backward moving signal publish
				msg_con.x = backspeed;
				msg_con.y = backspeed;
				msg_con.z = 0.0;
				control_pub.publish(msg_con);
				if(co == 6)
					bcwd=0;
				co = co + 1;
			}

		}else{// if fail to solve, publish the shifted pre_states
			std::cout<<" fail to solve!!!!!"<<std::endl;
			exit(0);

		}

		ros::spinOnce();// send the solution ASAP after the solving
		loop_rate_sim.sleep();

	
	}
	std::cout << "Experiment stopped by user. Stopping Robot..." << std::endl;
	geometry_msgs::Vector3 msg_con;	
	msg_con.x = 0;
	msg_con.y = 0;
	msg_con.z = 0.0;
	control_pub.publish(msg_con);
	std::cout << "THE END." << std::endl;

};


void NeighborCallback1(const mymsg::neighborpos& msg)
{
std::cout<<"Data retrieved from robot 1!"<<std::endl;
	std::vector<double> x = msg.xpos;
	std::vector<double> y = msg.ypos;
	std::lock_guard<std::mutex> lk(neig_mtx);
	for(size_t i = 1; i< neig[0].size();i++){
		neig[0][i-1][0] = x[i];
		neig[0][i-1][1] = y[i];
	}
	neig[0][neig[0].size()-1][0] = neig[0][neig[0].size()-2][0];
	neig[0][neig[0].size()-1][1] = neig[0][neig[0].size()-2][1];
	first_solution_v1 = true;
};


void NeighborCallback2(const mymsg::neighborpos& msg)
{
std::cout<<"Data retrieved from robot 2!"<<std::endl;

	std::vector<double> x = msg.xpos;
	std::vector<double> y = msg.ypos;
	std::lock_guard<std::mutex> lk(neig_mtx);
	for(int i = 1; i< neig[0].size();i++){
		neig[1][i-1][0] = x[i];
		neig[1][i-1][1] = y[i];
		

	}
	neig[1][neig[1].size()-1][0] = neig[1][neig[1].size()-2][0];
	neig[1][neig[1].size()-1][1] = neig[1][neig[1].size()-2][1];	
	first_solution_v2 = true;
};

void LocalizationCallback(const apriltag_ros::AprilTagDetectionArray& msg){
	if(msg.detections.size() == 0) return;
        for(size_t i = 0; i < msg.detections.size() ; i++){
		if(msg.detections[i].id[0] == 6){
                        //std::cout<<"inininininiiiinii"<<std::endl;
			Eigen::Quaterniond q(msg.detections[i].pose.pose.pose.orientation.w,msg.detections[i].pose.pose.pose.orientation.x,\
			msg.detections[i].pose.pose.pose.orientation.y,msg.detections[i].pose.pose.pose.orientation.z) ;
			Eigen::Matrix4d Tct = Eigen::Matrix4d::Zero();
			Tct.block<3,3>(0,0) =  q.toRotationMatrix();
			Tct(0,3) = msg.detections[i].pose.pose.pose.position.x;
			Tct(1,3) = msg.detections[i].pose.pose.pose.position.y;
			Tct(2,3) = msg.detections[i].pose.pose.pose.position.z;	
			Tct(3,3) = 1.0;

			Eigen::Matrix4d Twt = Twc * Tct;
			double xtag = Twt(0,3);
			double ytag = Twt(1,3);

	
			std::lock_guard<std::mutex> lk(init_mtx);	
			thetainit = atan2(Twt(1,0),Twt(0,0));
			xinit = xtag - 0.07 * cos(thetainit);
			yinit = ytag - 0.07 * sin(thetainit);
			std::cout<<"x : "<<xinit<<", y : "<<yinit<<", theta : "<<thetainit*180/PI<<std::endl;

                }


        }
	
};



/*N O T E : Using Tag5 for object 1, Tag1 for obj2, Tag2 for obj3, and Tag7 for obj 4*/


//Object 1 detection
void LocalizationCallback1(const apriltag_ros::AprilTagDetectionArray& msg){
	if(msg.detections.size() == 0) return;
        for(size_t i = 0; i < msg.detections.size() ; i++){
		if(msg.detections[i].id[0] == 5){
                        //std::cout<<"inininininiiiinii"<<std::endl;
			Eigen::Quaterniond q(msg.detections[i].pose.pose.pose.orientation.w,msg.detections[i].pose.pose.pose.orientation.x,\
			msg.detections[i].pose.pose.pose.orientation.y,msg.detections[i].pose.pose.pose.orientation.z) ;
			Eigen::Matrix4d Tct = Eigen::Matrix4d::Zero();
			Tct.block<3,3>(0,0) =  q.toRotationMatrix();
			Tct(0,3) = msg.detections[i].pose.pose.pose.position.x;
			Tct(1,3) = msg.detections[i].pose.pose.pose.position.y;
			Tct(2,3) = msg.detections[i].pose.pose.pose.position.z;	
			Tct(3,3) = 1.0;

			Eigen::Matrix4d Twt = Twc * Tct;
			double xtag = Twt(0,3);
			double ytag = Twt(1,3);

	
			std::lock_guard<std::mutex> lk(init_mtx);	
			theta[0] = atan2(Twt(1,0),Twt(0,0));
			object[0][0] = xtag - 0.07 * cos(theta[0]);
			object[0][1] = ytag - 0.07 * sin(theta[0]);
			std::cout<<"x : "<<object[0][0]<<", y : "<<object[0][1]<<", theta : "<<theta[0]*180/PI<<std::endl;
			rec_data[1] = true;
                }
//geometry_msgs::Vector3 msg_con;	

        }
	
}

 //Object 2 detection
void LocalizationCallback2(const apriltag_ros::AprilTagDetectionArray& msg){
	if(msg.detections.size() == 0) return;
        for(size_t i = 0; i < msg.detections.size() ; i++){
		if(msg.detections[i].id[0] == 1){
                        //std::cout<<"inininininiiiinii"<<std::endl;
			Eigen::Quaterniond q(msg.detections[i].pose.pose.pose.orientation.w,msg.detections[i].pose.pose.pose.orientation.x,\
			msg.detections[i].pose.pose.pose.orientation.y,msg.detections[i].pose.pose.pose.orientation.z) ;
			Eigen::Matrix4d Tct = Eigen::Matrix4d::Zero();
			Tct.block<3,3>(0,0) =  q.toRotationMatrix();
			Tct(0,3) = msg.detections[i].pose.pose.pose.position.x;
			Tct(1,3) = msg.detections[i].pose.pose.pose.position.y;
			Tct(2,3) = msg.detections[i].pose.pose.pose.position.z;	
			Tct(3,3) = 1.0;

			Eigen::Matrix4d Twt = Twc * Tct;
			double xtag = Twt(0,3);
			double ytag = Twt(1,3);

	
			std::lock_guard<std::mutex> lk(init_mtx);	
			theta[1] = atan2(Twt(1,0),Twt(0,0));
			object[1][0] = xtag - 0.07 * cos(theta[1]);
			object[1][1] = ytag - 0.07 * sin(theta[1]);
			std::cout<<"x : "<<object[1][0]<<", y : "<<object[1][1]<<", theta : "<<theta[1]*180/PI<<std::endl;
			std::cout << "dsdsd"<< command.data << std::endl; 
            std::cout << "object1"<< min_object_2 << std::endl; 
			std::cout << "place1"<< min_place_2 << std::endl; 

			rec_data[2] = true;

                }


        }
	
};

void LocalizationCallback3(const apriltag_ros::AprilTagDetectionArray& msg){
	if(msg.detections.size() == 0) return;
        for(size_t i = 0; i < msg.detections.size() ; i++){
		if(msg.detections[i].id[0] == 2){
			Eigen::Quaterniond q(msg.detections[i].pose.pose.pose.orientation.w,msg.detections[i].pose.pose.pose.orientation.x,\
			msg.detections[i].pose.pose.pose.orientation.y,msg.detections[i].pose.pose.pose.orientation.z) ;
			Eigen::Matrix4d Tct = Eigen::Matrix4d::Zero();
			Tct.block<3,3>(0,0) =  q.toRotationMatrix();
			Tct(0,3) = msg.detections[i].pose.pose.pose.position.x;
			Tct(1,3) = msg.detections[i].pose.pose.pose.position.y;
			Tct(2,3) = msg.detections[i].pose.pose.pose.position.z;	
			Tct(3,3) = 1.0;

			Eigen::Matrix4d Twt = Twc * Tct;
			double xtag = Twt(0,3);
			double ytag = Twt(1,3);

	
			std::lock_guard<std::mutex> lk(init_mtx);	
			theta[2] = atan2(Twt(1,0),Twt(0,0));
			object[2][0] = xtag - 0.07 * cos(theta[2]);
			object[2][1] = ytag - 0.07 * sin(theta[2]);
			std::cout<<"xapril2 : "<<object[2][0]<<", y : "<<object[2][1]<<", theta : "<<theta[2]*180/PI<<std::endl;
			rec_data[3] = true;
                }


        }
	
};

 //Object 4 detection
void LocalizationCallback4(const apriltag_ros::AprilTagDetectionArray& msg){
	if(msg.detections.size() == 0) return;
        for(size_t i = 0; i < msg.detections.size() ; i++){
		if(msg.detections[i].id[0] == 7){//geometry_msgs::Vector3 msg_con;	
                        //std::cout<<"inininininiiiinii"<<std::endl;
			Eigen::Quaterniond q(msg.detections[i].pose.pose.pose.orientation.w,msg.detections[i].pose.pose.pose.orientation.x,\
			msg.detections[i].pose.pose.pose.orientation.y,msg.detections[i].pose.pose.pose.orientation.z) ;
			Eigen::Matrix4d Tct = Eigen::Matrix4d::Zero();
			Tct.block<3,3>(0,0) =  q.toRotationMatrix();
			Tct(0,3) = msg.detections[i].pose.pose.pose.position.x;
			Tct(1,3) = msg.detections[i].pose.pose.pose.position.y;
			Tct(2,3) = msg.detections[i].pose.pose.pose.position.z;	
			Tct(3,3) = 1.0;

			Eigen::Matrix4d Twt = Twc * Tct;
			double xtag = Twt(0,3);
			double ytag = Twt(1,3);

	
			std::lock_guard<std::mutex> lk(init_mtx);	
			theta[3] = atan2(Twt(1,0),Twt(0,0));
			object[3][0] = xtag - 0.07 * cos(theta[2]);
			object[3][1] = ytag - 0.07 * sin(theta[2]);
			std::cout<<"xapril2 : "<<object[3][0]<<", y : "<<object[3][1]<<", theta : "<<theta[2]*180/PI<<std::endl;
			rec_data[3] = true;
                }


        }
	
};

void SendData(){ //this robot sends data to robot1 about which obj and place to go to
	//Assume i for place, j for object, and k = 0 for robot 1
	//This is an example where the Obj3 is being placed at P4 (this command is being sent to robot 1)
	int i = min_object_2; int j = min_place_2; //recieves these values from distance calculations

	
		command.data = 0; //reset the number
		command.data = command.data | (1 << j); //set (j+1)th bit, starting from LSB
		command.data = command.data | (1 << i+8); //set (i+9)th bit, starting from LSB
	    //dectobinary(command.data);
		central_command.publish(command);//send this data (in this example, data = 0b0000100000000100). Now the reciever has to decode this.

};



void Robot1_Busy(const std_msgs::Bool::ConstPtr& msg){
	robot1_busy.data = msg->data; //now the robot 1 has completed its task
	datacounter = 1; //recieved data atleast once

	int prev_object_2 = min_object_2; //store previous object2
	
	//Recalling Distance Calculator for only robot 2
	if(robot1_busy.data == false){
	 if(datacounter == 1){ //if robot2 sends data atleast once
	 obj_filled[min_object_2] = true;
	 place_filled[min_place_2] = true;
	 }
	 double robot[2][2] =           //define position of place with array
		{    
			{xinit, yinit},            //robot3
			{xrob1, yrob1},            //robot1
		};

	double dis3[1][4];

    for (int i = 1 ;i <2; i++)   // this for-loop is for the number of robot
    {     
		for (int j = 0 ; j < 4 ; j++)   // this for-loop is for the number of place
        { 
		std::vector<std::pair<double, double>> vec = 
	                                   { {robot[i][0], robot[i][1]}, {object[j][0], object[j][1]}, }; // the values are from the array robot and object
	                dis3[i][j] = hypot(vec[0].first - vec[1].first, vec[0].second - vec[1].second); // robot i to obj j
					if (object[j][0] == 0 && object[j][1] == 0)
					{
						obj_filled[j] = true; // ignore the object that's not present
					}
		}
	
	}

	double dis4[4][8];
    for (int i = 0 ;i <4; i++)   // this for-loop is for the number of object
    {     
		for (int j = 0 ; j < 8 ; j++)   // this for-loop is for the number of place
        {   
	        std::vector<std::pair<double, double>> vec = 
	                { {object[i][0], object[i][1]}, {place[j][0], place[j][1]}, }; // the values are from the array object and place
	                dis4[i][j] = std::hypot(vec[0].first - vec[1].first, vec[0].second - vec[1].second);  // object i to place j
					
		}
	}
    double min2 = 100;
    for (int i=1; i<2; i++)      // this for-loop is for the number of robot
	{
        for (int j=0; j<4; j++)    // this for-loop is for the number of object 
		{
            for (int k=0; k<8; k++)    // this for-loop is for the number of place
			{
                if((dis3[i][j] + dis4[j][k])< min2)
				{
					
					if(obj_filled[j] == false && place_filled[k] == false && j != min_object_1 && k != min_place_1 && k != min_place_2)
					{
					min_object_2 = j;  
					min_place_2 = k;
					min2 = dis3[i][j] + dis4[j][k];
					}
           
				}
			}
		}
	}
	if(prev_object_2 != min_object_2){
		robot1_busy.data = true;
		SendData(); //send the data for the min obj and place to robot 2
	}
	}
};

bool DistanceCalcualtor(){
	//store previous recommendation to see if it has changed
	distfunc++;
 int prev_object_1 = min_object_1;
 int prev_object_2 = min_object_2;
 int prev_place_1 = min_place_1;
 int prev_place_2 = min_place_2;

double robot[2][2] =           //define position of place with array
{    
	{xinit, yinit},            //robot3
	{xrob1, yrob1},            //robot1
};





//the distance between the two point minmum distance - robot1


	double dis1[1][4];
    for (int i = 0; i <1; i++)   // this for-loop is for the number of robot
    {     
		
		for (int j = 0; j < 4; j++)   // this for-loop is for the number of object
        {  
	        std::vector<std::pair<double, double>> vec = 
	                                   { {robot[i][0], robot[i][1]}, {object[j][0], object[j][1]}, }; // the values are from the array robot and object
	                dis1[i][j] = std::hypot(vec[0].first - vec[1].first, vec[0].second - vec[1].second);  // robot i to obj j
					if (object[j][0] == 0 && object[j][1] == 0)
					{
						obj_filled[j] = true; // ignore the object that's not present
					}     
		}
	}

	
	double dis2[4][8];
    for (int i = 0; i <4; i++)   // this for-loop is for the number of object
    {     
		for (int j = 0; j < 8; j++)   // this for-loop is for the number of place
        {   
	        std::vector<std::pair<double, double>> vec = 
	                                   { {object[i][0], object[i][1]}, {place[j][0], place[j][1]}, }; // the values are from the array object and place
	                dis2[i][j] = std::hypot(vec[0].first - vec[1].first, vec[0].second - vec[1].second);  // object i to place j
		}
	}

    double min1 = 100;
    for (int i=0; i<1; i++)      // this for-loop is for the number of robot
	{
        for (int j=0; j<4; j++)    // this for-loop is for the number of object 
		{
            for (int k=0; k<8; k++)    // this for-loop is for the number of place
			{
                if( (dis1[i][j] + dis2[j][k]) < min1)
				{	
					if(obj_filled[j] == false && place_filled[k] == false && j != min_object_2 && k != min_place_1 && k != min_place_2)
					{
					min1 = dis1[i][j] + dis2[j][k];
					min_object_1 = j;
					min_place_1 = k;
					}         
				}
			}
		}
	}
	    

	

//the distance between the two point minmum distance - robot2

if(robot1_busy.data == false){
	 if(datacounter == 1){ //if robot2 sends data atleast once
	 obj_filled[min_object_2] = true;
	 place_filled[min_place_2] = true;
	 }

	double dis3[1][4];

    for (int i = 1 ;i <2; i++)   // this for-loop is for the number of robot
    {     
		for (int j = 0 ; j < 4 ; j++)   // this for-loop is for the number of place
        { 
		std::vector<std::pair<double, double>> vec = 
	                                   { {robot[i][0], robot[i][1]}, {object[j][0], object[j][1]}, }; // the values are from the array robot and object
	                dis3[i][j] = hypot(vec[0].first - vec[1].first, vec[0].second - vec[1].second); // robot i to obj j
					if (object[j][0] == 0 && object[j][1] == 0)
					{
						obj_filled[j] = true; // ignore the object that's not present
					}
		}
	
	}

	double dis4[4][8];
    for (int i = 0 ;i <4; i++)   // this for-loop is for the number of object
    {     
		for (int j = 0 ; j < 8 ; j++)   // this for-loop is for the number of place
        {   
	        std::vector<std::pair<double, double>> vec = 
	                { {object[i][0], object[i][1]}, {place[j][0], place[j][1]}, }; // the values are from the array object and place
	                dis4[i][j] = std::hypot(vec[0].first - vec[1].first, vec[0].second - vec[1].second);  // object i to place j
					
		}
	}
    double min2 = 100;
    for (int i=1; i<2; i++)      // this for-loop is for the number of robot
	{
        for (int j=0; j<4; j++)    // this for-loop is for the number of object 
		{
            for (int k=0; k<8; k++)    // this for-loop is for the number of place
			{
                if((dis3[i][j] + dis4[j][k])< min2)
				{
					
					if(obj_filled[j] == false && place_filled[k] == false && j != min_object_1 && k != min_place_1 && k != min_place_2)
					{
					min_object_2 = j;  
					min_place_2 = k;
					min2 = dis3[i][j] + dis4[j][k];
					}
           
				}
			}
		}
	}
	if(prev_object_2 != min_object_2){
			robot1_busy.data = true;
			SendData(); //send the data for the min obj and place to robot 2
	}
}
	
	
	if(min_object_1 == prev_object_1){
		min_object_1 = 100; //to tell obstacleavoidance
		ObstacleAvoidance(); // turn the last object into an obstacle
		bs->set_ref_states(0,0,0);
		//command.data = 9999999; //store 1s in 12 bits
		//central_command.publish(command);
		return true;
	}
	
	return false;	
};


void ObstacleAvoidance(){ //any other object besides the selected object is treated as an obstacle
	obst.clear();
	
	for(int i = 0; i < 4; i++){
		std::vector<double> obst1;

		if((object[i][0] == 0 && object[i][1] == 0) || i == min_object_1 || (i == min_object_2 && robot1_busy.data == true)){
				continue; //go back to the for loop
		}
		
		obst1.push_back(object[i][0]);
		obst1.push_back(object[i][1]);
		obst.push_back(obst1);
						
	}	
}

void Retreat(){
	bcwd=1;
}
/*
long long 
void dectobinary(int n)
{
	for (int i = 16; i>=0; i--)
	int s = n >> i;
	if(s&1)
	    cout << "1";
	else
	    cout << "0";
}
*/