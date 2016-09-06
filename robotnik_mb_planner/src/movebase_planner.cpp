/** \file movebase_planner.cc
 * \author Robotnik Automation S.L.L.
 * \version 1.0
 * \date    2014
 *
 * \brief movebase_planner 
 * Component to follow a set of waypoints (path) via ROS move_base 
 * (C) 2014 Robotnik Automation, SLL
*/
#include <string.h>
#include <vector>
#include <queue>
#include <stdint.h>
#include <ros/ros.h>
#include <math.h>
#include <cstdlib>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"
#include <actionlib/server/simple_action_server.h>
#include <robotnik_mb_msgs/GoToAction.h>
#include <robotnik_mb_msgs/goal.h>
#include <robotnik_mb_msgs/CommandAction.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>


#define ODOM_TIMEOUT_ERROR					0.2				// max num. of seconds without receiving odom values
#define MAP_TIMEOUT_ERROR					0.2				// max num. of seconds without receiving map transformations

#define MAX_SPEED							1.0

#define WAYPOINT_POP_DISTANCE_M				0.10		//min dist to reach waypoint 

#define OK    								0
#define ERROR 								-1

#define IDLE_STATE                 0
#define SEND_WAYPOINT              1
#define REACHING_WAYPOINT          2
#define PAUSE_STATE                3
#define TWIST_LEFT                 4
#define TWIST_ZERO				   5
#define TWIST_RIGHT                6

#define MAX_PARAMETERS  		   5

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//! Data structure for a Waypoint
typedef struct Waypoint{
    //! Id of the magnet
    int iID;
    //! X position
    double dX;
    //! Y position
    double dY;
    //! Orientation
    double dA;
    //! Speed to arrive to the point
    double dSpeed;
}Waypoint;

//! class to manage the waypoints and magnets of the current path
class Path{
	public:
	//! Current waypoint
	int iCurrentWaypoint;

	private:
	//! Mutex to control the access
	pthread_mutex_t mutexPath;
	//! Vector to store all the Waypoints
	vector <Waypoint> vPoints;

	public:

	//! public constructor
	Path(){
		iCurrentWaypoint = -1;
		pthread_mutex_init(&mutexPath, NULL);//Initialization for WaypointRoutes' mutex
	}

	//! Destructor
	~Path(){
		pthread_mutex_destroy(&mutexPath);
	}

	//! Adds a new waypoint
	int AddWaypoint(Waypoint point){
		Waypoint aux;

		pthread_mutex_lock(&mutexPath);
			if(vPoints.size() > 0){
				aux = vPoints.back();
				// Only adds the waypoint if it's different from waypoint before
				if( (aux.dX != point.dX) || (aux.dY != point.dY) )
					vPoints.push_back(point);
			} else { // First point
				if(iCurrentWaypoint < 0){ //First point
					iCurrentWaypoint = 0;
				}

				vPoints.push_back(point);
			}

		pthread_mutex_unlock(&mutexPath);

		return OK;
	}

	//! Adds a vector of waypoints
	int AddWaypoint(vector <Waypoint> po){
		pthread_mutex_lock(&mutexPath);
			if(iCurrentWaypoint < 0){ //First point
				iCurrentWaypoint = 0;
			}
			for(int i = 0; i < po.size(); i++){
				vPoints.push_back(po[i]);
			}
		pthread_mutex_unlock(&mutexPath);
	}

	//! Clears the waypoints 
	void Clear(){
		pthread_mutex_lock(&mutexPath);
			iCurrentWaypoint = -1;
			vPoints.clear();
		pthread_mutex_unlock(&mutexPath);
	}
	
	//! Returns the size of the vector points
	unsigned int Size(){
		return vPoints.size();
	}

	//! Returns the next waypoint
	int GetNextWaypoint(Waypoint *wp){
		int ret = ERROR;
		pthread_mutex_lock(&mutexPath);
			if( (iCurrentWaypoint >= 0) && (iCurrentWaypoint < (vPoints.size() - 1)) ){
				*wp = vPoints[iCurrentWaypoint + 1];
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);
		return ret;
	}

	//! Returns the last waypoint
	int BackWaypoint(Waypoint *wp){
		int ret = ERROR;
		pthread_mutex_lock(&mutexPath);
			if( vPoints.size() > 0){
				*wp = vPoints.back();
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);
		return ret;
	}

	//! Gets the current waypoint
	int GetCurrentWaypoint(Waypoint *wp){
		int ret = ERROR;
		pthread_mutex_lock(&mutexPath);
			if( (iCurrentWaypoint >= 0) && (iCurrentWaypoint < vPoints.size()) ){
				*wp = vPoints[iCurrentWaypoint];
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);
		return ret;
	}

	//! Gets selected waypoint
	int GetWaypoint(int index, Waypoint *wp){
		int ret = ERROR;
		pthread_mutex_lock(&mutexPath);
			if( (index >= 0) && ( index< vPoints.size() ) ){
				*wp = vPoints[index];
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);
		return ret;
	}

	//! Gets the current Waypoint in the path
	int GetCurrentWaypointIndex(){
		return iCurrentWaypoint;
	}

	//!	Sets the current Waypoint to index
	int SetCurrentWaypoint(int index){
		int ret = ERROR;
//		if(index < (vPoints.size() - 1)){
// Changed robert	
		if(index <= (vPoints.size() - 1)){
			pthread_mutex_lock(&mutexPath);
				iCurrentWaypoint = index;
			pthread_mutex_unlock(&mutexPath);
			ret = OK;
		}
		return ret;
	}

	 //! Increase waypoint's number
	void NextWaypoint(){
		pthread_mutex_lock(&mutexPath);
			iCurrentWaypoint++;
		pthread_mutex_unlock(&mutexPath);
	}

	//! Returns the number of waypoints
	int NumOfWaypoints(){
		return vPoints.size();
	}

	//! Overloaded operator +=
	Path &operator+=(const Path &a){
		AddWaypoint(a.vPoints);
		return *this;
	}

	//! Cross product
	double dot2( Waypoint w, Waypoint v) {
		return (w.dX*v.dX + w.dY*v.dY);
	}

	//! Prints all the waypoints
	void Print(){
		cout << "Path::Print: Printing all the waypoints..." << endl;
		if(vPoints.size() > 0){
			for(int i = 0; i < vPoints.size(); i++){
				cout << "(" << i << ")\t " << vPoints[i].dX << ", " << vPoints[i].dY << endl;
			}
		}else
			cout << "Path::Print: No waypoints..." << endl;
	}

};

class movebase_planner_node
{

private:	
	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	double desired_freq_;
    bool bRunning;

    //! Object with the current path that the robot is following
    Path pathCurrent_;
    //! Object with the path that is being filled
    Path pathFilling_;
    //! Vector with next paths to follow
    queue <Path> qPath;
    //! current robot's position 
    geometry_msgs::Pose2D pose2d_robot_;
    //! current robot's odometry
    nav_msgs::Odometry odometry_robot_;
    //! current robot's linear speed
    double dLinearSpeed_;
    //! Max allowed speed
    double max_speed_;
    //! Flag to cancel the following path
    bool bCancel_;
    //! Internal state machine
    int iState_;
    
	//////// ROS
	//! Publishes the status of the robot
	ros::Publisher status_pub_;
	//! publish the transformation between map->base_link
	ros::Publisher tranform_map_pub_;
	//! it subscribes to /odom
	ros::Subscriber odom_sub_;
	//! Topic name to read the odometry from
	std::string odom_topic_;
    //! Topic to publishes /cmd_vel (states where the planner controls the base directly)
    ros::Publisher command_pub_;
    //! Topic name to command topic
    std::string command_topic_;
	//! Waypoint path reference frame
	std::string path_frame_id_;
    // DIAGNOSTICS
	//! Diagnostic to control the frequency of the published odom
	diagnostic_updater::TopicDiagnostic *updater_diagnostic_odom; 
	//! General status diagnostic updater
	diagnostic_updater::Updater updater_diagnostic;	
	//! Diagnostics min & max odometry freq
	double min_odom_freq, max_odom_freq; 
	//! Saves the time whenever receives a command msg and a transform between map and base link (if configured)
	ros::Time last_odom_time, last_map_time;
    // ACTIONLIB SERVER
	actionlib::SimpleActionServer<robotnik_mb_msgs::GoToAction> action_server_goto;
	robotnik_mb_msgs::GoToFeedback goto_feedback_;
	robotnik_mb_msgs::GoToResult goto_result_;
	robotnik_mb_msgs::GoToGoal goto_goal_;
	actionlib::SimpleActionServer<robotnik_mb_msgs::CommandAction> action_server_command;
	robotnik_mb_msgs::CommandGoal command_goal_;
	std::string command_;
	double param_[MAX_PARAMETERS];
	
	// Specific base commands
	double twist_speed_;
	double twist_angle_;
	double dist_since_last_twist_;
    bool first_waypoint_reached_;
    bool last_waypoint_reached_;
    bool navigation_stuck_;
	ros::Publisher dist_pub_;
    ros::Publisher first_waypoint_reached_pub_;
    ros::Publisher last_waypoint_reached_pub_;
    ros::Publisher navigation_stuck_pub_;
  
    // ACTIONLIB CLIENT
    MoveBaseClient ac_;

	// TFs
	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	// Odometry orientation saved
	// tf::Quaternion ref_orientation;
	double theta_ref_;
	
public:
	
	/*!	\fn movebase_planner_node::movebase_planner_node()
	 * 	\brief Public constructor
	*/
	movebase_planner_node(ros::NodeHandle h) : node_handle_(h), private_node_handle_("~"),
    desired_freq_(100.0), 
    action_server_goto(node_handle_, ros::this_node::getName() + "/path", false),
    action_server_command(node_handle_, ros::this_node::getName() + "/command", false),
    ac_("move_base", true)   //tell the action client that we want to spin a thread by default
	{
		bRunning = false;		
		ROSSetup();		
		pose2d_robot_.x = pose2d_robot_.y = pose2d_robot_.theta = 0.0;
    bCancel_ = false;
    iState_ = IDLE_STATE;
    theta_ref_ = 0.0;
    twist_speed_ = 0.0;
    twist_angle_ = 0.0;
    dist_since_last_twist_ = 0.0;
    first_waypoint_reached_ = false;
    last_waypoint_reached_ = false;
    navigation_stuck_ = false;
    command_ = "NONE";
	}

	/*!	\fn movebase_planner::~movebase_planner()
	 * 	\brief Public destructor
	*/
	~movebase_planner_node(){	
		
	}

/*!	\fn oid ROSSetup()
 * 	\brief Setups ROS' stuff
*/
void ROSSetup(){
	
	private_node_handle_.param<std::string>("odom_topic", odom_topic_, "/odom");
  private_node_handle_.param("command_topic", command_topic_, std::string("/cmd_vel"));
  private_node_handle_.param("max_speed", max_speed_, MAX_SPEED);
	private_node_handle_.param("desired_freq", desired_freq_, desired_freq_);
	private_node_handle_.param<std::string>("path_frame_id", path_frame_id_, "odom");
	ROS_INFO("Path refered to frame: %s ", path_frame_id_.c_str());
    
  // Subscriptions
  odom_sub_ = private_node_handle_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &movebase_planner_node::OdomCallback, this );

  // Publishers
  // Publish through the node handle Twist type messages to the guardian_controller/command topic
	command_pub_ = private_node_handle_.advertise<geometry_msgs::Twist>(command_topic_, 1);
  last_waypoint_reached_pub_ = private_node_handle_.advertise<std_msgs::Bool>("/last_waypoint_reached", 50);
  first_waypoint_reached_pub_ = private_node_handle_.advertise<std_msgs::Bool>("/first_waypoint_reached", 50);
  navigation_stuck_pub_ = private_node_handle_.advertise<std_msgs::Bool>("/navigation_stuck", 50);


	// Diagnostics
	updater_diagnostic.setHardwareID("MoveBase-Planner");
	// Topics freq control 
	min_odom_freq = 5.0;
	max_odom_freq = 100.0;
	updater_diagnostic_odom = new diagnostic_updater::TopicDiagnostic(odom_topic_, updater_diagnostic,
						diagnostic_updater::FrequencyStatusParam(&min_odom_freq, &max_odom_freq, 0.1, 10),
						diagnostic_updater::TimeStampStatusParam(0.001, 0.1));
			
	// Action server 
	action_server_goto.registerGoalCallback(boost::bind(&movebase_planner_node::GoalCB, this));
	action_server_goto.registerPreemptCallback(boost::bind(&movebase_planner_node::PreemptCB, this));

	action_server_command.registerGoalCallback(boost::bind(&movebase_planner_node::GoalCommandCB, this));
	action_server_command.registerPreemptCallback(boost::bind(&movebase_planner_node::PreemptCommandCB, this));
				
	ROS_INFO("movebase_planner::ROSSetup(): odom_topic = %s, desired_hz=%.1lf", 
				odom_topic_.c_str(), desired_freq_);				

	// Planner Odometer
	dist_pub_ = private_node_handle_.advertise<std_msgs::Float32>("/distance_since_last_twist", 50);
  


	// Starts action server (GotoAction)
	action_server_goto.start();		
	
	// Starts action server (CommandAction)
	action_server_command.start();

    // Connect to movebase action server
    // COMMENT FOR DEBUG !!!
    //while (!this->ConnectToActionServer()) {};
	}


/*!	\fn bool ConnectToActionServer()
 * 	\brief
*/
bool ConnectToActionServer()
{
  //wait for the action server to come up
  while(!ac_.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }
  return true;
}


/*!	\fn int Start()
 * 	\brief Start Controller
*/
int Start(){

	if(bRunning){
		ROS_INFO("movebase_planner::Start: the component's thread is already running");
		return ERROR;
		}				
	bRunning = true;
	return OK;
}

/*!	\fn int Stop()
 * 	\brief Stop Controller
*/
int Stop(){
	
	if(!bRunning){
		ROS_INFO("movebase_planner::Stop: Thread not running");		
		return ERROR;
		}		
	bRunning = false;		
	return OK;
}

/*! \fn void ControlThread()
*/
void ControlThread()
{
	ROS_INFO("movebase_planner::ControlThread(): Init");
	ros::Rate r(desired_freq_);
	
	//! For the twist action	
	geometry_msgs::Twist vel;
	double val =0.0;
		
	while(ros::ok()) {
		
		//ROS_INFO("iState = %d", iState_);
		// STATE MACHINE
        switch(iState_){
                
        case IDLE_STATE:
					// Transitions
					if (command_ != "NONE") {
						// Transitions to SEND_WAYPOINT (via GoTo goal or TWIST via TWIST command)
						if (command_ == "TWIST") {
              theta_ref_ = pose2d_robot_.theta;
							twist_speed_ = param_[0];
							twist_angle_ = param_[1];
							dist_since_last_twist_ = 0.0;
							iState_= TWIST_RIGHT;
							command_ = "NONE";
							}
						else if (command_ == "STARTFROM") {
							pathCurrent_.SetCurrentWaypoint( (int) param_[0] );
							iState_ = SEND_WAYPOINT;
							param_[0] = 0;
							command_ = "NONE";
							}
						else {
							ROS_INFO("GoalCommandCB::Planner in IDLE_STATE, command: %s is rejected", command_.c_str());
							command_ = "NONE";
							}
						}												
          break;
                    
          case SEND_WAYPOINT:
            // Actions 
            this->SendWaypoint();
            ROS_INFO("Following Wayponts State");
            // Transitions
            iState_ = REACHING_WAYPOINT;
                    
            // once the robot starts moving again, set the navigation_stuck feedback topic to false
            navigation_stuck_ = false;							
          break;
                
                case REACHING_WAYPOINT:
					// Actions
					if (ControlPath()) {
						int iWaypoint = pathCurrent_.GetCurrentWaypointIndex();
						if (iWaypoint >= pathCurrent_.NumOfWaypoints()-1) {
							ROS_INFO("Last waypoint is reached - end of path");
							
							// Set goal as succeded
							goto_result_.route_result = 1;
							action_server_goto.setSucceeded(goto_result_);
              
              last_waypoint_reached_ = true;
							iState_ = IDLE_STATE;
							}
						else {
							ROS_INFO("iWaypoint=%d numofwaypoints=%d", iWaypoint, pathCurrent_.NumOfWaypoints());							
							pathCurrent_.SetCurrentWaypoint( iWaypoint + 1 );
							iState_ = SEND_WAYPOINT;
							}					                
						}
					// Transitions
					if (command_ == "PAUSE") {
						ROS_INFO("Switching to PAUSE state");
						iState_ = PAUSE_STATE;
						ac_.cancelGoal(); // Cancel move_base goal
						command_ = "NONE";
						}
					if (command_ == "TWIST") {
						theta_ref_ = pose2d_robot_.theta;
						twist_speed_ = param_[0];
						twist_angle_ = param_[1];
						dist_since_last_twist_ = 0.0;						
						iState_ = TWIST_RIGHT;
						ac_.cancelGoal();
						command_ = "NONE";
						}
					break;
					
				case PAUSE_STATE:				
					// Transitions
					if (command_ == "RESTART") {
						iState_=SEND_WAYPOINT;
						command_ = "NONE";
						}					
					// Transitions
					if (command_ == "TWIST") {
						theta_ref_ = pose2d_robot_.theta;
						dist_since_last_twist_ = 0.0;
						twist_speed_ = param_[0];
						twist_angle_ = param_[1];						
						iState_ = TWIST_RIGHT;						
						//ac_.cancelGoal();
						command_ = "NONE";
						}					
					if (command_ == "STARTFROM") {
						pathCurrent_.SetCurrentWaypoint( (int) param_[0] );
						iState_ = SEND_WAYPOINT;
						param_[0] =0;
						command_ = "NONE";							
						}
					break;
					
				case TWIST_RIGHT:
					// Actions					
					vel.linear.x = 0.0; vel.linear.y = 0.0; vel.linear.z = 0.0;
					vel.angular.x = 0.0; vel.angular.y = 0.0; vel.angular.z = twist_speed_;
					command_pub_.publish( vel );					
					// Transitions 						
					if (angle_diff( pose2d_robot_.theta, theta_ref_) <= -twist_angle_) iState_ = TWIST_ZERO;					
					break;
					
				case TWIST_ZERO:
					// Actions 						
					vel.linear.x = 0.0; vel.linear.y = 0.0; vel.linear.z = 0.0;
					vel.angular.x = 0.0; vel.angular.y = 0.0; vel.angular.z = -twist_speed_;
					command_pub_.publish( vel );										
					// Transitions
					if (angle_diff( pose2d_robot_.theta, theta_ref_) >= 0.0) {
						if (command_ == "RESTART") {
							vel.linear.x = 0.0; vel.linear.y = 0.0; vel.linear.z = 0.0;
							vel.angular.x = 0.0; vel.angular.y = 0.0; vel.angular.z = 0.0;
							command_pub_.publish( vel );												
							iState_ = SEND_WAYPOINT;
							command_ = "NONE";
							}
						else if (command_ == "PAUSE") {
							vel.linear.x = 0.0; vel.linear.y = 0.0; vel.linear.z = 0.0;
							vel.angular.x = 0.0; vel.angular.y = 0.0; vel.angular.z = 0.0;
							command_pub_.publish( vel );
							iState_ = PAUSE_STATE;
							command_ = "NONE";
							}
						else iState_ = TWIST_LEFT;
						}
					break;
				
				case TWIST_LEFT:
					// Actions
					vel.linear.x = 0.0; vel.linear.y = 0.0; vel.linear.z = 0.0;
					vel.angular.x = 0.0; vel.angular.y = 0.0; vel.angular.z = -twist_speed_;
					command_pub_.publish( vel );					
					//ROS_INFO("TWIST_LEFT: twist_speed_ =%5.2f twist_angle_=%5.2f", -twist_speed_, twist_angle_);
					// Transitions 
					if (angle_diff( pose2d_robot_.theta, theta_ref_) >= twist_angle_) iState_ = TWIST_RIGHT;				
					break;
				}
        AllState();
		ros::spinOnce();
		r.sleep();
		}				
	
	ROS_INFO("movebase_planner::ControlThread(): End");		
}

	/*! \fn double angle_diff
	 *  \brief calculates angle difference from angle1 to angle2 
	 *  \return the angle difference
	 */
	double angle_diff( double alfa1, double alfa2 )
	{
		double alfa = alfa2 - alfa1;
		while (alfa < -M_PI) alfa += 2.0*M_PI;
		while (alfa > M_PI) alfa -= 2.0*M_PI;
		return alfa;
	}
							
	/*! \fn double Dot2( double x1, double y1, double x2, double y2)
	*   \brief Obtains vector cross product w x v
	*   \return w.x * v.x + w.y * w.y
	*/
	double Dot2( double x1, double y1, double x2, double y2) {
		return (x1*x2 + y1*y2); // cross product
	}


	/*! \fn double Dist(double x1, double y1, double x2, double y2)
	*   \brief obtains distance between points p1 and p2
	*/
	double Dist(double x1, double y1, double x2, double y2) {
		double diff_x = (x2 - x1);
		double diff_y = (y2 - y1);
		return sqrt( diff_x*diff_x + diff_y*diff_y );
	}

	

	/*!	\fn int FollowPath()
	 * \brief High level control 
	 *  \return 0 if the iteration is OK
	 *  \return -1 if there's a problem
	 *  \return 1 if the route finishes
	 */
	int FollowPath(){
		Waypoint last_waypoint, next_waypoint;
		
		double dAuxSpeed = 0.0;
		double aux = 0.0, dDistCovered = 0.0;
		int ret = 0;
				
		if(pathCurrent_.NumOfWaypoints() < 2)	{
			ROS_ERROR("movebase_planner::FollowPath: not enough waypoints");			
			return -1 ;
			}
								
		if(pathCurrent_.BackWaypoint(&last_waypoint) == ERROR){
			ROS_ERROR("movebase_planner::FollowPath: Error getting the last point in the path");
			return -1;
			}
		
		if(pathCurrent_.GetNextWaypoint(&next_waypoint) == ERROR){
			ROS_ERROR("movebase_planner::FollowPath: Error getting next waypoint in the path");
			return -1;
			}

		dAuxSpeed = next_waypoint.dSpeed;

		// End of path condition		
		if ( pathCurrent_.GetCurrentWaypointIndex() >= (pathCurrent_.NumOfWaypoints() - 1) ){
			pathCurrent_.Clear();
			ROS_INFO("movebase_planner::FollowPath: target position reached  Ending current path");
			
			// Inform about successful end of the action
			// action_server_goto.setSucceeded(goto_result_);
										
			return 1;
			}
	
		return 0;
	}
	
	/*!	\fn void CancelPath()
	 * Removes all the waypoints introduced in the system
	*/
	void CancelPath(){
		
		pathCurrent_.Clear();	// Clears current path
		pathFilling_.Clear();	// Clears the auxiliary path
		while(!qPath.empty())	// Clears the queue of paths
			qPath.pop();
		
        bCancel_ = false;
		// Cancels current action
		ROS_INFO("movebase_planner::CancelPath: action server preempted");
		action_server_goto.setPreempted();
	}
	
					
	/*!	\fn void AllState()
	*/
	void AllState(){
			
        AnalyseCB();        // Checks action server state
		
        updater_diagnostic.update();    // Updates diagnostics


        // publish navigation stuck feedback topic
        std_msgs::Bool navigation_stuck_msg;
        navigation_stuck_msg.data = navigation_stuck_;
        navigation_stuck_pub_.publish(navigation_stuck_msg);
        
        // publish first waypoint reached flag
        std_msgs::Bool first_waypoint_reached_msg;
        first_waypoint_reached_msg.data = first_waypoint_reached_;
        first_waypoint_reached_pub_.publish(first_waypoint_reached_msg);
        
        // publish last waypoint reached flag
        std_msgs::Bool last_waypoint_reached_msg;
        last_waypoint_reached_msg.data = last_waypoint_reached_;
        last_waypoint_reached_pub_.publish(last_waypoint_reached_msg);

//        if(bCancel_)		// Performs the cancel in case of required
//			CancelPath();
	}
	
	/*! \fn void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_value)
		* Receives odom values
	*/
	void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_value)
	{
		// Safety check
		last_odom_time = ros::Time::now();

		// compute_distance only if the robot is in motion
		if (iState_ == REACHING_WAYPOINT) {
			double dx = odom_value->pose.pose.position.x - odometry_robot_.pose.pose.position.x;
			double dy = odom_value->pose.pose.position.y - odometry_robot_.pose.pose.position.y;
			double d = sqrt( dx*dx + dy*dy );
			dist_since_last_twist_ = dist_since_last_twist_ + d;
			}
		
		// publish the odometer distance
		std_msgs::Float32 dist_msg;
		dist_msg.data = dist_since_last_twist_;
		dist_pub_.publish(dist_msg);
    
    
				
    // converts the odom to pose 2d
    pose2d_robot_.x = odom_value->pose.pose.position.x;
    pose2d_robot_.y = odom_value->pose.pose.position.y;
    pose2d_robot_.theta = tf::getYaw(odom_value->pose.pose.orientation);

		// Copies the current odom
        odometry_robot_ = *odom_value;
		// Gets the linear speed
        dLinearSpeed_ = odometry_robot_.twist.twist.linear.x;
	}
	
	/*! \fn int CheckOdomReceive()
		* Checks whether or not it's receiving odom values and/or map transformations
		* \return 0 if OK
		* \return -1 if ERROR
	*/
	int CheckOdomReceive()
	{		
		// Safety check
		if((ros::Time::now() - last_odom_time).toSec() > ODOM_TIMEOUT_ERROR)
			return -1;		
	}
		
	void executeCB(const robotnik_mb_msgs::GoToGoalConstPtr &goal)
	{
		
	}


	/*!	\fn int movebase_planner_node::MergePath()
	 * 	\brief Merges the current path with the next path
	*/
	int AcceptPath()
	{
		
		Waypoint new_waypoint;
		Path aux;

		pathCurrent_.Clear();

		// If a new goal is added
		if(action_server_goto.isNewGoalAvailable()){          
            goto_goal_.target = action_server_goto.acceptNewGoal()->target; // Reads points from action server
			if(goto_goal_.target.size() > 0){
								
				for(int i = 0; i < goto_goal_.target.size(); i++){
					ROS_INFO("movebase_planner::AcceptPath: Point %d (%.3lf, %.3lf, %.3lf) speed = %.3lf", i,  goto_goal_.target[i].pose.x, 
					goto_goal_.target[i].pose.y, goto_goal_.target[i].pose.theta, goto_goal_.target[i].speed  );
					
					new_waypoint.dX = goto_goal_.target[i].pose.x;
					new_waypoint.dY = goto_goal_.target[i].pose.y;
					new_waypoint.dA = goto_goal_.target[i].pose.theta;
					new_waypoint.dSpeed = fabs(goto_goal_.target[i].speed);
					
					pathCurrent_.AddWaypoint(new_waypoint);							
					}
				goto_feedback_.current_waypoint = 0;	// Inits the waypoint number feedback
				return OK;
				}
			}
		return ERROR;
	}

			
	/*!	\fn int movebase_planner_node::MergePath()
	 * 	\brief Merges the current path with the next path
	*/
	int MergePath(){
		Waypoint new_waypoint, wFirst, wLast;
		Path aux;
				
		// If a new goal is added
		if(action_server_goto.isNewGoalAvailable()){          
            goto_goal_.target = action_server_goto.acceptNewGoal()->target; // Reads points from action server
			if(goto_goal_.target.size() > 0){
								
				for(int i = 0; i < goto_goal_.target.size(); i++){
					ROS_INFO("movebase_planner::MergePath: Point %d (%.3lf, %.3lf, %.3lf) speed = %.3lf", i,  goto_goal_.target[i].pose.x, 
					goto_goal_.target[i].pose.y, goto_goal_.target[i].pose.theta, goto_goal_.target[i].speed  );
					
					new_waypoint.dX = goto_goal_.target[i].pose.x;
					new_waypoint.dY = goto_goal_.target[i].pose.y;
					new_waypoint.dA = goto_goal_.target[i].pose.theta;
					new_waypoint.dSpeed = fabs(goto_goal_.target[i].speed);
					
					pathFilling_.AddWaypoint(new_waypoint);							
				}

				//pathFilling_.Print();
				// Adds the new path to the queue
				qPath.push(pathFilling_);
				// Clears temporary path object
				pathFilling_.Clear();
				
				goto_feedback_.current_waypoint = 0;	// Inits the waypoint feedback
				
				// Only if exists any path into the queue
				if(qPath.size() > 0){					
					aux = qPath.front();
					aux.GetWaypoint(0, &wFirst);
					aux.BackWaypoint(&wLast);
					ROS_INFO("movebase_planner::MergePath: Adding new %d points from (%.2lf, %.2lf) to (%.2lf, %.2lf)", aux.NumOfWaypoints() ,wFirst.dX, wFirst.dY, wLast.dX, wLast.dY);
					ROS_INFO("movebase_planner::MergePath: Current number of points = %d", pathCurrent_.NumOfWaypoints() );
					
					// Adds the first path in the queue to the current path
					pathCurrent_+=qPath.front();					
					ROS_INFO("::MergePath: New number of points = %d", pathCurrent_.NumOfWaypoints() );
					
					// Pops the extracted path
					qPath.pop();					
					goto_goal_.target.clear();	// removes current goals					
					return OK;
					}
			}
		
		}
		
		return ERROR;
	}
	
	/*! \fn void GoalCB()
		* Called when receiving a new target. (ActionServer)
	*/
	void GoalCB()
	{	
        ROS_INFO("Action: GoalCB call");

		// Check, needs 2x Go to run
        //if(bEnabled_ && !bCancel_ ){
		if (iState_ == IDLE_STATE) {
        // if(pathCurrent_.Size() > 0 || MergePath() == OK){
		   if (AcceptPath() == OK) {
                ROS_INFO("Action: GoalCB - route available");                                             
                iState_ = SEND_WAYPOINT;
                pathCurrent_.SetCurrentWaypoint(0);
                }
            }
    }
	
	/*! \fn void PreemptCB()
		* Called to cancel or replace current mision. (ActionServer)
	*/
	void PreemptCB()
	{	
        ROS_INFO("Action: PreemptCB");
		if (iState_ != IDLE_STATE) {
			CancelPath();			
			iState_ == IDLE_STATE;
			}
      
      // restart navigation variables
      first_waypoint_reached_ = false;
      last_waypoint_reached_ = false;
	}
	
	/*! \fn void AnalyseCB()
		* Checks the status. (ActionServer)
	*/
	void AnalyseCB(){
		
		if (!action_server_goto.isActive()){
            //ROS_INFO("movebase_planner::AnalyseCB: Not active");
			return;
			}
		
		goto_feedback_.current_waypoint = pathCurrent_.GetCurrentWaypointIndex();
				
		action_server_goto.publishFeedback(goto_feedback_);
		
		/*
		if(goto_feedback_.percent_complete == 100.0){
			//action_server_goto.setSucceeded(goto_result_);
			action_server_goto.setAborted(goto_result_);
			ROS_INFO("movebase_planner::AnalyseCB: Action finished");
		    }
		*/
	}

    /*! \fn void SendWaypoint( int iWaypoint )
        * Sends the goal to the movebase action server
    */
    bool SendWaypoint()
    {
        Waypoint w;

        if(pathCurrent_.NumOfWaypoints() < 1)	{
            ROS_ERROR("SendGoal: not enough waypoints");
            return -1 ;
            }

		int i = pathCurrent_.GetCurrentWaypointIndex();
				
		// Send waypoint as goal to move_base
		if(pathCurrent_.GetWaypoint(i, &w) == OK) {
			
			// Set Speed parameters with dynamic_reconfigure
			///move_base/TrajectoryPlannerROS/max_vel_theta
			///move_base/TrajectoryPlannerROS/max_vel_x
			char buf[120];
			sprintf(buf, "rosrun dynamic_reconfigure dynparam set move_base/TrajectoryPlannerROS max_vel_x %5.2f", w.dSpeed);
			system(buf);


			// Prepare message
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.frame_id = path_frame_id_;
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position.x = w.dX;
			goal.target_pose.pose.position.y = w.dY;
			
			// Convert theta from yaw (rads) to quaternion.
			double theta = w.dA;
			geometry_msgs::Quaternion wp_quat = tf::createQuaternionMsgFromYaw( theta );
			goal.target_pose.pose.orientation = wp_quat;

			ROS_INFO("movebase_planner::SendWaypoint (wp:%d  x:%5.3f y:%5.3f a:%5.3f", i, w.dX, w.dY, w.dA);
			ac_.sendGoal(goal);
			}
        
        return true;
    }

	/*! \fn void ControlPath()
		* Checks if the robot has reached the waypoint in course
	*/
	bool ControlPath()
	{
		ac_.waitForResult(ros::Duration(0.1) );
    
    // get action client goal state
    actionlib::SimpleClientGoalState ac_state = ac_.getState();
    
		if(ac_state == actionlib::SimpleClientGoalState::SUCCEEDED) {		
			int iWaypoint = pathCurrent_.GetCurrentWaypointIndex();
			ROS_INFO("Reached waypoint %d", iWaypoint);
      
      if(iWaypoint == 0)
        first_waypoint_reached_ = true;
      
			return true;
			}
    // checks if the navigation was aborted for due to a navigation error
    else if(ac_state == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_WARN("Received aborted state from Move Base. Restarting planner. A new goal must be resent");
      navigation_stuck_ = true;
    
      // set the planner on IDLE_STATE
      PreemptCB();
      
      }
		else {
			return false;
			}
	}


	/*! \fn void GoalCommandCB()
		* Called when receiving a new command (ActionServer)
	*/
	void GoalCommandCB()
	{	
        ROS_INFO("Action: GoalCommandCB call");

		// If a new goal is added, a command is received, run the TRANSITIONS OF THE STATE MACHINE
		if(action_server_command.isNewGoalAvailable()){
            ROS_INFO("New Goal Command Available");
            command_goal_ = *action_server_command.acceptNewGoal();                       
			ROS_INFO("Received cmd=%s", command_goal_.cmd.c_str() );
			command_ = command_goal_.cmd;				
			if (command_goal_.param.size() > 0) {
				for(int i = 0; i < command_goal_.param.size(); i++){
					param_[i] = command_goal_.param[i];	
					//ROS_INFO("param[%d]=%5.2f", i, param_[i]);
					}
				}	
			}
    }

	/*! \fn void PreemptCommandCB()
		* Called when receiving a new target. (ActionServer)
	*/
	void PreemptCommandCB()
	{	
        ROS_INFO("Action: PreemptCommandCB call");
    }



}; // class movebase_planner_node

// MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "movebase_planner_node");
	
	ros::NodeHandle n;		
  	movebase_planner_node planner(n);
	
	planner.ControlThread();

	return (0);
}
// EOF
