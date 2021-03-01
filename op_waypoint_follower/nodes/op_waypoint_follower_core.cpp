
#include "op_waypoint_follower_core.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "op_utility/UtilityH.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include "math.h"

namespace op_waypoint_follower
{

WaypointFollower::WaypointFollower()
{
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bNewTrajectory = false;
	bNewVelocity = false;
	bNewBehaviorState = false;
	bNewVehicleCmd = false;
	bInitPos = false;
	ros::NodeHandle _nh;

	ReadParamFromLaunchFile(_nh);

	m_PathFollower.Init(m_ControlParams, m_ControllerHyperParams, m_CarInfo); //1.0, m_CmdParams.bVelocityMode, m_CmdParams.bAngleMode, true, false);
	m_State.Init(m_CarInfo.wheel_base, m_CarInfo.max_wheel_angle, m_ControlParams.SteeringDelay);

	//Control signal to publish
	pub_TwistRaw = _nh.advertise<geometry_msgs::TwistStamped>("twist_raw", 1); // published to Twist Filter
	pub_ControlRaw = _nh.advertise<autoware_msgs::ControlCommandStamped>("ctrl_raw", 1); // published to Twist Filter

	//Localization Information to publish
	pub_SimuPose = _nh.advertise<geometry_msgs::PoseStamped>(m_Topics.pose_topic_name, 1);
	pub_SimuLocalizerPose = _nh.advertise<geometry_msgs::PoseStamped>(m_Topics.localizer_pose_topic_name, 1);
	pub_SimuVelocity = _nh.advertise<geometry_msgs::TwistStamped>(m_Topics.velocity_topic_name, 1);
	pub_SimuVehicleStatus = _nh.advertise<autoware_msgs::VehicleStatus>(m_Topics.vehicle_status_topic_name, 1);

	//For rviz visualization
	pub_CurrPoseRviz = _nh.advertise<visualization_msgs::Marker>("op_curr_simu_pose", 1);
	pub_ForwardSimuPoseRviz = _nh.advertise<visualization_msgs::Marker>("op_forward_simu_pose", 1);
	pub_FollowPointRviz	= _nh.advertise<visualization_msgs::Marker>("follow_pose", 1);
	pub_VelocityRviz = _nh.advertise<std_msgs::Float32>("linear_velocity_viz", 1);

	// define subscribers.
	sub_current_pose = _nh.subscribe("/current_pose", 1, &WaypointFollower::callbackGetCurrentPose, this);
	sub_behavior_state = _nh.subscribe("/op_current_behavior",	1,  &WaypointFollower::callbackGetBehaviorState, 	this);
	sub_current_trajectory = _nh.subscribe("/op_local_selected_trajectory", 	1,	&WaypointFollower::callbackGetCurrentTrajectory, this);

	if(m_iSimulationMode == 1)
	{
		sub_initialpose = _nh.subscribe("/initialpose", 1, &WaypointFollower::callbackGetInitPose, this); // receive from RViz
		sub_vehicle_command = _nh.subscribe("vehicle_cmd", 1, &WaypointFollower::callbackVehicleCommand, this); // Receive from Twist Gate
	}
	else if(m_iSimulationMode == 2) // direct command
	{
		pub_VehicleCommandOP = _nh.advertise<autoware_msgs::VehicleCmd>("op_controller_cmd", 1);
	}


	if(m_iVelSource == 0)
	{
		sub_robot_odom = _nh.subscribe(m_Topics.velocity_source_topic_name, 1, &WaypointFollower::callbackGetRobotOdom, this);
	}
	else if(m_iVelSource == 1)
	{
		sub_current_velocity = _nh.subscribe(m_Topics.velocity_source_topic_name, 1, &WaypointFollower::callbackGetAutowareStatus, this);
	}
	else if(m_iVelSource == 2)
	{
		sub_can_info = _nh.subscribe(m_Topics.velocity_source_topic_name, 1, &WaypointFollower::callbackGetCANInfo, this);
	}
	else if(m_iVelSource == 3)
	{
		sub_vehicle_status = _nh.subscribe(m_Topics.velocity_source_topic_name, 1, &WaypointFollower::callbackGetVehicleStatus, this);
	}

	std::cout << "op_waypoint_follower initialized successfully " << std::endl;
}

void WaypointFollower::ReadParamFromLaunchFile(ros::NodeHandle& nh)
{
	nh.getParam("/op_common_params/width", m_CarInfo.width);
	nh.getParam("/op_common_params/length", m_CarInfo.length);
	nh.getParam("/op_common_params/wheelBaseLength", m_CarInfo.wheel_base);
	nh.getParam("/op_common_params/turningRadius", m_CarInfo.turning_radius);
	nh.getParam("/op_common_params/maxWheelAngle", m_CarInfo.max_wheel_angle);
	nh.getParam("/op_common_params/maxAcceleration", m_CarInfo.max_acceleration);
	nh.getParam("/op_common_params/maxDeceleration", m_CarInfo.max_deceleration);
	nh.getParam("/op_common_params/maxVelocity", m_CarInfo.max_speed_forward);
	nh.getParam("/op_common_params/minVelocity", m_CarInfo.min_speed_forward);
	nh.getParam("/op_common_params/steeringDelay", m_ControlParams.SteeringDelay);
	nh.getParam("/op_common_params/minPursuiteDistance", m_ControlParams.minPursuiteDistance );
	nh.getParam("/op_common_params/pathDensity", m_ControllerHyperParams.path_density);

	nh.getParam("/op_common_params/velocitySource", m_iVelSource);
	std::string velocity_topic;
	if(m_iVelSource == 0)
	{
		nh.getParam("/op_common_params/vel_odom_topic", m_Topics.velocity_source_topic_name);
	}
	else if(m_iVelSource == 1)
	{
		nh.getParam("/op_common_params/vel_curr_topic", m_Topics.velocity_source_topic_name);
	}
	else if(m_iVelSource == 2)
	{
		nh.getParam("/op_common_params/vel_can_topic", m_Topics.velocity_source_topic_name);
	}
	else if(m_iVelSource == 3)
	{
		nh.getParam("/op_common_params/vehicle_status_topic", m_Topics.velocity_source_topic_name);
	}

	int steering_mode = 0;
	int drive_mode = 0;
	nh.getParam("/op_waypoint_follower/steer_mode",  steering_mode);
	if(steering_mode == 0)
	{
		m_ControllerHyperParams.bEnableSteeringMode = true;
	}
	else
	{
		m_ControllerHyperParams.bEnableSteeringMode = false;
	}

	nh.getParam("/op_waypoint_follower/drive_mode", drive_mode);
	if(drive_mode == 0)
	{
		m_ControllerHyperParams.bEnableVelocityMode = true;
	}
	else
	{
		m_ControllerHyperParams.bEnableVelocityMode = false;
	}

	nh.getParam("/op_common_params/use_internal_acc", m_ControllerHyperParams.bUseInternalACC);

	nh.getParam("/op_common_params/experimentName", m_ControllerHyperParams.log_file_path);
	if(m_ControllerHyperParams.log_file_path.size() > 0)
	{
		if(m_ControllerHyperParams.log_file_path.at(m_ControllerHyperParams.log_file_path.size()-1) != '/')
		{
			m_ControllerHyperParams.log_file_path.push_back('/');
		}
	}


	m_ControllerHyperParams.bEnableConstantVelocity = true; //only for testing
	m_ControllerHyperParams.bEnableLogs = true;
	m_ControllerHyperParams.bEnableCalibration = false;
	m_ControllerHyperParams.bEnableVelocityFF = false;
	if(m_ControlParams.SteeringDelay > 0)
	{
		m_ControllerHyperParams.bEnableSteeringFF = true;
	}
	else
	{
		m_ControllerHyperParams.bEnableSteeringFF = false;
	}

	nh.getParam("/op_waypoint_follower/min_follow_safe_distance", m_ControlParams.min_safe_follow_distance);
	nh.getParam("/op_waypoint_follower/control_frequency", m_ControlParams.ControlFrequency );

	nh.getParam("/op_waypoint_follower/max_steer_value", m_CarInfo.max_steer_value );

	nh.getParam("/op_waypoint_follower/max_steer_torque", m_CarInfo.max_steer_torque );
	nh.getParam("/op_waypoint_follower/min_steer_torque", m_CarInfo.min_steer_torque );

	nh.getParam("/op_waypoint_follower/max_accel_value", m_CarInfo.max_accel_value );
	nh.getParam("/op_waypoint_follower/max_brake_value", m_CarInfo.max_brake_value );

	nh.getParam("/op_waypoint_follower/steerAngleGainKP", m_ControlParams.Steering_Gain.kP );
	nh.getParam("/op_waypoint_follower/steerAngleGainKI", m_ControlParams.Steering_Gain.kI );
	nh.getParam("/op_waypoint_follower/steerAngleGainKD", m_ControlParams.Steering_Gain.kD );

	nh.getParam("/op_waypoint_follower/steerTorqueGainKP", m_ControlParams.Torque_Gain.kP );
	nh.getParam("/op_waypoint_follower/steerTorqueGainKI", m_ControlParams.Torque_Gain.kI );
	nh.getParam("/op_waypoint_follower/steerTorqueGainKD", m_ControlParams.Torque_Gain.kD );

	nh.getParam("/op_waypoint_follower/accelGainKP", m_ControlParams.Accel_Gain.kP );
	nh.getParam("/op_waypoint_follower/accelGainKI", m_ControlParams.Accel_Gain.kI );
	nh.getParam("/op_waypoint_follower/accelGainKD", m_ControlParams.Accel_Gain.kD );

	nh.getParam("/op_waypoint_follower/brakeGainKP", m_ControlParams.Brake_Gain.kP );
	nh.getParam("/op_waypoint_follower/brakeGainKI", m_ControlParams.Brake_Gain.kI );
	nh.getParam("/op_waypoint_follower/brakeGainKD", m_ControlParams.Brake_Gain.kD );

	nh.getParam("/op_waypoint_follower/accelInitDelay", m_ControlParams.accel_init_delay );
	nh.getParam("/op_waypoint_follower/accelAvgDelay", m_ControlParams.accel_avg_delay );
	nh.getParam("/op_waypoint_follower/avgAcceleration", m_ControlParams.avg_acceleration );

	nh.getParam("/op_waypoint_follower/avg_engine_brake_accel", m_ControlParams.avg_engine_brake_accel );
	nh.getParam("/op_waypoint_follower/brakeInitDelay", m_ControlParams.brake_init_delay );
	nh.getParam("/op_waypoint_follower/brakeAvgDelay", m_ControlParams.brake_avg_delay );
	nh.getParam("/op_waypoint_follower/avgDeceleration", m_ControlParams.avg_deceleration );

	m_PlanningParams.maxSpeed = m_CarInfo.max_speed_forward;
	m_PlanningParams.minSpeed = m_CarInfo.min_speed_forward;

	nh.getParam("/op_waypoint_follower/enableSimulationMode", m_iSimulationMode);
	nh.getParam("/op_waypoint_follower/pose_topic", m_Topics.pose_topic_name);
	nh.getParam("/op_waypoint_follower/localizer_pose_topic", m_Topics.localizer_pose_topic_name);
	nh.getParam("/op_waypoint_follower/velocity_topic", m_Topics.velocity_topic_name);
	nh.getParam("/op_waypoint_follower/vehicle_status_topic", m_Topics.vehicle_status_topic_name);

	nh.getParam("/op_waypoint_follower/accelerationPushRatio", m_ControlParams.accelPushRatio);
	nh.getParam("/op_waypoint_follower/brakingPushRatio", m_ControlParams.brakePushRatio);
}

WaypointFollower::~WaypointFollower()
{
}

void WaypointFollower::callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	tf::StampedTransform transform;
	GetTransformFromTF("map", msg->header.frame_id, transform);

	geometry_msgs::Pose p;
	p.position.x  = msg->pose.pose.position.x + transform.getOrigin().x();
	p.position.y  = msg->pose.pose.position.y + transform.getOrigin().y();
	p.position.z  = msg->pose.pose.position.z + transform.getOrigin().z();
	p.orientation = msg->pose.pose.orientation;

	m_InitPos =  PlannerHNS::WayPoint(p.position.x, p.position.y, p.position.z , tf::getYaw(p.orientation));
	m_State.FirstLocalizeMe(m_InitPos);
	m_CurrentPos = m_InitPos;
	bInitPos = true;
}

void WaypointFollower::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	bNewCurrentPos = true;
}

void WaypointFollower::callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_CurrVehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_CurrVehicleStatus.speed;

	if(fabs(m_CurrentPos.v) > 0.1)
	{
		m_CurrVehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/m_CurrentPos.v);
	}

	UtilityHNS::UtilityH::GetTickCount(m_CurrVehicleStatus.tStamp);
}

void WaypointFollower::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
	m_CurrVehicleStatus.speed = msg->speed/3.6;
	m_CurrentPos.v = m_CurrVehicleStatus.speed;
	m_CurrVehicleStatus.steer = msg->angle * m_CarInfo.max_wheel_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_CurrVehicleStatus.tStamp);

	std::cout << "Velocity From CAN: " << m_CurrVehicleStatus.speed << std::endl;
}

void WaypointFollower::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_CurrVehicleStatus.speed = msg->twist.twist.linear.x;
	m_CurrentPos.v = m_CurrVehicleStatus.speed ;
	if(msg->twist.twist.linear.x != 0)
	{
		m_CurrVehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	}
	UtilityHNS::UtilityH::GetTickCount(m_CurrVehicleStatus.tStamp);

	std::cout << "Velocity From Odometer: " << m_CurrVehicleStatus.speed << std::endl;
}

void WaypointFollower::callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg)
{
	m_CurrVehicleStatus.speed = msg->speed/3.6;
	m_CurrVehicleStatus.steer = -msg->angle*DEG2RAD;
	m_CurrentPos.v = m_CurrVehicleStatus.speed;

	//std::cout << "Velocity From Vehicle Status : " << m_CurrVehicleStatus.speed << std::endl;
}

void WaypointFollower::callbackGetBehaviorState(const autoware_msgs::WaypointConstPtr& msg)
{
	m_CurrentBehavior = PlannerHNS::ROSHelpers::ConvertAutowareWaypointToBehaviorState(*msg);
	bNewBehaviorState = true;
}

void WaypointFollower::callbackGetCurrentTrajectory(const autoware_msgs::LaneConstPtr &msg)
{
	m_Path.clear();
	PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(*msg, m_Path);
	std::cout << "Receive new Trajectory From Behavior Selector : " << msg->waypoints.size() << std::endl;
	bNewTrajectory = true;
}

void WaypointFollower::callbackVehicleCommand(const autoware_msgs::VehicleCmdConstPtr& msg)
{
	m_CurrVehicleCommand = *msg;
	bNewVehicleCmd = true;
}

void WaypointFollower::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
	static tf::TransformListener listener;

	while (1)
	{
		try
		{
			listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
			break;
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}
}

void WaypointFollower::displayFollowingInfo()
{
	visualization_msgs::Marker m1,m2,m3;
	m1.header.frame_id = "map";
	m1.header.stamp = ros::Time();
	m1.ns = "op_curr_simu_pose";
	m1.type = visualization_msgs::Marker::ARROW;
	m1.action = visualization_msgs::Marker::ADD;
	m1.pose.position.x = m_CurrentPos.pos.x;
	m1.pose.position.y = m_CurrentPos.pos.y;
	m1.pose.position.z = m_CurrentPos.pos.z;
	m1.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(m_CurrentPos.pos.a));
	std_msgs::ColorRGBA green;
	green.a = 1.0;
	green.b = 0.0;
	green.r = 0.0;
	green.g = 1.0;
	m1.color = green;
	m1.scale.x = 1.8;
	m1.scale.y = 0.5;
	m1.scale.z = 0.5;
	m1.frame_locked = true;
	pub_CurrPoseRviz.publish(m1);

	m2.header.frame_id = "map";
	m2.header.stamp = ros::Time();
	m2.ns = "op_forward_simu_pose";
	m2.type = visualization_msgs::Marker::ARROW;
	m2.action = visualization_msgs::Marker::ADD;
	m2.pose.position.x = m_PathFollower.m_ForwardSimulationPoint.pos.x;
	m2.pose.position.y = m_PathFollower.m_ForwardSimulationPoint.pos.y;
	m2.pose.position.z = m_PathFollower.m_ForwardSimulationPoint.pos.z;
	m2.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(m_PathFollower.m_ForwardSimulationPoint.pos.a));
	std_msgs::ColorRGBA yellow;
	yellow.a = 0.75;
	yellow.b = 0.0;
	yellow.r = 0.8;
	yellow.g = 0.8;
	m2.color = yellow;
	m2.scale.x = 1.8;
	m2.scale.y = 0.5;
	m2.scale.z = 0.5;
	m2.frame_locked = true;
	pub_ForwardSimuPoseRviz.publish(m2);

	m3.header.frame_id = "map";
	m3.header.stamp = ros::Time();
	m3.ns = "follow_pose";
	m3.type = visualization_msgs::Marker::SPHERE;
	m3.action = visualization_msgs::Marker::ADD;
	m3.pose.position.x = m_PathFollower.m_FollowMePoint.pos.x;
	m3.pose.position.y = m_PathFollower.m_FollowMePoint.pos.y;
	m3.pose.position.z = m_PathFollower.m_FollowMePoint.pos.z;
	m3.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(m_PathFollower.m_FollowMePoint.pos.a));
	std_msgs::ColorRGBA red;
	red.a = 1.0;
	red.b = 0.0;
	red.r = 1.0;
	red.g = 0.0;
	m3.color = red;
	m3.scale.x = 0.7;
	m3.scale.y = 0.7;
	m3.scale.z = 0.7;
	m3.frame_locked = true;
	pub_FollowPointRviz.publish(m3);

	std_msgs::Float32 vel_rviz;
	vel_rviz.data = m_CurrVehicleStatus.speed;
	pub_VelocityRviz.publish(vel_rviz);
}

void WaypointFollower::SimulatedMoveStep(double dt)
{
	PlannerHNS::VehicleState vehicleCmd;
	vehicleCmd.speed = m_CurrVehicleCommand.ctrl_cmd.linear_velocity;
	vehicleCmd.speed = m_CurrVehicleCommand.ctrl_cmd.linear_velocity;
	vehicleCmd.steer = m_CurrVehicleCommand.ctrl_cmd.steering_angle;
	vehicleCmd.shift = PlannerHNS::SHIFT_POS_DD;

#ifndef AVOID_USING_TWIST_FILTER
	m_State.SimulateOdoPosition(dt, vehicleCmd);
#else
	//This modification to avoid using twist gate or twist filter
	m_State.SimulateOdoPosition(dt, m_curr_target_status);
	m_curr_motion_status.speed = m_State.getStateVelocity();
	m_curr_motion_status.steer = m_State.getStateSteering();
#endif

	geometry_msgs::TwistStamped vehicle_velocity;
	vehicle_velocity.header.stamp = ros::Time::now();
	vehicle_velocity.twist.linear.x = m_State.getStateVelocity();
	vehicle_velocity.twist.angular.z = ( vehicle_velocity.twist.linear.x / m_CarInfo.wheel_base ) * tan(m_State.getStateSteering());
	pub_SimuVelocity.publish(vehicle_velocity);

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.pose.position.x = m_State.getStatePose().pos.x;
	pose.pose.position.y = m_State.getStatePose().pos.y;
	pose.pose.position.z = m_State.getStatePose().pos.z;
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(m_State.getStatePose().pos.a));
	pub_SimuPose.publish(pose);

	static tf::TransformBroadcaster tf_broadcaster;
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "map";
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.x = pose.pose.position.x;
	odom_trans.transform.translation.y = pose.pose.position.y;
	odom_trans.transform.translation.z = pose.pose.position.z;
	odom_trans.transform.rotation = pose.pose.orientation;
	tf_broadcaster.sendTransform(odom_trans);

	// send lidar transform
//	geometry_msgs::TransformStamped lidar_trans;
//	lidar_trans.header.stamp = ros::Time::now();
//	lidar_trans.header.frame_id = "base_link";
//	lidar_trans.child_frame_id = "velodyne";
//	lidar_trans.transform.translation.z = 1.2;
//	lidar_trans.transform.rotation.w = 1.0;
//	tf_broadcaster.sendTransform(lidar_trans);
}

void WaypointFollower::PathFollowingStep(double dt)
{
	//To Define steering delay function based on velocity , the following is a piecewise first order linear function
	//c_params.SteeringDelay = m_ControlParams.SteeringDelay / (1.0- UtilityHNS::UtilityH::GetMomentumScaleFactor(m_CurrVehicleStatus.speed));

#ifndef AVOID_USING_TWIST_FILTER
	m_curr_target_status = m_PathFollower.DoOneStep(dt, m_CurrentBehavior, m_Path, m_CurrentPos, m_CurrVehicleStatus, bNewTrajectory);
#else
	//This modification to avoid using twist gate or twist filter
	m_curr_target_status = m_PathFollower.DoOneStep(dt, m_CurrentBehavior, m_Path, m_State.getStatePose(), m_curr_motion_status, bNewTrajectory);
#endif

	bNewTrajectory = false;
	m_State.UpdateStateHeight(m_PathFollower.m_PerpendicularPoint.pos.z);

	//Publish Twist_raw and ctrl_raw,
	geometry_msgs::TwistStamped twist_raw;
	twist_raw.header.stamp = ros::Time::now();
	twist_raw.twist.linear.x = m_curr_target_status.speed;
	//Steering angle from the controller is tuned as the steering wheel angle. Convert it to wheel turning angle in radians.
	//double front_wheel_angle = -curr_step_status.steer*m_CarInfo.max_wheel_angle / m_CarInfo.max_steer_value;
	double front_wheel_angle = m_curr_target_status.steer;
	twist_raw.twist.angular.z = (m_curr_target_status.speed / m_CarInfo.wheel_base ) * tan(front_wheel_angle);
	pub_TwistRaw.publish(twist_raw);

	autoware_msgs::ControlCommandStamped ctrl_raw;
	ctrl_raw.header.stamp = ros::Time::now();
	ctrl_raw.cmd.linear_velocity = m_curr_target_status.speed;
	ctrl_raw.cmd.steering_angle = m_curr_target_status.steer;
	ctrl_raw.cmd.linear_acceleration = m_curr_target_status.target_accel;
	pub_ControlRaw.publish(ctrl_raw);

	//publish vehicle status in case of simulation, later in case of controlling a real vehicle directly from this node
	if(m_iSimulationMode == 2)
	{
		autoware_msgs::VehicleCmd cmd;

		// select lateral control mode
		if (m_ControllerHyperParams.bEnableSteeringMode){
			cmd.steer_cmd.steer = m_curr_target_status.steer*(180/3.141);
		} 
		else
		{
			cmd.steer_cmd.steer = -m_curr_target_status.steer_torque;
		}

		// select longitudinal control mode
		if (m_ControllerHyperParams.bEnableVelocityMode){
			cmd.ctrl_cmd.linear_velocity = m_curr_target_status.speed;
		} 
		else
		{
			cmd.accel_cmd.accel = m_curr_target_status.accel_stroke;
			cmd.brake_cmd.brake = m_curr_target_status.brake_stroke;
		}
		
		pub_VehicleCommandOP.publish(cmd);
	}
}

void WaypointFollower::RunMainLoop()
{
	int freq = m_ControlParams.ControlFrequency;
	ros::Rate loop_rate(freq);

	struct timespec dt_timer;
	UtilityHNS::UtilityH::GetTickCount(dt_timer);
	double curr_dt = 1.0/(double)freq;
	double avg_dt = curr_dt;

	while (ros::ok())
	{
		ros::spinOnce();

		curr_dt = UtilityHNS::UtilityH::GetTimeDiffNow(dt_timer);
		UtilityHNS::UtilityH::GetTickCount(dt_timer);

		dt_list.push_back(curr_dt);
		if(dt_list.size() > freq)
		{
			double dt_sum = 0;
			for(auto& step_dt: dt_list)
			{
				dt_sum += step_dt;
			}
			avg_dt = dt_sum / dt_list.size();
			dt_list.erase(dt_list.begin()+0);
		}

		/**
		 * Path Following Part
		 * -----------------------------------------------------------------------------------------------
		 */
		PathFollowingStep(avg_dt);

		if(m_iSimulationMode == 1)
		{
			/**
			 * Localization and Status Reading Part
			 * -----------------------------------------------------------------------------------
			 */
			SimulatedMoveStep(avg_dt);
		}

		/**
		 * Rviz Visualization
		 * -----------------------------------------------------------------------------------------------
		 */
		displayFollowingInfo();

		loop_rate.sleep();
	}
}

}
