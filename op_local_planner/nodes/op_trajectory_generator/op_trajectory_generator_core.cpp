/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "op_trajectory_generator_core.h"
#include "op_ros_helpers/op_ROSHelpers.h"


namespace TrajectoryGeneratorNS
{

TrajectoryGen::TrajectoryGen()
{
	bInitPos = false;
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bWayGlobalPath = false;
	bEnableSmoothGlobalPathForCARLA = false;
	bEnableVisualizeGlobalPathForCARLA = false;
	bFrontAxelStart = false;
	m_SteeringDelay = 0;
	m_MinPursuitDistance = 0;
	m_bEnableForwardSimulation = false;

	ros::NodeHandle _nh;
	UpdatePlanningParams(_nh);

	tf::StampedTransform transform;
	tf::TransformListener tf_listener;
	PlannerHNS::ROSHelpers::getTransformFromTF("world", "map", tf_listener, transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_PathsRviz = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_rviz", 1, true);
	pub_LocalTrajectories = nh.advertise<autoware_msgs::LaneArrayStamped>("local_trajectories", 1);
	pub_LocalTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories_gen_rviz", 1);

	sub_initialpose = nh.subscribe("/initialpose", 1, &TrajectoryGen::callbackGetInitPose, this);
	sub_current_pose = nh.subscribe("/current_pose", 1, &TrajectoryGen::callbackGetCurrentPose, this);

	int bVelSource = 1;
	_nh.getParam("/op_common_params/velocitySource", bVelSource);
	std::string velocity_topic;
	if(bVelSource == 0)
	{
		sub_robot_odom = nh.subscribe("/carla/ego_vehicle/odometry", 1,	&TrajectoryGen::callbackGetRobotOdom, this);
	}
	else if(bVelSource == 1)
	{
		sub_current_velocity = nh.subscribe("/current_velocity", 1, &TrajectoryGen::callbackGetAutowareStatus, this);
	}
	else if(bVelSource == 2)
	{
		sub_can_info = nh.subscribe("/can_info", 1, &TrajectoryGen::callbackGetCANInfo, this);
	}
	else if(bVelSource == 3)
	{
		nh.getParam("/op_common_params/vehicle_status_topic", velocity_topic);
		sub_vehicle_status = _nh.subscribe(velocity_topic, 1, &TrajectoryGen::callbackGetVehicleStatus, this);
	}

	sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 1, &TrajectoryGen::callbackGetGlobalPlannerPath, this);

	UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);
	m_distance_moved_since_stuck = 0;
	m_distance_moved = 0;
	m_bStuckState = false;
}

TrajectoryGen::~TrajectoryGen()
{
}

void TrajectoryGen::UpdatePlanningParams(ros::NodeHandle& _nh)
{
	_nh.getParam("/op_trajectory_generator/samplingTipMargin", m_PlanningParams.carTipMargin);
	_nh.getParam("/op_trajectory_generator/samplingOutMargin", m_PlanningParams.rollInMargin);
	_nh.getParam("/op_trajectory_generator/samplingSpeedFactor", m_PlanningParams.rollInSpeedFactor);
	_nh.getParam("/op_trajectory_generator/enableHeadingSmoothing", m_PlanningParams.enableHeadingSmoothing);
	_nh.getParam("/op_trajectory_generator/startFromFrontAxel", bFrontAxelStart);
	_nh.getParam("/op_trajectory_generator/enableForwardSimulation", m_bEnableForwardSimulation);

	_nh.getParam("/op_common_params/enableSwerving", m_PlanningParams.enableSwerving);
	if(m_PlanningParams.enableSwerving)
	{
		m_PlanningParams.enableFollowing = true;
	}
	else
	{
		_nh.getParam("/op_common_params/enableFollowing", m_PlanningParams.enableFollowing);
	}

	_nh.getParam("/op_common_params/enableTrafficLightBehavior", m_PlanningParams.enableTrafficLightBehavior);
	_nh.getParam("/op_common_params/enableStopSignBehavior", m_PlanningParams.enableStopSignBehavior);

	_nh.getParam("/op_common_params/maxVelocity", m_PlanningParams.maxSpeed);
	_nh.getParam("/op_common_params/minVelocity", m_PlanningParams.minSpeed);
	_nh.getParam("/op_common_params/maxLocalPlanDistance", m_PlanningParams.microPlanDistance);

	_nh.getParam("/op_common_params/pathDensity", m_PlanningParams.pathDensity);
	_nh.getParam("/op_common_params/rollOutDensity", m_PlanningParams.rollOutDensity);
	_nh.getParam("/op_common_params/rollOutsNumber", m_PlanningParams.rollOutNumber);
	m_nOriginalRollOuts = m_PlanningParams.rollOutNumber;

	if(!m_PlanningParams.enableSwerving)
	{
		m_PlanningParams.rollOutNumber = 0;
	}

	_nh.getParam("/op_common_params/horizonDistance", m_PlanningParams.horizonDistance);
	_nh.getParam("/op_common_params/minFollowingDistance", m_PlanningParams.minFollowingDistance);
	_nh.getParam("/op_common_params/minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
	_nh.getParam("/op_common_params/maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
	_nh.getParam("/op_common_params/speedProfileFactor", m_PlanningParams.speedProfileFactor);

	_nh.getParam("/op_common_params/smoothingDataWeight", m_PlanningParams.smoothingDataWeight);
	_nh.getParam("/op_common_params/smoothingSmoothWeight", m_PlanningParams.smoothingSmoothWeight);

	_nh.getParam("/op_common_params/horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
	_nh.getParam("/op_common_params/verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);

	_nh.getParam("/op_common_params/enableLaneChange", m_PlanningParams.enableLaneChange);

	_nh.getParam("/op_common_params/front_length", m_CarInfo.front_length);
	_nh.getParam("/op_common_params/back_length", m_CarInfo.back_length);
	_nh.getParam("/op_common_params/height", m_CarInfo.height);
	_nh.getParam("/op_common_params/width", m_CarInfo.width);
	_nh.getParam("/op_common_params/length", m_CarInfo.length);
	_nh.getParam("/op_common_params/wheelBaseLength", m_CarInfo.wheel_base);
	_nh.getParam("/op_common_params/turningRadius", m_CarInfo.turning_radius);
	_nh.getParam("/op_common_params/maxWheelAngle", m_CarInfo.max_wheel_angle);
	_nh.getParam("/op_common_params/maxAcceleration", m_CarInfo.max_acceleration);
	_nh.getParam("/op_common_params/maxDeceleration", m_CarInfo.max_deceleration);

	m_CarInfo.max_speed_forward = m_PlanningParams.maxSpeed;
	m_CarInfo.min_speed_forward = m_PlanningParams.minSpeed;

	_nh.getParam("/op_common_params/steeringDelay", m_SteeringDelay);
	_nh.getParam("/op_common_params/minPursuiteDistance", m_MinPursuitDistance);

}

void TrajectoryGen::callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	if(!bInitPos)
	{
		m_InitPos = PlannerHNS::WayPoint(msg->pose.pose.position.x+m_OriginPos.position.x,
				msg->pose.pose.position.y+m_OriginPos.position.y,
				msg->pose.pose.position.z+m_OriginPos.position.z,
				tf::getYaw(msg->pose.pose.orientation));
		m_CurrentPos = m_InitPos;
		bInitPos = true;
	}
}

void TrajectoryGen::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	PlannerHNS::GPSPoint prev_pos = m_CurrentPos.pos;
	m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	m_InitPos = m_CurrentPos;
	bNewCurrentPos = true;
	bInitPos = true;

	//just For CARLA 
	if(m_PlanningParams.enableTimeOutAvoidance)
	{
		if(m_bStuckState)
		{
			m_distance_moved_since_stuck += hypot(prev_pos.y-m_CurrentPos.pos.y, prev_pos.x-m_CurrentPos.pos.x);;
			if(m_distance_moved_since_stuck > m_PlanningParams.microPlanDistance)
			{
				UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);
				m_distance_moved_since_stuck = 0;
				m_distance_moved = 0;
				m_bStuckState = false;
				m_PlanningParams.enableSwerving = false;
				m_PlanningParams.rollOutNumber = 0;
			}
		}
		else
		{
			if(m_VehicleStatus.speed < 0.1) //start monitor stuck
			{
				m_distance_moved += hypot(prev_pos.y-m_CurrentPos.pos.y, prev_pos.x-m_CurrentPos.pos.x);

				double time_since_stop = UtilityHNS::UtilityH::GetTimeDiffNow(m_PlanningTimer);
				if(m_distance_moved < m_DistanceLimitInTimeOut && time_since_stop  > m_PlanningParams.avoidanceTimeOut)
				{
					m_bStuckState = true;
					m_distance_moved_since_stuck = 0;
					m_PlanningParams.enableSwerving = true;
					m_PlanningParams.rollOutNumber = m_nOriginalRollOuts;
				}
			}
			else //vehicle is moving , nothing to worry about
			{
				UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);
				m_distance_moved = 0;
			}
		}
	}
}

void TrajectoryGen::callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;
	if(fabs(msg->twist.linear.x) > 0.25)
	{
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);
	}
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryGen::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_VehicleStatus.steer = msg->angle * m_CarInfo.max_wheel_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryGen::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.twist.linear.x;

	if(msg->twist.twist.linear.x != 0)
		m_VehicleStatus.steer += atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void TrajectoryGen::callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_VehicleStatus.steer = -msg->angle*DEG2RAD;
	m_CurrentPos.v = m_VehicleStatus.speed;
	bVehicleStatus = true;
	//std::cout << "Vehicle Real Status, Speed: " << m_VehicleStatus.speed << ", Steer Angle: " << m_VehicleStatus.steer << ", Steermode: " << msg->steeringmode << ", Org angle: " << msg->angle <<  std::endl;
}

void TrajectoryGen::callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0)
	{
		bool bOldGlobalPath = m_GlobalPaths.size() == msg->lanes.size();

		m_GlobalPaths.clear();

		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), m_temp_path);

			m_GlobalPaths.push_back(m_temp_path);

			if(bOldGlobalPath)
			{
				bOldGlobalPath = PlannerHNS::PlanningHelpers::CompareTrajectories(m_temp_path, m_GlobalPaths.at(i));
			}
		}

		if(!bOldGlobalPath)
		{
			if(bEnableSmoothGlobalPathForCARLA)
			{
				m_prev_index.clear();
				for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
				{
					PlannerHNS::PlanningHelpers::FixPathDensity(m_GlobalPaths.at(i), m_PlanningParams.pathDensity);
					PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_GlobalPaths.at(i));
					PlannerHNS::PlanningHelpers::SmoothPath(m_GlobalPaths.at(i), 0.48, 0.2, 0.05); // this line could slow things , if new global path is generated frequently. only for carla
					PlannerHNS::PlanningHelpers::SmoothPath(m_GlobalPaths.at(i), 0.48, 0.2, 0.05); // this line could slow things , if new global path is generated frequently. only for carla
					PlannerHNS::PlanningHelpers::SmoothPath(m_GlobalPaths.at(i), 0.48, 0.2, 0.05); // this line could slow things , if new global path is generated frequently. only for carla
					PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_GlobalPaths.at(i));
					m_prev_index.push_back(0);
				}
			}
			else
			{
				m_prev_index.clear();
				for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
				{
					PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_GlobalPaths.at(i));
					m_prev_index.push_back(0);		
				}
			}

			bWayGlobalPath = true;
			std::cout << "Received New Global Path Generator ! " << std::endl;
		}
		else
		{
			m_GlobalPaths.clear();
		}


		if(bEnableVisualizeGlobalPathForCARLA)
		{
			autoware_msgs::LaneArray lane_array;
			visualization_msgs::MarkerArray pathsToVisualize;

			for(unsigned int i=0; i < m_GlobalPaths.size(); i++)
			{
				autoware_msgs::Lane lane;
				PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(m_GlobalPaths.at(i), lane);
				lane_array.lanes.push_back(lane);
			}

			std_msgs::ColorRGBA total_color;
			total_color.r = 0;
			total_color.g = 0.7;
			total_color.b = 1.0;
			total_color.a = 0.6;
			PlannerHNS::ROSHelpers::createGlobalLaneArrayMarker(total_color, lane_array, pathsToVisualize);
			PlannerHNS::ROSHelpers::createGlobalLaneArrayOrientationMarker(lane_array, pathsToVisualize);
			PlannerHNS::ROSHelpers::createGlobalLaneArrayVelocityMarker(lane_array, pathsToVisualize);
			pub_PathsRviz.publish(pathsToVisualize);
		}
	}
}

void TrajectoryGen::GenerateSmoothTrajectory(const std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > >& rollOuts_in, std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > >& rollOuts_out)
{
	std::vector<std::vector<PlannerHNS::WayPoint > > road_out;
	std::vector<PlannerHNS::WayPoint > path_out;

	rollOuts_out.clear();
	for(auto& road : rollOuts_in)
	{
		road_out.clear();
		int min_size = DBL_MAX;
		for(auto& path : road)
		{
			path_out.clear();
			PlannerHNS::PlannerH traj_planner;
			traj_planner.GenerateKinematicallyFeasibleTrajectory(m_VehicleStatus, m_CurrentPos, m_CarInfo,
					m_SteeringDelay, m_PlanningParams.pathDensity, m_MinPursuitDistance, m_CarInfo.max_speed_forward, path, path_out, true);
			road_out.push_back(path_out);
			if(path_out.size() < min_size)
			{
				min_size = path_out.size();
			}
		}

		//Trim the simulated trajectory so it has the same number of waypoints as the
		// smallest trajectory size in all rollouts, it is important to keep consistency with the local planning stack
		for(auto& path : road_out)
		{
			if(path.size() > min_size)
			{
				path.resize(min_size);
			}
		}

		rollOuts_out.push_back(road_out);
	}
}

void TrajectoryGen::MainLoop()
{
	ros::Rate loop_rate(50);

	PlannerHNS::WayPoint prevState, state_change;

	while (ros::ok())
	{
		ros::spinOnce();

		if(bInitPos && m_GlobalPaths.size()>0)
		{
			m_GlobalPathSections.clear();

			PlannerHNS::WayPoint start_point = m_CurrentPos;
			if(bFrontAxelStart)
			{
				start_point.pos.x += m_CarInfo.wheel_base*cos(start_point.pos.a);
				start_point.pos.y += m_CarInfo.wheel_base*sin(start_point.pos.a);
			}

			for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
			{
				t_centerTrajectorySmoothed.clear();

				m_prev_index.at(i) = PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_GlobalPaths.at(i), start_point, m_PlanningParams.horizonDistance ,
						m_PlanningParams.pathDensity ,t_centerTrajectorySmoothed, m_prev_index.at(i));

				if(m_prev_index.at(i) > 0 ) m_prev_index.at(i) = m_prev_index.at(i) -1;

				m_GlobalPathSections.push_back(t_centerTrajectorySmoothed);
			}

			if(m_PlanningParams.enableHeadingSmoothing)
			{
				double follow_distance = PlannerHNS::PlanningHelpers::CalculateLookAheadDistance(m_SteeringDelay, m_VehicleStatus.speed, m_MinPursuitDistance);
				PlannerHNS::WayPoint estimatedPose = start_point;
				PlannerHNS::PlanningHelpers::EstimateFuturePosition(start_point, m_VehicleStatus.steer, follow_distance, m_PlanningParams.pathDensity/2.0, m_CarInfo.wheel_base, estimatedPose);
				start_point = estimatedPose;
			}

			std::vector<PlannerHNS::WayPoint> sampledPoints_debug;
			m_Planner.GenerateRunoffTrajectory(m_GlobalPathSections, start_point,
								m_PlanningParams.enableLaneChange,
								m_VehicleStatus.speed,
								m_PlanningParams.microPlanDistance,
								m_PlanningParams.maxSpeed,
								m_PlanningParams.minSpeed,
								m_PlanningParams.carTipMargin,
								m_PlanningParams.rollInMargin,
								m_PlanningParams.rollInSpeedFactor,
								m_PlanningParams.pathDensity,
								m_PlanningParams.rollOutDensity,
								m_PlanningParams.rollOutNumber,
								m_PlanningParams.smoothingDataWeight,
								m_PlanningParams.smoothingSmoothWeight,
								m_PlanningParams.smoothingToleranceError,
								m_PlanningParams.speedProfileFactor,
								m_PlanningParams.enableHeadingSmoothing,
								-1 , -1,
								m_RollOuts, sampledPoints_debug);


			if(m_bEnableForwardSimulation)
			{
				GenerateSmoothTrajectory(m_RollOuts, m_SmoothRollOuts);
			}
			else
			{
				m_SmoothRollOuts = m_RollOuts;
			}

			autoware_msgs::LaneArrayStamped local_lanes;
			for(unsigned int i=0; i < m_SmoothRollOuts.size(); i++)
			{
				for(unsigned int j=0; j < m_SmoothRollOuts.at(i).size(); j++)
				{
					autoware_msgs::Lane lane;
					PlannerHNS::PlanningHelpers::PredictConstantTimeCostForTrajectory(m_SmoothRollOuts.at(i).at(j), start_point, m_PlanningParams.minSpeed, m_PlanningParams.microPlanDistance);
					PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(m_SmoothRollOuts.at(i).at(j), lane);
					lane.closest_object_distance = 0;
					lane.closest_object_velocity = 0;
					lane.cost = 0;
					lane.is_blocked = false;
					lane.lane_index = i;
					local_lanes.lanes.push_back(lane);
				}
			}
                        local_lanes.header.stamp = ros::Time::now();
			pub_LocalTrajectories.publish(local_lanes);
		}
		else
			sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 	1,		&TrajectoryGen::callbackGetGlobalPlannerPath, 	this);

		visualization_msgs::MarkerArray all_rollOuts;
		PlannerHNS::ROSHelpers::TrajectoriesToMarkers(m_SmoothRollOuts, all_rollOuts);
		pub_LocalTrajectoriesRviz.publish(all_rollOuts);

		loop_rate.sleep();
	}
}

}
