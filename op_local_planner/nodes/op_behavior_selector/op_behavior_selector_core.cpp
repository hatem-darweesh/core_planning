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

#include "op_behavior_selector_core.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/KmlMapLoader.h"
#include "op_planner/Lanelet2MapLoader.h"
#include "op_planner/VectorMapLoader.h"

namespace BehaviorGeneratorNS
{

BehaviorGen::BehaviorGen()
{
	// m_ControlFrequency = 50;
	// make control frequency configureable, because it is related to the desired maximum velocity.
	// The control frequency must be adapted to the message frequency of /current_velocity.
	// If not, the control delta time (dt) for the calculation of the target acceleration is based
	// on old velocity inputs and therefore, wrong.

	bNewCurrentPos = false;
	bVehicleStatus = false;
	bNewLightStatus = false;
	bNewLightSignal = false;
	bBestCost = false;
	bMap = false;
	m_bRequestNewPlanSent = false;
	m_bShowActualDrivingPath = false;

	ros::NodeHandle _nh;
	UpdatePlanningParams(_nh);

	tf::StampedTransform transform;
	tf::TransformListener tf_listener;
	PlannerHNS::ROSHelpers::getTransformFromTF("world", "map", tf_listener, transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_TotalLocalPath = nh.advertise<autoware_msgs::Lane>("op_local_selected_trajectory", 1,true);
	pub_LocalPath = nh.advertise<autoware_msgs::Lane>("final_waypoints", 1,true);
	pub_LocalBasePath = nh.advertise<autoware_msgs::Lane>("base_waypoints", 1,true);
	pub_ClosestIndex = nh.advertise<std_msgs::Int32>("closest_waypoint", 1,true);
	//pub_BehaviorState = nh.advertise<geometry_msgs::TwistStamped>("current_behavior", 1);
	pub_BehaviorState = nh.advertise<autoware_msgs::Waypoint>("op_current_behavior", 1);
	pub_SimuBoxPose	  = nh.advertise<geometry_msgs::PoseArray>("sim_box_pose_ego", 1);
	pub_BehaviorStateRviz = nh.advertise<visualization_msgs::MarkerArray>("behavior_state", 1);
	pub_SelectedPathRviz = nh.advertise<visualization_msgs::MarkerArray>("local_selected_trajectory_rviz", 1);
	pub_TargetSpeedRviz = nh.advertise<std_msgs::Float32>("op_target_velocity_rviz", 1);
	pub_ActualSpeedRviz = nh.advertise<std_msgs::Float32>("op_actual_velocity_rviz", 1);
	pub_CurrDrivingPathRviz = nh.advertise<visualization_msgs::MarkerArray>("op_actual_driving_path", 1);
	pub_DetectedLight = nh.advertise<autoware_msgs::ExtractedPosition>("op_detected_light", 1);
	pub_CurrGlobalLocalPathsIds = nh.advertise<std_msgs::Int32MultiArray>("op_curr_global_local_ids", 1);
	pub_RequestReplan = nh.advertise<std_msgs::Bool>("op_global_replan", 1);

	//Path Planning Section
	//----------------------------
	sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 1, &BehaviorGen::callbackGetGlobalPlannerPath, this);
	sub_LocalPlannerPaths = nh.subscribe("/local_weighted_trajectories", 1, &BehaviorGen::callbackGetLocalPlannerPath, this);
	sub_Trajectory_Cost = nh.subscribe("/local_trajectory_cost", 1, &BehaviorGen::callbackGetLocalTrajectoryCost, this);
	//----------------------------

	//Traffic Information Section
	//----------------------------
	sub_TrafficLightStatus = nh.subscribe("/light_color", 1, &BehaviorGen::callbackGetTrafficLightStatus, this);
	sub_TrafficLightSignals	= nh.subscribe("/roi_signal", 1, &BehaviorGen::callbackGetTrafficLightSignals, this);
	//----------------------------

	// Control Topics Sections
	//----------------------------
	sub_current_pose = nh.subscribe("/current_pose", 1,	&BehaviorGen::callbackGetCurrentPose, this);
	sub_twist_raw = nh.subscribe("/twist_raw", 1, &BehaviorGen::callbackGetTwistRaw, this);
	sub_twist_cmd = nh.subscribe("/twist_cmd", 1, &BehaviorGen::callbackGetTwistCMD, this);
	//sub_ctrl_cmd = nh.subscribe("/ctrl_cmd", 1, &BehaviorGen::callbackGetCommandCMD, this);
	int bVelSource = 1;
	_nh.getParam("/op_common_params/velocitySource", bVelSource);
	std::string velocity_topic;
	if(bVelSource == 0)
	{
		sub_robot_odom = nh.subscribe("/carla/ego_vehicle/odometry", 1, &BehaviorGen::callbackGetRobotOdom, this);
	}
	else if(bVelSource == 1)
	{
		sub_current_velocity = nh.subscribe("/current_velocity", 1, &BehaviorGen::callbackGetAutowareStatus, this);
	}
	else if(bVelSource == 2)
	{
		sub_can_info = nh.subscribe("/can_info", 1, &BehaviorGen::callbackGetCANInfo, this);
	}
	else if(bVelSource == 3)
	{
		_nh.getParam("/op_common_params/vehicle_status_topic", velocity_topic);
		sub_vehicle_status = _nh.subscribe(velocity_topic, 1, &BehaviorGen::callbackGetVehicleStatus, this);
	}
	//----------------------------


	//Mapping Section
	if(m_MapType == PlannerHNS::MAP_AUTOWARE)
	{
		sub_bin_map = nh.subscribe("/lanelet_map_bin", 1, &BehaviorGen::callbackGetLanelet2, this);
		sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &BehaviorGen::callbackGetVMLanes,  this);
		sub_points = nh.subscribe("/vector_map_info/point", 1, &BehaviorGen::callbackGetVMPoints,  this);
		sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &BehaviorGen::callbackGetVMdtLanes,  this);
		sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &BehaviorGen::callbackGetVMIntersections,  this);
		sup_area = nh.subscribe("/vector_map_info/area", 1, &BehaviorGen::callbackGetVMAreas,  this);
		sub_lines = nh.subscribe("/vector_map_info/line", 1, &BehaviorGen::callbackGetVMLines,  this);
		sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &BehaviorGen::callbackGetVMStopLines,  this);
		sub_signals = nh.subscribe("/vector_map_info/signal", 1, &BehaviorGen::callbackGetVMSignal,  this);
		sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &BehaviorGen::callbackGetVMVectors,  this);
		sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &BehaviorGen::callbackGetVMCurbs,  this);
		sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &BehaviorGen::callbackGetVMRoadEdges,  this);
		sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &BehaviorGen::callbackGetVMWayAreas,  this);
		sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &BehaviorGen::callbackGetVMCrossWalks,  this);
		sub_nodes = nh.subscribe("/vector_map_info/node", 1, &BehaviorGen::callbackGetVMNodes,  this);
	}
}

BehaviorGen::~BehaviorGen()
{
#ifdef LOG_LOCAL_PLANNING_DATA
	std::ostringstream fileName;
	if(m_ExperimentFolderName.size() == 0)
		fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::StatesLogFolderName;
	else
		fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + m_ExperimentFolderName + UtilityHNS::DataRW::StatesLogFolderName;

	UtilityHNS::DataRW::WriteLogData(fileName.str(), "MainLog",
				"time,dt, Behavior_i, Behavior_str, RollOuts_n, Blocked_i, Central_i, Selected_i, StopSign_id, Light_id, minStopDist, Follow_Dist, Follow_Vel,"
				"Max_Vel, Target_Vel, PID_Vel, T_cmd_Vel, C_cmd_Vel, Vel, Steer, X, Y, Z, Theta, Goal_Dist, Stop_Dist,"
				, m_LogData);
#endif
}

void BehaviorGen::UpdatePlanningParams(ros::NodeHandle& _nh)
{
	_nh.getParam("/op_common_params/control_frequency", m_ControlFrequency);

	_nh.getParam("/op_common_params/enableSwerving", m_PlanningParams.enableSwerving);
	if(m_PlanningParams.enableSwerving)
		m_PlanningParams.enableFollowing = true;
	else
		_nh.getParam("/op_common_params/enableFollowing", m_PlanningParams.enableFollowing);

	_nh.getParam("/op_common_params/enableTrafficLightBehavior", m_PlanningParams.enableTrafficLightBehavior);
	_nh.getParam("/op_common_params/enableStopSignBehavior", m_PlanningParams.enableStopSignBehavior);
	_nh.getParam("/op_common_params/maxLocalPlanDistance", m_PlanningParams.microPlanDistance);

	_nh.getParam("/op_common_params/pathDensity", m_PlanningParams.pathDensity);

	_nh.getParam("/op_common_params/rollOutDensity", m_PlanningParams.rollOutDensity);
	if(m_PlanningParams.enableSwerving)
		_nh.getParam("/op_common_params/rollOutsNumber", m_PlanningParams.rollOutNumber);
	else
		m_PlanningParams.rollOutNumber = 0;

	_nh.getParam("/op_common_params/horizonDistance", m_PlanningParams.horizonDistance);
	_nh.getParam("/op_common_params/minFollowingDistance", m_PlanningParams.minFollowingDistance);
	_nh.getParam("/op_common_params/minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
	_nh.getParam("/op_common_params/maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
	_nh.getParam("/op_common_params/speedProfileFactor", m_PlanningParams.speedProfileFactor);
	nh.getParam("/op_common_params/curveSlowDownRatio", m_PlanningParams.curveSlowDownRatio);

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
	_nh.getParam("/op_common_params/maxVelocity", m_PlanningParams.maxSpeed);
	_nh.getParam("/op_common_params/minVelocity", m_PlanningParams.minSpeed);
	m_CarInfo.max_speed_forward = m_PlanningParams.maxSpeed;
	m_CarInfo.min_speed_forward = m_PlanningParams.minSpeed;


	m_ControlParams.Steering_Gain = PlannerHNS::PID_CONST(0.07, 0.02, 0.01);
	m_ControlParams.Velocity_Gain = PlannerHNS::PID_CONST(0.1, 0.005, 0.1);
	m_ControlParams.min_safe_follow_distance = m_PlanningParams.maxDistanceToAvoid;
	nh.getParam("/op_common_params/steeringDelay", m_ControlParams.SteeringDelay);
	nh.getParam("/op_common_params/minPursuiteDistance", m_ControlParams.minPursuiteDistance );

	//Internal ACC parameters
	nh.getParam("/op_common_params/use_internal_acc", m_BehaviorGenerator.m_bUseInternalACC);
	nh.getParam("/op_common_params/accelerationPushRatio", m_ControlParams.accelPushRatio);
	nh.getParam("/op_common_params/brakingPushRatio", m_ControlParams.brakePushRatio);

	nh.getParam("/op_common_params/additionalBrakingDistance", m_PlanningParams.additionalBrakingDistance );
	nh.getParam("/op_common_params/goalDiscoveryDistance", m_PlanningParams.goalDiscoveryDistance);
	nh.getParam("/op_common_params/giveUpDistance", m_PlanningParams.giveUpDistance );

	int iSource = 0;
	_nh.getParam("/op_common_params/mapSource" , iSource);
	if(iSource == 0)
		m_MapType = PlannerHNS::MAP_AUTOWARE;
	else if (iSource == 1)
		m_MapType = PlannerHNS::MAP_FOLDER;
	else if(iSource == 2)
		m_MapType = PlannerHNS::MAP_KML_FILE;
	else if(iSource == 3)
	{
		m_MapType = PlannerHNS::MAP_LANELET_2;
		std::string str_origin;
		nh.getParam("/op_common_params/lanelet2_origin" , str_origin);
		std::vector<std::string> lat_lon_alt = PlannerHNS::MappingHelpers::SplitString(str_origin, ",");
		if(lat_lon_alt.size() == 3)
		{
			m_Map.origin.pos.lat = atof(lat_lon_alt.at(0).c_str());
			m_Map.origin.pos.lon = atof(lat_lon_alt.at(1).c_str());
			m_Map.origin.pos.alt = atof(lat_lon_alt.at(2).c_str());
		}
	}

	_nh.getParam("/op_common_params/mapFileName" , m_MapPath);
	_nh.getParam("/op_behavior_selector/evidence_trust_number", m_PlanningParams.nReliableCount);
	_nh.getParam("/op_behavior_selector/show_driving_path", m_bShowActualDrivingPath);



	_nh.getParam("/op_common_params/experimentName" , m_ExperimentFolderName);
	if(m_ExperimentFolderName.size() > 0)
	{
		if(m_ExperimentFolderName.at(m_ExperimentFolderName.size()-1) != '/')
			m_ExperimentFolderName.push_back('/');
	}

	UtilityHNS::DataRW::CreateLoggingMainFolder();
	if(m_ExperimentFolderName.size() > 1)
	{
		UtilityHNS::DataRW::CreateExperimentFolder(m_ExperimentFolderName);
	}

	//std::cout << "nReliableCount: " << m_PlanningParams.nReliableCount << std::endl;

	m_BehaviorGenerator.Init(m_ControlParams, m_PlanningParams, m_CarInfo);
	m_BehaviorGenerator.m_pCurrentBehaviorState->m_Behavior = PlannerHNS::INITIAL_STATE;

}

// Control Topics Sections
//----------------------------
void BehaviorGen::callbackGetTwistRaw(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_Twist_raw = *msg;
}

void BehaviorGen::callbackGetTwistCMD(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_Twist_cmd = *msg;
}

void BehaviorGen::callbackGetCommandCMD(const autoware_msgs::ControlCommandConstPtr& msg)
{
	m_Ctrl_cmd = *msg;
}

void BehaviorGen::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	bNewCurrentPos = true;
}

void BehaviorGen::callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;

	if(fabs(m_CurrentPos.v) > 0.1)
	{
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/m_CurrentPos.v);
	}
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void BehaviorGen::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_CurrentPos.v = m_VehicleStatus.speed;
	m_VehicleStatus.steer = msg->angle * m_CarInfo.max_wheel_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void BehaviorGen::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed ;
	if(msg->twist.twist.linear.x != 0)
	{
		m_VehicleStatus.steer += atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	}
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void BehaviorGen::callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_VehicleStatus.steer = -msg->angle*DEG2RAD;
	m_CurrentPos.v = m_VehicleStatus.speed;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
//	std::cout << "Vehicle Real Status, Speed: " << m_VehicleStatus.speed << ", Steer Angle: " << m_VehicleStatus.steer << ", Steermode: " << msg->steeringmode << ", Org angle: " << msg->angle <<  std::endl;
}

//----------------------------

//Path Planning Section
//----------------------------
void BehaviorGen::callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0 && bMap)
	{
		m_GlobalPaths.clear();
		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), m_temp_path);

			if(bMap)
			{
				PlannerHNS::Lane* pPrevValid = 0;
				for(unsigned int j = 0 ; j < m_temp_path.size(); j++)
				{
					PlannerHNS::Lane* pLane = 0;
					pLane = PlannerHNS::MappingHelpers::GetLaneById(m_temp_path.at(j).laneId, m_Map);
					if(!pLane)
					{
						pLane = PlannerHNS::MappingHelpers::GetClosestLaneFromMap(m_temp_path.at(j), m_Map, 1, true);

						if(!pLane && !pPrevValid)
						{
							ROS_ERROR("Map inconsistency between Global Path and Local Planer Map, Can't identify current lane.");
							return;
						}

						if(!pLane)
							m_temp_path.at(j).pLane = pPrevValid;
						else
						{
							m_temp_path.at(j).pLane = pLane;
							pPrevValid = pLane ;
						}

						m_temp_path.at(j).laneId = m_temp_path.at(j).pLane->id;
					}
					else
						m_temp_path.at(j).pLane = pLane;
				}
			}

			m_GlobalPaths.push_back(m_temp_path);
		}

		bool bOldGlobalPath = true;
		if(m_GlobalPathsToUse.size() == m_GlobalPaths.size())
		{
			for(unsigned int i=0; i < m_GlobalPaths.size(); i++)
			{
				bOldGlobalPath = PlannerHNS::PlanningHelpers::CompareTrajectories(m_GlobalPaths.at(i), m_GlobalPathsToUse.at(i));
			}
		}
		else
		{
			bOldGlobalPath = false;
		}

		if(!bOldGlobalPath)
		{
			//bWayGlobalPathLogs = true;
			for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
			{
				PlannerHNS::PlanningHelpers::FixPathDensity(m_GlobalPaths.at(i), m_PlanningParams.pathDensity);
				PlannerHNS::PlanningHelpers::SmoothPath(m_GlobalPaths.at(i), 0.4, 0.4, 0.05);
				PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_temp_path);
				PlannerHNS::PlanningHelpers::GenerateRecommendedSpeed(m_GlobalPaths.at(i), m_CarInfo.max_speed_forward, m_PlanningParams.speedProfileFactor);

#ifdef LOG_LOCAL_PLANNING_DATA
				std::ostringstream str_out;
				str_out << UtilityHNS::UtilityH::GetHomeDirectory();
				if(m_ExperimentFolderName.size() == 0)
					str_out << UtilityHNS::DataRW::LoggingMainfolderName;
				else
					str_out << UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + m_ExperimentFolderName;

				str_out << UtilityHNS::DataRW::GlobalPathLogFolderName;
				str_out << "GlobalPath_";
				str_out << i;
				str_out << "_";
				PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), m_GlobalPaths.at(i));
#endif
			}

			std::cout << "Received New Global Path Selector! " << std::endl;
		}
	}
}

void BehaviorGen::callbackGetLocalTrajectoryCost(const autoware_msgs::LaneConstPtr& msg)
{
	bBestCost = true;
	m_TrajectoryBestCost.bBlocked = msg->is_blocked;
	m_TrajectoryBestCost.lane_index = msg->lane_id;
	m_TrajectoryBestCost.index = msg->lane_index;
	m_TrajectoryBestCost.cost = msg->cost;
	m_TrajectoryBestCost.closest_obj_distance = msg->closest_object_distance;
	m_TrajectoryBestCost.closest_obj_velocity = msg->closest_object_velocity;
}

void BehaviorGen::callbackGetLocalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0)
	{
		//m_RollOuts.clear();
		std::vector< std::vector<PlannerHNS::WayPoint> > received_local_rollouts;
		std::vector<int> globalPathsId_roll_outs;

		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			std::vector<PlannerHNS::WayPoint> path;
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), path);
			received_local_rollouts.push_back(path);
			//m_RollOuts.push_back(path);

			int roll_out_gid = -1;
			if(path.size() > 0)
			{
				roll_out_gid = path.at(0).gid;
			}

			if(std::find(globalPathsId_roll_outs.begin(), globalPathsId_roll_outs.end(), roll_out_gid) == globalPathsId_roll_outs.end())
			{
				globalPathsId_roll_outs.push_back(roll_out_gid);
			}
		}

		if(CompareTrajectoriesWithIds(m_GlobalPathsToUse, globalPathsId_roll_outs) == true)
		{
			CollectRollOutsByGlobalPath(received_local_rollouts);
			m_BehaviorGenerator.m_LanesRollOuts = m_LanesRollOutsToUse;
		}
		else if(CompareTrajectoriesWithIds(m_GlobalPaths, globalPathsId_roll_outs) == true)
		{
			m_GlobalPathsToUse.clear();
			for(auto& path: m_GlobalPaths)
			{
				m_GlobalPathsToUse.push_back(path);
			}
			CollectRollOutsByGlobalPath(received_local_rollouts);

			m_BehaviorGenerator.SetNewGlobalPath(m_GlobalPathsToUse);
			m_BehaviorGenerator.m_LanesRollOuts = m_LanesRollOutsToUse;
		}
		else
		{
			m_LanesRollOutsToUse.clear();
			m_GlobalPathsToUse.clear();
		}

		// For CARLA challenge
#ifndef DISABLE_CARLA_SPECIAL_CODE
		if(m_RollOuts.size() > 1)
		{
			m_PlanningParams.enableSwerving = true;
			m_PlanningParams.rollOutNumber = m_RollOuts.size() - 1;
		}
		else
		{
			m_PlanningParams.enableSwerving = false;
			m_PlanningParams.rollOutNumber = 0;
		}
#endif

	}
}

void BehaviorGen::CollectRollOutsByGlobalPath(std::vector< std::vector<PlannerHNS::WayPoint> >& local_rollouts)
{
	m_LanesRollOutsToUse.clear();
	std::vector< std::vector<PlannerHNS::WayPoint> > local_category;
//	std::cout << "Collecting Rollouts: ------------------ " << std::endl;
	for(auto& g_path: m_GlobalPathsToUse)
	{
		if(g_path.size() > 0)
		{
			local_category.clear();
			for(auto& l_traj: local_rollouts)
			{
				if(l_traj.size() > 0 && l_traj.at(0).gid == g_path.at(0).gid)
				{
					local_category.push_back(l_traj);
	//				std::cout << "Global Lane ID: " << g_path.at(0).gid << ", Cost Global: " << g_path.at(0).laneChangeCost << ", Cost Local: " << l_traj.at(0).laneChangeCost << std::endl;
				}
			}
			m_LanesRollOutsToUse.push_back(local_category);
		}
	}
//	std::cout << " ------------------ " << std::endl;
}

bool BehaviorGen::CompareTrajectoriesWithIds(std::vector<std::vector<PlannerHNS::WayPoint> >& paths, std::vector<int>& local_ids)
{
	if(local_ids.size() != paths.size())
	{
		std::cout << "Warning From Trajectory Selector, paths size mismatch, GlobalPaths: " << paths.size() << ", LocalPaths: " << local_ids.size() << std::endl;
		return false;
	}

	for(auto& id : local_ids)
	{
		bool bFound = false;
		for(auto& path: paths)
		{
			if(path.size() > 0 && path.at(0).gid == id)
			{
				bFound = true;
			}
		}

		if(bFound == false)
		{
			std::cout << "Synchronization At Trajectory Selector: " << std::endl;
			std::cout << "## Local IDs: ";
			for(auto& id : local_ids)
			{
				std::cout << id << ",";
			}
			std::cout << std::endl << "## Global IDs: ";
			for(auto& path: paths)
			{
				if(path.size() > 0)
				{
					std::cout << path.at(0).gid << ",";
				}
			}
			std::cout << std::endl;

			return false;
		}
	}

	return true;
}

//----------------------------

//Traffic Information Section
//----------------------------
void BehaviorGen::callbackGetTrafficLightStatus(const autoware_msgs::TrafficLight& msg)
{
	// std::cout << "Received Traffic Light Status : " << msg.traffic_light << std::endl;
	bNewLightStatus = true;
	if(msg.traffic_light == 1) // green
		m_CurrLightStatus = PlannerHNS::GREEN_LIGHT;
	else //0 => RED , 2 => Unknown
		m_CurrLightStatus = PlannerHNS::RED_LIGHT;
}

void BehaviorGen::callbackGetTrafficLightSignals(const autoware_msgs::Signals& msg)
{
	bNewLightSignal = true;
	std::vector<PlannerHNS::TrafficLight> simulatedLights;
	for(unsigned int i = 0 ; i < msg.Signals.size() ; i++)
	{
		PlannerHNS::TrafficLight tl;
		tl.id = msg.Signals.at(i).signalId;

		for(unsigned int k = 0; k < m_Map.trafficLights.size(); k++)
		{
			if(m_Map.trafficLights.at(k).id == tl.id)
			{
				tl.pose = m_Map.trafficLights.at(k).pose;
				break;
			}
		}

		switch(msg.Signals.at(i).type)
		{
		case 1:
			tl.lightType = PlannerHNS::RED_LIGHT;
			break;
		case 2:
			tl.lightType = PlannerHNS::GREEN_LIGHT;
			break;
		case 3:
			tl.lightType = PlannerHNS::YELLOW_LIGHT; //r = g = 1
			break;
		case 4:
			tl.lightType = PlannerHNS::CROSS_RED;
			break;
		case 5:
			tl.lightType = PlannerHNS::CROSS_GREEN;
			break;
		default:
			tl.lightType = PlannerHNS::UNKNOWN_LIGHT;
			break;
		}

		simulatedLights.push_back(tl);
	}

	//std::cout << "Received Traffic Lights : " << lights.markers.size() << std::endl;

	m_CurrTrafficLight = simulatedLights;

//	int stopLineID = -1, stopSignID = -1, trafficLightID=-1;
//	PlannerHNS::PlanningHelpers::GetDistanceToClosestStopLineAndCheck(m_BehaviorGenerator.m_Path, m_CurrentPos, stopLineID, stopSignID, trafficLightID);
//	m_CurrTrafficLight.clear();
//	if(trafficLightID > 0)
//	{
//		for(unsigned int i=0; i < simulatedLights.size(); i++)
//		{
//			if(simulatedLights.at(i).id == trafficLightID)
//			{
//				m_CurrTrafficLight.push_back(simulatedLights.at(i));
//			}
//		}
//	}
}
//----------------------------

void BehaviorGen::VisualizeLocalPlanner()
{
	visualization_msgs::Marker behavior_rviz;
	int iDirection = 0;
	if(m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory > m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
		iDirection = 1;
	else if(m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory < m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory)
		iDirection = -1;
	PlannerHNS::ROSHelpers::VisualizeBehaviorState(m_CurrentPos, m_CurrentBehavior, !m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->bTrafficIsRed , iDirection, behavior_rviz, "beh_state");
	//pub_BehaviorStateRviz.publish(behavior_rviz);

	visualization_msgs::MarkerArray markerArray;

	//PlannerHNS::ROSHelpers::GetIndicatorArrows(m_CurrentPos, m_CarInfo.width, m_CarInfo.length, m_CurrentBehavior.indicator, 0, markerArray);

	markerArray.markers.push_back(behavior_rviz);

	pub_BehaviorStateRviz.publish(markerArray);

	std_msgs::Float32 target_speed;
	target_speed.data = m_CurrentBehavior.maxVelocity * 3.6;
	pub_TargetSpeedRviz.publish(target_speed);
	target_speed.data = m_CurrentPos.v * 3.6;
	pub_ActualSpeedRviz.publish(target_speed);

	visualization_msgs::MarkerArray selected_path;
	pub_SelectedPathRviz.publish(selected_path);
	std::vector<PlannerHNS::WayPoint> path = m_BehaviorGenerator.m_Path;
	PlannerHNS::PlanningHelpers::FixPathDensity(path, 1.5);
	PlannerHNS::ROSHelpers::TrajectoryToMarkersWithCircles(path, 1,0,1, 1,0,1, m_CarInfo.width/2.0+m_PlanningParams.horizontalSafetyDistancel, selected_path);
	pub_SelectedPathRviz.publish(selected_path);

	//To Test Synchronization Problem
//	visualization_msgs::MarkerArray selected_path;
//	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > paths;
//  paths.push_back(std::vector<std::vector<PlannerHNS::WayPoint> >());
//	paths.at(0).push_back(m_BehaviorGenerator.m_Path);
//	paths.push_back(m_GlobalPathsToUse);
//	paths.push_back(m_RollOuts);
//	PlannerHNS::ROSHelpers::TrajectoriesToMarkers(paths, selected_path);
//	pub_SelectedPathRviz.publish(selected_path);
}

void BehaviorGen::SendLocalPlanningTopics()
{
	//Send Behavior State

	autoware_msgs::Waypoint wp_state = PlannerHNS::ROSHelpers::ConvertBehaviorStateToAutowareWaypoint(m_CurrentBehavior);
	pub_BehaviorState.publish(wp_state);

	if(m_LanesRollOutsToUse.size() > 0)
	{
		std_msgs::Int32MultiArray ilane_itraj_path_ids;
		ilane_itraj_path_ids.data.push_back(m_CurrentBehavior.iLane);
		ilane_itraj_path_ids.data.push_back(m_CurrentBehavior.iTrajectory);
		for(auto& path: m_GlobalPathsToUse)
		{
			if(path.size() > 0)
			{
				ilane_itraj_path_ids.data.push_back(path.at(0).gid);
			}
		}
		pub_CurrGlobalLocalPathsIds.publish(ilane_itraj_path_ids);
	}

	//Send Ego Vehicle Simulation Pose Data
	geometry_msgs::PoseArray sim_data;
	geometry_msgs::Pose p_id, p_pose, p_box;

	sim_data.header.frame_id = "map";
	sim_data.header.stamp = ros::Time();
	p_id.position.x = 0;
	p_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, UtilityHNS::UtilityH::SplitPositiveAngle(m_BehaviorGenerator.state.pos.a));

	PlannerHNS::WayPoint pose_center = PlannerHNS::PlanningHelpers::GetRealCenter(m_BehaviorGenerator.state, m_CarInfo.wheel_base);

	p_pose.position.x = pose_center.pos.x;
	p_pose.position.y = pose_center.pos.y;
	p_pose.position.z = pose_center.pos.z;
	p_box.position.x = m_BehaviorGenerator.m_CarInfo.width;
	p_box.position.y = m_BehaviorGenerator.m_CarInfo.length;
	p_box.position.z = 2.2;
	sim_data.poses.push_back(p_id);
	sim_data.poses.push_back(p_pose);
	sim_data.poses.push_back(p_box);
	//pub_SimuBoxPose.publish(sim_data); // enable to simulate collision with other simulated vehicles

	//Send Trajectory Data to path following nodes
	std_msgs::Int32 closest_waypoint;
	PlannerHNS::RelativeInfo info;
	PlannerHNS::PlanningHelpers::GetRelativeInfo(m_BehaviorGenerator.m_Path, m_BehaviorGenerator.state, info);
	PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(m_BehaviorGenerator.m_Path, m_CurrentTrajectoryToSend, info.iBack);
	//std::cout << "Path Size: " << m_BehaviorGenerator.m_Path.size() << ", Send Size: " << m_CurrentTrajectoryToSend << std::endl;

	if(m_CurrentTrajectoryToSend.waypoints.size() > 2)
	{
		closest_waypoint.data = 1;
		pub_ClosestIndex.publish(closest_waypoint);
		pub_LocalBasePath.publish(m_CurrentTrajectoryToSend);
		pub_LocalPath.publish(m_CurrentTrajectoryToSend);
	}

	if(m_CurrentBehavior.bNewPlan)
	{
		autoware_msgs::Lane curr_slected_trajectory;
		PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(m_BehaviorGenerator.m_Path, curr_slected_trajectory, 0);
		pub_TotalLocalPath.publish(curr_slected_trajectory);
	}

	autoware_msgs::ExtractedPosition _signal;

	_signal.signalId  = m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->currentTrafficLightID;
	if(m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->bTrafficIsRed)
		_signal.type = PlannerHNS::RED_LIGHT;
	else
		_signal.type = PlannerHNS::GREEN_LIGHT;
	pub_DetectedLight.publish(_signal);
}

void BehaviorGen::LogLocalPlanningInfo(double dt)
{

#ifndef DISABLE_CARLA_SPECIAL_CODE //just for carla
	int curr_lucky_index = 0;
	if(m_BehaviorGenerator.m_prev_index.size() > 0)
	{
		curr_lucky_index = m_BehaviorGenerator.m_prev_index.at(0);
	}
#endif

	double target_vel = 0;
	if(m_BehaviorGenerator.m_Path.size() > 0)
	{
		target_vel = m_BehaviorGenerator.m_Path.at(0).v;
	}

	timespec log_t;
	UtilityHNS::UtilityH::GetTickCount(log_t);
	std::ostringstream dataLine;
	dataLine << UtilityHNS::UtilityH::GetLongTime(log_t) <<"," << dt << "," << m_CurrentBehavior.state << ","<< PlannerHNS::ROSHelpers::GetBehaviorNameFromCode(m_CurrentBehavior.state) << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->m_pParams->rollOutNumber << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->bFullyBlock << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->iCentralTrajectory << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory << "," <<
			//curr_lucky_index << "," <<// only for carla
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->currentStopSignID << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->currentTrafficLightID << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->minStoppingDistance << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->distanceToNext << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->velocityOfNext << "," <<
			m_CurrentBehavior.maxVelocity << "," <<
			target_vel << "," <<
			m_Twist_raw.twist.linear.x << "," <<
			m_Twist_cmd.twist.linear.x << "," <<
			m_Ctrl_cmd.linear_velocity << "," <<
			m_VehicleStatus.speed << "," <<
			m_VehicleStatus.steer << "," <<
			m_BehaviorGenerator.state.pos.x << "," << m_BehaviorGenerator.state.pos.y << "," << m_BehaviorGenerator.state.pos.z << "," << UtilityHNS::UtilityH::SplitPositiveAngle(m_BehaviorGenerator.state.pos.a)+M_PI << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->distanceToGoal << "," <<
			m_BehaviorGenerator.m_pCurrentBehaviorState->GetCalcParams()->distanceToStop() << ",";

	if(m_LogData.size() < 150000 && m_CurrentBehavior.state != PlannerHNS::INITIAL_STATE) //in case I forget to turn off this node .. could fill the hard drive
	{
		m_LogData.push_back(dataLine.str());
	}

	if(m_CurrentBehavior.bNewPlan)
	{
		std::ostringstream str_out;
		str_out << UtilityHNS::UtilityH::GetHomeDirectory();
		if(m_ExperimentFolderName.size() == 0)
			str_out << UtilityHNS::DataRW::LoggingMainfolderName;
		else
			str_out << UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + m_ExperimentFolderName;
		str_out << UtilityHNS::DataRW::PathLogFolderName;
		str_out << "Local_Trajectory_";
		PlannerHNS::PlanningHelpers::WritePathToFile(str_out.str(), m_BehaviorGenerator.m_Path);
	}
}

void BehaviorGen::InsertNewActualPathPair(const double& min_record_distance)
{
	bool bInsertNew = false;
	if(m_ActualDrivingPath.size() == 0)
	{
		bInsertNew = true;
	}
	else
	{
		PlannerHNS::WayPoint prev_p = m_ActualDrivingPath.back().first;
		double d = hypot(prev_p.pos.y-m_CurrentPos.pos.y, prev_p.pos.x-m_CurrentPos.pos.x);
		if(d > min_record_distance)
		{
			bInsertNew = true;
		}
	}

	if(bInsertNew == true)
	{
		PlannerHNS::PolygonShape car_poly;
		PlannerHNS::PlanningHelpers::InitializeSafetyPolygon(m_CurrentPos, m_CarInfo, m_VehicleStatus, m_PlanningParams.horizontalSafetyDistancel, m_PlanningParams.verticalSafetyDistance, false, car_poly);
		m_ActualDrivingPath.push_back(make_pair(m_CurrentPos, car_poly));
	}

	if(m_ActualDrivingPath.size() > 1)
	{
		visualization_msgs::MarkerArray driving_path;
		PlannerHNS::ROSHelpers::DrivingPathToMarkers(m_ActualDrivingPath, driving_path);
		pub_CurrDrivingPathRviz.publish(driving_path);
	}
}

void BehaviorGen::MainLoop()
{
	ros::Rate loop_rate(m_ControlFrequency);
	double dt = 1.0/(double)m_ControlFrequency;
	double avg_dt = dt;

	timespec planningTimer;
	UtilityHNS::UtilityH::GetTickCount(planningTimer);

	while (ros::ok())
	{

		dt  = UtilityHNS::UtilityH::GetTimeDiffNow(planningTimer);
		UtilityHNS::UtilityH::GetTickCount(planningTimer);

		dt_list.push_back(dt);
		if(dt_list.size() > m_ControlFrequency)
		{
			double dt_sum = 0;
			for(auto& step_dt: dt_list)
			{
				dt_sum += step_dt;
			}
			avg_dt = dt_sum / dt_list.size();
			dt_list.erase(dt_list.begin()+0);
		}

		ros::spinOnce();

		if(m_MapType == PlannerHNS::MAP_KML_FILE && !bMap)
		{
			bMap = true;
			PlannerHNS::KmlMapLoader kml_loader;
			kml_loader.LoadKML(m_MapPath, m_Map);
			PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
		}
		else if (m_MapType == PlannerHNS::MAP_FOLDER && !bMap)
		{
			bMap = true;
			PlannerHNS::VectorMapLoader vec_loader(1, m_PlanningParams.enableLaneChange);
			vec_loader.LoadFromFile(m_MapPath, m_Map);
			PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
		}
		else if (m_MapType == PlannerHNS::MAP_LANELET_2 && !bMap)
		{
			bMap = true;
			PlannerHNS::Lanelet2MapLoader map_loader;
			map_loader.LoadMap(m_MapPath, m_Map);
			PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
		}
		else if (m_MapType == PlannerHNS::MAP_AUTOWARE && !bMap)
		{
			if(m_MapRaw.AreMessagesReceived())
			{
				bMap = true;
				PlannerHNS::VectorMapLoader vec_loader(1, m_PlanningParams.enableLaneChange);
				vec_loader.LoadFromData(m_MapRaw, m_Map);
				PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
			}
		}

		if(bNewCurrentPos && m_GlobalPathsToUse.size() > 0)
		{

			for(auto& x: m_CurrTrafficLight)
			{
				x.lightType = m_CurrLightStatus;
			}

#ifndef DISABLE_CARLA_SPECIAL_CODE //just for carla
			m_BehaviorGenerator.UpdateAvoidanceParams(m_PlanningParams.enableSwerving, m_PlanningParams.rollOutNumber);
#endif

			// --- find boundary/area ---
			bool bInsideIntersection = false;
			int currentLaneID = m_BehaviorGenerator.m_LanesRollOuts.at(0).at(0).at(0).laneId;
			for (int iter = 0; iter<m_Map.roadSegments.at(0).Lanes.size(); iter++)
			// iterate torugh all lanes inside the map
			{
				if (m_Map.roadSegments.at(0).Lanes.at(iter).id == currentLaneID)
				// find the lane, the vehicle is using
				{
					m_CurrentPos.boundaryId = m_Map.roadSegments.at(0).Lanes.at(iter).points.at(0).boundaryId;
					// assign the boundary our acutal lanes is starting in (prone to errors if lane does not start 
					// inside the boudary!!!)
				} 
			}

			m_CurrentBehavior = m_BehaviorGenerator.DoOneStep(avg_dt, m_CurrentPos, m_VehicleStatus, m_CurrTrafficLight, m_TrajectoryBestCost, 0 );
			
			if(m_bShowActualDrivingPath)
			{
				InsertNewActualPathPair();
			}

			//if(!m_bRequestNewPlanDone && m_BehaviorGenerator.m_bRequestNewGlobalPlan)
			if(m_BehaviorGenerator.m_bRequestNewGlobalPlan)
			{
				if(m_bRequestNewPlanSent == false)
				{
					std_msgs::Bool replan_req;
					replan_req.data = true;
					pub_RequestReplan.publish(replan_req);
					m_bRequestNewPlanSent = true;
				}
			}
			else
			{
				m_bRequestNewPlanSent = false;
			}


			SendLocalPlanningTopics();
			VisualizeLocalPlanner();
#ifdef LOG_LOCAL_PLANNING_DATA
			LogLocalPlanningInfo(dt);
#endif
		}
		else
		{
			sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 	1,		&BehaviorGen::callbackGetGlobalPlannerPath, 	this);
		}

		loop_rate.sleep();
	}
}
//----------------------------

//Mapping Section
//----------------------------
void BehaviorGen::callbackGetLanelet2(const autoware_lanelet2_msgs::MapBin& msg)
{
	PlannerHNS::Lanelet2MapLoader map_loader;
	map_loader.LoadMap(msg, m_Map);
	PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
	bMap = true;
}

void BehaviorGen::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
	std::cout << "Received Lanes" << endl;
	if(m_MapRaw.pLanes == nullptr)
		m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
}

void BehaviorGen::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
	std::cout << "Received Points" << endl;
	if(m_MapRaw.pPoints  == nullptr)
		m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
}

void BehaviorGen::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg)
{
	std::cout << "Received dtLanes" << endl;
	if(m_MapRaw.pCenterLines == nullptr)
		m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
}

void BehaviorGen::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg)
{
	std::cout << "Received CrossRoads" << endl;
	if(m_MapRaw.pIntersections == nullptr)
		m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
}

void BehaviorGen::callbackGetVMAreas(const vector_map_msgs::AreaArray& msg)
{
	std::cout << "Received Areas" << endl;
	if(m_MapRaw.pAreas == nullptr)
		m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
}

void BehaviorGen::callbackGetVMLines(const vector_map_msgs::LineArray& msg)
{
	std::cout << "Received Lines" << endl;
	if(m_MapRaw.pLines == nullptr)
		m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);
}

void BehaviorGen::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
	std::cout << "Received StopLines" << endl;
	if(m_MapRaw.pStopLines == nullptr)
		m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
}

void BehaviorGen::callbackGetVMSignal(const vector_map_msgs::SignalArray& msg)
{
	std::cout << "Received Signals" << endl;
	if(m_MapRaw.pSignals  == nullptr)
		m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
}

void BehaviorGen::callbackGetVMVectors(const vector_map_msgs::VectorArray& msg)
{
	std::cout << "Received Vectors" << endl;
	if(m_MapRaw.pVectors  == nullptr)
		m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
}

void BehaviorGen::callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg)
{
	std::cout << "Received Curbs" << endl;
	if(m_MapRaw.pCurbs == nullptr)
		m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
}

void BehaviorGen::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg)
{
	std::cout << "Received Edges" << endl;
	if(m_MapRaw.pRoadedges  == nullptr)
		m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
}

void BehaviorGen::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg)
{
	std::cout << "Received Wayareas" << endl;
	if(m_MapRaw.pWayAreas  == nullptr)
		m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
}

void BehaviorGen::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg)
{
	std::cout << "Received CrossWalks" << endl;
	if(m_MapRaw.pCrossWalks == nullptr)
		m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
}

void BehaviorGen::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
	std::cout << "Received Nodes" << endl;
	if(m_MapRaw.pNodes == nullptr)
		m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
}
//----------------------------

}
