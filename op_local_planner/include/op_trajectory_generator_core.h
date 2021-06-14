/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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

#ifndef OP_TRAJECTORY_GENERATOR_CORE
#define OP_TRAJECTORY_GENERATOR_CORE

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/LaneArrayStamped.h>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_can_msgs/CANInfo.h>

#include "op_planner/PlannerH.h"
#include "op_planner/PlannerCommonDef.h"

namespace TrajectoryGeneratorNS
{

class TrajectoryGen
{
protected:
	PlannerHNS::PlannerH m_Planner;
	geometry_msgs::Pose m_OriginPos;
	PlannerHNS::WayPoint m_InitPos;
	bool bInitPos;
	bool bEnableSmoothGlobalPathForCARLA;
	bool bEnableVisualizeGlobalPathForCARLA;

	PlannerHNS::WayPoint m_CurrentPos;
	bool bNewCurrentPos;

	PlannerHNS::VehicleState m_VehicleStatus;
	bool bVehicleStatus;
	bool bFrontAxelStart;

	std::vector<PlannerHNS::WayPoint> m_temp_path;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathSections;
	std::vector<int> m_prev_index;
	std::vector<PlannerHNS::WayPoint> t_centerTrajectorySmoothed;
	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > m_RollOuts;
	std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > > m_SmoothRollOuts;
	bool bWayGlobalPath;
  	std::vector<std::string>    m_LogData;
  	PlannerHNS::PlanningParams m_PlanningParams;
  	PlannerHNS::CAR_BASIC_INFO m_CarInfo;

	//for CARLA challenge 
  	const double m_DistanceLimitInTimeOut = 25; //meters
  	struct timespec m_PlanningTimer;
  	double m_distance_moved_since_stuck;
  	double m_distance_moved;
  	bool m_bStuckState;
  	int m_nOriginalRollOuts;

  	double m_SteeringDelay;
  	double m_MinPursuitDistance;
  	bool m_bEnableForwardSimulation;

  	//ROS messages (topics)
	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_PathsRviz;
	ros::Publisher pub_LocalTrajectories;
	ros::Publisher pub_LocalTrajectoriesRviz;

	// define subscribers.
	ros::Subscriber sub_initialpose;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_vehicle_status;
	ros::Subscriber sub_can_info;
	ros::Subscriber sub_GlobalPlannerPaths;


	// Callback function for subscriber.
	void callbackGetInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);
	void callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg);
	void callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg);

	//Helper Functions
  void UpdatePlanningParams(ros::NodeHandle& _nh);
  void GenerateSmoothTrajectory(const std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > >& rollOuts_in, std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > >& rollOuts_out);

public:
	TrajectoryGen();
  ~TrajectoryGen();
  void MainLoop();
};

}

#endif  // OP_TRAJECTORY_GENERATOR_CORE
