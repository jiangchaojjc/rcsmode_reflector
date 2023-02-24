/*
 *  RCSORE ROBOTICS
 *  Copyright (c) 2015-2018, umore robotics company
 *  All rights reserved.
 */


#ifndef RCS_MODE_REFLECTORNEW_H
#define RCS_MODE_REFLECTORNEW_H

#include "RsMode.h"
#include "RsBase.h"
#include "pose.h"
#include "RsActionReflectorNew.h"
#include "RsActionAvoid.h"
#include "RsActionGroup.h"
#include "RsPlugin.h"

#define MODE_NAME "ModeReflectorNew"
#define MODE_VERSION "1.0.0"
#define TASK_NAME "reflector"

namespace rcs
{
/**
 *  @class	RsModeFollow
 *  @brief	mode RsModeFollow to handle set follow mode
 *  @author	sheldon
 *  @date	2018.01.27
 */
class RsModeReflectorNew : public RsMode
{
  public:
	RsModeReflectorNew(const std::string& name);
	bool init();
	void Start(const RsArg &task) final;
	void Run() final;
  private:
	bool is_updated;
	bool reflector_computed_;
	Pose mid_pose;
	Pose result_pose;
	RsOdometry odom_pose;
	std::shared_ptr<RsObsAvoid> mObsAvoid;
	std::shared_ptr<RsActionGroup> mGroupReflector;
	std::shared_ptr<RsActionReflectorNew> mActionReflector;
	std::shared_ptr<RsActionAvoid> mActionObsAvoid;

	double max_distance;
	//    double scan_edge_left;
	//    double scan_dege_right;
	//    double target_width;

	//params from command
	double final_distance,ahead_distance;
	double reflector_offset;
	double move_speed;
	bool old_behavior;
	double distance_between_reflector;

	double final_error_accept_offset;
	double final_error_accept_theta;
	double final_error_accept_distance;

	//distinct return 0 and 1
	double distinct01_distance;
	bool distinct01_is_on;

	double limit_time;

	double rotate_acc,rotate_dec;

	RsArg mParams;

	int mTryedTimes;
	int mPermitBackTimes;
	bool mFirstFailed;
	RsOdometry mStartOdom;
};
MODEPLUGIN(RsModeReflectorNew, MODE_NAME, MODE_VERSION)

}


#endif //RCS_MODE_GOTO_H
