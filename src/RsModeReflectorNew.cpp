//
// Created by sheldon on 2018.2.6.
//

#include "RsModeReflectorNew.h"
using namespace rcs_msg;
namespace rcs
{

	RsModeReflectorNew::RsModeReflectorNew(const std::string &name) : RsMode(name),
																	  mGroupReflector(std::make_shared<RsActionGroup>("ModeGroupreflector")),
																	  mActionReflector(std::make_shared<RsActionReflectorNew>())
	{
		AddTask(TASK_NAME);
		mObsAvoid = std::make_shared<RsObsAvoid>();
		mObsAvoid->SetCallBack();
		mActionObsAvoid = std::make_shared<RsActionAvoid>("ActionAvoid", mObsAvoid);
		// mGroupReflector->AddAction(mActionObsAvoid);
		mGroupReflector->AddAction(mActionReflector);
	}
	bool RsModeReflectorNew::init()
	{
		mActionReflector->SetParams(mParams);

		// init param from file
		// final_distance = GetArgAsDouble("distance");
		distance_between_reflector = GetArgAsDouble("gap");
		reflector_offset = -GetArgAsDouble("offset");
		move_speed = GetArgAsDouble("speed");
		//  yAwayFromWall = GetArgAsDouble("yAwayFromWall");

		max_distance = RsJson::GetDouble(mParams, "max_distance", 2000, true);
		ahead_distance = RsJson::GetDouble(mParams, "ahead_distance", 500, true);
		final_error_accept_offset = RsJson::GetDouble(mParams, "final_error_accept_offset", 15, true);
		final_error_accept_distance = RsJson::GetDouble(mParams, "final_error_accept_distance", 20, true);
		final_error_accept_theta = RsJson::GetDouble(mParams, "final_error_accept_theta", 3, true);
		distinct01_distance = RsJson::GetDouble(mParams, "distinct01_distance", 100, true);
		distinct01_is_on = RsJson::GetBool(mParams, "distinct01_is_on", false, true);

		LOG_INFO("--now-- distinct01_is_on %d distinct01_distance %lf", distinct01_is_on, distinct01_distance);

		mActionReflector->SetDistance(max_distance);

		// final distance
		mActionReflector->SetFinalDistance(final_distance);

		mActionReflector->SetAheadDistance(ahead_distance);

		mActionReflector->SetSpeed(move_speed);

		mActionReflector->SetGap(distance_between_reflector);
		//	LOG_INFO( "--now-- param init finished");

		return true;
	}
	void RsModeReflectorNew::Start(const RsArg &task)
	{
		mParams = MSGPROXY->GetSection(SECTION_TASK)["reflector"]; // 根据这个关键字获取Json::Value中的mparams,再根据mparams在init中获取Getdouble或这GetInit等值
		init();
		MSGPROXY->EnableMotor();
		RsArg arg = MSGPROXY->GetSection(SECTION_TASK)["reflector_obsavoid"];
		mActionObsAvoid->SetArg(arg);							   // 用来避障的参数
		mTryedTimes = 0;										   // 这个值是干什么的？
		mPermitBackTimes = RsJson::GetInt(task, "back_times", -1); // 这个值是干什么的？
		final_distance = RsJson::GetDouble(task, "distance", 100);
		mFirstFailed = false;
		SetStatus("reflector");
	}

	void RsModeReflectorNew::Stop()
	{
		MSGPROXY->DisableMotor();
	}

	void RsModeReflectorNew::Run()
	{
		if (!mGroupReflector->IsActive())
		{
			LOG_DEBUG("start reflect");
			SetStatus(std::string("reflecting"));
			mGroupReflector->Active();
		}
		else // 根据RsActionReflectorNew.cpp 中的act返回情况返回给外部一些状态
		{
			if (mActionReflector->GetState() == RsActionReflectorNew::STATE_ACHIEVED_DISTANCE) // 这个什么状态？？
			{
				if (!mFirstFailed)
				{
					mFirstFailed = true;
					mStartOdom = MSGPROXY->GetOdometry();
				}
				else
				{
					double dist = hypot(MSGPROXY->GetOdometry().GetX() - mStartOdom.GetX(), // 第一次里程计和第二次里程计差》1000
										MSGPROXY->GetOdometry().GetY() - mStartOdom.GetY());
					if (dist > 1000)
					{
						mPermitBackTimes = -1;
					}
				}
				LOG_INFO("--now-- myActionReflector.haveAchievedDistance");
				if (mPermitBackTimes < 0 || mTryedTimes >= mPermitBackTimes) //
				{
					TaskRet(0); // 失败的情况，任务返回，taskRet任务返回
					return;
				}
			}
			if (mActionReflector->GetState() == RsActionReflectorNew::STATE_REACHED_FINAL_DISTANCE) // 到达目标点
			{
				SetStatus(std::string("finish reflection"));
				TaskRet(1); // 任务成功
				return;
			}
			else if (mActionReflector->GetState() == RsActionReflectorNew::STATE_FAILED) // 任务失败的情况
			{
				LOG_INFO("--now-- myActionReflector.is_find_reflector_failed");
				if (distinct01_is_on)
				{
					LOG_INFO("--now-- have approached %lf distinct distance %lf ", mActionReflector->GetTravelledDistance(),
							 distinct01_distance);
					if (mActionReflector->GetTravelledDistance() < distinct01_distance)
					{
						if (!mFirstFailed)
						{
							mFirstFailed = true;
							mStartOdom = MSGPROXY->GetOdometry();
						}
						else
						{
							double dist = hypot(MSGPROXY->GetOdometry().GetX() - mStartOdom.GetX(),
												MSGPROXY->GetOdometry().GetY() - mStartOdom.GetY());
							if (dist > 1000)
							{
								mPermitBackTimes = -1;
							}
						}
						if (mPermitBackTimes < 0 || mTryedTimes >= mPermitBackTimes) // 假如这里不满足，程序会怎么运行？
						{
							SetStatus(std::string("reflect failed"));
							TaskRet(0);
							return;
						}
					}
					else
					{
						if (!mFirstFailed)
						{
							mFirstFailed = true;
							mStartOdom = MSGPROXY->GetOdometry();
						}
						else
						{
							double dist = hypot(MSGPROXY->GetOdometry().GetX() - mStartOdom.GetX(),
												MSGPROXY->GetOdometry().GetY() - mStartOdom.GetY());
							if (dist > 1000)
							{
								mPermitBackTimes = -1;
							}
						}
						if (mPermitBackTimes < 0 || mTryedTimes >= mPermitBackTimes)
						{
							SetStatus(std::string("reflect failed"));
							TaskRet(2);
							return;
						}
					}
				}
				else
				{
					if (!mFirstFailed)
					{
						mFirstFailed = true;
						mStartOdom = MSGPROXY->GetOdometry();
					}
					else
					{
						double dist = hypot(MSGPROXY->GetOdometry().GetX() - mStartOdom.GetX(),
											MSGPROXY->GetOdometry().GetY() - mStartOdom.GetY());
						if (dist > 1000)
						{
							mPermitBackTimes = -1;
						}
					}
					if (mPermitBackTimes < 0 || mTryedTimes >= mPermitBackTimes)
					{
						SetStatus(std::string("reflect failed"));
						TaskRet(2);
						return;
					}
				}
			}
		}
	}

}