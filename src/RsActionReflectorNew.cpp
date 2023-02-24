//
// Created by sheldon on 18-3-7.
//

#include "RsActionReflectorNew.h"
#include "RsMsgProxy.h"
using namespace rcs_msg;
namespace rcs
{

RsActionReflectorNew::RsActionReflectorNew(std::string name, double speed) : RsAction(name)
{
	mReflector = std::make_shared<RsReflectorNew>();
	mSpeed = speed;
	mState = STATE_NO_DISTANCE;
}

RsActionReflectorNew::~RsActionReflectorNew()
{

}

void RsActionReflectorNew::CancelDistance()
{
	SetState(STATE_NO_DISTANCE);
}

void RsActionReflectorNew::SetDistance(double distance, bool useEncoders)
{
	mDistance = distance;
	SetState(STATE_START);
	mLastPose = MSGPROXY->GetOdometry().GetPose();
	mDistTravelled = 0;
	mLaserValidTime = CTime::GetCurTime();
}

void RsActionReflectorNew::Act()
{
	double vel = 0, rvel = 0;
	double dd = hypot(fabs(MSGPROXY->GetOdometry().GetX() - mLastPose.GetX()),
	                  fabs(MSGPROXY->GetOdometry().GetY() - mLastPose.GetY()));
	mDistTravelled += dd;
	mLastPose = MSGPROXY->GetOdometry().GetPose();
	double distToGo;
	if (mDistance >= 0)
	{
		distToGo = mDistance - mDistTravelled;
	}
	else
	{
		distToGo = -mDistance - mDistTravelled;
	}
	if (distToGo <= 0 && fabs(MSGPROXY->GetOdometry().GetV()) < 5)
	{
		LOG_INFO("Achieved distance: %f > %f", mDistTravelled, mDistance);
		SetState(STATE_ACHIEVED_DISTANCE);
		vel = 0;
		rvel = 0;
	}
	State state = GetState();
	if (state == STATE_START)
	{
		if (mReflector->Update())
		{
			mReflector->GetTargetPosition(mTargetX, mTargetY, mTargetTh);
			LOG_DEBUG("target pose %f %f %f",mTargetX, mTargetY, mTargetTh);
			mStartOdom = MSGPROXY->GetOdometry();
			SetState(STATE_GOING_DISTANCE);
			mLaserValidTime = CTime::GetCurTime();
		}
		else
		{
			if (CTime::GetCurTime() - mLaserValidTime > 3)
			{
				SetState(STATE_FAILED);
			}
		}
	}
	else if (state == STATE_GOING_DISTANCE)
	{
		if (mReflector->Update())
		{
			mReflector->GetTargetPosition(mTargetX, mTargetY, mTargetTh);
			//todo
			LOG_DEBUG("target pose %f %f %f",mTargetX, mTargetY, mTargetTh);
//			if ((-mTargetX - mFinalDistance) < mErrorDist)
//			{
//				if (fabs(mTargetY) < mErrorOffset && fabs(mTargetTh) < mErrorTheta)
//				{
//					SetState(STATE_REACHED_FINAL_DISTANCE);
//				}
//				else
//				{
//					SetState(STATE_FAILED);
//				}
//				rvel = 0;
//				vel = 0;
//			}
//			else
//			{
//				vel = mSpeed / 1000.0;
//				if ((-mTargetX - mFinalDistance) < 2 * mErrorDist)
//				{
//					double tmp = vel;
//					vel = std::min(tmp / 2.0, Max_Final_Speed);
//				}
//				rvel = -(mTargetY) / 10 * mGain / 180.0 * M_PI;
//				if (vel > Max_Speed)
//				{
//					vel = Max_Speed;
//				}
//				if (vel < Min_Speed)
//				{
//					vel = Min_Speed;
//				}
//				if (rvel > Max_Rot)
//				{
//					rvel = Max_Rot;
//				}
//				if (rvel < Min_Rot)
//				{
//					rvel = Min_Rot;
//				}
//			}
		}
		else
		{
			if (CTime::GetCurTime() - mLaserValidTime > 3)
			{
				SetState(STATE_FAILED);
			}
		}
	}
	RsAction act;
	act.SetVal(int(RsAction::ActionKeys::TransVel), vel * 1000, 1.0);
	act.SetVal(int(RsAction::ActionKeys::RotateVel), rvel * 180 / M_PI, 1.0);
	SetAllData(act.GetAllData());
}

double RsActionReflectorNew::GetTravelledDistance()
{
	return mDistTravelled;
}

void RsActionReflectorNew::SetFinalDistance(double p_final_distance)
{
	mFinalDistance = p_final_distance;
}

void RsActionReflectorNew::SetAheadDistance(double p_ahead_distance)
{
	mAheadDistance = p_ahead_distance;
}

void RsActionReflectorNew::SetGap(double p_distance_between)
{
	mReflector->SetGap(p_distance_between);
}

void RsActionReflectorNew::SetSpeed(double speed)
{
	mSpeed = speed;
}

void RsActionReflectorNew::SetParams(RsArg mArg)
{
	mReflector->Init(mArg);
	mGain = mArg["gain"].asDouble();
	mErrorOffset = mArg["final_error_accept_offset"].asDouble();
	mErrorTheta = mArg["final_error_accept_theta"].asDouble();
	mErrorDist = mArg["final_error_accept_distance"].asDouble();
}
}