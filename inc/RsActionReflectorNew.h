//
// Created by sheldon on 18-3-7.
//

#ifndef MCL_RsActionReflectorNewNEW_H
#define MCL_RsActionReflectorNewNEW_H

#include "RsBase.h"
#include "RsReflectorNew.h"
#include "RsAction.h"
#define Max_Speed 0.3
#define Min_Speed 0.03
#define Max_Final_Speed 0.05

#define Max_Rot 15/180.0*M_PI
#define Min_Rot -15/180.0*M_PI
using namespace rcs;
namespace rcs
{
class RsActionReflectorNew : public RsAction
{
  public:
	enum State
	{
		STATE_START,
		STATE_GOING_DISTANCE,
		STATE_REACHED_FINAL_DISTANCE,
		STATE_FAILED,
		STATE_NO_DISTANCE,
		STATE_ACHIEVED_DISTANCE
	};
	RsActionReflectorNew(std::string name = "reflector", double speed = 200);
	virtual ~RsActionReflectorNew();
	void SetDistance(double distance, bool useEncoders = true);
	void SetParams(RsArg mArg);
	void SetFinalDistance(double p_final_distance);
	void SetAheadDistance(double p_ahead_distance);
	void SetGap(double p_distance_between);
	void SetSpeed(double speed);
	double GetTravelledDistance();
	void CancelDistance();
	virtual void Act();

	State GetState()
	{
		mStateMutex.lock();
		auto value = mState;
		mStateMutex.unlock();
		return value;
	}

  protected:
	void SetState(State value)
	{
		mStateMutex.lock();
		mState = value;
		mStateMutex.unlock();
	}

	State mState;
	double mTargetX;
	double mTargetY;
	double mTargetTh;
	double mGain = 1.5;
	std::shared_ptr<RsReflectorNew> mReflector;
	RsMutexR mStateMutex;
	RsOdometry mStartOdom;
	double mStartAngle;
	double mLaserValidTime;
	double mFinalDistance;
	double mAheadDistance;
	double mErrorOffset;
	double mErrorTheta;
	double mErrorDist;
	double mSpeed;
	RsPose mLastPose;
	double mDistTravelled;
	double mDistance;
};
}

#endif //MCL_RsActionReflectorNew_H
