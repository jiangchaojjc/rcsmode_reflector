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
#define Max_Final_Speed 0.08 // 单位:m
#define Normalspeed 0.05

#define Max_Rot 15 / 180.0 * M_PI
#define Min_Rot -15 / 180.0 * M_PI
#define slow_Rot 7 / 180.0 * M_PI
#define KP 1.15
#define KDD 40 // 加速收敛
#define KD 70  // 之后环节较大时，加快减小超调量
#define robotRadius 0.25
#define obstacleDE 20
#define angleRotate 20 // 机器人朝向与墙面的夹角不的大于30

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
			STATE_ACHIEVED_DISTANCE // ？？
		};

		enum MoveStatus
		{
			STATE_ROTATE,
			STATE_ARC_FIRST_HALF,
			STATE_ARC_SECOND_HALF,
			STATE_BACK,
			STATE_FORWARD
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
		int32_t pi2pi(int32_t angle);

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

		void SetMoveState(MoveStatus value)
		{
			MoveStateMutex.lock();
			MoveState = value;
			MoveStateMutex.unlock();
		}

		State mState;
		MoveStatus MoveState;
		double mTargetX;
		double mTargetY;
		double mTargetTh;
		double mGain = 1.5;
		std::shared_ptr<RsReflectorNew> mReflector;
		RsMutexR mStateMutex;
		RsMutexR MoveStateMutex;
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
		double slope;
	};
}

#endif // MCL_RsActionReflectorNew_H
