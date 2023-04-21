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

	void RsActionReflectorNew::Act() // run() 之后会执行act，act根据mGroupReflector里面的放进去的action的对象,即mGroupReflector->AddAction(mActionReflector);
	{
		double vel = 0, rvel = 0;
		double dd = hypot(fabs(MSGPROXY->GetOdometry().GetX() - mLastPose.GetX()),
						  fabs(MSGPROXY->GetOdometry().GetY() - mLastPose.GetY()));
		mDistTravelled += dd;
		mLastPose = MSGPROXY->GetOdometry().GetPose();
		RsPose robotPose = MSGPROXY->GetOdometry().GetPose(); // 获取机器人位姿
		auto laser_map = MSGPROXY->GetMsgLaserScan();		  // 获取激光数据

		// 检测机器人四个方向的激光数据是否在机身以内，碰撞检测
		for (const auto &laser : laser_map)
		{
			auto count = static_cast<int>(std::floor(RsMath::Abs(laser.second.scan().angle_max() -
																 laser.second.scan().angle_min()) /
													 laser.second.scan().angle_increment()) +
										  1);
			LOG_DEBUG("obstalce count %d %f %f %f", count, laser.second.scan().angle_max(), laser.second.scan().angle_min(), laser.second.scan().angle_increment());
			for (int i = 0; i < count; i++)
			{
				double point_angle = laser.second.scan().angle_min() + laser.second.scan().angle_increment() * i;
				if (laser.second.scan().ranges(i) == 0)
				{
					continue;
				}
				double point_x = cos(point_angle * M_PI / 180) * laser.second.scan().ranges(i);
				double point_y = sin(point_angle * M_PI / 180) * laser.second.scan().ranges(i);
				// 计算机器人坐标系下的反光板坐标x,y
				double laser_th = RsMath::DegToRad(laser.second.scan().laser_pose().orientation().yaw()); // laser_th为激光雷达相对于机器人坐标系的角度差
				double point_to_robot_x = laser.second.scan().laser_pose().position().x() + point_x * cos(laser_th) - point_y * sin(laser_th);
				double point_to_robot_y = laser.second.scan().laser_pose().position().y() + point_x * sin(laser_th) + point_y * cos(laser_th);
				// LOG_DEBUG("reflector position %f %f", point_to_robot_x, point_to_robot_y);
				if (point_to_robot_x < MSGPROXY->GetRobotLength() / 2 + obstacleDE && point_to_robot_x > -MSGPROXY->GetRobotLength() / 2 - obstacleDE &&
					point_to_robot_y < MSGPROXY->GetRobotWidth() / 2 + obstacleDE && point_to_robot_y > -MSGPROXY->GetRobotWidth() / 2 - obstacleDE)
				{
					LOG_DEBUG("obstalce detected %f %f length %f %f %f %f %f %f count %d", point_to_robot_x, point_to_robot_y,
							  MSGPROXY->GetRobotLength() / 2 + obstacleDE, MSGPROXY->GetRobotWidth() / 2 + obstacleDE, laser.second.scan().ranges(i), point_angle, point_x, point_y, count);
					vel = 0;
					rvel = 0;
					SetState(STATE_FAILED);
				}
			}
		}
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
			SetState(STATE_ACHIEVED_DISTANCE); // 一个周期内的行走距离大于mDistance，
			vel = 0;
			rvel = 0;
		}
		State state = GetState(); // 初始为STATE_START
		if (state == STATE_START)
		{
			if (mReflector->Update())
			{
				mReflector->GetTargetPosition(mTargetX, mTargetY, mTargetTh);
				LOG_DEBUG("target pose %f %f %f", mTargetX, mTargetY, mTargetTh);
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
				mReflector->GetTargetPosition(mTargetX, mTargetY, mTargetTh);	  // 机器人坐标系下目标点的x坐标和y坐标
				LOG_DEBUG("target pose %f %f %f", mTargetX, mTargetY, mTargetTh); // mTargetTh传过来是角度，// 机器人坐标系下 目标点在墙上的朝向在机器人坐标系下方向
				double phi, Rx, Ry, output;
				double ReftargetY, ReftargetX;
				double lastError = 0.0;
				double error = 0.0;
				static double lastRx = 1;
				static int backAndForth = 0;
				// double angle_threshold = mTargetTh - MSGPROXY->GetOdometry().GetPose().GetTh();
				// auto laser_map = MSGPROXY->GetMsgLaserScan(); // 获取激光数据

				// 将mTargetX, mTargetY, mTargetTh转换到世界坐标系，如果没有目标就以上一个世界坐标系下的目标点为目标
				// double Rotheta = RsMath::DegToRad(robotPose.GetTh());
				// double TargetGlobalX = robotPose.GetX() + mTargetX * cos(Rotheta) - mTargetX * sin(Rotheta);
				// double TargetGlobalY = robotPose.GetY() + mTargetX * sin(Rotheta) + mTargetX * cos(Rotheta);

				// 以反光板中心建立坐标系，墙体为x轴，
				double rol1 = atan2(mTargetY, mTargetX) * 180 / M_PI; // 机器人坐标系下的目标方向
				double distance = sqrt(pow(mTargetX, 2) + pow(mTargetY, 2));
				ReftargetX = 0.0;
				ReftargetY = -700.0;
				double Dcheck;
				// int lineDelta = 10;
				double minrotate = mErrorTheta;
				// 如果反光板在机器人左边，rol1>0,目标点为（0，-500）
				if (fabs(mTargetTh) < 90)
				{

					if (mTargetTh > 0) // 车头朝向墙外
					{
						phi = -1 * (180.0 - rol1 + mTargetTh); // phi反光办坐标系下的机器人中心点的方向
						Rx = distance * cos(phi * M_PI / 180);
						Ry = distance * sin(phi * M_PI / 180);
					}
					else if (rol1 > 0) // 车头朝向墙内
					{
						phi = -1 * (180.0 - rol1 - fabs(mTargetTh));
						Rx = distance * cos(phi * M_PI / 180);
						Ry = distance * sin(phi * M_PI / 180);
					}
					else
					{
						phi = -1 * (180.0 + fabs(rol1) - fabs(mTargetTh));
						Rx = distance * cos(phi * M_PI / 180);
						Ry = distance * sin(phi * M_PI / 180);
					}
				}

				////如果反光板在机器人右边
				else
				{
					if (mTargetTh > 0)
					{
						phi = -1 * (fabs(rol1) - 180.0 + mTargetTh); // phi反光板坐标系下的机器人方向,一般都只有这种情况
						Rx = distance * cos(phi * M_PI / 180);
						Ry = distance * sin(phi * M_PI / 180);
					}
					else if (rol1 < 0) // 车头朝向墙内,并且车头方向在目标点上面,一般都只有这中情况
					{
						phi = -1 * (fabs(rol1) + 180.0 - fabs(mTargetTh));
						Rx = distance * cos(phi * M_PI / 180);
						Ry = distance * sin(phi * M_PI / 180);
					}
					else // 车头朝向墙内,并且车头方向在目标点下面
					{
						phi = -1 * (180.0 - fabs(rol1) - fabs(mTargetTh));
						Rx = distance * cos(phi * M_PI / 180);
						Ry = distance * sin(phi * M_PI / 180);
					}
				}

				double gapRX = ReftargetX - Rx;
				double gapRy = ReftargetY - Ry;
				double gapthe = atan2(gapRy, gapRX) * 180 / M_PI;
				if (Rx * lastRx < 0) // 来回行走的情况，来回3次就停下
				{
					backAndForth++;
				}
				lastRx = Rx;
				LOG_INFO("jcjc rol1 : %f   : %f  Rx : %f  Ry : %f  mTargetTh : %f gapthe: %f ", rol1, phi, Rx, Ry, mTargetTh, gapthe);
				// 第一步先调试到y坐标符合的地方
				if (backAndForth >= 3)
				{
					rvel = 0;
					vel = 0;
					LOG_INFO("move near the goal  %d", backAndForth);
					backAndForth = 0;
					SetState(STATE_REACHED_FINAL_DISTANCE);
				}
				else if (fabs(ReftargetY - Ry) < mErrorDist) // mErrorDist 15// y坐标符合，x坐标不符合的情况。 x轴移动机器人至目标点，前进或者后退
				{
					if (fabs(Rx - ReftargetX) < mErrorOffset && (fabs(fabs(mTargetTh) - 180) < mErrorTheta || fabs(mTargetTh) < mErrorTheta)) // mErrorTheta 5 ;mErrorOffset 10
					{
						SetState(STATE_REACHED_FINAL_DISTANCE);
						rvel = 0;
						vel = 0;
						LOG_INFO("move get goal  ");
						backAndForth = 0;
					}
					else
					{
						// 如果反光板在机器人左边，rol1>0,反光板x轴正向上
						if (fabs(mTargetTh) < 90) // 机器人正对着墙时候为90
						{
							if (Rx - ReftargetX > 0) // 如果反光板在机器人左边,机器人在目标点前面,机器人往后退
							{
								if (fabs(fabs(mTargetTh)) < mErrorTheta + 3) // 机器人角度误差不大的情况
								{
									// vel = -1 * std ::min(Normalspeed, Max_Final_Speed);
									// rvel = 0;

									vel = -1 * std::min(Normalspeed, Max_Final_Speed);
									Dcheck = 0.1 * vel * sin(-1 * mTargetTh / 180.0 * M_PI); // 转换到x轴向上的右手定则坐标系 偏向角-1×mTargetTh， KD控制需要取反;0.1为控制周期

									error = (ReftargetY - Ry) / 1000; // 误差距离：cm
									// Dcheck = error - lastError;
									output = -1 * (KP * error + Dcheck * KD) / robotRadius; // v/r = w
									rvel = output;
									LOG_INFO("move arc back output %f error : %f Dcheck: %f", output, error, Dcheck);
								}
								else
								{
									vel = 0;
									double gaptarThe = mTargetTh;
									if (fabs(gaptarThe) < minrotate)
									{
										rvel = minrotate * fabs(gaptarThe) / gaptarThe / 180.0 * M_PI;
										LOG_INFO("move rotate rvel = minrotate %f  gaptarThe %f", minrotate, gaptarThe);
									}
									else
									{
										rvel = mTargetTh / 180.0 * M_PI;
										LOG_INFO("move rotate rvel %f  gaptarThe %f", rvel, gaptarThe);
									}
								}
							}
							else // 如果反光板在机器人左边,机器人在目标点后面,机器人往前  一般都只有这种情况
							{
								if (fabs(mTargetTh) < mErrorTheta + 3) // mErrorTheta =5
								{
									// vel = std::min(Normalspeed, Max_Final_Speed);
									// rvel = 0;

									vel = std::min(Normalspeed, Max_Final_Speed);
									Dcheck = 0.1 * vel * sin(-1 * mTargetTh / 180.0 * M_PI);
									error = (ReftargetY - Ry) / 1000; // 误差距离：cm
									output = (KP * error - Dcheck * KD) / robotRadius;
									rvel = output;
									LOG_INFO("move arc forward output %f error : %f Dcheck: %f", output, error, Dcheck);
								}
								else
								{
									vel = 0;
									double gaptarThe = mTargetTh;
									if (fabs(gaptarThe) < minrotate)
									{
										rvel = minrotate * fabs(gaptarThe) / gaptarThe / 180.0 * M_PI;
										LOG_INFO("move rotate rvel = minrotate %f  gaptarThe %f", minrotate, gaptarThe);
									}
									else
									{
										rvel = mTargetTh / 180.0 * M_PI;
										LOG_INFO("move rotate rvel %f  gaptarThe %f", rvel, gaptarThe);
									}
								}
							}
						}
						else //  如果反光板在机器人右侧，反光板x轴正向下
						{
							// y坐标符合，x坐标不符合的情况。 x轴移动机器人至目标点，前进或者后退
							// Rx机器人位置在目标点下面，机器人往前走
							if (Rx - ReftargetX > 0)
							{
								if (fabs(180 - fabs(mTargetTh)) < mErrorTheta + 3)
								{
									// vel = std::min(Normalspeed, Max_Final_Speed);
									// rvel = 0;

									vel = std::min(Normalspeed, Max_Final_Speed);
									Dcheck = 0.1 * vel * sin(pi2pi(180 - mTargetTh) / 180.0 * M_PI); // 转换到x轴向上的右手定则坐标系， 偏向角pi2pi(180-mTargetTh)，
									error = (ReftargetY - Ry) / 1000;								 // 误差距离：cm
									output = -1 * (KP * error + Dcheck * KD) / robotRadius;
									rvel = output;
									LOG_INFO("move arc forward output %f error : %f Dcheck: %f", output, error, Dcheck);
								}
								else
								{
									vel = 0;
									double gaptarThe = pi2pi(180 - mTargetTh);
									if (fabs(gaptarThe) < minrotate)
									{
										rvel = minrotate * fabs(gaptarThe) / gaptarThe / 180.0 * M_PI;
										LOG_INFO("move rotate rvel = minrotate %f  gaptarThe %f", minrotate, gaptarThe);
									}
									else
									{
										rvel = -1 * pi2pi(180 - mTargetTh) / 180.0 * M_PI;
										LOG_INFO("move rotate rvel %f  gaptarThe %f", rvel, gaptarThe);
									}
								}
							}
							else // 一般都只有这种情况,机器人往后退
							{
								if (fabs(fabs(mTargetTh) - 180) < mErrorTheta + 3) // ry满足之后,和目标角度角度差不大的情况PID
								{
									// 机器人往后退
									// vel = -1 * std::min(Normalspeed, Max_Final_Speed);
									// rvel = 0;

									vel = -1 * std::min(Normalspeed, Max_Final_Speed);
									Dcheck = 0.1 * vel * sin(pi2pi(180 - mTargetTh) / 180.0 * M_PI);
									error = (ReftargetY - Ry) / 1000; // 误差距离：cm
									output = (KP * error + Dcheck * KD) / robotRadius;
									rvel = output;
									LOG_INFO("move arc back output %f error : %f Dcheck: %f", output, error, Dcheck);
								}
								else // ry满足之后角度差太大，只能停下来转
								{
									vel = 0;
									double gaptarThe = pi2pi(180 - mTargetTh);
									if (fabs(gaptarThe) < minrotate) // 如果小于2度就按2度转
									{
										rvel = minrotate * fabs(gaptarThe) / gaptarThe / 180.0 * M_PI;
										LOG_INFO("move rotate rvel = minrotate %f  gaptarThe %f", minrotate, gaptarThe);
									}
									else
									{
										rvel = -1 * pi2pi(180 - mTargetTh) / 180.0 * M_PI;
										LOG_INFO("move rotate rvel %f  gaptarThe %f", rvel, gaptarThe);
									}
								}
							}
						}
					}
				}
				// x坐标y坐标都不符合的情况，先移动机器人让y坐标至目标点，前进或后退走弧形
				else
				{

					// 如果反光板在机器人左边，rol1>0,反光板x轴正向上
					if (fabs(mTargetTh) < 90)
					{

						if (ReftargetX - Rx < 0) ////机器人位置ReftargetX 小于目标点 机器人往后退
						{

							vel = -1 * std::min(Normalspeed, Max_Final_Speed);
							Dcheck = 0.1 * vel * sin(-1 * mTargetTh / 180.0 * M_PI); // 转换到x轴向上的右手定则坐标系 偏向角-1×mTargetTh， KD控制需要取反;0.1为控制周期
							error = (ReftargetY - Ry) / 1000;						 // 误差距离：cm
							output = -1 * (KP * error + Dcheck * KDD) / robotRadius; // v/r = w
							double angleT = fabs(mTargetTh);
							if (angleT > angleRotate) // 转的太厉害了，直接撞墙了，得控制角度
							{
								rvel = 0;
							}
							else
							{
								rvel = output;
							}
							LOG_INFO("move arc back output %f error: %f Dcheck: %f angleT: %f", output, error, Dcheck, angleT);
						}
						else // 机器人往前走 一般都只有这种情况
						{

							vel = std::min(Normalspeed, Max_Final_Speed);
							Dcheck = 0.1 * vel * sin(-1 * mTargetTh / 180.0 * M_PI);
							error = (ReftargetY - Ry) / 1000; // 误差距离：cm
							output = (KP * error - Dcheck * KDD) / robotRadius;
							double angleT = fabs(mTargetTh);
							if (angleT > angleRotate) // 转的太厉害了，直接撞墙了，得控制角度
							{
								rvel = 0;
							}
							else
							{
								rvel = output;
							}
							LOG_INFO("move arc front output %f error : %f Dcheck: %f angleT : %f ", output, error, Dcheck, angleT);
						}
					}
					else // 如果反光板在机器人右边， 反光板x轴正向下
					{
						if (ReftargetX - Rx < 0) // ry不满足的情况,机器人位置ReftargetX 小于目标点 机器人往前
						{

							vel = std::min(Normalspeed, Max_Final_Speed);
							Dcheck = 0.1 * vel * sin(pi2pi(180 - mTargetTh) / 180.0 * M_PI); // 转换到x轴向上的右手定则坐标系， 偏向角pi2pi(180-mTargetTh)，
							error = (ReftargetY - Ry) / 1000;								 // 误差距离：cm
							output = -1 * (KP * error + Dcheck * KDD) / robotRadius;
							double angleT = fabs(pi2pi(180 - mTargetTh));
							if (angleT > angleRotate) // 转的太厉害了，直接撞墙了，得控制角度
							{
								rvel = 0;
							}
							else
							{
								rvel = output;
							}

							LOG_INFO("move arc forward output %f error : %f Dcheck: %f angleT : %f", output, error, Dcheck, angleT);
						}
						else // ry不满足的情况,机器人往后 一般都只有这种情况
						{

							vel = -1 * std::min(Normalspeed, Max_Final_Speed);
							Dcheck = 0.1 * vel * sin(pi2pi(180 - mTargetTh) / 180.0 * M_PI);
							error = (ReftargetY - Ry) / 1000; // 误差距离：cm
							output = (KP * error + Dcheck * KDD) / robotRadius;
							double angleT = fabs(pi2pi(180 - mTargetTh)); // angleT机器人朝向与墙面的夹角的pi2pi
							if (angleT > angleRotate)					  // 转的太厉害了，直接撞墙了，得控制角度
							{
								rvel = 0;
							}
							else
							{
								rvel = output;
							}

							LOG_INFO("move arc back output %f error : %f Dcheck: %f angleT: %f", output, error, Dcheck, angleT);
						}
					}

					lastError = error;
				}

				if (rvel > Max_Rot)
				{
					rvel = Max_Rot;
				}
				if (rvel < Min_Rot)
				{
					rvel = Min_Rot;
				}
				LOG_INFO("jcjc vel : %f rvel : %f ", vel, rvel * 180.0 / M_PI);
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
		act.SetVal(int(RsAction::ActionKeys::TransVel), vel * 1000, 1.0);		  // 线速度vel:米/s
		act.SetVal(int(RsAction::ActionKeys::RotateVel), rvel * 180 / M_PI, 1.0); // 角速度rvel:弧度/s
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
		mErrorOffset = mArg["final_error_accept_offset"].asDouble(); // 10
		mErrorTheta = mArg["final_error_accept_theta"].asDouble();	 // 2
		mErrorDist = mArg["final_error_accept_distance"].asDouble(); // 15
	}

	// RsPose RsActionReflectorNew::RCoor2WCoor(RsPose point, RsPose robot)
	// {
	// 	RsPose wcoor;
	// 	double Rotheta = RsMath::DegToRad(robot.GetTh());
	// 	double x = robot.GetX() + point.GetX() * cos(Rotheta) - point.GetY() * sin(Rotheta);
	// 	double y = robot.GetY() + point.GetX() * sin(Rotheta) + point.GetY() * cos(Rotheta);
	// 	wcoor.SetX(x);
	// 	wcoor.SetY(y);
	// 	return wcoor;
	// }

	int32_t RsActionReflectorNew::pi2pi(int32_t angle)
	{
		int32_t tmp = static_cast<int32_t>(angle % 360);
		if (tmp > 180)
		{
			tmp = tmp - 360;
		}
		else if (tmp <= -180)
		{
			tmp = tmp + 360;
		}
		return tmp;
	}

}