//
// Created by sheldon on 6/19/17.
//

#ifndef MCL_RsReflector_H
#define MCL_RsReflector_H

#include <bitset>
#include "math.h"
#include "RsJson.h"
#include "RsLog.h"
namespace rcs
{
class RsReflectorNew
{
  public:
	class ReflectPiece
	{
	  public:
		//reflect raw data
		ReflectPiece() : left_x(0), left_y(0), left_angle(0), right_x(0), right_y(0), right_angle(0), position_x(0),
				position_y(0), width(0), count(0)
		{}
		struct LaserMsg
		{
			LaserMsg():laser_x(0),laser_y(0),laser_th(0)
			{}
			std::string laser_id;
			double laser_x;
			double laser_y;
			double laser_th;
			void show()
			{
				LOG_DEBUG("laser_%s_msg %f %f %f",laser_id.c_str(),laser_x,laser_y,laser_th);
			}
		};
		double left_x;
		double left_y;
		double left_angle;
		double right_x;
		double right_y;
		double right_angle;
		double position_x;
		double position_y;
		double width;
		int count;
		int start_index;
		int end_index;
		LaserMsg laser_msg;
		void show()
		{
			LOG_DEBUG("left_x: %f, left_y: %f, left_angle: %f, right_x: %f, right_y: %f, right_angle: %f,"
			          "position_x: %f, position_y: %f, count: %d, width: %f", left_x, left_y, left_angle, right_x, right_y,
			          right_angle, position_x, position_y, count, width);
			laser_msg.show();
		}
	};

	RsReflectorNew()
	{
		mTargetX = 0;
		mTargetY = 0;
		mTargetTh = 0;
	};
	bool Init(RsArg mArg);
	bool Update();
	void GetTargetPosition(double &x, double &y, double &angle);
	void SetGap(double gap)
	{
		mReliableReflectorGap = gap;
	}

  private:
	void ShowParams();
	bool Calculate();
	bool ChangeCoordinate();
	std::vector<ReflectPiece> mReflectorPieces;
	std::map<std::string,std::vector<ReflectPiece>> mReliablePieces;
	std::pair<ReflectPiece,ReflectPiece> mReflectorPair;
	double mReliableReflectorWidth;
	int mReliableReflectorCount;
	double mReliableReflectorDistance;
	double mMinX,mMaxX,mMinY,mMaxY;
	double mReliableReflectorGap;
	double mReliableReflectorGapError;
	double mTargetX;
	double mTargetY;
	double mTargetTh;
};
}
#endif //MCL_RsReflector_H
