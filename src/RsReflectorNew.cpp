//
// Created by sheldon on 6/20/17.
//
#include "RsMsgProxy.h"
#include "RsReflectorNew.h"
using namespace rcs_msg;
namespace rcs
{
bool RsReflectorNew::Update()
{
	auto laser_map = MSGPROXY->GetMsgLaserScan();
	mReliablePieces.clear();
	for(const auto& laser:laser_map)
	{
		mReflectorPieces.clear();
		if (laser.second.scan().ranges().empty() || laser.second.scan().intensities().empty())
		{
			continue;
		}
		std::vector<int> intensity;
		std::vector<double> ranges;
		auto count = static_cast<int>(std::floor(RsMath::Abs(laser.second.scan().angle_max() -
				laser.second.scan().angle_min()) / laser.second.scan().angle_increment()) + 1);
		ranges.resize(count, 0);
		intensity.resize(count, 0);
		for (int i = 0; i < count; i++)
		{
			ranges[i] = laser.second.scan().ranges(i);
			intensity[i] = laser.second.scan().intensities(i);
		}
		if (ranges.empty() || intensity.empty() || ranges.size() != intensity.size())
		{
			LOG_INFO("no laser data");
			continue;
		}
		//raw laser data
		std::string range_data;
		std::string reflect_data;
		std::string show_reflector_data;
		for (int i = 0; i < intensity.size(); i++)
		{
			show_reflector_data += std::to_string(intensity[i]);
		}
		LOG_DEBUG("received reflector data : %s", show_reflector_data.c_str());
		//pre deal data
		for (int i = 1; i < intensity.size() - 1; i++)
		{
			if (intensity[i] == 0)
			{
				if (intensity[i - 1] == 1 && intensity[i + 1] == 1)
				{
					intensity[i] = 1;
					ranges[i] = (ranges[i - 1] + ranges[i + 1]) / 2.0;
				}
			}
			if (intensity[i] == 1)
			{
				if (intensity[i - 1] == 0 && intensity[i + 1] == 0)
				{
					intensity[i] = 0;
					ranges[i] = (ranges[i - 1] + ranges[i + 1]) / 2.0;
				}
			}
		}
		show_reflector_data.clear();
		for (int i = 0; i < intensity.size(); i++)
		{
			show_reflector_data += std::to_string(intensity[i]);
		}
		LOG_DEBUG("after pre deal data : %s", show_reflector_data.c_str());
		for (int i = 0; i < intensity.size() - 1; i++)
		{
			if (intensity[i])
			{
				ReflectPiece reflect_piece;
				reflect_piece.laser_msg.laser_id = laser.first;
				reflect_piece.laser_msg.laser_x = laser.second.scan().laser_pose().position().x();
				reflect_piece.laser_msg.laser_y = laser.second.scan().laser_pose().position().y();
				reflect_piece.laser_msg.laser_th = laser.second.scan().laser_pose().orientation().yaw();
				double start_angle = laser.second.scan().angle_min() + laser.second.scan().angle_increment() * i;
				double start_x = cos(start_angle * M_PI / 180) * ranges[i];
				double start_y = sin(start_angle * M_PI / 180) * ranges[i];
				reflect_piece.right_x = start_x;
				reflect_piece.right_y = start_y;
				reflect_piece.right_angle = start_angle;
				reflect_piece.start_index = i;

				double average_x = 0;
				double average_y = 0;
				int average_count = 0;
				if (hypot(start_x, start_y) < 10000)
				{
					average_x += start_x;
					average_y += start_y;
					average_count++;
					LOG_DEBUG("start %f %f",start_x,start_y);
				}
				else
				{
					LOG_DEBUG("point(%f %f) is far away from the robot", start_x, start_y);
					continue;
				}
				for (int j = i + 1; j < intensity.size(); j++)
				{
					if (intensity[j] != 1)
					{
						double end_angle = laser.second.scan().angle_min() + laser.second.scan().angle_increment() * (j - 1);
						double end_x = cos(end_angle * M_PI / 180) * ranges[j - 1];
						double end_y = sin(end_angle * M_PI / 180) * ranges[j - 1];
						reflect_piece.left_x = end_x;
						reflect_piece.left_y = end_y;
						reflect_piece.left_angle = end_angle;
						reflect_piece.end_index = j - 1;
						reflect_piece.width = sqrt(pow(start_x - end_x, 2) + pow(start_y - end_y, 2));
						if (average_count > 1)
						{
							average_x = average_x / average_count;
							average_y = average_y / average_count;
							reflect_piece.position_x = average_x;
							reflect_piece.position_y = average_y;
							reflect_piece.count = average_count;
							mReflectorPieces.push_back(reflect_piece);
						}
						i = j;
						break;
					}
					else if (intensity[j] == 1 && j == intensity.size() - 1)
					{
						double end_angle = laser.second.scan().angle_min() + laser.second.scan().angle_increment() * j;
						double end_x = cos(end_angle * M_PI / 180) * ranges[j];
						double end_y = sin(end_angle * M_PI / 180) * ranges[j];
						reflect_piece.left_angle = end_angle;
						reflect_piece.left_x = end_x;
						reflect_piece.left_y = end_y;
						reflect_piece.end_index = j;
						reflect_piece.width = sqrt(pow(start_x - end_x, 2) + pow(start_y - end_y, 2));

						if (hypot(end_x, end_y) < 10000)
						{
							average_x += end_x;
							average_y += end_y;
							average_count++;
						}
						if (average_count > 0)
						{
							average_x = average_x / average_count;
							average_y = average_y / average_count;
							reflect_piece.position_x = average_x;
							reflect_piece.position_y = average_y;
							reflect_piece.count = average_count;
							mReflectorPieces.push_back(reflect_piece);
						}
						i = j;
						break;
					}
					double angle = laser.second.scan().angle_min() + laser.second.scan().angle_increment() * j;
					double xx = cos(angle * M_PI / 180) * ranges[j];
					double yy = sin(angle * M_PI / 180) * ranges[j];
					if (hypot(xx, yy) < 10000)
					{
						average_x += xx;
						average_y += yy;
						average_count++;
					}
				}
			}
		}
		if (mReflectorPieces.size() < 2)
		{
			LOG_DEBUG("not enough reflector pieces");
			continue;
		}
		for (int i = 0; i < mReflectorPieces.size(); i++)
		{
			if (mReflectorPieces[i].count < mReliableReflectorCount)
			{
				LOG_INFO("reflect %d not enough cout %d", i, mReflectorPieces[i].count);
				continue;
			}
			if (mReflectorPieces[i].width < mReliableReflectorWidth)
			{
				LOG_INFO("reflect %d not enough width %lf", i, mReflectorPieces[i].width);
				continue;
			}
			double laser_th = RsMath::DegToRad(mReflectorPieces[i].laser_msg.laser_th);
			double reflector_to_robot_x = mReflectorPieces[i].laser_msg.laser_x +
			                              mReflectorPieces[i].position_x * cos(laser_th) - mReflectorPieces[i].position_y * sin(laser_th);
			double reflector_to_robot_y = mReflectorPieces[i].laser_msg.laser_y +
			                              mReflectorPieces[i].position_x * sin(laser_th) + mReflectorPieces[i].position_y * cos(laser_th);
			LOG_DEBUG("reflector position %f %f", reflector_to_robot_x, reflector_to_robot_y);
			double distance = hypot(reflector_to_robot_x, reflector_to_robot_y);
			if (distance > mReliableReflectorDistance)
			{
				LOG_DEBUG("reflect %d is too far away %lf", i, distance);
				continue;
			}
			if (reflector_to_robot_x > mMaxX || reflector_to_robot_x < mMinX)
			{
				LOG_DEBUG("reflect %d %lf is out of x edge (%f %f)", i, reflector_to_robot_x, mMinX, mMaxX);
				continue;
			}
			if (reflector_to_robot_y > mMaxY || reflector_to_robot_y < mMinY)
			{
				LOG_DEBUG("reflect %d %lf is out of y edge", i, reflector_to_robot_y, mMinY, mMaxY);
				continue;
			}
			mReflectorPieces[i].show();
			mReliablePieces[laser.first].push_back(mReflectorPieces[i]);
		}
	}
	if (Calculate())
	{
		return ChangeCoordinate();
	}
	else
	{
		return false;
	}

}

bool RsReflectorNew::Calculate()
{
	LOG_DEBUG("mReliablePieces size %d",mReliablePieces.size())
	for(auto& reflector_pieces:mReliablePieces)
	{
		if (reflector_pieces.second.size() < 2)
		{
			LOG_DEBUG("not enough reliable reflector pieces");
			continue;
		}
		else if (reflector_pieces.second.size() == 2)
		{
			double left_x = reflector_pieces.second.at(1).position_x;
			double left_y = reflector_pieces.second.at(1).position_y;
			double right_x = reflector_pieces.second.at(0).position_x;
			double right_y = reflector_pieces.second.at(0).position_y;
			double detect_distance = hypot(left_x-right_x,left_y-right_y);
			double error = fabs(detect_distance - mReliableReflectorGap);
			if (error > mReliableReflectorGapError)
			{
				LOG_INFO("reflector distance %lf reliable distance bet %lf acc %lf  is not accept", detect_distance,
						mReliableReflectorGap, mReliableReflectorGapError);
				reflector_pieces.second.clear();
				continue;
			}
		}
		else
		{
			LOG_INFO("%d reliable reflector is detected", reflector_pieces.second.size());
			std::vector<int> reliable_distance_left;
			std::vector<int> reliable_distance_right;
			for (int i = 0; i <  reflector_pieces.second.size() - 1; i++)
			{
				for (int j = i + 1; j <  reflector_pieces.second.size(); j++)
				{
					double left_x = reflector_pieces.second[j].position_x;
					double left_y = reflector_pieces.second[j].position_y;
					double right_x = reflector_pieces.second[i].position_x;
					double right_y = reflector_pieces.second[i].position_y;
					double detect_distance = hypot(left_x-right_x,left_y-right_y);
					double error = fabs(detect_distance - mReliableReflectorGap);
					if (error > mReliableReflectorGapError)
					{
						LOG_INFO("refuse %d %d distance %lf error %lf > acc %lf", i, j, detect_distance, error, mReliableReflectorGapError);
					}
					else
					{
						LOG_INFO("accept %d %d distance %lf error %lf < acc %lf", i, j, detect_distance, error, mReliableReflectorGapError);
						reliable_distance_left.push_back(j);
						reliable_distance_right.push_back(i);
					}
				}
			}
			LOG_INFO("reliable ditance reflector pair count %d", reliable_distance_left.size(),
					reliable_distance_right.size());
			if (reliable_distance_left.size() == 1 && reliable_distance_right.size() == 1)
			{
				LOG_INFO("--now-- reliable ditance reflector pair %d %d", reliable_distance_left.at(0), reliable_distance_right.at(0));
				ReflectPiece right_piece = reflector_pieces.second[reliable_distance_right[0]];
				ReflectPiece left_piece = reflector_pieces.second[reliable_distance_left[0]];
				reflector_pieces.second.clear();
				reflector_pieces.second.push_back(right_piece);
				reflector_pieces.second.push_back(left_piece);
			}
			else
			{
				LOG_INFO("still no reliable reflector found");
				reflector_pieces.second.clear();
				continue;
			}
		}
	}
	int valid_count = 0;
	for(auto& reflector_pieces:mReliablePieces)
	{
		if(reflector_pieces.second.size() == 2)
		{
			mReflectorPair.first = reflector_pieces.second[0];
			mReflectorPair.second = reflector_pieces.second[1];
			valid_count++;
		}
	}
	return valid_count == 1;
}

bool RsReflectorNew::ChangeCoordinate()
{
	double r_laser_th = RsMath::DegToRad(mReflectorPair.first.laser_msg.laser_th);
	double rx = mReflectorPair.first.laser_msg.laser_x +
			    mReflectorPair.first.position_x*cos(r_laser_th) - mReflectorPair.first.position_y*sin(r_laser_th);
	double ry = mReflectorPair.first.laser_msg.laser_y +
	            mReflectorPair.first.position_x*sin(r_laser_th) + mReflectorPair.first.position_y*cos(r_laser_th);

	double l_laser_th = RsMath::DegToRad(mReflectorPair.second.laser_msg.laser_th);
	double lx = mReflectorPair.second.laser_msg.laser_x +
	            mReflectorPair.second.position_x*cos(l_laser_th) - mReflectorPair.second.position_y*sin(l_laser_th);
	double ly = mReflectorPair.second.laser_msg.laser_y +
	            mReflectorPair.second.position_x*sin(l_laser_th) + mReflectorPair.second.position_y*cos(l_laser_th);
	mTargetX = (rx+lx)/2;
	mTargetY = (ry+ly)/2;
	mTargetTh = atan2(ry-ly,rx-lx);
	return true;
}

void RsReflectorNew::GetTargetPosition(double &x, double &y, double &angle)
{
	x = mTargetX;
	y = mTargetY;
	angle = mTargetTh * 180.0 / M_PI;
}

bool RsReflectorNew::Init(RsArg arg)
{
	mReliableReflectorWidth = RsJson::GetDouble(arg, "reliable_reflector_width", 20, true);
	mReliableReflectorCount = RsJson::GetInt(arg, "reliable_reflector_count", 5, true);
	mReliableReflectorDistance = RsJson::GetDouble(arg, "reliable_reflector_distance", 800, true);
	mMinX = RsJson::GetDouble(arg, "min_x", -2000, true);
	mMaxX = RsJson::GetDouble(arg, "max_x", 2000, true);
	mMinY = RsJson::GetDouble(arg, "min_y", -1000, true);
	mMaxY = RsJson::GetDouble(arg, "max_y", 1000, true);
	mReliableReflectorGapError = RsJson::GetDouble(arg, "reliable_re_bet_dis_accept_error", 100, true);
	ShowParams();
}

void RsReflectorNew::ShowParams()
{
	LOG_INFO("--now-- re para reliable_reflector_width %lf", mReliableReflectorWidth);
	LOG_INFO("--now-- re para reliable_reflector_count  %d", mReliableReflectorCount);
	LOG_INFO("--now-- re para reliable_reflector_distance %lf", mReliableReflectorDistance);
	LOG_INFO("--now-- re  para x range %lf %lf", mMinX,mMaxX);
	LOG_INFO("--now-- re  para y range %lf %lf", mMinY,mMaxY);
	LOG_INFO("--now-- re  para reliable_reflector_between_distance %lf", mReliableReflectorGap);
	LOG_INFO("--now-- re  para reliable_re_bet_dis_accept_error %lf", mReliableReflectorGapError);
}
}