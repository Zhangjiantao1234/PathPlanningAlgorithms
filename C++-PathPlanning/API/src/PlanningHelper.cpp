#include "../include/PlanningHelper.h"
#include <cassert>
#include <algorithm>


namespace CCSP_PlanningAlgorithm
{
	PlanningHelper::PlanningHelper()
	{
		scenario_ptr_ = nullptr;
		sample_distance_ = 0;
	}

	PlanningHelper::PlanningHelper(PScenario* scenario_ptr, double sample_distance)
	{
		scenario_ptr_ = scenario_ptr;
		sample_distance_ = sample_distance;
	}

	bool PlanningHelper::IsGoalBlock(FPoint3& wpt, FPoint3& wptGoal) const
	{
		//	global variables
		const double divide_pieces = ceil(FPoint3::DistanceBetweenTwoPlanePoints(wpt, wptGoal) / sample_distance_);

		//减少构造/析构次数
		//让他多COPY几次又何妨
		FPoint3 location;

		// initial is NOT blocked
		bool isBlock = false;


		for (int i = 0; i <= divide_pieces; i++)
		{
			//定比分点
			location = (1 - i / divide_pieces) * wpt + i / divide_pieces*wptGoal;
			// location.xyz[0] = (1 - i / 20.0) * wpt.xyz[0] + i / 20.0 * wptGoal.xyz[0];
			// location.xyz[1] = (1 - i / 20.0) * wpt.xyz[1] + i / 20.0 * wptGoal.xyz[1];
			// location.xyz[2] = (1 - i / 20.0) * wpt.xyz[2] + i / 20.0 * wptGoal.xyz[2];
			
			if (!IsSafePoint(location))
			{
				isBlock = true;
				break;
			}
		}
		return(isBlock);

	}

	bool PlanningHelper::IsSafePoint(const FPoint3& mPoint) const
	{
		assert(scenario_ptr_ != NULL);
		return scenario_ptr_->m_flight_scene.m_threats.IsSafePointCurrent(mPoint);

	}

	bool PlanningHelper::IsSafeLine(const FPoint3& lhs, const FPoint3& rhs) const
	{
		bool isSafe = true;
		//FPoint3 mTempPoint;

		//计算采样点数量
		//int iSampleNumber =(FPoint3.DistanceBetweenTwoSpacePointsXY(startPoint, endPoint) / m_fSampleDistance));//

		double iSampleNumber = ceil(FPoint3::DistanceBetweenTwoPlanePoints(lhs, rhs) / sample_distance_);

		//采样判断(判断起始点和终止点)
		for (int i = 0; i < iSampleNumber + 1; ++i)
		{
			const auto mTempPoint = lhs + (rhs - lhs) * (i / iSampleNumber); //先算实数部分会加快运算速度
			if (!IsSafePoint(mTempPoint))
			{
				isSafe = false;
				break;
			}
		}
		//返回
		return isSafe;
	}

}
