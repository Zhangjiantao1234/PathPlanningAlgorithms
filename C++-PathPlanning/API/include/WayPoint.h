#pragma once
#include <BasicLibraryCode/Basic/include/FPoint.hpp>


namespace CCSP_PlanningAlgorithm
{
	using CCSP_BasicLibrary::FPoint3;

	class WayPoint
	{
	public:
		int index;
		int stage_index;
		FPoint3 location;
		double time_stamp;
		double velocity;
	};
}