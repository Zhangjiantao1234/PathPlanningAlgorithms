#pragma once

#include <BasicLibraryCode/PScenario.hpp>
#include "MInput.h"
#include "WayPoint.h"
#include <map>


using CCSP_BasicLibrary::PScenario;
using CCSP_BasicLibrary::SEUAV;
using CCSP_BasicLibrary::FPoint3;

namespace CCSP_PlanningAlgorithm
{
	using PathMap = std::map<SEUAV*, std::vector<WayPoint>>;

	/**
	 * \brief 航迹规划输出类
	 */
	class MOutput
	{
	public:
		long path_result_ID;					//!<任务结果ID
		PathMap path_map;						//!< 无人机指针(数据源来自PScenario)，value = 无人机轨迹
	};



}
