// PathPlanning.cpp : 定义控制台应用程序的入口点。
//


#include "API/include/MInput.h"
#include "RRT/include/RRTAlgorithm.h"
#include "BasicLibraryCode/Mission/include/SEMission.hpp"
#include <iostream>


int main()
{
	using CCSP_BasicLibrary::SEMission;
	using CCSP_BasicLibrary::MissionTargetPtr;
	
	//读取场景
	CCSP_PlanningAlgorithm::MInput algorithm_input
	{
		"2018121010000003.json",
		CCSP_PlanningAlgorithm::AssignmentMap(),
		1
	};

	

	//瞎编一个输入输出关系吧。
	auto tmp_Target_ptr = std::make_shared<CCSP_BasicLibrary::SEMissionTarget>();
	tmp_Target_ptr->m_target_pos = algorithm_input.scenario.m_uav_group.v_UAVs[0].m_start_state.m_location;
	tmp_Target_ptr->index = 1;
	
	

	algorithm_input.assignment_result.insert_or_assign(
		&algorithm_input.scenario.m_uav_group.v_UAVs[0], 
		std::vector<MissionTargetPtr>{ algorithm_input.scenario.m_flight_mission.v_target_ptr[0], tmp_Target_ptr
		});


	//读取任务信息

	//读取算法
	auto myRRT = CCSP_PlanningAlgorithm::RRTAlgorithm(algorithm_input);
	//参数设置（para_ptr在构造函数里已经创建）
	myRRT.para_ptr->i_maxNodeNumber = 500;
	myRRT.para_ptr->d_biasedSamplingProb = 0.2;
	myRRT.para_ptr->n_SelectionType = CCSP_PlanningAlgorithm::RRTParameter::Biased;
	myRRT.para_ptr->n_PlanningStepType = CCSP_PlanningAlgorithm::RRTParameter::Random;
	
	//所有威胁全部打开
	myRRT.SetAllThreatsDetectedStatus(true);

	//运行算法
	for (auto &item : algorithm_input.assignment_result)
	{
		std::cout << "===========================================" << std::endl;
		myRRT.RRT4SingleUAVTaskStatic(item.first);
		auto& result = myRRT.m_planning_results;
		for each (auto tmp in result)
		{
			std::cout << " X: " << tmp.location.x << " Y: " << tmp.location.y << " Z: " << tmp.location.z << std::endl;
		}
	}
	return 0;


}

