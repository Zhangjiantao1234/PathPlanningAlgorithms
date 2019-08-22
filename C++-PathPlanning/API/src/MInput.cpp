#include "../include/MInput.h"
#include <iostream>

namespace CCSP_PlanningAlgorithm
{
	MInput::MInput()
	{
	}

	MInput::MInput(std::string json_str, const AssignmentMap& assignment_map, double sample_distance, bool is_path)
	{
		scenario = PScenario();
		try
		{
			if (is_path)
			{
				auto tmp_tuple = scenario.ReadJsonFile(json_str);
				if (!std::get<0>(tmp_tuple))
					throw;
				if (!scenario.GetInfoFromJson(std::get<1>(tmp_tuple)))
					throw;
			}
			else
			{
				if (!scenario.GetInfoFromJson(json_str))
					throw;
			}
		}
		catch (...)
		{
			std::cerr << "Illegal Scenario Infomaton." << std::endl;;
		}

		//STL copy
		assignment_result = assignment_map;

		//class create;
		helper = PlanningHelper(&scenario, sample_distance);
	}
}
