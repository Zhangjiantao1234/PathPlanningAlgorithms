
#include "../include/RRTParameter.h"



CCSP_PlanningAlgorithm::RRTParameter::RRTParameter(MInput& algo_input)
{
	n_PlanningStepType = CCSP_PlanningAlgorithm::RRTParameter::Constant;
	n_SelectionType = CCSP_PlanningAlgorithm::RRTParameter::Classical;
	d_biasedSamplingProb = 0.3;
	i_maxNodeNumber = 50000;
	this->AutoOptimizePara(algo_input);
}

inline void CCSP_PlanningAlgorithm::RRTParameter::AutoOptimizePara(MInput& algo_input)
{
	auto& scene = algo_input.scenario.m_flight_scene.m_coordinate;

	const double tmpValue = std::max(scene.d_X_MAX - scene.d_X_MIN, scene.d_Y_MAX - scene.d_Y_MIN);
	this->constStep = tmpValue / 20;
	this->randomStepMin = tmpValue / 100;
	if (randomStepMin < 1)
		randomStepMin = 1;
	this->randomStepMax = tmpValue / 10;
	this->d_biasedSamplingProb = 0.3;
}
