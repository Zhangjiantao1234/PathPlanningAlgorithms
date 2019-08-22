#pragma once

#include <algorithm>
#include "../../API/include/MInput.h"

namespace CCSP_PlanningAlgorithm
{

	/**
	 * \brief RRT算法参数类
	 */
	class RRTParameter
	{
	public:

		/**
		 * \brief 构造函数
		 * \param scene 任务想定输入
		 */
		RRTParameter(MInput &scene);

		///节点生成策略
		enum NodeSelectionType
		{
			Classical,	///<均匀采样
			Biased		///<有偏采样
		};

		///步长选择模式
		enum PlanningStepType
		{
			Constant,	///<固定步长生成
			Random		///<随机步长生成
		};

		int i_maxNodeNumber;	///<最大节点个数
		double constStep;		///<固定仿真步长
		double randomStepMin;	///<随即步长最小值
		double randomStepMax;	///<随机步长最大值
		double d_biasedSamplingProb;	///<有偏采样概率
		NodeSelectionType n_SelectionType;		///<新节点构造方式
		PlanningStepType  n_PlanningStepType;	///<新节点延展步长

		/**
		 * \brief 自动化调参
		 * \param algo_input 任务想定输入
		 */
		inline void AutoOptimizePara(MInput& algo_input);
	};
};

