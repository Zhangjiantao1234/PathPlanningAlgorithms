


#include <random>
#include <memory>
#include <ctime>
#include <cassert>
#include <ppltasks.h>
#include "../include/RRTAlgorithm.h"
#include <iostream>


class RRTNode;

namespace CCSP_PlanningAlgorithm
{
	void RRTAlgorithm::SetAllThreatsDetectedStatus(bool is_detected)
	{
		algo_input.scenario.m_flight_scene.m_threats.InitDetectThreat(is_detected);
	}

	RRTAlgorithm::RRTAlgorithm(MInput& mInput)
	{
		//copy;
		algo_input = mInput;

		//定义航路
		m_planning_results.clear();

		//定义树中间节点
		rand_node_ptr = nullptr;
		near_node_ptr = nullptr;
		steer_node_ptr = nullptr;
		//起始节点
		start_node_ptr = nullptr;
		//目标节点
		target_node_ptr = nullptr;
		//初始化计数器
		nCount = 1;
		//是否到达目标 - 初始未到达
		is_reach_target = false;
		//参数为空
		para_ptr =  std::make_shared<RRTParameter>(mInput);
	}


	void RRTAlgorithm::RRT_kernel(PlanningHelper& helper)
	{
		//到达目标时退出
		while (!is_reach_target)
		{
			rand_node_ptr = nullptr;
			near_node_ptr = nullptr;
			steer_node_ptr = nullptr;

			//============================随机生成节点==============================//
			if (RRTParameter::NodeSelectionType::Classical == para_ptr->n_SelectionType)
			{
				//随机生成节点-标准RRT
				rand_node_ptr = RandomConfiguration();
			}
			else
			{
				//方案2:按照一定概率随机生成节点或者采用目标点 (改进方案)
				rand_node_ptr = BiasConfiguration(para_ptr->d_biasedSamplingProb);
			}
			//============================随机生成节点 end==============================//

			//----------------------------搜索距离最近的节点-----------------------------//
			near_node_ptr = NearestNeighbor();
			//-----------------------------------end-------------------------------------//

			//--------------------------------生成新节点---------------------------------//
			if ((RRTParameter::PlanningStepType::Constant == para_ptr->n_PlanningStepType))
			{
				steer_node_ptr = NewConfiguration();
			}
			else if (RRTParameter::PlanningStepType::Random == para_ptr->n_PlanningStepType)
			{
				steer_node_ptr = NewConfigurationWithRandomStep();
			}
			//-----------------------------------end-------------------------------------//

			//计数器累加
			nCount++;

			//-----------------------如果到达允许的搜索上限,则退出-----------------------//
			if (nCount >= para_ptr->i_maxNodeNumber)
			{
				std::cerr << "Algorithm Timeout!" << std::endl;
				break;
			}
			//-----------------------------------end-------------------------------------//


			//----------------威胁规避(禁止在威胁区域内选择/生成新点/连线)---------------//
			if (!helper.IsSafePoint(steer_node_ptr->location))
			{
				continue;
			}

			if (!helper.IsSafeLine(steer_node_ptr->location, near_node_ptr->location))
			{
				continue;
			}
			//-----------------------------------end-------------------------------------//


			//------------------------------扩展树 - 核心--------------------------------//
			is_reach_target = ExpandTree();
			//-----------------------------------end-------------------------------------//

			//------------判断新结点到目标连线是否安全, 若安全则直接连接目标点-----------//

			is_reach_target = is_reach_target || DirectlyToTarget();

			//-----------------------------------end-------------------------------------//

		}
	}

	/**
	 * \brief 静态轨迹规划
	 *			算法流程：
	 *			各种初始化检查；
	 			foreach(该无人机的各个任务阶段)
				{
					加入起始点
					加入终止点
					创建最近节点
					创建RRT树
					while (最近节点与GOAL距离过远)
					{
						if (扩展最新节点与终点可直接连接)
							结束
						以一定几率选择GOAL作为延伸方向，否则随机生成一点作为延伸方向；
						寻找最近节点；
						做该最近节点的延伸；
						if (该节点是安全的)
							添加至RRT树
					}
				}
	 * \param uav_ptr 无人机指针
	 * \return 轨迹
	 */
	std::vector<WayPoint> RRTAlgorithm::RRT4SingleUAVTaskStatic(SEUAV* uav_ptr)
	{
		//初始化检查
		auto &helper = algo_input.helper;
		m_planning_results.clear();

		//获取阶段vector
		for(int stage_index=0 ;stage_index< algo_input.assignment_result[uav_ptr].size();stage_index++)
		{

			//============================加入起始点，终止点=====================//
			RRTInitilize(uav_ptr,stage_index);
			//============================加入起始点，终止点 end=================//

			assert(InitStatusCheck());

			//============================RRT树扩展==============================//
			RRT_kernel(helper);
			//============================RRT树扩展 end==========================//

			//============================填充数据===============================//
			GeneratePlanningResult(stage_index);
			//============================填充数据.end===========================//

			
		}//stage_index end;
		return m_planning_results;
	}

	std::vector<WayPoint> RRTAlgorithm::RRT4SingleUAVTaskRealtime(SEUAV* uav_ptr, const WayPoint* current_state_ptr)
	{
		// current_state_ptr->stage_index == 0 代表第一次规划

		//已经飞过的轨迹就不要了，现在规划没飞过的轨迹
		//显示的问题由甲方处理，这里仅保证功能正确
		auto &helper = algo_input.helper;
		m_planning_results.clear();

		
		//获取阶段vector
		int stage_index =0;
		if (current_state_ptr != nullptr)
			stage_index = current_state_ptr->stage_index;
		for(; stage_index< algo_input.assignment_result[uav_ptr].size(); stage_index++)
		{

			//============================加入起始点，终止点=====================//
			RRTInitilize(uav_ptr, stage_index,current_state_ptr);
			//============================加入起始点，终止点 end=================//
			
			assert(InitStatusCheck());

			//============================RRT树扩展==============================//
			RRT_kernel(helper);
			//============================RRT树扩展 end==========================//

			//============================填充数据===============================//
			if(current_state_ptr == nullptr)
				GeneratePlanningResult(stage_index,0);
			else
				GeneratePlanningResult(stage_index,current_state_ptr->stage_index);
			//============================填充数据.end===========================//

		}//stage_index end;
		return m_planning_results;
	}

	void RRTAlgorithm::GeneratePlanningResult(int current_stage_index , int init_stage_index /* =0 */)
	{
		auto stage_path_vect = GeneratePlanningStageResult();
		//mPlanningResults.resize(mPlanningResults.size() + stage_path_vect.size());
		if (current_stage_index == init_stage_index)
		{
			auto count = 0;
			for (auto &item : stage_path_vect)
				m_planning_results.push_back(WayPoint{ count++,current_stage_index,item->location,0,0 });
		}
		else
		{
			assert(stage_path_vect.size() >= 2);
			//从第二个开始往后累加
			for (int i = 1; i < stage_path_vect.size(); i++)
				m_planning_results.push_back(WayPoint{ i ,current_stage_index,stage_path_vect[i]->location,0,0 });
		}
	}

	std::vector<PRRTNode> RRTAlgorithm::GeneratePlanningStageResult()
	{
		std::vector<PRRTNode> tmp_result;
		//tmp_result.resize(rrt_tree.size());

		PRRTNode tmp = rrt_tree.back(); //target_node_ptr 的坐标
		while (tmp->m_pParentNode != nullptr)
		{
			tmp_result.push_back(tmp);
			tmp = tmp->m_pParentNode;
		}
		tmp_result.push_back(tmp);
		//反转一下顺序
		std::reverse(tmp_result.begin(), tmp_result.end());
		return tmp_result;
	}

	void RRTAlgorithm::RRTInitilize(SEUAV* uav_ptr,int stage_index_current,const WayPoint* current_state_ptr)
	{
		rrt_tree.clear();
		is_reach_target = false;

		//计数器初始化
		nCount = 0;

		//数组边界检查
		assert(stage_index_current < algo_input.assignment_result.at(uav_ptr).size());

		//初始化start_node_ptr
		start_node_ptr = std::make_shared<RRTNode>();

		//将起始点压入RRTTREE中
		steer_node_ptr = std::make_shared<RRTNode>();
		
		//无人机 对应的 STAGE压入 tree

		//当WayPoint 空的时候为静态规划初始化
		if(current_state_ptr == nullptr)
		{
			if (stage_index_current == 0)
			{
				//两块内存不一样....
				start_node_ptr->Set(uav_ptr->m_start_state.m_location, nullptr);
				//steer_node_ptr->Set(uav_ptr->m_start_state.m_location, nullptr);
			}
			else
			{
				//两块内存不一样....
				start_node_ptr->Set(algo_input.assignment_result.at(uav_ptr)[stage_index_current - 1]->m_target_pos, nullptr);
				//steer_node_ptr->Set(algo_input.assignment_result.at(uav_ptr)[stage_index_current - 1]->m_target_pos, nullptr);
			}
		}
		else  //当WayPoint不空的时候为实时规划初始化
		{
			if (stage_index_current == current_state_ptr->stage_index)
			{
				//两块内存不一样....
				start_node_ptr->Set(current_state_ptr->location, nullptr);
				//steer_node_ptr->Set(current_state_ptr->location, nullptr);
			}
			else
			{
				//两块内存不一样....
				start_node_ptr->Set(algo_input.assignment_result.at(uav_ptr)[stage_index_current - 1]->m_target_pos, nullptr);
				//steer_node_ptr->Set(algo_input.assignment_result.at(uav_ptr)[stage_index_current - 1]->m_target_pos, nullptr);
			}
		}
		rrt_tree.push_back(start_node_ptr);

		//将终止点加入到target_node_ptr变量中
		target_node_ptr = std::make_shared<RRTNode>();
		target_node_ptr->Set(algo_input.assignment_result.at(uav_ptr)[stage_index_current]->m_target_pos, nullptr);

		//初始化nearNode（即为根节点）
		near_node_ptr = std::make_shared<RRTNode>(*start_node_ptr);


		//准备随机数的种子；
		//myclock::duration d = myclock::now() - beginning;
		//defaultGenerator.seed((unsigned)d.count());
		defaultGenerator.seed(static_cast<unsigned>(time(nullptr)));
		const std::uniform_int_distribution<unsigned> dis(0, 1e10);
		generatorX.seed(dis(defaultGenerator));
		generatorY.seed(dis(defaultGenerator));
	}

	bool RRTAlgorithm::InitStatusCheck()
	{
		auto & helper = algo_input.helper;

		if ( !helper.IsSafePoint(start_node_ptr->location) || !helper.IsSafePoint(target_node_ptr->location))
			return false;
		else
			return true;
	}

	PRRTNode RRTAlgorithm::RandomConfiguration()
	{
		auto &sceneInfo = algo_input.scenario.m_flight_scene.m_coordinate;

		static std::uniform_real_distribution<double> distX(0, sceneInfo.d_X_MAX - sceneInfo.d_X_MIN);
		static std::uniform_real_distribution<double> distY(0, sceneInfo.d_Y_MAX - sceneInfo.d_Y_MIN);
		const auto ranX = distX(generatorX);
		const auto ranY = distY(generatorY);
		PRRTNode result = std::make_shared<RRTNode>();
		result->Set(ranX, ranY, nullptr,start_node_ptr->location.z);
		return result;
	}


	PRRTNode RRTAlgorithm::BiasConfiguration(double bias_threshold)
	{
		static std::uniform_real_distribution<double> dist(0, 1);
		const auto ran = dist(defaultGenerator);
		if (ran < bias_threshold)
			return RandomConfiguration();
		else
			return target_node_ptr;
	}

	PRRTNode RRTAlgorithm::NearestNeighbor()
	{
		//near_node_ptr = NearestNeighbor(rrt_tree, rand_node_ptr);
		double minDis = 1e9;
		PRRTNode minNode = nullptr;
		for each (auto node in rrt_tree)
		{
			double result = RRTNode::DistanceBetweenTwoLocation2D(node, rand_node_ptr);
			if (result < minDis)
			{
				minDis = result;
				minNode = node;
			}
		}
		return minNode;
	}

	PRRTNode RRTAlgorithm::NewConfiguration()
	{
		//steer_node_ptr = NewConfiguration(near_node_ptr, rand_node_ptr);

		PRRTNode to = rand_node_ptr;
		PRRTNode from = near_node_ptr;
		FPoint3 intermediate = to->location - from->location;
		intermediate = intermediate / intermediate.norm();
		FPoint3 ret = from->location + intermediate * para_ptr->constStep;
		PRRTNode result = std::make_shared<RRTNode>();
		result->Set(ret, from);

		if (RRTNode::DistanceBetweenTwoLocation2D(result, from) > RRTNode::DistanceBetweenTwoLocation2D(to, from))
		{
			result->Set(to->location, from);
		}

		return result;

	}


	//考虑该点与目标点距离，如果距离较远，则步长应该较大
	PRRTNode RRTAlgorithm::NewConfigurationWithRandomStep()
	{

		PRRTNode to = rand_node_ptr;
		PRRTNode from = near_node_ptr;
		FPoint3 intermediate = to->location - from->location;
		intermediate = intermediate / intermediate.norm();

		//生成范围内随机数(均匀分布)
		static std::uniform_real_distribution<double> dist(para_ptr->randomStepMin, para_ptr->randomStepMax);
		FPoint3 tmp_node = from->location + intermediate * dist(defaultGenerator);

		PRRTNode result = std::make_shared<RRTNode>();
		result->Set(tmp_node, from);

		if (RRTNode::DistanceBetweenTwoLocation2D(result, from) > RRTNode::DistanceBetweenTwoLocation2D(to, from))
		{
			result->Set(to->location, from);
		}

		return result;
	}

	bool RRTAlgorithm::ExpandTree()
	{
		//is_reach_target = ExpandTree(rrt_tree, near_node_ptr, steer_node_ptr);

		steer_node_ptr->m_pParentNode = near_node_ptr;
		rrt_tree.push_back(steer_node_ptr);
		if (steer_node_ptr->location == target_node_ptr->location)
			return true;
		else
			return false;
	}

	bool RRTAlgorithm::DirectlyToTarget()
	{
		auto &helper = algo_input.helper;
		//DirectlyToTarget(steer_node_ptr, rrt_tree)
		if (helper.IsSafeLine(target_node_ptr->location, steer_node_ptr->location))
		{
			target_node_ptr->m_pParentNode = steer_node_ptr;
			rrt_tree.push_back(target_node_ptr);
			return true;
		}
		else
			return false;
	}


}