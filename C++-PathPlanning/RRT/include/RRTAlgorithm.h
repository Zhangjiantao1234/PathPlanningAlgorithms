#pragma once

#include <vector>
#include <random>

#include "RRTNode.h"
#include "RRTParameter.h"
#include"BasicLibraryCode/PScenario.hpp"
#include "../../API/include/MInput.h"
#include"BasicLibraryCode/Basic/include/FPoint.hpp"
#include "../../API/include/WayPoint.h"


namespace CCSP_PlanningAlgorithm
{

	/**
	 * \brief RRT算法类，对外的算法调用类
	 */
	class RRTAlgorithm 
	{
	public:
		MInput algo_input;							///<算法输入信息
		std::shared_ptr<RRTParameter> para_ptr;		///<参数定义（在RRTAlgorithm中开辟了相关内存，可以从外界导入）
		std::vector<WayPoint> m_planning_results;	///<规划结果输出
	private:
		//定义树
		//定义为指针的缘由是由于RRTTree要保留父被指针，
		//因为VECTOR超过一定数量会重新开辟空间并复制粘贴，
		//所以当重新划空间并将数据挪动时，其创建的指针就不再是有效的父辈节点所在地址。
		//所以这里保存的是RRTNODE的指针

		//RRTAlgorithm tree
		std::vector<PRRTNode> rrt_tree;		///<RRTAlgorithm tree

		//定义树中间节点
		PRRTNode rand_node_ptr;		///<随机的采样点（算法流程用）
		PRRTNode near_node_ptr;		///<邻居节点（算法流程用）
		PRRTNode steer_node_ptr;	///<延申节点（算法流程用）
		PRRTNode start_node_ptr;	///<无人机初始节点
		PRRTNode target_node_ptr;	///<无人机目标节点

		//算法临时变量
		int nCount;					///<计数器（Timeout用）
		bool is_reach_target;		///<是否到达目标 - 初始未到达
		
		//随机数种子初始化
		//考虑到随机的X,Y坐标应该是保持独立的，所以这里使用两个种子分别对X,Y取随机数。
		std::default_random_engine generatorX;		///<X轴 变量 随机数生成器（保证是均匀分布）
		std::default_random_engine generatorY;		///<Y轴 变量 随机数生成器（保证是均匀分布）
		std::default_random_engine defaultGenerator;///<其他参数 随机数生成器

	public:

		/**
		 * \brief 构造函数
		 * \param mInput 算法输入信息（数据源来自外部） 
		 */
		RRTAlgorithm(MInput& mInput);


		/**
		 * \brief 为单架无人机单任务执行航迹规划算法
		 *			参数包括场景，无人机信息，参数，起始点终止点信息,
		 *			并不考虑任务类型，只考虑起始，终止节点，初始方位角。
		 * \param uav_ptr 无人机指针，数据直接来源来自 MInput 类的 assignment_result，原始数据来自 PScenario 类的无人机组
		 * \return 规划轨迹结果
		 */
		std::vector<WayPoint> RRT4SingleUAVTaskStatic(SEUAV* uav_ptr);

		
		/**
		 * \brief 
		 * \param uav_ptr 
		 * \param current_state 
		 * \return 
		 */
		std::vector<WayPoint> RRT4SingleUAVTaskRealtime(SEUAV* uav_ptr, const WayPoint* current_state);
	
		
		/**
		 * \brief 所有威胁的探测状态设定
		 * \param is_detected 状态值
		 */
		void SetAllThreatsDetectedStatus(bool is_detected);
	private:

		/**
		 * \brief RRT 算法核心
		 * \param helper 
		 */
		void RRT_kernel(PlanningHelper& helper);

		/**
		 * \brief 获取完整（所有STAGE）的轨迹结果
		 * 
		 *			NOTE:
		 *			当 current_stage_index == init_stage_index时，保存完整的path
		 *			当 current_stage_index != init_stage_index时，保存[1,last]的轨迹（没有初始的坐标，因为重复）
		 * \param current_stage_index	当前stage_id 
		 * \param init_stage_index		初始规划所属的stage_id（实时规划时可能为非0值）
		 */
		void GeneratePlanningResult(int current_stage_index, int init_stage_index = 0);

		///获取一个 stage 下的航迹结果
		std::vector<PRRTNode> GeneratePlanningStageResult();
	


		/**
		 * \brief RRT树构造初始化
		 * \param uav_ptr				无人机指针
		 * \param stage_index_current	当前任务阶段的index
		 * \param current_state_ptr		重规划时无人机状态(默认参数为 nullptr)
		 */
		void RRTInitilize(SEUAV* uav_ptr, int stage_index_current, const WayPoint* current_state_ptr = nullptr);

		///初始状态检查
		bool InitStatusCheck();

		///随机生成节点-标准RRT
		PRRTNode RandomConfiguration();

		///按照一定概率随机生成节点或者采用目标点(改进方案)
		PRRTNode BiasConfiguration(double bias_threshold);

		///寻找最近节点
		PRRTNode NearestNeighbor();

		///得到新的延伸节点
		PRRTNode NewConfiguration();

		///得到随机的新的延伸节点
		PRRTNode NewConfigurationWithRandomStep();

		///延伸RRT树
		bool ExpandTree();

		///直接连接到目标
		bool DirectlyToTarget();


	};
}



