#pragma once

#include <map>
#include "BasicLibraryCode/PScenario.hpp"
#include "PlanningHelper.h"

using CCSP_BasicLibrary::PScenario;
using CCSP_BasicLibrary::SEUAV;
using CCSP_BasicLibrary::MissionTargetPtr;

/**
 * \brief 航迹规划算法类
 * 
 * 航迹规划算法类，当前包含以下内容
 *  - MInput 算法输入信息
 *  - PlanningHelper 算法基础公共函数类
 *  - Algorithms 算法本体
 *		- RRTAlgorithm	RRTAlgorithm 算法
 *		- RRTNode		RRT算法数据结构基础
 *		- RRTParameter	RRT算法参数
 *	本版本仅提供RRT算法。但当前框架可以快速添加其他算法。
 */
namespace CCSP_PlanningAlgorithm
{
	using AssignmentMap = std::map<SEUAV*, std::vector<MissionTargetPtr>>;

	/**
	 * \brief 航迹规划算法输入信息类
	 * 
	 * 注意：
	 * assignment_result 其实可以使用 shared_ptr 来定义
	 * 但由于scenario 不是new 出来的
	 * 可能 shared_ptr 会删除两次相同的内存
	 * 所以这里就不用了（考虑 d11 兼容性）
	 *
	 * 如果这里计划使用 shared_ptr 请这样修改：
	 *	- 1. scenario 保存 make_shared 获取的ptr
	 *	- 2. AssignmentResult 的变量定义为 shared_ptr (可能有的目标会被访问多次)
	 *	- 3. PlanningHelper 中的指针也变成 shared_ptr (防止野指针)
	 */
	class MInput
	{
	//====================parameter===========================//
	public:
		PScenario scenario;					//!<场景实例
		AssignmentMap assignment_result;	//!<无人机任务结果,所有的指针数据源来自scenario相关的引用
	
		PlanningHelper helper;				//!<规划帮助类
		
		//说明：
		//assignment_result
		//其实可以使用 shared_ptr 来定义
		//但由于scenario 不是new 出来的
		//可能shared_ptr会删除两次相同的内存
		//所以这里就不用了（考虑c11兼容性）

		//如果这里计划使用 shared_ptr 请这样修改：
		//1. scenario 保存 make_shared 获取的ptr
		//2. AssignmentResult 的变量定义为shared_ptr (可能有的目标会被访问多次)
		//3. PlanningHelper 中的指针也变成shared_ptr (防止野指针)

	//====================function===========================//
	public:
		///空构造函数，无数据
		MInput();

		/**
		 * \brief 带数据的构造函数
		 * \param json_str JSON 原始任务想定数据（来自外部 - CCSP）
		 * \param assignment_map 任务分配结果（来自外部）
		 * \param sample_distance 最小采样间隔（来自外部，该参数会影响 PlanningHelper 相关函数的性能）
		 * \param is_path  true:json_str为JSON文件的地址，调用相关函数解析文件得到JSON字符串再解析为 SCENARIO 类，false:json_str为JSON字符串并直接解析为SCENARIO类				
		 */
		MInput(std::string json_str,const AssignmentMap & assignment_map , double sample_distance ,bool is_path = true );

	};
}

