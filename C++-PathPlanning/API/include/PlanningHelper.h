#pragma once
#include"./BasicLibraryCode/PScenario.hpp"

using CCSP_BasicLibrary::PScenario;
using CCSP_BasicLibrary::FPoint3;

//算法Helper 类
//提供各种杂七杂八的功能
namespace CCSP_PlanningAlgorithm
{

	/**
	 * \brief 算法帮助类，提供各种航迹规划算法公共函数
	 */
	class PlanningHelper
	{

	protected:
		
		/// 任务想定输入类（weak_ptr 的使用策略，此变量不需要析构函数处理内存。)
		PScenario* scenario_ptr_;

		///采样间隔
		double sample_distance_;

	public:

		///构造函数，参数为空
		PlanningHelper();

		
		/**
		 * \brief 构造函数
		 * \param scenario_ptr 任务想定输入，数据来源来自MInput赋值。
		 * \param sample_distance 采样间隔
		 */
		PlanningHelper(PScenario *  scenario_ptr, double sample_distance);

		
		/**
		 * \brief 检查当前位置是否可以直连目标点
		 * \param wpt 待检测坐标点
		 * \param wptGoal 目标坐标点
		 * \return 是否可以直连目标点
		 */
		bool IsGoalBlock(FPoint3 &wpt, FPoint3 & wptGoal) const;

		
		/**
		 * \brief 检查测试位置是否安全
		 * \param mPoint 待测目标点
		 * \return 是否安全
		 * 
		 * 该函数调用了 BasicLibrary 中 SEThread 类中 IsSafePointCurrent。
		 * 即算法时以当前已知的Threads来确定该点是否是安全的，
		 * 并不是以上帝视角（IsSafePointGlobal）来确定是否是安全的。
		 * 具体逻辑行为见函数 IsSafePointCurrent
		 */
		bool IsSafePoint(const FPoint3 &mPoint) const ;

		
		/**
		 * \brief 检查测试线段是否安全
		 * \param lhs 线段端点1
		 * \param rhs 线段端点2
		 * \return 是否是安全的线段
		 * 
		 * 该函数调用 IsSafePoint ，以一定间隔将线段采样为点集合
		 * 具体逻辑见 IsSafePoint 函数
		 */
		bool IsSafeLine(const FPoint3 &lhs, const FPoint3 &rhs) const;

	};

}
