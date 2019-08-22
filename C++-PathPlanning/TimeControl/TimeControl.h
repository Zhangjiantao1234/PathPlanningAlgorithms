#pragma once
#include <vector>
#include <map>
#include"../API/include/MOutput.h"
#include"../API/include/WayPoint.h"


namespace CCSP_PlanningAlgorithm 
{
	using std::vector;
	using std::map;
	/**
	 * \brief 无人机到达该区域时的航路点信息
	 */
	class TimeInformation
	{
	public:
		/// 该点所属区域的编号
		int UAVRegionIndex;
			/// 到达该区域第一个点的状态
		WayPoint Firstpoint;
			/// 到达区域前一个点
		WayPoint Lastpoint;
			/// 插入位置
		 int InsertLocation;
			/// 到达该点的时间
		 double ArriveRegiontime;
			/// 正玄值，计算坐标用
		 double SinTheta;
			/// 余弦值，计算坐标用
		 double CosTheta;
	};

	/**
     * \brief 访问区域的无人机的信息类
     */
	class RegionUAV
	{
	public:
	     /// 无人机编号
		 int RegionUAVIndex;
		/// 访问区域的无人机航路
		 SEUAV* RegionUAVPath;
		/// 到达该区域时的相关信息
		 vector<TimeInformation> ArriveRegionIformation;
	};

	/**
     * \brief 有多架无人机协同搜索的区域目标的基本信息
     */
	class RegionTargetBase
	{
	public:
		/// 区域编号
		 int RegionIndex;
	  /// 访问此区域协同搜索的无人机集合
		 vector<RegionUAV> RegionUAV;
			/// 区域顶点集合
		 vector<FPoint3> RegionVertexe;
			/// 在没调整路线前开始搜索的时间，会按照这个对区域进行一个排序
		 double SearchBegin;

	public:
			/// 更新到达时间
		bool SearchBeginUpdate();

	};

	/**
	 * \brief 基本计算类，实现航路调整，达到时间戳的控制功能
	 */
	class TimeControlCompute
	{
	public:
     /**
      * \brief 获取有多架无人机搜索的区域的相关信息
	  * \param mResult 航路规划最终结果，来源于MOutPut类
	  * \param mPScenario 场景相关信息，来源于Minput的场景实例
	  * \return 有多架无人机搜索的区域集合
      */
		static vector<RegionTargetBase> FindRegionInformation(MOutput &mResult, const PScenario &mPScenario);
		/**
         * \brief 对协同搜索区域的无人机进行航路调整得到调整后的航迹，达到时间控制的功能
         * \param mResult 航路规划最终结果，来源于MOutPut类
         * \param mPScenario 场景相关信息，来源于Minput的场景实例
		 * \param step 无人机盘旋的两点间的距离
         * \return 如果航路调整成功，返回true
         */
		static bool Adjustpath(MOutput &mResult, const PScenario &mPScenario, double step);
	};

}