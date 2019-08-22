#pragma once

#include <memory>
#include "BasicLibraryCode/Basic/include/FPoint.hpp"

using CCSP_BasicLibrary::FPoint3;

namespace CCSP_PlanningAlgorithm
{

	class RRTNode;

	using PRRTNode = std::shared_ptr<RRTNode> ;

	
	/**
	 * \brief RRT算法基础数据结构，定义RRT 的节点
	 */
	class RRTNode
	{
	public:
		FPoint3 location;		///<NODE坐标
		PRRTNode m_pParentNode;	///<RRT父备坐标NODE
		bool mIsInRRTPath;		///<是否是路径上的节点

	//==========================function ====================================//
	public:
		/**
		 * \brief 赋值操作
		 * \param location		坐标
		 * \param parentNode	父节点指针
		 */
		void Set(const FPoint3 &location, PRRTNode parentNode);

		
		/**
		 * \brief 赋值操作
		 * \param posX			x坐标
		 * \param posY			y坐标
		 * \param parentNode 	父节点指针
		 * \param posZ			z坐标
		 */
		void Set(double posX, double posY, PRRTNode parentNode, double posZ = 10);

		
		/**
		 * \brief 距离函数(调用了FPoint3::DistanceBetweenTwoPlanePoints)
		 * \param lhs NODE指针1
		 * \param rhs NODE指针2
		 * \return 两点距离
		 */
		static double DistanceBetweenTwoLocation2D(PRRTNode lhs, PRRTNode rhs);
	};
}
