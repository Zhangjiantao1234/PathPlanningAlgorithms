


#include "../include/RRTNode.h"
#include <BasicLibraryCode/Basic/include/FPoint.hpp>

namespace CCSP_PlanningAlgorithm
{
	void RRTNode::Set(const FPoint3 &position, PRRTNode parentNode)
	{
		location.Set(position.x, position.y, position.z);
		m_pParentNode = parentNode;
	}

	void RRTNode::Set(double posX, double posY, PRRTNode parentNode, double posZ /*= 10*/)
	{
		location.Set(posX, posY, posZ);
		m_pParentNode = parentNode;
	}

	double RRTNode::DistanceBetweenTwoLocation2D(PRRTNode lhs, PRRTNode rhs)
	{
		return FPoint3::DistanceBetweenTwoPlanePoints(lhs->location, rhs->location);
	}
}



