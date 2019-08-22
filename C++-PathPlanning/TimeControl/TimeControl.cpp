#include <algorithm>
#include <map>
#include "TimeControl.h"
#include <BasicLibraryCode/Mission/include/SEMissionRegionTarget.hpp>
#include <BasicLibraryCode/Basic/include/FPoint.hpp> 

namespace CCSP_PlanningAlgorithm
{
	using     CCSP_BasicLibrary::SEMissionRegionTarget;
	using     CCSP_BasicLibrary::FPoint2;
	using     CCSP_BasicLibrary::FPoint3;
	using  std::map;
	using  std::sort;
	using std::find_if;
	////更新区域开始搜索时间
	bool  RegionTargetBase::SearchBeginUpdate()
	{
		vector<double> newsearchtime ;
		for(auto t :  RegionUAV)
		{
			for(auto f : t.ArriveRegionIformation)
			{
				if (f.UAVRegionIndex == this->RegionIndex)
				{
					newsearchtime.push_back(f.ArriveRegiontime);
				}
			}
		}
		auto a= std::max_element(newsearchtime.begin(), newsearchtime.end());
		this->SearchBegin = *a;
		return true;
	}

	vector<RegionTargetBase>  TimeControlCompute::FindRegionInformation( MOutput &mResult, const PScenario &mPScenario)
	{
		vector<RegionTargetBase> mRegionTarget ;//存储区域结果类，包含各架无人机及他们到达区域时间
		vector<RegionUAV> mRegionUAVs ;//存储无人机信息类，包含到达每个区域的时间链

			////区域循环找路过此区域的无人机
		for (auto m : mPScenario.m_flight_mission.v_target_ptr)
		{
			if (m.get()->target_type == "SEMissionRegionTarget" )
			{
				//SEMissionRegionTarget tmpRegionTarget = m as SEMissionRegionTarget;
				auto tmpRegionTarget=static_cast<SEMissionRegionTarget*>(m.get());
				RegionTargetBase tmpRegionTargetbase;////存储一个区域的信息
				tmpRegionTargetbase.RegionIndex = tmpRegionTarget->index;  //区域索引
				tmpRegionTargetbase.RegionVertexe = tmpRegionTarget->v_vertexs;//区域顶点

				/////得到无人机信息//////
				vector<double> Timelist ;//用来存储当前区域内各无人机到达的时间；
				///对每一架无人机的路径进行循环搜索
				for(auto n : mResult.path_map)
				{
					RegionUAV mRegionUAV ;
				//	mRegionUAV.ArriveRegionIformation ;
				//	mRegionUAV.RegionUAVPath;

					mRegionUAV.RegionUAVIndex = n.first->UAV_Index;
					double accumulatedistance = 0;
#pragma region 对一条路径上的每一个点进行循环, 找到此路径中在当前区域中的第一个点
						for (int w = 0; w < n.second.size();w++)
						{
							TimeInformation mTimeInformation ;
							FPoint3 mlocation = n.second[w].location;
							////距离积累
							if (w > 0)
							{
								FPoint2 Currentmlocation =  FPoint2(mlocation.x, mlocation.y);
								FPoint2 Lastmlocation = FPoint2(n.second[w - 1].location.x, n.second[w - 1].location.y);
								accumulatedistance = accumulatedistance + FPoint2:: DistanceBetweenTwoPlanePoints(Currentmlocation, Lastmlocation);
							}
							//if (BasicComputation::InPolygon(mlocation, tmpRegionTargetbase.RegionVertexe.ToArray()))
							if(tmpRegionTarget->IsPointInside(mlocation))
							{
								mTimeInformation.UAVRegionIndex = tmpRegionTargetbase.RegionIndex;
								mTimeInformation.Firstpoint = n.second[w]; 
								mTimeInformation.Lastpoint = n.second[w - 1]; 

								////从第一个区域点往回找边界点
								FPoint2 O = FPoint2(mTimeInformation.Lastpoint.location.x, mTimeInformation.Lastpoint.location.y);
								FPoint2 A = FPoint2(mTimeInformation.Firstpoint.location.x, mTimeInformation.Firstpoint.location.y);
								mTimeInformation.SinTheta = (A.y - O.y) / FPoint2::DistanceBetweenTwoPlanePoints(O, A);
								mTimeInformation.CosTheta = (A.x - O.x) / FPoint2::DistanceBetweenTwoPlanePoints(O, A);
		
								while (tmpRegionTarget->IsPointInside(mTimeInformation.Firstpoint.location))
								{
									mTimeInformation.Firstpoint.location.x -= 1 * mTimeInformation.CosTheta;
									mTimeInformation.Firstpoint.location.y -= 1 * mTimeInformation.SinTheta;
									accumulatedistance = accumulatedistance - 1;
								}

								mTimeInformation.InsertLocation = w; 
								mTimeInformation.Firstpoint.velocity = n.first->m_start_state.d_flight_velocity;//给第一个区域点的速度信息赋值
								
								mRegionUAV.RegionUAVPath = n.first;//这里存为map中的索引类型

								double time = accumulatedistance / n.first->m_start_state.d_flight_velocity;//    mTimeInformation.Firstpoint.State.FlightVelocity;
								Timelist.push_back(time);
								mTimeInformation.ArriveRegiontime = time;

#pragma region 判断当前无人机是否已经在无人机链表中，如果在，加入当前点；如果不在，将无人机加入链表
									auto it = find_if(mRegionUAVs.begin(), mRegionUAVs.end(), [mRegionUAV](const RegionUAV &uav) {return uav.RegionUAVIndex == mRegionUAV.RegionUAVIndex; });
								if (it!=mRegionUAVs.end())
								{
									it->ArriveRegionIformation.push_back(mTimeInformation);
									sort(it->ArriveRegionIformation.begin(),
										it->ArriveRegionIformation.end(),
										[](const TimeInformation &a, const TimeInformation &b) {return a.ArriveRegiontime < b.ArriveRegiontime; });
									tmpRegionTargetbase.RegionUAV.push_back(*it);
									break;
								}
								else
								{
									mRegionUAV.ArriveRegionIformation.push_back(mTimeInformation);
									mRegionUAVs.push_back(mRegionUAV);
									tmpRegionTargetbase.RegionUAV.push_back(mRegionUAV);
								}
#pragma endregion
									break;///只要找到一个航路点落入了这个区域，就说明此无人机会搜索此区域
							}
						}
#pragma endregion
				}///j结束无人机循环

				if (tmpRegionTargetbase.RegionUAV.size() > 1)
				{
					///选取到达这个区域最晚的那个时间作为这个区域开始搜索的时间
					tmpRegionTargetbase.SearchBegin = *(std::max_element(Timelist.begin(),Timelist.end()));
					mRegionTarget.push_back(tmpRegionTargetbase);
				}
			}
		}
		///对区域按照原始时间做个排序
		sort(mRegionTarget.begin(), mRegionTarget.end(), [](const RegionTargetBase &a, const RegionTargetBase &b) {return  a.SearchBegin < b.SearchBegin; });
		return mRegionTarget;
	}

	bool  TimeControlCompute:: Adjustpath( MOutput &mResult, const PScenario &mPScenario, double step)
	{
		 vector<RegionTargetBase> mRegionTargetBase = FindRegionInformation(mResult, mPScenario);

		 //vector<MPath> RegionUAVPath;
		///对每个区域进行循环一遍，更改航迹
		for (int m = 0; m < mRegionTargetBase.size(); m++)
		{
			for(auto n : mRegionTargetBase[m].RegionUAV)
			{
				for(auto j : n.ArriveRegionIformation)
				{
					if (j.UAVRegionIndex == mRegionTargetBase[m].RegionIndex && j.ArriveRegiontime < mRegionTargetBase[m].SearchBegin)
					{
						double timediffer = mRegionTargetBase[m].SearchBegin - j.ArriveRegiontime;///此无人机应该等待的时间
						double CircleDistance = timediffer * j.Firstpoint.velocity; ////此无人机在此时间内应该飞行的距离
						//////根据盘旋距离计算出插入点位置////////////
#pragma region 找到插入的航路点序列
						vector <WayPoint>CirclePoint;
						WayPoint *mwaypoint1 ;
						WayPoint *mwaypoint2 ;
						mwaypoint1 =&j.Firstpoint;
						mwaypoint2 =&j.Firstpoint;
						// int step = 2;
						 ////找到进入区域的点了
						CirclePoint.push_back(*mwaypoint1);
						mwaypoint2->location.x = mwaypoint1->location.x - step * j.CosTheta;
						mwaypoint2->location.y = mwaypoint1->location.y - step * j.SinTheta;

						double GetCircleDistance = 0;
						while (GetCircleDistance < CircleDistance)
						{
							CirclePoint.push_back(*mwaypoint2);
							CirclePoint.push_back(*mwaypoint1);
							GetCircleDistance = GetCircleDistance + 2 * step;
						}
#pragma endregion
#pragma region 将补充的航路点插入相应位置											 
						auto it = mResult.path_map.find(n.RegionUAVPath);//找到这架无人机对应的航路
						it->second.insert(it->second.begin()+ j.InsertLocation, CirclePoint.begin(), CirclePoint.end());////这架无人机的航路改变了
#pragma endregion
#pragma region 以下是对此区域以后区域中的航路点的时间和插入位置进行更新
						for (int w = m + 1; w < mRegionTargetBase.size(); w++)
						{
							auto it = find_if(mRegionTargetBase[w].RegionUAV.begin(), 
								mRegionTargetBase[w].RegionUAV.end(), 
								[n](const RegionUAV &uav) {return uav.RegionUAVIndex == n.RegionUAVIndex; });
							if(it!= mRegionTargetBase[w].RegionUAV.end())
							{
							  for(auto y : it->ArriveRegionIformation)
							  {
								if(y.ArriveRegiontime>j.ArriveRegiontime)
								{
									y.ArriveRegiontime = y.ArriveRegiontime + timediffer;
									y.InsertLocation += CirclePoint.size();
								}
							  }
							  mRegionTargetBase[w].SearchBeginUpdate();

							}
						}
#pragma endregion
					}
				}
			}

		}
		return true;
	}
}