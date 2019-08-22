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
	////��������ʼ����ʱ��
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
		vector<RegionTargetBase> mRegionTarget ;//�洢�������࣬�����������˻������ǵ�������ʱ��
		vector<RegionUAV> mRegionUAVs ;//�洢���˻���Ϣ�࣬��������ÿ�������ʱ����

			////����ѭ����·������������˻�
		for (auto m : mPScenario.m_flight_mission.v_target_ptr)
		{
			if (m.get()->target_type == "SEMissionRegionTarget" )
			{
				//SEMissionRegionTarget tmpRegionTarget = m as SEMissionRegionTarget;
				auto tmpRegionTarget=static_cast<SEMissionRegionTarget*>(m.get());
				RegionTargetBase tmpRegionTargetbase;////�洢һ���������Ϣ
				tmpRegionTargetbase.RegionIndex = tmpRegionTarget->index;  //��������
				tmpRegionTargetbase.RegionVertexe = tmpRegionTarget->v_vertexs;//���򶥵�

				/////�õ����˻���Ϣ//////
				vector<double> Timelist ;//�����洢��ǰ�����ڸ����˻������ʱ�䣻
				///��ÿһ�����˻���·������ѭ������
				for(auto n : mResult.path_map)
				{
					RegionUAV mRegionUAV ;
				//	mRegionUAV.ArriveRegionIformation ;
				//	mRegionUAV.RegionUAVPath;

					mRegionUAV.RegionUAVIndex = n.first->UAV_Index;
					double accumulatedistance = 0;
#pragma region ��һ��·���ϵ�ÿһ�������ѭ��, �ҵ���·�����ڵ�ǰ�����еĵ�һ����
						for (int w = 0; w < n.second.size();w++)
						{
							TimeInformation mTimeInformation ;
							FPoint3 mlocation = n.second[w].location;
							////�������
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

								////�ӵ�һ������������ұ߽��
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
								mTimeInformation.Firstpoint.velocity = n.first->m_start_state.d_flight_velocity;//����һ���������ٶ���Ϣ��ֵ
								
								mRegionUAV.RegionUAVPath = n.first;//�����Ϊmap�е���������

								double time = accumulatedistance / n.first->m_start_state.d_flight_velocity;//    mTimeInformation.Firstpoint.State.FlightVelocity;
								Timelist.push_back(time);
								mTimeInformation.ArriveRegiontime = time;

#pragma region �жϵ�ǰ���˻��Ƿ��Ѿ������˻������У�����ڣ����뵱ǰ�㣻������ڣ������˻���������
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
									break;///ֻҪ�ҵ�һ����·��������������򣬾�˵�������˻�������������
							}
						}
#pragma endregion
				}///j�������˻�ѭ��

				if (tmpRegionTargetbase.RegionUAV.size() > 1)
				{
					///ѡȡ�����������������Ǹ�ʱ����Ϊ�������ʼ������ʱ��
					tmpRegionTargetbase.SearchBegin = *(std::max_element(Timelist.begin(),Timelist.end()));
					mRegionTarget.push_back(tmpRegionTargetbase);
				}
			}
		}
		///��������ԭʼʱ����������
		sort(mRegionTarget.begin(), mRegionTarget.end(), [](const RegionTargetBase &a, const RegionTargetBase &b) {return  a.SearchBegin < b.SearchBegin; });
		return mRegionTarget;
	}

	bool  TimeControlCompute:: Adjustpath( MOutput &mResult, const PScenario &mPScenario, double step)
	{
		 vector<RegionTargetBase> mRegionTargetBase = FindRegionInformation(mResult, mPScenario);

		 //vector<MPath> RegionUAVPath;
		///��ÿ���������ѭ��һ�飬���ĺ���
		for (int m = 0; m < mRegionTargetBase.size(); m++)
		{
			for(auto n : mRegionTargetBase[m].RegionUAV)
			{
				for(auto j : n.ArriveRegionIformation)
				{
					if (j.UAVRegionIndex == mRegionTargetBase[m].RegionIndex && j.ArriveRegiontime < mRegionTargetBase[m].SearchBegin)
					{
						double timediffer = mRegionTargetBase[m].SearchBegin - j.ArriveRegiontime;///�����˻�Ӧ�õȴ���ʱ��
						double CircleDistance = timediffer * j.Firstpoint.velocity; ////�����˻��ڴ�ʱ����Ӧ�÷��еľ���
						//////���������������������λ��////////////
#pragma region �ҵ�����ĺ�·������
						vector <WayPoint>CirclePoint;
						WayPoint *mwaypoint1 ;
						WayPoint *mwaypoint2 ;
						mwaypoint1 =&j.Firstpoint;
						mwaypoint2 =&j.Firstpoint;
						// int step = 2;
						 ////�ҵ���������ĵ���
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
#pragma region ������ĺ�·�������Ӧλ��											 
						auto it = mResult.path_map.find(n.RegionUAVPath);//�ҵ�������˻���Ӧ�ĺ�·
						it->second.insert(it->second.begin()+ j.InsertLocation, CirclePoint.begin(), CirclePoint.end());////������˻��ĺ�·�ı���
#pragma endregion
#pragma region �����ǶԴ������Ժ������еĺ�·���ʱ��Ͳ���λ�ý��и���
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