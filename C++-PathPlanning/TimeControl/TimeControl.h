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
	 * \brief ���˻����������ʱ�ĺ�·����Ϣ
	 */
	class TimeInformation
	{
	public:
		/// �õ���������ı��
		int UAVRegionIndex;
			/// ����������һ�����״̬
		WayPoint Firstpoint;
			/// ��������ǰһ����
		WayPoint Lastpoint;
			/// ����λ��
		 int InsertLocation;
			/// ����õ��ʱ��
		 double ArriveRegiontime;
			/// ����ֵ������������
		 double SinTheta;
			/// ����ֵ������������
		 double CosTheta;
	};

	/**
     * \brief ������������˻�����Ϣ��
     */
	class RegionUAV
	{
	public:
	     /// ���˻����
		 int RegionUAVIndex;
		/// ������������˻���·
		 SEUAV* RegionUAVPath;
		/// ���������ʱ�������Ϣ
		 vector<TimeInformation> ArriveRegionIformation;
	};

	/**
     * \brief �ж�����˻�Эͬ����������Ŀ��Ļ�����Ϣ
     */
	class RegionTargetBase
	{
	public:
		/// ������
		 int RegionIndex;
	  /// ���ʴ�����Эͬ���������˻�����
		 vector<RegionUAV> RegionUAV;
			/// ���򶥵㼯��
		 vector<FPoint3> RegionVertexe;
			/// ��û����·��ǰ��ʼ������ʱ�䣬�ᰴ��������������һ������
		 double SearchBegin;

	public:
			/// ���µ���ʱ��
		bool SearchBeginUpdate();

	};

	/**
	 * \brief ���������࣬ʵ�ֺ�·�������ﵽʱ����Ŀ��ƹ���
	 */
	class TimeControlCompute
	{
	public:
     /**
      * \brief ��ȡ�ж�����˻�����������������Ϣ
	  * \param mResult ��·�滮���ս������Դ��MOutPut��
	  * \param mPScenario ���������Ϣ����Դ��Minput�ĳ���ʵ��
	  * \return �ж�����˻����������򼯺�
      */
		static vector<RegionTargetBase> FindRegionInformation(MOutput &mResult, const PScenario &mPScenario);
		/**
         * \brief ��Эͬ������������˻����к�·�����õ�������ĺ������ﵽʱ����ƵĹ���
         * \param mResult ��·�滮���ս������Դ��MOutPut��
         * \param mPScenario ���������Ϣ����Դ��Minput�ĳ���ʵ��
		 * \param step ���˻������������ľ���
         * \return �����·�����ɹ�������true
         */
		static bool Adjustpath(MOutput &mResult, const PScenario &mPScenario, double step);
	};

}