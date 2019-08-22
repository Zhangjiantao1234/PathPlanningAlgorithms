using PlanningAlgorithmInterface.AlgorithmInterface;
using PlanningAlgorithmInterface.Define.Input;
using PlanningAlgorithmInterface.Define.Output;
using System;
using System.Collections.Generic;
using PathPlaningDataVisualization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RRTOrigin
{
    public class RRTOriginAlgorithm : RRTBase, ICompletePlanningAlgorithm
    {

        RrtOriginVisualization Visualization = new RrtOriginVisualization();

        public MPath BuildPathForSingleUAVForRealTime(int iTaskIndex, MWaypoint mWaypoint, MPath mPath)
        {
            return BuildPathForSingleUAV(iTaskIndex, mWaypoint, mPath);
        }

        public MPath BuildPathForSingleUAVInStatic(int iTaskIndex)
        {
            return BuildPathForSingleUAV(iTaskIndex);
        }

        public void InitParameter()
        {
            m_RRTParameter = AlgoParameter as RRTParameter;

            if (m_RRTParameter.AutoOptimizeParameter == true)
            {
                //自动化参数
            }
        }



        /// <summary>
        /// 为单架无人机规划航迹
        /// </summary>
        /// <param name="iTaskIndex">无人机/任务索引</param>
        /// <returns></returns>
        protected virtual MPath BuildPathForSingleUAV(int iTaskIndex)
        {
            //实例化航路
            MPath mPath = new MPath();
            mPath.Index = AlgoInput.UAVTask[iTaskIndex].Index;
            mPath.Waypoints = new List<MWaypoint>();
            int iWaypointIndex = 0;

            //定义树
            List<RRTNode> mRRTTree = new List<RRTNode>();
            //定义航路
            List<RRTNode> mRRTPath = new List<RRTNode>();
            //定义树节点
            RRTNode mRRTNode = null;
            //定义树中间节点
            RRTNode mRRTRandNode = null;
            RRTNode mRRTNearNode = null;
            RRTNode mRRTNewNode = null;

            //计数器
            int nCount = 1;

            //对每一个阶段规划航路
            for (int iStageIndex = 0; iStageIndex < AlgoInput.UAVTask[iTaskIndex].Stages.Count; ++iStageIndex)
            {
                //实例化
                mRRTTree = new List<RRTNode>();
                //加入开始点为第一个点
                mRRTNode = new RRTNode(0, AlgoInput.UAVTask[iTaskIndex].Stages[iStageIndex].StartState.Location,
                    AlgoInput.UAVTask[iTaskIndex].Stages[iStageIndex].StartState.Direction, null);
                mRRTTree.Add(mRRTNode);

                //初始化计数器
                nCount = 1;
                //是否到达目标 - 初始未到达
                bool isReachTarget = false;

                //循环构建新点 - 到达目标时自动退出
                while (!isReachTarget)
                {
                    //-------------------------------随机生成节点--------------------------------//
                    if (NodeSelectionType.Classical == m_RRTParameter.SelectionType)
                    {
                        //方案1:随机生成节点 - 标准RRT采用的方案
                        mRRTRandNode = RandomConfiguration(iTaskIndex, iStageIndex, mRRTTree);
                    }
                    else //if (NodeSelectionType.Optimzed == m_RRTParameter.SelectionType)
                    {
                        //方案2:按照一定概率随机生成节点或者采用目标点 (改进方案)
                        mRRTRandNode = RandomConfigurationOrTargetConfiguration(iTaskIndex, iStageIndex, mRRTTree);
                    }
                    //-----------------------------------end-------------------------------------//


                    //----------------------------搜索距离最近的节点-----------------------------//
                    mRRTNearNode = NearestNeighbor(mRRTTree, mRRTRandNode);
                    //-----------------------------------end-------------------------------------//


                    //--------------------------------生成新节点---------------------------------//
                    if (PlanningStepType.Constant == m_RRTParameter.StepType)
                    {
                        mRRTNewNode = NewConfiguration(iTaskIndex, iStageIndex, mRRTNearNode, mRRTRandNode);
                    }
                    else //if (PlanningStepType.Random == m_RRTParameter.m_StepType)
                    {
                        mRRTNewNode = NewConfigurationWithRandomStep(iTaskIndex, iStageIndex, mRRTNearNode, mRRTRandNode);
                    }
                    //-----------------------------------end-------------------------------------//


                    //计数器累加
                    nCount = nCount + 1;


                    //-----------------------如果到达允许的搜索上限,则退出-----------------------//
                    if (nCount >= m_RRTParameter.MaxNodeNumber)
                    {
                        Console.WriteLine("");
                        break;
                    }
                    //-----------------------------------end-------------------------------------//


                    //----------------威胁规避(禁止在威胁区域内选择/生成新点/连线)---------------//
                    if (!IsSafePoint(mRRTNewNode.NodeLocation))
                    {
                        continue;
                    }

                    if (!IsSafeLine(mRRTNewNode.NodeLocation, mRRTNearNode.NodeLocation))
                    {
                        continue;
                    }
                    //-----------------------------------end-------------------------------------//


                    //------------------------------扩展树 - 核心--------------------------------//
                    isReachTarget = ExpandTree(iTaskIndex, iStageIndex, ref mRRTTree, mRRTNearNode, mRRTNewNode);
                    //-----------------------------------end-------------------------------------//



                    //------------判断新结点到目标连线是否安全, 若安全则直接连接目标点-----------//
                    if (TargetReachMode.Direct == m_RRTParameter.ReachMode)
                    {
                        isReachTarget = isReachTarget || DirectlyToTarget(iTaskIndex, iStageIndex, mRRTNewNode, ref mRRTTree);
                    }
                    //-----------------------------------end-------------------------------------//
                }

                //构建当前阶段航路
                mRRTPath = BuildPath(mRRTTree);

                //为可视化输出保存
                Visualization.MPathForVisualizaition = mRRTTree;
                if (!PathPlaningDataVisualization.PathPlaningDataVisualization.AlgoDataDict.ContainsKey(mPath.Index.ToString() + "-" + iStageIndex.ToString()))
                {
                    //为ResultShowForm分析窗口保存数据
                    //3.30.2018 刘洋添加
                    PathPlaningDataVisualization.PathPlaningDataVisualization.AlgoDataDict.Add(mPath.Index.ToString() + "-" + iStageIndex.ToString(), Visualization.MyTreeNodeConverter(Visualization.MPathForVisualizaition));
                }
                else
                {
                    PathPlaningDataVisualization.PathPlaningDataVisualization.AlgoDataDict[mPath.Index.ToString() + "-" + iStageIndex.ToString()] = Visualization.MyTreeNodeConverter(Visualization.MPathForVisualizaition);
                }
                //添加当前阶段航路到总航路(起始航路点为当前阶段, 目标航路点为下一个阶段, 注意总航路的最后一个航路点属于最后一个阶段)
                for (int k = 0; k < mRRTPath.Count - 1; ++k)
                {
                    mPath.Waypoints.Add(new MWaypoint(iWaypointIndex, RRTNode.ConvertNodeToUAVState(mRRTPath[k]), iStageIndex));
                    iWaypointIndex = iWaypointIndex + 1;
                }
            }
            //增加最后的目标点
            mPath.Waypoints.Add(new MWaypoint(iWaypointIndex, RRTNode.ConvertNodeToUAVState(mRRTPath[mRRTPath.Count - 1]),
                AlgoInput.UAVTask[iTaskIndex].Stages.Count - 1));

            //返回路径
            return mPath;
        }

        /// <summary>
        /// 从指定航路点开始为单架无人机规划航迹
        /// </summary>
        /// <param name="iTaskIndex">无人机/任务索引</param>
        /// <param name="mWaypoint">指定的起始航路点</param>
        /// <param name="mPath">当前路径</param>
        /// <returns></returns>
        protected MPath BuildPathForSingleUAV(int iTaskIndex, MWaypoint mWaypoint, MPath mPath)
        {
            //移除起始航路点之后(含)的所有航路点
            if (mPath.Waypoints != null)
            {
                mPath.Waypoints.RemoveRange(mWaypoint.Index, mPath.Waypoints.Count - mWaypoint.Index);
                //从数据库移除
            }

            //定义树
            List<RRTNode> mRRTTree = null;
            //定义航路
            List<RRTNode> mRRTPath = null;
            //定义树节点
            RRTNode mRRTNode = null;
            //定义树中间节点
            RRTNode mRRTRandNode = null;
            RRTNode mRRTNearNode = null;
            RRTNode mRRTNewNode = null;

            //以当前航路点为当前阶段的起始状态点(以后的阶段均不变), 构建新的阶段
            List<MStage> mNewStages = new List<MStage>();
            MStage mTempStage = null;
            mTempStage = new MStage();
            mTempStage.StageIndex = mWaypoint.StageIndex;
            mTempStage.StartState = new MKeyState(mWaypoint.State);
            mTempStage.TargetState = AlgoInput.UAVTask[iTaskIndex].Stages[mWaypoint.StageIndex].TargetState;
            mNewStages.Add(mTempStage);
            for (int i = mWaypoint.StageIndex + 1; i < AlgoInput.UAVTask[iTaskIndex].Stages.Count; ++i)
            {
                mTempStage = new MStage();
                mTempStage = AlgoInput.UAVTask[iTaskIndex].Stages[i];
                mNewStages.Add(mTempStage);
            }

            //当前航路点编号
            int iWaypointIndex = mWaypoint.Index;

            //计数器
            int nCount = 1;

            //对每一个阶段规划航路
            for (int iStageIndex = mNewStages[0].StageIndex; iStageIndex < mNewStages[0].StageIndex + mNewStages.Count; ++iStageIndex)
            {
                //实例化
                mRRTTree = new List<RRTNode>();
                //加入开始点为第一个点
                mRRTNode = new RRTNode(0, mNewStages[iStageIndex - mNewStages[0].StageIndex].StartState.Location,
                    mNewStages[iStageIndex - mNewStages[0].StageIndex].StartState.Direction, null);
                mRRTTree.Add(mRRTNode);

                //初始化计数器
                nCount = 1;
                //是否到达目标 - 初始未到达
                bool isReachTarget = false;

                //循环构建新点 - 到达目标时自动退出
                while (!isReachTarget)
                {
                    //-------------------------------随机生成节点--------------------------------//
                    if (NodeSelectionType.Classical == m_RRTParameter.SelectionType)
                    {
                        //方案1:随机生成节点 - 标准RRT采用的方案
                        mRRTRandNode = RandomConfiguration(iTaskIndex, iStageIndex, mRRTTree);
                    }
                    else //if (NodeSelectionType.Optimzed == m_RRTParameter.SelectionType)
                    {
                        //方案2:按照一定概率随机生成节点或者采用目标点 (改进方案)
                        mRRTRandNode = RandomConfigurationOrTargetConfiguration(iTaskIndex, iStageIndex, mRRTTree);
                    }
                    //-----------------------------------end-------------------------------------//


                    //----------------------------搜索距离最近的节点-----------------------------//
                    mRRTNearNode = NearestNeighbor(mRRTTree, mRRTRandNode);
                    //-----------------------------------end-------------------------------------//


                    //--------------------------------生成新节点---------------------------------//
                    if (PlanningStepType.Constant == m_RRTParameter.StepType)
                    {
                        mRRTNewNode = NewConfiguration(iTaskIndex, iStageIndex, mRRTNearNode, mRRTRandNode);
                    }
                    else //if (PlanningStepType.Random == m_RRTParameter.m_StepType)
                    {
                        mRRTNewNode = NewConfigurationWithRandomStep(iTaskIndex, iStageIndex, mRRTNearNode, mRRTRandNode);
                    }
                    //-----------------------------------end-------------------------------------//


                    //计数器累加
                    nCount = nCount + 1;


                    //-----------------------如果到达允许的搜索上限,则退出-----------------------//
                    if (nCount >= m_RRTParameter.MaxNodeNumber)
                    {
                        break;
                    }
                    //-----------------------------------end-------------------------------------//


                    //----------------威胁规避(禁止在威胁区域内选择/生成新点/连线)---------------//
                    if (!IsSafePoint(mRRTNewNode.NodeLocation))
                    {
                        continue;
                    }

                    if (!IsSafeLine(mRRTNewNode.NodeLocation, mRRTNearNode.NodeLocation))
                    {
                        continue;
                    }
                    //-----------------------------------end-------------------------------------//


                    //------------------------------扩展树 - 核心--------------------------------//
                    isReachTarget = ExpandTree(iTaskIndex, iStageIndex, ref mRRTTree, mRRTNearNode, mRRTNewNode);
                    //-----------------------------------end-------------------------------------//

                    //------------判断新结点到目标连线是否安全, 若安全则直接连接目标点-----------//
                    if (TargetReachMode.Direct == m_RRTParameter.ReachMode)
                    {
                        isReachTarget = isReachTarget || DirectlyToTarget(iTaskIndex, iStageIndex, mRRTNewNode, ref mRRTTree);
                    }
                    //-----------------------------------end-------------------------------------//
                }
                //构建当前阶段航路
                mRRTPath = BuildPath(mRRTTree);

                //添加当前阶段航路到总航路
                for (int k = 0; k < mRRTPath.Count - 1; ++k)
                {
                    mPath.Waypoints.Add(new MWaypoint(iWaypointIndex, RRTNode.ConvertNodeToUAVState(mRRTPath[k]), iStageIndex));
                    iWaypointIndex = iWaypointIndex + 1;
                }
            }
            //增加最后的目标点
            mPath.Waypoints.Add(new MWaypoint(iWaypointIndex, RRTNode.ConvertNodeToUAVState(mRRTPath[mRRTPath.Count - 1]),
                AlgoInput.UAVTask[iTaskIndex].Stages.Count - 1));

            //返回路径
            return mPath;
        }

    }
}
