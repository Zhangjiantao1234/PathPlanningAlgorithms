using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
//
using PlanningAlgorithmInterface.Define.Input;
using PlanningAlgorithmInterface.Define.Output;

using SceneElementDll.Basic;
using PlanningAlgorithmInterface.AlgorithmInterface;

namespace RRTStar
{
    /// <summary>
    /// RRT算法公用函数类
    /// </summary>
    public class MrrtStarBase 
    {
        public MInput AlgoInput { get; set; }
        public IParameter AlgoParameter { get; set; }
        public Func<FPoint3, bool> IsSafePoint { get; set; }
        public Func<FPoint3, FPoint3, bool> IsSafeLine { get; set; }
        public Func<int, int, MPath, bool> IsSafePath { get; set; }

        /// <summary>
        /// 随机数生成
        /// </summary>
        protected Random MRandom = new Random();

        /// <summary>
        /// RRT算法参数
        /// </summary>
        protected RrtStarParameter MRrtParameter = null;

        //可视化参数
        RrtStarVisualization _visualization = new RrtStarVisualization();



        /// <summary>
        /// 为单架无人机规划航迹
        /// </summary>
        /// <param name="iTaskIndex">无人机/任务索引</param>
        /// <returns></returns>
        protected virtual MPath BuildPathForSingleUav(int iTaskIndex)
        {
            //实例化航路
            MPath mPath = new MPath();
            mPath.Index = AlgoInput.UAVTask[iTaskIndex].Index;
            mPath.Waypoints = new List<MWaypoint>();
            int iWaypointIndex = 0;
            //基本RRT函数库
            RrtStarHelper helper = null;

            //定义树
            HashSet<RrtStarNode> mRrtStarTree = null;
            //定义航路
            List<RrtStarNode> mRrtPath = null;
            //定义树节点
            RrtStarNode mRrtNode = null;
            //定义树中间节点
            RrtStarNode mRrtRandNode = null;
            RrtStarNode mRrtNearNode = null;
            RrtStarNode mRrtNewSteerNode = null;

            HashSet<RrtStarNode> mFinalNodeSet = null;

            //计数器
            int nCount = 1;

            //对每一个阶段规划航路
            for (int iStageIndex = 0; iStageIndex < AlgoInput.UAVTask[iTaskIndex].Stages.Count; ++iStageIndex)
            {
                //初始化函数库
                helper = new RrtStarHelper(MRandom, MRrtParameter, iTaskIndex, iStageIndex, AlgoInput,IsSafePoint,IsSafeLine);
                //加载委托函数
                helper.MLineCostFunc += CostFunc;
                
                //实例化
                mRrtStarTree = new HashSet<RrtStarNode>();
                mFinalNodeSet = new HashSet<RrtStarNode>();

                //初始化计数器
                nCount = 1;

                //加入开始点为第一个点
                mRrtNode = new RrtStarNode(0, AlgoInput.UAVTask[iTaskIndex].Stages[iStageIndex].StartState.Location, nCount, null);
                mRrtStarTree.Add(mRrtNode);


                //是否到达目标 - 初始未到达
                bool isReachTarget = false;

       
                //循环构建新点 - 到达目标时自动退出
                while (!isReachTarget)
                {
                    //-------------------------------随机生成节点--------------------------------//
                    //随机生成节点 
                    mRrtRandNode = helper.RandomConfiguration(mFinalNodeSet.Count == 0);
                    //-----------------------------------end-------------------------------------//
                    
                    //----------------------------搜索距离最近的节点-----------------------------//
                    mRrtNearNode = helper.NearestNeighbor(mRrtStarTree, mRrtRandNode);
                    //-----------------------------------end-------------------------------------//
                    
                    //------------------------------延伸生成新节点-------------------------------//
                    mRrtNewSteerNode = helper.Steer(mRrtNearNode, mRrtRandNode);
                    //-----------------------------------end-------------------------------------//
                    
                    //计数器累加
                    nCount = nCount + 1;
                    
                    //-----------------------如果到达允许的搜索上限,则退出-----------------------//
                    if (nCount >= MRrtParameter.MaxNodeNumber)
                    {
                        Console.WriteLine("MaxNodeNumber!");
                        break;
                    }
                    //-----------------------------------end-------------------------------------//

                    //------------------------------扩展树 - 核心--------------------------------//
                    if (IsSafeLine(mRrtNewSteerNode.NodeLocation, mRrtNearNode.NodeLocation) && IsSafePoint(mRrtNewSteerNode.NodeLocation))
                    {
                        //此处SteerNode已经添加到Tree中
                        helper.ExpandTree(ref mRrtStarTree, mRrtNearNode, mRrtNewSteerNode, nCount);  
                    }
                    else
                        continue;
                   
                    //-----------------------------------end-------------------------------------//
                    


                    //------------判断新结点到目标连线是否安全, 若安全则直接连接目标点-----------//
                    if (MRrtParameter.ReachMode == TargetReachMode.Direct)
                    {
                        isReachTarget = isReachTarget || helper.DirectlyToTarget(mRrtNewSteerNode, ref mRrtStarTree);
                    }
                    else if (MRrtParameter.ReachMode == TargetReachMode.Gradual)
                    {
                        helper.MarkingValid(mRrtNewSteerNode,ref mFinalNodeSet);
                    }
                    //-----------------------------------end-------------------------------------//


                    //----------------------------Branch and Bound-------------------------------//
                    if(mFinalNodeSet.Count > 10)
                    {
                        helper.BranchAndBound(ref mFinalNodeSet, ref mRrtStarTree);
                    }
                    //-----------------------------------end-------------------------------------//
                }
                if (nCount >= MRrtParameter.MaxNodeNumber)
                {
                    if (MRrtParameter.ReachMode == TargetReachMode.Direct)
                    {
                        helper.ConstructFailedPath(ref mRrtStarTree);

                    }
                    else if(MRrtParameter.ReachMode == TargetReachMode.Gradual)
                    {
                        //GraduallyToTarget方法就是检查当 nCount == m_RRTParameter.MaxNodeNumber 时的结果
                        //所以在本部分需要为IsVailid添加标签。
                        helper.GraduallyToTarget(nCount, ref mRrtStarTree);
                    }
                    
                    
                }
                mRrtPath = BuildPath(mRrtStarTree);


                //为可视化输出保存
                _visualization.MPathForVisualizaition = mRrtStarTree.ToList();
                if (!PathPlaningDataVisualization.PathPlaningDataVisualization.AlgoDataDict.ContainsKey(mPath.Index.ToString() + "-" + iStageIndex.ToString()))
                {
                    //为ResultShowForm分析窗口保存数据
                    //3.30.2018 刘洋添加
                    PathPlaningDataVisualization.PathPlaningDataVisualization.AlgoDataDict.Add(mPath.Index.ToString() + "-" + iStageIndex.ToString(), _visualization.MyTreeNodeConverter(_visualization.MPathForVisualizaition));
                }
                else
                {
                    PathPlaningDataVisualization.PathPlaningDataVisualization.AlgoDataDict[mPath.Index.ToString() + "-" + iStageIndex.ToString()] = _visualization.MyTreeNodeConverter(_visualization.MPathForVisualizaition);
                }


                //添加当前阶段航路到总航路(起始航路点为当前阶段, 目标航路点为下一个阶段, 注意总航路的最后一个航路点属于最后一个阶段)
                for (int k = 0; k < mRrtPath.Count - 1; ++k)
                {
                    mPath.Waypoints.Add(new MWaypoint(iWaypointIndex, RrtStarNode.ConvertNodeToUavState(mRrtPath[k]), iStageIndex));
                    iWaypointIndex = iWaypointIndex + 1;
                }
            }
            //增加最后的目标点
            mPath.Waypoints.Add(new MWaypoint(iWaypointIndex, RrtStarNode.ConvertNodeToUavState(mRrtPath[mRrtPath.Count - 1]),
                AlgoInput.UAVTask[iTaskIndex].Stages.Count - 1));

            //返回路径
            return mPath;
        }

        /// <summary>
        /// 构造RRT树上的路径
        /// </summary>
        /// <param name="mRrtTree">从中选择路径的树</param>
        /// <returns>路径</returns>
        protected List<RrtStarNode> BuildPath(HashSet<RrtStarNode> mRrtTree)
        {
            //如果树为空,则返回空
            if ((null == mRrtTree))
            {
                return null;
            }

            //选取并存储路径
            int targetNodeIndex = mRrtTree.Max(e => e.NodeIndex);
            RrtStarNode mTempNode = mRrtTree.First(e => e.NodeIndex == targetNodeIndex);
            List<RrtStarNode> mTempRrtPath = new List<RrtStarNode>();
            //循环搜索
            while (mTempNode.ParentNode != null)
            {
                //增加节点
                mTempRrtPath.Add(mTempNode);
                //递增
                mTempNode = mTempNode.ParentNode;
            }
            mTempRrtPath.Add(mTempNode);//实际上是初始节点

            //由于是按照倒序添加,这里将顺序反转
            mTempRrtPath.Reverse();

            //返回值
            return mTempRrtPath;
        }


        /// <summary>
        /// 本DEMO中给出的目标函数的定义,以最短距离为最优目标
        /// </summary>
        /// <param name="lNode"></param>
        /// <param name="rNode"></param>
        /// <returns></returns>
        protected double CostFunc(RrtStarNode lNode,RrtStarNode rNode)
        {
            return FPoint3.DistanceBetweenTwoSpacePointsXY(lNode.NodeLocation, rNode.NodeLocation);
        }
    }
}
