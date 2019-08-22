using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
//
using PlanningAlgorithmInterface.Define.Input;
using PlanningAlgorithmInterface.Define.Output;
using PlanningAlgorithmInterface.AlgorithmInterface;
using SceneElementDll.Basic;

namespace RRTOrigin
{
    /// <summary>
    /// RRT算法公用函数类
    /// </summary>
    public class RRTBase 
    {
        #region 算法属性
        public MInput AlgoInput { get; set; }//算法输入，场景信息
        public IParameter AlgoParameter { get; set; }//算法参数的属性
        /// <summary>
        /// 检查Point3 点是否安全，委托类型的变量
        /// </summary>
        public Func<FPoint3, bool> IsSafePoint { get; set; }
        /// <summary>
        /// 返回lhs,rhs构成的线段是否安全，委托类型的变量
        /// </summary>
        public Func<FPoint3, FPoint3, bool> IsSafeLine { get; set; }
        /// <summary>
        /// 判断航路是否安全
        /// <param name="startWaypointIndex">起始航路点编号</param>
        /// <param name="endWaypointIndex">终止航路点编号</param>
        /// <param name="mPath">航路</param>
        /// <returns>bool Path是否安全</returns>
        /// </summary>
        public Func<int, int, MPath, bool> IsSafePath { get; set; }
        #endregion



        /// <summary>
        /// 随机数生成
        /// </summary>
        protected Random m_Random = new Random();

        /// <summary>
        /// RRT算法参数
        /// </summary>
        protected RRTParameter m_RRTParameter = null;


        /// <summary>
        /// 在探测范围内随机生成一个节点
        /// </summary>
        /// <param name="iTaskIndex">无人机/任务编号</param>
        /// <param name="iStageIndex">阶段编号</param>
        /// <param name="mRRTTree">RRT树</param>
        /// <returns></returns>
        protected RRTNode RandomConfiguration(int iTaskIndex, int iStageIndex, List<RRTNode> mRRTTree)
        {
            RRTNode mRRTRandNode = new RRTNode();
            mRRTRandNode.NodeLocation.X = m_Random.NextDouble() * AlgoInput.Scenario.FlightScene.Coordinate.MaxX;
            mRRTRandNode.NodeLocation.Y = m_Random.NextDouble() * AlgoInput.Scenario.FlightScene.Coordinate.MaxY;
            mRRTRandNode.NodeLocation.Z = AlgoInput.Scenario.UAVGroup.GetUAV(AlgoInput.UAVTask[iTaskIndex].Index).StartState.PointLocation.Z; //m_RRTParameter.MissionAltitude;
            mRRTRandNode.ParentNode = null;

            //
            return mRRTRandNode;
        }

        /// <summary>
        /// 按照一定概率选择目标点或者随机生成一个新节点
        /// </summary>
        /// <param name="iTaskIndex">无人机/任务编号</param>
        /// <param name="iStageIndex">阶段编号</param>
        /// <param name="mRRTTree">RRT树</param>
        /// <returns>节点</returns>
        protected RRTNode RandomConfigurationOrTargetConfiguration(int iTaskIndex, int iStageIndex, List<RRTNode> mRRTTree)
        {
            //新节点
            RRTNode mRRTNode = null;

            //是否直接把目标点当作新点
            if (m_Random.NextDouble() > m_RRTParameter.ChooseTargetThreshold)
            {
                mRRTNode = RandomConfiguration(iTaskIndex, iStageIndex, mRRTTree);
            }
            else
            {
                mRRTNode = RRTNode.ConvertUAVStateToNode(AlgoInput.UAVTask[iTaskIndex].Stages[iStageIndex].TargetState);
            }

            //
            return mRRTNode;
        }

        /// <summary>
        /// 寻找树mRRTTree中距离mRRTRandNode最近的节点
        /// </summary>
        /// <param name="mRRTTree">RRT树</param>
        /// <param name="mRRTRandNode">随机生成的节点</param>
        /// <returns></returns>
        protected RRTNode NearestNeighbor(List<RRTNode> mRRTTree, RRTNode mRRTRandNode)
        {
            RRTNode mRRTNearNode = new RRTNode();
            double fMinDistance = double.MaxValue;
            double fDistance = 0;
            foreach (RRTNode mRRTNode in mRRTTree)
            {
                fDistance = DistanceBetweenTwoNode(mRRTRandNode, mRRTNode);
                if (fDistance <= fMinDistance)
                {
                    mRRTNearNode = mRRTNode;
                    fMinDistance = fDistance;
                }
            }
            //
            return mRRTNearNode;
        }

        /// <summary>
        /// 计算新的节点
        /// </summary>
        /// <param name="iTaskIndex">无人机任务索引</param>
        /// <param name="iStageIndex">阶段编号</param>
        /// <param name="mRRTNearNode">距离mRRTRandNode最近的节点</param>
        /// <param name="mRRTRandNode">随机生成的节点</param>
        /// <returns></returns>
        protected RRTNode NewConfiguration(int iTaskIndex, int iStageIndex, RRTNode mRRTNearNode, RRTNode mRRTRandNode)
        {
            RRTNode mRRTNewNode = new RRTNode();
            if (DistanceBetweenTwoNode(mRRTNearNode, mRRTRandNode) <= m_RRTParameter.Step)
            {
                mRRTNewNode = mRRTRandNode;
                return mRRTNewNode;
            }

            //生成新点
            mRRTNewNode = NewNode(iTaskIndex, iStageIndex, m_RRTParameter.Step, mRRTNearNode, mRRTRandNode);
            //
            return mRRTNewNode;
        }

        /// <summary>
        /// 计算新的节点
        /// </summary>
        /// <param name="iTaskIndex">无人机任务索引</param>
        /// <param name="iStageIndex">阶段编号</param>
        /// <param name="mRRTNearNode">距离mRRTRandNode最近的节点</param>
        /// <param name="mRRTRandNode">随机生成的节点</param>
        /// <returns></returns>
        protected RRTNode NewConfigurationWithRandomStep(int iTaskIndex, int iStageIndex, RRTNode mRRTNearNode, RRTNode mRRTRandNode)
        {
            //在指定范围内随机生成同一个步长
            double currentSTEP = m_RRTParameter.RandomStepMin + (m_RRTParameter.RandomStepMax - m_RRTParameter.RandomStepMin) * m_Random.NextDouble();
            //
            RRTNode mRRTNewNode = new RRTNode();
            if (currentSTEP >= DistanceBetweenTwoNode(mRRTNearNode, mRRTRandNode))
            {
                mRRTNewNode = mRRTRandNode;
                return mRRTNewNode;
            }

            //生成新点
            mRRTNewNode = NewNode(iTaskIndex, iStageIndex, currentSTEP, mRRTNearNode, mRRTRandNode);

            return mRRTNewNode;
        }

        /// <summary>
        /// 计算新的节点
        /// </summary>
        /// <param name="iTaskIndex">无人机任务索引</param>
        /// <param name="iStageIndex">阶段编号</param>
        /// <param name="fStep">步长</param>
        /// <param name="mRRTNearNode">距离mRRTRandNode最近的节点</param>
        /// <param name="mRRTRandNode">随机生成的节点</param>
        /// <returns></returns>
        protected RRTNode NewNode(int iTaskIndex, int iStageIndex, double fStep, RRTNode mRRTNearNode, RRTNode mRRTRandNode)
        {
            //liuwei.2012.09.24.修正计算错误
            RRTNode mRRTNewNode = new RRTNode();

            mRRTNewNode.NodeLocation.X = mRRTNearNode.NodeLocation.X + (fStep * (mRRTNearNode.NodeLocation.X - mRRTRandNode.NodeLocation.X)) / DistanceBetweenTwoNode(mRRTNearNode, mRRTRandNode);
            mRRTNewNode.NodeLocation.Y = mRRTNearNode.NodeLocation.Y + (fStep * (mRRTNearNode.NodeLocation.Y - mRRTRandNode.NodeLocation.Y)) / DistanceBetweenTwoNode(mRRTNearNode, mRRTRandNode);
            mRRTNewNode.NodeLocation.Z = AlgoInput.Scenario.UAVGroup.GetUAV(AlgoInput.UAVTask[iTaskIndex].Index).StartState.PointLocation.Z; //m_RRTParameter.MissionAltitude;

            if (DistanceBetweenTwoNode(mRRTRandNode, mRRTNewNode) >= DistanceBetweenTwoNode(mRRTRandNode, mRRTNearNode))
            {
                mRRTNewNode.NodeLocation.X = mRRTNearNode.NodeLocation.X - (fStep * (mRRTNearNode.NodeLocation.X - mRRTRandNode.NodeLocation.X)) / DistanceBetweenTwoNode(mRRTNearNode, mRRTRandNode);
                mRRTNewNode.NodeLocation.Y = mRRTNearNode.NodeLocation.Y - (fStep * (mRRTNearNode.NodeLocation.Y - mRRTRandNode.NodeLocation.Y)) / DistanceBetweenTwoNode(mRRTNearNode, mRRTRandNode);
                mRRTNewNode.NodeLocation.Z = AlgoInput.Scenario.UAVGroup.GetUAV(AlgoInput.UAVTask[iTaskIndex].Index).StartState.PointLocation.Z; //m_RRTParameter.MissionAltitude;
            }
            return mRRTNewNode;
        }

        /// <summary>
        /// 计算两个节点之间的距离
        /// </summary>
        /// <param name="mNode1">节点1</param>
        /// <param name="mNode2">节点2</param>
        /// <returns></returns>
        protected double DistanceBetweenTwoNode(RRTNode mNode1, RRTNode mNode2)
        {
            //二维算法使用
            return FPoint3.DistanceBetweenTwoSpacePointsXY(mNode1.NodeLocation, mNode2.NodeLocation);
            //三维算法使用
            //return FPoint3.DistanceBetweenTwoSpacePoints(mNode1.NodeLocation, mNode2.NodeLocation);
        }

        /// <summary>
        /// 扩展树
        /// </summary>
        /// <param name="iTaskIndex">无人机/任务编号</param>
        /// <param name="iStageIndex">阶段编号</param>
        /// <param name="mRRTTree">RRT树</param>
        /// <param name="mRRTNearNode">最近节点</param>
        /// <param name="mRRTNewNode">新生成节点</param>
        /// <returns>1,是否有效(安全); 2,如果有效(安全),是否到达目标</returns>
        protected bool ExpandTree(int iTaskIndex, int iStageIndex, ref List<RRTNode> mRRTTree, RRTNode mRRTNearNode, RRTNode mRRTNewNode)
        {
            //设置是否到达目标的标志
            bool isReachTarget = false;
            //
            if (mRRTNewNode != null)
            {
                //设置节点编号
                mRRTNewNode.NodeIndex = mRRTTree.Count;

                //设置父节点
                mRRTNewNode.ParentNode = mRRTNearNode;

                //设置是否在路径上
                mRRTNewNode.IsInRRTPath = 0;

                //增加新节点
                mRRTTree.Add(mRRTNewNode);

                //到达目标点
                if (DistanceBetweenTwoNode(mRRTNewNode, RRTNode.ConvertUAVStateToNode(AlgoInput.UAVTask[iTaskIndex].Stages[iStageIndex].TargetState)) <= RRTParameter.Error)
                {
                    //设置到达标记
                    isReachTarget = true;

                    //此判断防止直接选择目标点作为新节点的时候，重复增加目标点到树中 - liuwei.2011.11.28增加
                    if (mRRTNewNode.NodeLocation != AlgoInput.UAVTask[iTaskIndex].Stages[iStageIndex].TargetState.Location)
                    {
                        //------------------------将目标点作为最后一个节点加入路径中---------------------//
                        RRTNode mRRTTempNewNode = new RRTNode();
                        //设置节点编号
                        mRRTTempNewNode.NodeIndex = mRRTTree.Count;
                        //设置父节点
                        mRRTTempNewNode.ParentNode = mRRTNewNode;
                        //设置是否在路径上
                        mRRTTempNewNode.IsInRRTPath = 0;
                        //新节点位置
                        mRRTTempNewNode.NodeLocation = AlgoInput.UAVTask[iTaskIndex].Stages[iStageIndex].TargetState.Location;
                        //增加新节点
                        mRRTTree.Add(mRRTTempNewNode);
                        //------------------------将目标点作为最后一个节点加入路径中end------------------//
                    }
                }
                else
                {
                    isReachTarget = false;
                }
            }

            return isReachTarget;
        }

        /// <summary>
        /// 判断新结点到目标连线是否安全, 若安全则直接连接目标点
        /// </summary>
        /// <param name="iTaskIndex">无人机/任务编号</param>
        /// <param name="iStageIndex">阶段编号</param>
        /// <param name="mRRTCurrentNode">当前节点</param>
        /// <param name="mRRTTree">RRT</param>
        /// <returns>1,是否有效(安全); 2,如果有效(安全),是否到达目标</returns>
        protected bool DirectlyToTarget(int iTaskIndex, int iStageIndex, RRTNode mRRTCurrentNode, ref List<RRTNode> mRRTTree)
        {
            bool isReachTarget = false;
            //进一步判断新结点到目标连线是否安全, 若安全则直接连接目标点
            if (IsSafeLine(mRRTCurrentNode.NodeLocation, AlgoInput.UAVTask[iTaskIndex].Stages[iStageIndex].TargetState.Location))
            {
                //设置为到达
                isReachTarget = true;
                //设置目标点为最后一个新结点, 并将当前节点设置为她的父节点
                mRRTCurrentNode = RRTNode.ConvertUAVStateToNode(AlgoInput.UAVTask[iTaskIndex].Stages[iStageIndex].TargetState, mRRTCurrentNode);
                //设节点编号
                mRRTCurrentNode.NodeIndex = mRRTTree.Count;
                //设置是否在路径上
                mRRTCurrentNode.IsInRRTPath = 0;
                //增加新节点
                mRRTTree.Add(mRRTCurrentNode);
            }

            return isReachTarget;
        }

        /// <summary>
        /// 判断是否能够避免碰撞
        /// </summary>
        /// <param name="mRRTNode">节点</param>
        /// <returns></returns>
        protected bool CollisionAvoidance(RRTNode mRRTNode)
        {
            bool bIsSafe = true;

            return bIsSafe;
        }

        /// <summary>
        /// 构造RRT树上的路径
        /// </summary>
        /// <param name="mRRTTree">从中选择路径的树</param>
        /// <returns>路径</returns>
        protected List<RRTNode> BuildPath(List<RRTNode> mRRTTree)
        {
            //如果树为空,则返回空
            if ((null == mRRTTree))
            {
                return null;
            }

            //选取并存储路径
            RRTNode mTempNode = mRRTTree[mRRTTree.Count - 1];
            mRRTTree[mRRTTree.Count - 1].IsInRRTPath = 1;
            List<RRTNode> mTempRRTPath = new List<RRTNode>();
            //循环搜索
            while (mTempNode.ParentNode != null)
            {
                //增加节点
                mTempRRTPath.Add(mTempNode);

                //递增
                mTempNode = mTempNode.ParentNode;
                mRRTTree[mTempNode.NodeIndex].IsInRRTPath = 1;
            }
            mTempRRTPath.Add(mTempNode);//实际上是初始节点

            //由于是按照倒序添加,这里将顺序反转
            mTempRRTPath.Reverse();

            //航迹简化
            if ((AlgoParameter as RRTParameter).DijkstraOptimizeParameter == true )
                WayPointList_Simplify_Method(ref mTempRRTPath);

            //返回值
            return mTempRRTPath;
        }
    
    
     
#region 航迹简化部分（9院需求）
        private void WayPointList_Simplify_Method(ref List<RRTNode> WayPointList)
        {

            //插入待处理的航迹
            List<RRTNode> temp_list = new List<RRTNode>();
            for (int i = 0; i < WayPointList.Count - 1; i++)
            {
                temp_list.Add(WayPointList[i]);
            }
            temp_list.Add(WayPointList[WayPointList.Count - 1]);
            //templist更新完毕
            //WayPointList = temp_list;
            PathSimplify(temp_list, ref WayPointList);

        }
        /// <summary>
        ///  路径化简，动态规划方法
        /// </summary>
        /// <param name="OriginNodeList"></param>
        /// <param name="OUTPUTWayPointList"></param>
        private void PathSimplify(List<RRTNode> OriginNodeList, ref List<RRTNode> OUTPUTWayPointList)
        {

            //             List<FPoint3> temp_WayPointList = new List<FPoint3>();
            //             for (int i = 0; i < OriginNodeList.Count; i++)
            //             {
            //                 temp_WayPointList.Add(OriginNodeList[i].NodeLocation * 1.0); //new
            //             }
            List<List<double>> DistanceMatrix = new List<List<double>>();
            for (int i = 0; i < OriginNodeList.Count; i++)
            {
                DistanceMatrix.Add(new List<double>());
                for (int j = 0; j < OriginNodeList.Count; j++)
                {
                    DistanceMatrix[i].Add(new double());
                    DistanceMatrix[i][j] = double.MaxValue;
                }
            }
            /*
             *      x0  x1  x2  x3
             * x0   max max max max
             * x1   X   max max max
             * x2   X   X   max max
             * x3   X   X   X   max
            */



            for (int i = 0; i < OriginNodeList.Count - 1; i++)
            {
                for (int j = i + 1; j < OriginNodeList.Count; j++)           //下三角阵
                {
                    //if (Line_ObstacleAvodiance(OriginNodeList[i], OriginNodeList[j], FlightScene) == true)
                    if (IsSafeLine(OriginNodeList[i].NodeLocation, OriginNodeList[j].NodeLocation))
                    {
                        DistanceMatrix[i][j] = FPoint3.DistanceBetweenTwoSpacePointsXY(OriginNodeList[i].NodeLocation, OriginNodeList[j].NodeLocation);
                        DistanceMatrix[j][i] = DistanceMatrix[i][j];
                    }
                    else
                    {
                        DistanceMatrix[i][j] = double.MaxValue;
                    }
                }
            }
            //距离矩阵生成结束




            //这里设X0为 初始点， Xn 为goalPoint
            List<int> OptimizePrePathPoint = new List<int>();  //记录到达该店最优路径的父节点
            for (int i = 0; i < OriginNodeList.Count; i++)
            {
                OptimizePrePathPoint.Add(new int());
                OptimizePrePathPoint[i] = int.MinValue;

            }

            List<double> OptimizeDistance = new List<double>(); //记录到达该点的最优距离
            for (int i = 0; i < OriginNodeList.Count; i++)
            {
                OptimizeDistance.Add(new double());
                OptimizeDistance[i] = double.MaxValue;

            }


            Dijkstra(OriginNodeList.Count, DistanceMatrix, OriginNodeList.Count - 1, ref OptimizePrePathPoint, ref OptimizeDistance);


            OUTPUTWayPointList.Clear();
            int temp_index = 0;
            do
            {
                OUTPUTWayPointList.Add(OriginNodeList[temp_index]);
                temp_index = OptimizePrePathPoint[temp_index];
            } while (temp_index != OptimizePrePathPoint.Count - 1);
            OUTPUTWayPointList.Add(OriginNodeList[temp_index]);

        }
        /// <summary>
        /// Dijkstra算法
        /// </summary>
        /// <param name="iTotalNumber">点的总数量</param>
        /// <param name="fCostMatrix">代价矩阵</param>
        /// <param name="iTargetIndex">目标点索引</param>
        /// <param name="prev"></param>
        /// <param name="dist"></param>
        private void Dijkstra(int iTotalNumber, List<List<double>> fCostMatrix, int iTargetIndex, ref List<int> prev, ref List<double> dist)
        {
            int[] s = new int[iTotalNumber];

            //初始化最小路径代价和前一跳节点值
            for (int i = 0; i < iTotalNumber; ++i)
            {
                dist[i] = fCostMatrix[iTargetIndex][i];
                s[i] = 0;
                if (dist[i] >= double.MaxValue)
                {
                    prev[i] = 0;
                }
                else
                {
                    prev[i] = iTargetIndex;
                }
            }
            dist[iTargetIndex] = 0;
            //源节点作为最初的s子集
            s[iTargetIndex] = 1;

            //
            double temp = 0;
            int u = 0;
            double newdist = 0;
            for (int i = 0; i < iTotalNumber; ++i)
            {
                temp = double.MaxValue;
                u = iTargetIndex;
                //加入具有最小代价的邻居节点到s子集    
                for (int j = 0; j < iTotalNumber; ++j)
                {
                    if ((0 == s[j]) && (dist[j] < temp))
                    {
                        u = j;
                        temp = dist[j];
                    }
                }
                s[u] = 1;
                //计算加入新的节点后，更新路径使得其产生代价最短    
                for (int j = 0; j < iTotalNumber; ++j)
                {
                    if ((0 == s[j]) && (fCostMatrix[u][j] < double.MaxValue))
                    {
                        newdist = dist[u] + fCostMatrix[u][j];
                        if (newdist < dist[j])
                        {
                            dist[j] = newdist;
                            prev[j] = u;
                        }
                    }
                }
            }
        }

#endregion
    
    }
}
