using PlanningAlgorithmInterface.Define.Input;
using SceneElementDll.Basic;
using System;
using System.Collections.Generic;
using System.Linq;

namespace RRTStar
{

    /// <summary>
    /// RRTStar各种子函数包
    /// 由于之前的子函数都以iTaskIndex 和 iStageIndex未参数
    /// 所以这里做了一个helper的class将这些参数抹去，使得 RRT 更容易编写。
    /// </summary>
    public class RrtStarHelper 
    {
        public RrtStarHelper(Random mRandom, RrtStarParameter mRrtParameter, int iTaskIndex, int iStageIndex, MInput mInput
            , Func<FPoint3, bool> isSafePoint, Func<FPoint3, FPoint3, bool> isSafeLine)
        {
            this._mRandom = mRandom;
            this.MRrtParameter = mRrtParameter;
            this.ITaskIndex = iTaskIndex;
            this.IStageIndex = iStageIndex;
            this._mInput = mInput;
            this._isSafeLine = isSafeLine;
            this._isSafePoint = isSafePoint;
        }


        Func<FPoint3, bool> _isSafePoint = null;
        Func<FPoint3, FPoint3, bool> _isSafeLine = null;

        /// <summary>
        /// 随机数生成
        /// </summary>
        private readonly Random _mRandom = null;

        /// <summary>
        /// RRT算法参数
        /// </summary>
        protected RrtStarParameter MRrtParameter = null;

        /// <summary>
        /// 无人机/任务编号
        /// </summary>
        protected int ITaskIndex = -1;

        /// <summary>
        /// 阶段编号
        /// </summary>
        protected int IStageIndex = -1;
        private MInput _mInput = null;



        /// <summary>
        /// 自定义目标函数委托(处理线段)
        /// </summary>
        /// <param name="startNode"></param>
        /// <param name="endNode"></param>
        /// <returns></returns>
        public delegate double LineCostFunc(RrtStarNode startNode, RrtStarNode endNode);

        /// <summary>
        /// 自定义目标函数委托
        /// </summary>
        public event LineCostFunc MLineCostFunc = null;


        /// <summary>
        /// Generate the Node by Random Configuration
        /// </summary>
        /// <param name="needBiased"></param>
        /// <returns></returns>
        public RrtStarNode RandomConfiguration(bool needBiased)
        {
            if (MRrtParameter.SelectionType == NodeSelectionType.Classical)
                return RandomConfigurationOrigin();
            else if (MRrtParameter.SelectionType == NodeSelectionType.Bias)
            {
                if (needBiased)
                    return BiasRandomConfiguration();
                else
                    return RandomConfigurationOrigin();
            }
            else
                return null;
        }

        /// <summary>
        /// 在探测范围内随机生成一个节点
        /// </summary>
        /// <returns>RRTStarNode</returns>
        private RrtStarNode RandomConfigurationOrigin()
        {
            RrtStarNode mRrtRandNode = new RrtStarNode();
            mRrtRandNode.NodeLocation.X = _mRandom.NextDouble() * _mInput.Scenario.FlightScene.Coordinate.MaxX;
            mRrtRandNode.NodeLocation.Y = _mRandom.NextDouble() * _mInput.Scenario.FlightScene.Coordinate.MaxY;
            mRrtRandNode.NodeLocation.Z = _mInput.Scenario.UAVGroup.GetUAV(_mInput.UAVTask[ITaskIndex].Index).StartState.PointLocation.Z; //m_RRTParameter.MissionAltitude;
            mRrtRandNode.ParentNode = null;

            //
            return mRrtRandNode;
        }


        /// <summary>
        /// 寻找树mRRTTree中距离mRRTRandNode最近的节点
        /// </summary>
        /// <param name="mRrtTree">RRT树</param>
        /// <param name="mRrtRandNode">随机生成的节点</param>
        /// <returns></returns>
        public RrtStarNode NearestNeighbor(HashSet<RrtStarNode> mRrtTree, RrtStarNode mRrtRandNode)
        {
            RrtStarNode mRrtNearNode = new RrtStarNode();
            double fMinDistance = double.MaxValue;
            double fDistance = 0;
            foreach (RrtStarNode mRrtNode in mRrtTree)
            {
                fDistance = DistanceBetweenTwoNode(mRrtRandNode, mRrtNode);
                if (fDistance <= fMinDistance)
                {
                    mRrtNearNode = mRrtNode;
                    fMinDistance = fDistance;
                }
            }
            //
            return mRrtNearNode;
        }

        /// <summary>
        /// 计算新的节点
        /// </summary>
        /// <param name="mRrtNearNode">距离mRRTRandNode最近的节点</param>
        /// <param name="mRrtRandNode">随机生成的节点</param>
        /// <returns></returns>
        public RrtStarNode Steer(RrtStarNode mRrtNearNode, RrtStarNode mRrtRandNode)
        {
            RrtStarNode mRrtNewNode = new RrtStarNode();
            if (DistanceBetweenTwoNode(mRrtNearNode, mRrtRandNode) <= MRrtParameter.Step)
            {
                mRrtNewNode = mRrtRandNode;
            }
            else
            {
                //生成新点
                mRrtNewNode = NewNode(MRrtParameter.Step, mRrtNearNode, mRrtRandNode);
            }
            return mRrtNewNode;
        }


        /// <summary>
        /// 计算新的节点
        /// </summary>
        /// <param name="fStep">步长</param>
        /// <param name="mRrtNearNode">距离mRRTRandNode最近的节点</param>
        /// <param name="mRrtRandNode">随机生成的节点</param>
        /// <returns></returns>
        protected RrtStarNode NewNode(double fStep, RrtStarNode mRrtNearNode, RrtStarNode mRrtRandNode)
        {
            //Alex 2018.09.18 修复错误
            //随便吐槽
            //不知道和妹子还能在一起多久，不过还是有点心塞吧。
            RrtStarNode mRrtNewNode = new RrtStarNode();

            mRrtNewNode.NodeLocation.X = mRrtNearNode.NodeLocation.X + (fStep * (mRrtRandNode.NodeLocation.X - mRrtNearNode.NodeLocation.X)) / DistanceBetweenTwoNode(mRrtNearNode, mRrtRandNode);
            mRrtNewNode.NodeLocation.Y = mRrtNearNode.NodeLocation.Y + (fStep * (mRrtRandNode.NodeLocation.Y - mRrtNearNode.NodeLocation.Y)) / DistanceBetweenTwoNode(mRrtNearNode, mRrtRandNode);
            mRrtNewNode.NodeLocation.Z = _mInput.Scenario.UAVGroup.GetUAV(_mInput.UAVTask[ITaskIndex].Index).StartState.PointLocation.Z; //m_RRTParameter.MissionAltitude;

            //DirectionGenerationMethod(mRRTRandNode, mRRTNearNode, mRRTNewNode, fStep);


            return mRrtNewNode;
        }

        /// <summary>
        /// 计算两个节点之间的距离
        /// </summary>
        /// <param name="mNode1">节点1</param>
        /// <param name="mNode2">节点2</param> 
        /// <returns></returns>
        public double DistanceBetweenTwoNode(RrtStarNode mNode1, RrtStarNode mNode2)
        {
            //二维算法使用
            return FPoint3.DistanceBetweenTwoSpacePointsXY(mNode1.NodeLocation, mNode2.NodeLocation);
            //三维算法使用
            //return FPoint3.DistanceBetweenTwoSpacePoints(mNode1.NodeLocation, mNode2.NodeLocation);
        }

        /// <summary>
        /// 扩展树算法
        /// </summary>
        /// <param name="mRrtStarTree"></param>
        /// <param name="mRrtNearNode"></param>
        /// <param name="mRrtNewSteerNode"></param>
        public void ExpandTree(ref HashSet<RrtStarNode> mRrtStarTree, RrtStarNode mRrtNearNode, RrtStarNode mRrtNewSteerNode, int index)
        {
            //设置是否到达目标的标志
            //bool isReachTarget = false;
            //保存Steer节点信息
            if (mRrtNewSteerNode != null)
            {
                //设置节点编号
                mRrtNewSteerNode.NodeIndex = index;
                //设置父节点
                mRrtNewSteerNode.ParentNode = mRrtNearNode;
                //增加新节点
                mRrtStarTree.Add(mRrtNewSteerNode);
            }
            //创建Near集合
            HashSet<RrtStarNode> nearNodes = NearNeighbors(ref mRrtStarTree, mRrtNewSteerNode);
            RrtStarNode minCostNode = mRrtNearNode;
            double minCost = minCostNode.CostFuncValue + MLineCostFunc(minCostNode, mRrtNewSteerNode);
            //记录初始化父节点
            mRrtNewSteerNode.ParentNode = minCostNode;
            //记录初始化最优minCost参数
            mRrtNewSteerNode.CostFuncValue = minCost;

            //寻找安全的Cost函数最小值
            foreach (var node in nearNodes)
            {
                if (_isSafeLine(mRrtNewSteerNode.NodeLocation, node.NodeLocation) && _isSafePoint(mRrtNewSteerNode.NodeLocation)
                    && node.CostFuncValue + MLineCostFunc(node, mRrtNewSteerNode) < minCost)
                {
                    minCostNode = node;
                    minCost = node.CostFuncValue + MLineCostFunc(node, mRrtNewSteerNode);
                    //记录父节点
                    mRrtNewSteerNode.ParentNode = minCostNode;
                    //记录临时最优minCost参数
                    mRrtNewSteerNode.CostFuncValue = minCost;

                }
            }

            //更新父节点的childlist
            mRrtNewSteerNode.ParentNode.ChildNodes.Add(mRrtNewSteerNode);
            //Rewrite the tree!
            foreach (var node in nearNodes)
            {
                if (_isSafeLine(mRrtNewSteerNode.NodeLocation, node.NodeLocation) && _isSafePoint(mRrtNewSteerNode.NodeLocation)
                    && mRrtNewSteerNode.CostFuncValue + MLineCostFunc(mRrtNewSteerNode, node) < node.CostFuncValue)
                {   //新创立的节点可能会影响tree的最优性，所以需要重写局部的连接情况

                    //去掉与原父节点的联系
                    node.ParentNode.ChildNodes.Remove(node);
                    //添加与新父节点的联系
                    node.ParentNode = mRrtNewSteerNode;
                    mRrtNewSteerNode.ChildNodes.Add(node);
                    //更新字节点的costfunc
                    UpdateChildnodeCostFunc(node);
                }

            }

        }

        /// <summary>
        /// 深度搜索更新Childnode 的costfunc
        /// </summary>
        /// <param name="node"></param>
        protected void UpdateChildnodeCostFunc(RrtStarNode node)
        {
            //不论如何都要更新的事情
            if (node.ParentNode != null)
            {
                node.CostFuncValue = node.ParentNode.CostFuncValue + MLineCostFunc(node.ParentNode, node);
            }

            //检查子类是否需要更新
            if (node.ChildNodes == null)
                return;
            else if (node.ChildNodes.Count == 0)
            {
                return;
            }
            else if (node.ChildNodes.Count > 0)
            {
                foreach (var childnode in node.ChildNodes)
                {
                    UpdateChildnodeCostFunc(childnode);
                }
            }

        }

        /// <summary>
        /// 判断新结点到目标连线是否安全, 若安全则直接连接目标点
        /// </summary>
        /// <param name="iTaskIndex">无人机/任务编号</param>
        /// <param name="iStageIndex">阶段编号</param>
        /// <param name="mRrtCurrentNode">当前节点</param>
        /// <param name="mRrtTree">RRT</param>
        /// <returns>1,是否有效(安全); 2,如果有效(安全),是否到达目标</returns>
        public bool DirectlyToTarget(RrtStarNode mRrtCurrentNode, ref HashSet<RrtStarNode> mRrtTree)
        {

            bool isReachTarget = false;
            //进一步判断新结点到目标连线是否安全, 若安全则直接连接目标点
            if (_isSafeLine(mRrtCurrentNode.NodeLocation, _mInput.UAVTask[ITaskIndex].Stages[IStageIndex].TargetState.Location))
            {
                //设置为到达
                isReachTarget = true;
                //设置目标点为最后一个新结点, 并将当前节点设置为她的父节点
                mRrtCurrentNode = RrtStarNode.ConvertUavStateToNode(_mInput.UAVTask[ITaskIndex].Stages[IStageIndex].TargetState, mRrtCurrentNode);
                //设节点编号
                mRrtCurrentNode.NodeIndex = mRrtTree.Count;
                //增加新节点
                mRrtTree.Add(mRrtCurrentNode);
            }

            return isReachTarget;
        }

        /// <summary>
        /// 寻找SteerNode 的附近节点
        /// </summary>
        /// <param name="mRrtStarTrees"></param>
        /// <param name="steerNode"></param>
        /// <returns></returns>
        public HashSet<RrtStarNode> NearNeighbors(ref HashSet<RrtStarNode> mRrtStarTrees, RrtStarNode steerNode)
        {
            double Tolerance = 10E-7;
            //原始算法测试结果
            //double radiusConst = m_RRTParameter.Step*1.1;
            //double radius = Math.Max(m_RRTParameter.FixedNearDistanceRatio * Math.Pow(Math.Log(2, mRRTStarTrees.Count) / mRRTStarTrees.Count, 1.0 / 2), radiusConst);

            double radius = MRrtParameter.Step * MRrtParameter.FixedNearDistanceRatio;

            HashSet<RrtStarNode> resultSet = new HashSet<RrtStarNode>();
            resultSet.Clear();
            /*
            //并行操作优化
            //貌似会慢1S左右。。。。。
            ResultSet = mRRTStarTrees.AsParallel().Where(node => DistanceBetweenTwoNode(node, SteerNode) <= radius && node != SteerNode).ToHashSet();
            */

            foreach (var node in mRrtStarTrees)
            {
                if (DistanceBetweenTwoNode(node, steerNode) <= radius && node != steerNode)
                    resultSet.Add(node);
            }
            

            if (resultSet.Count > MRrtParameter.TriggerNum && MRrtParameter.NearNodesFilter)
            {

                var query = resultSet.GroupBy(e => e.ParentNode);
                var tmpResultSet = new HashSet<RrtStarNode>();
                foreach (var group in query)
                {
                    var minDis = group.Min(e => e.CostFuncValue + DistanceBetweenTwoNode(e, steerNode));
                    var minNode = group.First(e => Math.Abs((e.CostFuncValue + DistanceBetweenTwoNode(e, steerNode)) - minDis) <= Tolerance);
                    tmpResultSet.Add(minNode);

                }

//                var tmpResultSet = new HashSet<RrtStarNode>();
//                foreach (var item in resultSet)
//                {
//                    if(item.ParentNode != null)
//                        tmpResultSet.Add(item.ParentNode);
//                }
//                if(tmpResultSet.Count != 0)
//                    resultSet = tmpResultSet;

                
            }

            return resultSet;
        }


        private RrtStarNode BiasRandomConfiguration()
        {
            //新节点
            RrtStarNode mRrtNode = null;

            //是否直接把目标点当作新点
            if (_mRandom.NextDouble() > MRrtParameter.ChooseTargetThreshold)
            {
                mRrtNode = RandomConfigurationOrigin();
            }
            else
            {
                mRrtNode = RrtStarNode.ConvertUavStateToNode(_mInput.UAVTask[ITaskIndex].Stages[IStageIndex].TargetState);
            }

            //
            return mRrtNode;
        }


        public void BranchAndBound(ref HashSet<RrtStarNode> mFinalNodeSet, ref HashSet<RrtStarNode> mRrtStarTree)
        {
            var targetNode = RrtStarNode.ConvertUavStateToNode(_mInput.UAVTask[ITaskIndex].Stages[IStageIndex].TargetState);
            var minCost = mFinalNodeSet.Min(e => e.CostFuncValue + MLineCostFunc(e, targetNode));
            RrtStarNode mOptimalNode = mFinalNodeSet.FirstOrDefault(e => e.CostFuncValue + MLineCostFunc(e, targetNode) == minCost);

            if (mOptimalNode == null)   //default
                return;

            //foreach(var item in mRRTStarTree.Where(e => e.IsValid == true)) //寻找mFinalNodeSet相同内容
            foreach(var item in mFinalNodeSet)
            {
                if (item.NodeIndex == mOptimalNode.NodeIndex)
                    continue;
                //跳过最优值，剩下的都删了

                //TODO 
                //Step 1. Find out the GGGGGrandParent that should be abandoned.

                var topNodeParent = item;
                RrtStarNode topNode = item; //TopNode的子节点, item 一定是 abandoned 的节点。
                while(topNodeParent.CostFuncValue + MLineCostFunc(topNodeParent, targetNode) > minCost)
                {
                    if (topNodeParent.ParentNode != null)
                    {
                        topNode = topNodeParent;
                        topNodeParent = topNodeParent.ParentNode;
                    }
                    else
                        break;
                }

                //Step 2. Delete ChildNode reference
                if (topNode.ParentNode != null)
                    topNode.ParentNode.ChildNodes.Remove(topNode);

                //Step 3. Set All child as abandoned
                SetDeleted(topNode);
            }

            //Step Finally. Delete! Them! All!
            mRrtStarTree.RemoveWhere(e => e.IsAbandoned);

            mFinalNodeSet.Clear();
            mFinalNodeSet.Add(mOptimalNode);
        }



        private void SetDeleted( RrtStarNode topNode)
        {
            topNode.IsAbandoned = true;
            foreach (var item in topNode.ChildNodes)
                SetDeleted( item);
        }


        public void MarkingValid(RrtStarNode mRrtCurrentNode, ref HashSet<RrtStarNode> finalNodeSet)
        {
            var targetNode = RrtStarNode.ConvertUavStateToNode(_mInput.UAVTask[ITaskIndex].Stages[IStageIndex].TargetState);

            //路径连通而且距离不长
            if (_isSafeLine(mRrtCurrentNode.NodeLocation, targetNode.NodeLocation))
            //&& DistanceBetweenTwoNode(mRRTCurrentNode, TargetNode) <= RRTParameter.Error)
            {
                //打上 Valid标记（gradually模式用）
                mRrtCurrentNode.IsValid = true;
                finalNodeSet.Add(mRrtCurrentNode);
            }
        }

        public void ConstructFailedPath(ref HashSet<RrtStarNode> mRrtTree)
        {
            //构建当前阶段航路
            mRrtTree = new HashSet<RrtStarNode>();
            mRrtTree.Add(new RrtStarNode(0, _mInput.UAVTask[ITaskIndex].Stages[IStageIndex].StartState.Location, 0, null));
            mRrtTree.Add(new RrtStarNode(0, _mInput.UAVTask[ITaskIndex].Stages[IStageIndex].TargetState.Location, 0, mRrtTree.Last()));
        }

        public void GraduallyToTarget(int count, ref HashSet<RrtStarNode> mRrtTree)
        {
            //当count < MaxNodeNumber 的木一个值时有可能之前被 continue掉，所以不能这么写
            //所以应该取count == m_RRTParameter.MaxNodeNumber并在最终异常部分处理
            //判断mRRTCurrentNode是不是有效的.

            var targetNode = RrtStarNode.ConvertUavStateToNode(_mInput.UAVTask[ITaskIndex].Stages[IStageIndex].TargetState);
            var finalCost = mRrtTree.Where(e => e.IsValid).Min(e => e.CostFuncValue + MLineCostFunc(e, targetNode));
            RrtStarNode finalNode = mRrtTree.FirstOrDefault(e => e.CostFuncValue + MLineCostFunc(e, targetNode) == finalCost);
            
/*
            var TargetNode = RRTStarNode.ConvertUAVStateToNode(m_Input.UAVTask[iTaskIndex].Stages[iStageIndex].TargetState);
            RRTStarNode FinalNode = null;
            double FinalCost = double.MaxValue;
            //寻找最短路径
            foreach (var tmpNode in mRRTTree)
            {
                if (tmpNode.IsValid)
                {
                    var tmpCost = tmpNode.CostFuncValue + mLineCostFunc(tmpNode, TargetNode);
                    if (tmpCost < FinalCost)
                    {
                        FinalCost = tmpCost;
                        FinalNode = tmpNode;
                    }
                }
            }*/

            if (finalNode == null)
            {
                ConstructFailedPath(ref mRrtTree);

            }
            else
            {
                //设置目标点为最后一个新结点, 并将当前节点设置为她的父节点
                var mRrtFinalNode = RrtStarNode.ConvertUavStateToNode(_mInput.UAVTask[ITaskIndex].Stages[IStageIndex].TargetState, finalNode);
                //设节点编号
                mRrtFinalNode.NodeIndex = count;
                //增加新节点
                mRrtTree.Add(mRrtFinalNode);
            }
        }
    }
}
