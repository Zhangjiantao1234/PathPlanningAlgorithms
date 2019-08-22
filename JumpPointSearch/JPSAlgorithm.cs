using PlanningAlgorithmInterface.AlgorithmInterface;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using PlanningAlgorithmInterface.Define.Output;
using MyGenerics;
using SceneElementDll.Basic;
using PlanningAlgorithmInterface.Define.Input;
using System.Threading;

namespace JumpPointSearch
{
    public class JPSAlgorithm : JPSAlgorithmHelper, ICompletePlanningAlgorithm
    {
        //可视化参数
        JPSVisualization Visualization = new JPSVisualization();

        /// <summary>
        /// 初始化Parameter
        /// </summary>
        public void InitParameter()
        {
            mPara = AlgoParameter as JPSParameter;

            if (mPara.AutoOptimizeParameter == true)
            {
                //自动化参数
            }

        }

        /// <summary>
        /// 构造静态的调用方法
        /// IPlanningAlgorithm接口
        /// </summary>
        /// <param name="iTaskIndex"></param>
        /// <returns></returns>
        public MPath BuildPathForSingleUAVInStatic(int iTaskIndex)
        {
            MPath mPath = new MPath();
            mPath.Waypoints = new List<MWaypoint>();
            mPath.Index = AlgoInput.UAVTask[iTaskIndex].Index;//得到当前无人机编号，就是这条路径的编号
            int iWaypointIndex = 0;  //所有阶段的点的集合的索引

            for (int iStageIndex = 0; iStageIndex < AlgoInput.UAVTask[iTaskIndex].Stages.Count; iStageIndex++)
            {

                //在每个阶段都先获取起始点和目标点，并进行处理
                var start = AlgoInput.UAVTask[iTaskIndex].Stages[iStageIndex].StartState.Location;
                var goal = AlgoInput.UAVTask[iTaskIndex].Stages[iStageIndex].TargetState.Location;
                HeursticInfo.StartNode = new Node(GetRegulatedPos(start), null);
                HeursticInfo.TargetNode = new Node(GetRegulatedPos(goal), null);

                HashSet<Node> mCloseList = null;
                //获取路径
                var mStagePath = BuildPathForSingleUAVwithSingleStageInStatic(iTaskIndex,out mCloseList);    
                //为可视化输出保存
                Visualization.mPathForVisualizaition = mCloseList;
                if (!PathPlaningDataVisualization.PathPlaningDataVisualization.AlgoDataDict.ContainsKey(mPath.Index.ToString() + "-" + iStageIndex.ToString()))
                {
                    //为ResultShowForm分析窗口保存数据
                    //3.30.2018 刘洋添加
                    PathPlaningDataVisualization.PathPlaningDataVisualization.AlgoDataDict.Add(mPath.Index.ToString() + "-" + iStageIndex.ToString(), Visualization.MyTreeNodeConverter(Visualization.mPathForVisualizaition));
                }
                else
                {
                    PathPlaningDataVisualization.PathPlaningDataVisualization.AlgoDataDict[mPath.Index.ToString() + "-" + iStageIndex.ToString()] = Visualization.MyTreeNodeConverter(Visualization.mPathForVisualizaition);
                }

                if (mStagePath == null)
                {
                    mPath.Waypoints = new List<MWaypoint>() {
                        new MWaypoint(iWaypointIndex++, new Node(start, null).ConvertTreeNodeToUAVState(), iStageIndex),
                        new MWaypoint(iWaypointIndex++, new Node(goal, null).ConvertTreeNodeToUAVState(), iStageIndex) 
                    };
                    return mPath;
                }

                //输出格式处理
                for (int k = 0; k < mStagePath.Count - 1; k++)
                {
                    mPath.Waypoints.Add(new MWaypoint(iWaypointIndex++, mStagePath[k].ConvertTreeNodeToUAVState(), iStageIndex));
                    
                }
                if (iStageIndex == AlgoInput.UAVTask[iTaskIndex].Stages.Count - 1)//如果到了最后一个阶段，不要忘记把最后一个点加进来
                {
                    mPath.Waypoints.Add(new MWaypoint(iWaypointIndex++, mStagePath[mStagePath.Count - 1].ConvertTreeNodeToUAVState(), AlgoInput.UAVTask[iTaskIndex].Stages.Count - 1));
                    //刷一个最后的点，防止不再运行～
                    mPath.Waypoints.Add(new MWaypoint(iWaypointIndex++, new Node(goal, null).ConvertTreeNodeToUAVState(), iStageIndex));
                }
            }

            return mPath;
        }


        /// <summary>
        /// 实时调用方法
        /// ICompletePlanningAlgorithm接口
        /// </summary>
        /// <param name="iTaskIndex"></param>
        /// <param name="mWaypoint"></param>
        /// <param name="mPath"></param>
        /// <returns></returns>
        public MPath BuildPathForSingleUAVForRealTime(int iTaskIndex, MWaypoint mWaypoint, MPath mPath)
        {
            //移除起始航路点之后(含)的所有航路点
            if (mPath.Waypoints != null)
            {
                mPath.Waypoints.RemoveRange(mWaypoint.Index, mPath.Waypoints.Count - mWaypoint.Index);
                //从数据库移除
            }

            //以当前航路点为当前阶段的起始状态点(以后的阶段均不变), 构建新的阶段
            List<MStage> mNewStages = new List<MStage>();
            var mFirstStage = new MStage()
            {
                StageIndex = mWaypoint.StageIndex,
                StartState = new MKeyState(mWaypoint.State),
                TargetState = AlgoInput.UAVTask[iTaskIndex].Stages[mWaypoint.StageIndex].TargetState
            };
            mNewStages.Add(mFirstStage);
            for (int i = mWaypoint.StageIndex + 1; i < AlgoInput.UAVTask[iTaskIndex].Stages.Count; ++i)
            {
                var mTempStage = AlgoInput.UAVTask[iTaskIndex].Stages[i];
                mNewStages.Add(mTempStage);
            }

            //当前航路点编号
            int iWaypointIndex = mWaypoint.Index;
            
            //对每一个阶段规划航路
            //Alex Liu评价：原来的FOR写的真糟糕。。。。。
            //for (int iStageIndex = mNewStages[0].StageIndex; iStageIndex < mNewStages[0].StageIndex + mNewStages.Count; ++iStageIndex)
            foreach(var mStage in mNewStages)
            {
                //在每个阶段都先获取起始点和目标点，并进行处理
                var start = mStage.StartState.Location;
                var goal = mStage.TargetState.Location;
                var iStageIndex = mStage.StageIndex;

                HeursticInfo.StartNode = new Node(GetRegulatedPos(start), null);
                HeursticInfo.TargetNode = new Node(GetRegulatedPos(goal), null);

                HashSet<Node> mCloseList = null;
                //获取路径
                var mStagePath = BuildPathForSingleUAVwithSingleStageInStatic(iTaskIndex, out mCloseList);

                //输出格式处理
                for (int k = 0; k < mStagePath.Count - 1; k++)
                {
                    mPath.Waypoints.Add(new MWaypoint(iWaypointIndex++, mStagePath[k].ConvertTreeNodeToUAVState(), iStageIndex));

                }
                if (iStageIndex == AlgoInput.UAVTask[iTaskIndex].Stages.Count - 1)//如果到了最后一个阶段，不要忘记把最后一个点加进来
                {
                    mPath.Waypoints.Add(new MWaypoint(iWaypointIndex++, mStagePath[mStagePath.Count - 1].ConvertTreeNodeToUAVState(), AlgoInput.UAVTask[iTaskIndex].Stages.Count - 1));
                    //刷一个最后的点，防止不再运行～
                    mPath.Waypoints.Add(new MWaypoint(iWaypointIndex++, new Node(goal, null).ConvertTreeNodeToUAVState(), iStageIndex));
                }
            }

            return mPath;
        }

        #region AlgorithmPart

        private long NodeCounter = 0;

        private JPSParameter mPara = null;

        private void InsertToHeap(Node mNode, ref MinBinaryHeap<Node> OpenList)
        {
            mNode.Index = NodeCounter++;
            OpenList.Insert(mNode);
        }

        private FPoint3 GetRegulatedPos(FPoint3 Pos)
        {
            var step = mPara.Step;
            double x = Math.Round(Pos.X / step) * step;
            double y = Math.Round(Pos.Y / step) * step;
            return new FPoint3(x, y, Pos.Z);
        }

        private List<Node> BuildPathForSingleUAVwithSingleStageInStatic(int iTaskIndex, out HashSet<Node> CloseList)
        {
            //Openlist 与 Closelist 定义
            MinBinaryHeap<Node> OpenList = new MinBinaryHeap<Node>();
            CloseList = new HashSet<Node>();
            //List<Node> CloseList = new List<Node>();
            //算法参数get
            if (mPara == null)//算法参数为空
                return null;

            //刷ID
            InsertToHeap(HeursticInfo.StartNode, ref OpenList);

            while (OpenList.HeapSize != 0)
            {
                var mExtentNode = OpenList.Extract();
                CloseList.Add(mExtentNode);
                var mSuccessors = IdentifySuccessors(mExtentNode);

                foreach (var item in mSuccessors)
                {
                    if (!CloseList.Any(a => item.NodeLocation == a.NodeLocation))
                    {
                        InsertToHeap(item, ref OpenList);
                        if (item.NodeLocation == HeursticInfo.TargetNode.NodeLocation)
                        {
                            return GetPathForSingleUAVwithSingleStage(item);
                        }
                    }
                }
            }
            return null;
        }

        private List<Node> GetPathForSingleUAVwithSingleStage(Node item)
        {
            List<Node> mResultNodeList = new List<Node>();
            var tmpNode = item;

            while (tmpNode.NodeLocation != HeursticInfo.StartNode.NodeLocation)
            {
                mResultNodeList.Add(tmpNode);
                tmpNode = tmpNode.ParentNode;
            }
            mResultNodeList.Add(HeursticInfo.StartNode);

            mResultNodeList.Reverse();

            List<Node> FAPath = new List<Node>();
            if (mPara.NeedPathSimplifed)
            {
                int j = 1;
                int i = 0;
                FAPath.Add(mResultNodeList[0]);
                bool IsSimplify = true;
                while (IsSimplify)
                {
                    if (!IsSafeLine(mResultNodeList[i].NodeLocation, mResultNodeList[j].NodeLocation))
                    {
                        FAPath.Add(mResultNodeList[j - 1]);
                        i = j - 1;
                    }
                    else
                    {
                        if (j == mResultNodeList.Count - 1)
                        {
                            FAPath.Add(mResultNodeList[mResultNodeList.Count - 1]);
                            IsSimplify = false;
                        }
                        else
                        {
                            j++;
                        }
                    }
                }
            }
            else
            {
                FAPath = mResultNodeList;
            }
            return FAPath;
        }

        private List<FPoint3> PruneByDirection(Node mCurrentNode)
        {
            var allNeighbors = AllNeighbors(mCurrentNode);
            var ResultNeighbors = new List<FPoint3>();
            if (mCurrentNode.ParentNode == null)
                return allNeighbors;
            else
            {
                var ParentPos = mCurrentNode.ParentNode.NodeLocation;
                var CurrentPos = mCurrentNode.NodeLocation;
                double step = mPara.Step;

                //CurrentPos-ParrentPos方向不用检查～
                var AcceptableDirection = GetDirection(CurrentPos, ParentPos);

                // search diagonally //说明上一步是搜索的水平的～
                if (AcceptableDirection.X * AcceptableDirection.Y == 0)
                {
                    if (IsSafePoint(CurrentPos + step * AcceptableDirection))
                        ResultNeighbors.Add(CurrentPos + step * AcceptableDirection);

                    //斜着的方向补一下～
                    if (AcceptableDirection.X == 0)
                    {
                        ResultNeighbors.AddRange(new FPoint3[]{
                            CurrentPos + step * new FPoint3(1 , AcceptableDirection.Y, 0),
                            CurrentPos + step * new FPoint3(-1, AcceptableDirection.Y, 0) });
                    }
                    else //(AcceptableDirection.Y == 0)
                    {
                        ResultNeighbors.AddRange(new FPoint3[]{
                            CurrentPos + step * new FPoint3(AcceptableDirection.X ,  1, 0),
                            CurrentPos + step * new FPoint3(AcceptableDirection.X , -1, 0) });
                    }


                }
                else //Search Horizontally and Vertically //说明上一步是搜索斜着的，
                {
                    //所以可以斜着或者横着走
                    if (IsSafePoint(CurrentPos + step * AcceptableDirection))
                        ResultNeighbors.Add(CurrentPos + step * AcceptableDirection);

                    if (IsSafePoint(CurrentPos + step * new FPoint3(0, AcceptableDirection.Y, 0)))
                        ResultNeighbors.Add(CurrentPos + step * new FPoint3(0, AcceptableDirection.Y, 0));

                    if (IsSafePoint(CurrentPos + step * new FPoint3(AcceptableDirection.X, 0, 0)))
                        ResultNeighbors.Add(CurrentPos + step * new FPoint3(AcceptableDirection.X, 0, 0));

                    //在特殊情况下可以90度转弯～
                    if (IsSafePoint(CurrentPos + step * new FPoint3(-AcceptableDirection.Y, AcceptableDirection.X, 0))
                        && !IsSafePoint(CurrentPos + step * new FPoint3(-AcceptableDirection.Y, 0, 0)))
                    {
                        ResultNeighbors.Add(CurrentPos + step * new FPoint3(-AcceptableDirection.Y, AcceptableDirection.X, 0));
                    }
                    if (IsSafePoint(CurrentPos + step * new FPoint3(AcceptableDirection.Y, -AcceptableDirection.X, 0))
                        && !IsSafePoint(CurrentPos + step * new FPoint3(0, -AcceptableDirection.X, 0)))
                    {
                        ResultNeighbors.Add(CurrentPos + step * new FPoint3(AcceptableDirection.Y, -AcceptableDirection.X, 0));
                    }
                }

            }
            return ResultNeighbors;
        }

        private List<Node> IdentifySuccessors(Node mParentNode)
        {
            List<Node> mSuccessors = new List<Node>();
            var TestNodePosList = PruneByDirection(mParentNode); //FindPruneNeighbors(mCurrentNode);

            //Shuffle不会影响算法过程，因为下一轮读取的是COST值最小的节点，只有当有多个最小COST值时才会影响。
            //嗯。。。。。JPS因为踩的点少，所以很少有一样的。。。所以影响不明显。
            //if (TestNodePosList.Count >= 2)
            //    TestNodePosList.Shuffle();
            foreach (var tmpNodePos in TestNodePosList)
            {
                var direction = GetDirection(mParentNode, tmpNodePos);
                Node newNode = Jump(mParentNode, direction);
                if (newNode != null)
                {
                    newNode.ParentNode = mParentNode;
                    mSuccessors.Add(newNode);
                }

            }
            return mSuccessors;
        }

        /// <summary>
        /// 核心剪纸策略
        /// </summary>
        /// <param name="mParentNode">init节点</param>
        /// <param name="direction">原前进方向</param>
        /// <returns></returns>
        private Node Jump(Node mParentNode, FPoint3 direction)
        {
            var size = mPara.Step;
            var TestNode = new Node(mParentNode.NodeLocation + direction * size, mParentNode, HeursticInfo);

            //这个点是废了的
            if (!IsSafePoint(TestNode.NodeLocation) || IsPointOutrange(TestNode.NodeLocation))
                return null;
            if (TestNode.NodeLocation == HeursticInfo.TargetNode.NodeLocation)
                return TestNode;
            foreach (var tmpNeighbor in FindPruneNeighbors4Jump(TestNode))
            {
                if (IsForcedNeighbor(tmpNeighbor, TestNode))    //判断TestNode是不是ForceNeighbor
                {
                    return TestNode;
                    //return new Node(TestNode.NodeLocation, TestNode.ParentNode, HeursticInfo);  
                    //return new Node(tmpNeighbor, TestNode, HeursticInfo);
                }
            }

            if (direction.X * direction.Y != 0)//对角线
            {
                FPoint3[] DirectionList = new FPoint3[2] { new FPoint3(direction.X, 0, 0), new FPoint3(0, direction.Y, 0) };
                foreach (var tmpDir in DirectionList)
                {
                    if (Jump(TestNode, tmpDir) != null)
                        return TestNode;
                }
            }
            return Jump(TestNode, direction);
        }

        private FPoint3 GetDirection(FPoint3 mCurrentPos, FPoint3 mParentPos)
        {
            FPoint3 Direction = mCurrentPos - mParentPos;
            return (new FPoint3(Math.Sign(Direction.X), Math.Sign(Direction.Y), Math.Sign(Direction.Z)));
        }

        private FPoint3 GetDirection(Node mCurrentNode, FPoint3 mNeighborPos)
        {
            FPoint3 Direction = mNeighborPos - mCurrentNode.NodeLocation;
            return (new FPoint3(Math.Sign(Direction.X), Math.Sign(Direction.Y), Math.Sign(Direction.Z)));
        }

        private List<FPoint3> AllNeighbors(Node mCurrentNode)
        {
            var ResultNeighbors = new List<FPoint3>();
            var step = mPara.Step;

            List<FPoint3> Directions = new List<FPoint3>
            {   new FPoint3(0, 1, 0), new FPoint3(1, 0, 0),
                new FPoint3(1, 1, 0), new FPoint3(0, -1, 0),
                new FPoint3(-1, 0, 0), new FPoint3(-1, -1, 0),
                new FPoint3(-1, 1, 0), new FPoint3(1, -1, 0) };     //可以加入缺陷

            foreach (var tmp in Directions)
            {
                if (IsSafePoint(mCurrentNode.NodeLocation + tmp * step))
                    ResultNeighbors.Add(mCurrentNode.NodeLocation + tmp * step);
            }

            return ResultNeighbors;
        }

        private List<FPoint3> FindPruneNeighbors4Jump(Node mCurrentNode)
        {
            Node mParent = mCurrentNode.ParentNode;
            double step = mPara.Step;
            List<FPoint3> ResultNeighbors = new List<FPoint3>();

            // directed pruning: can ignore most neighbors, unless forced.
            if (mParent != null)
            {
                var ParentPos = mParent.NodeLocation;
                var CurrentPos = mCurrentNode.NodeLocation;

                //CurrentPos-ParrentPos方向不用检查～
                var AcceptableDirection = GetDirection(CurrentPos, ParentPos);

                // search diagonally //说明上一步是搜索的水平的～
                if (AcceptableDirection.X * AcceptableDirection.Y != 0)
                {
                    if (IsSafePoint(CurrentPos + step * AcceptableDirection))
                        ResultNeighbors.Add(CurrentPos + step * AcceptableDirection);
                    if (IsSafePoint(CurrentPos + step * new FPoint3(0, AcceptableDirection.Y, 0)))
                        ResultNeighbors.Add(CurrentPos + step * new FPoint3(0, AcceptableDirection.Y, 0));
                    if (IsSafePoint(CurrentPos + step * new FPoint3(AcceptableDirection.X, 0, 0)))
                        ResultNeighbors.Add(CurrentPos + step * new FPoint3(AcceptableDirection.X, 0, 0));
                }
                else //Search Horizontally and Vertically //说明上一步是搜索斜着的
                {
                    if (IsSafePoint(CurrentPos + step * AcceptableDirection))
                        ResultNeighbors.Add(CurrentPos + step * AcceptableDirection);
                }
            }
            // return all neighbors
            else
            {
                ResultNeighbors = AllNeighbors(mCurrentNode);
            }

            return ResultNeighbors;
        }

        private bool IsForcedNeighbor(FPoint3 mNeighborPos, Node mTestNode)
        {
            //参数拿mNeighborPos是为了算离开mCurrentNode的方向
            var step = mPara.Step;
            var direction = GetDirection(mTestNode, mNeighborPos) * step;
            var mTestPos = mTestNode.NodeLocation;

            // check for forced neighbors
            // along the diagonal
            if (direction.X * direction.Y != 0)
            {
                if (IsSafePoint(mTestPos + new FPoint3(-direction.X, direction.Y, 0)) && !IsSafePoint(mTestPos + new FPoint3(direction.X, -direction.Y, 0)) ||
                    IsSafePoint(mTestPos + new FPoint3(direction.X, -direction.Y, 0)) && !IsSafePoint(mTestPos + new FPoint3(-direction.X, direction.Y, 0)))
                {
                    return true;
                }
            }
            // horizontally/vertically
            else
            {
                if (direction.X != 0)
                {
                    if (IsSafePoint(mTestPos + new FPoint3(direction.X, step, 0)) && !IsSafePoint(mTestPos + new FPoint3(0, step, 0)) ||
                        IsSafePoint(mTestPos + new FPoint3(direction.X, -step, 0)) && !IsSafePoint(mTestPos + new FPoint3(0, -step, 0)))
                    {
                        return true;
                    }
                }
                else //direction.y !=0
                {
                    if (IsSafePoint(mTestPos + new FPoint3(step, direction.Y, 0)) && !IsSafePoint(mTestPos + new FPoint3(step, 0, 0)) ||
                        IsSafePoint(mTestPos + new FPoint3(-step, direction.Y, 0)) && !IsSafePoint(mTestPos + new FPoint3(-step, 0, 0)))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        #endregion
    }

    #region ShuffleExtensions
 
    /// <summary>
    /// Shuffle算法
    /// </summary>
    static class MyExtensions
    {
        private static Random rng = new Random();
        public static void Shuffle<T>(this IList<T> list)
        {
            int n = list.Count;
            while (n > 1)
            {
                n--;
                int k = rng.Next(n + 1);
                T value = list[k];
                list[k] = list[n];
                list[n] = value;
            }
        }
    }
    #endregion


}
