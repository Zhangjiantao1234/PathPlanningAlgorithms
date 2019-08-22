using PlanningAlgorithmInterface.AlgorithmInterface;
using PlanningAlgorithmInterface.Define.Output;
using SceneElementDll.Basic;
using System;
using System.Collections.Generic;

namespace LStar
{
    public partial class LStarAlgorithm : LStarAlgorithmHelper, IPlanningAlgorithm
    {
        private LStarVisualization _visualization;
        private LStarParameter _para;
        private long _nodeCounter;
        private double _minValue;
        private double _maxValue;
        private double _valueInit;  //论文中的f0
        private double _df;

        /// <summary>
        /// 初始化Parameter
        /// </summary>
        public void InitParameter()
        {
            _para = AlgoParameter as LStarParameter;
            _visualization = new LStarVisualization();
            if (_para.AutoOptimizeParameter == true)
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
                //第一次赋值给 HeursticInfo 刷入坐标
                HeursticInfo.StartNode = new Node(GetRegulatedPos(start), null);
                HeursticInfo.TargetNode = new Node(GetRegulatedPos(goal), null);
                //第二次赋值给 HeursticInfo 刷入启发式参数(第一次直接刷启发式的话起点终点坐标不对)
                HeursticInfo.StartNode = new Node(GetRegulatedPos(start), null, HeursticInfo);
                HeursticInfo.TargetNode = new Node(GetRegulatedPos(goal), null, HeursticInfo);

                //获取路径
                var mStagePath = BuildPathForSingleUavStageInStatic(iTaskIndex, out HashSet<Node> mCloseList);

                //为可视化输出保存
                _visualization.mPathForVisualizaition = mCloseList;
                if (!PathPlaningDataVisualization.PathPlaningDataVisualization.AlgoDataDict.ContainsKey(mPath.Index.ToString() + "-" + iStageIndex.ToString()))
                {
                    //为ResultShowForm分析窗口保存数据
                    PathPlaningDataVisualization.PathPlaningDataVisualization.AlgoDataDict.Add(mPath.Index.ToString() + "-" + iStageIndex.ToString(), _visualization.MyTreeNodeConverter(_visualization.mPathForVisualizaition));
                }
                else
                {
                    PathPlaningDataVisualization.PathPlaningDataVisualization.AlgoDataDict[mPath.Index.ToString() + "-" + iStageIndex.ToString()] = _visualization.MyTreeNodeConverter(_visualization.mPathForVisualizaition);
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


        private FPoint3 GetRegulatedPos(FPoint3 Pos)
        {
            var step = _para.Step;
            double x = Math.Round(Pos.X / step) * step;
            double y = Math.Round(Pos.Y / step) * step;
            return new FPoint3(x, y, Pos.Z);
        }

        private void InitOpenList()
        {
            //如果是面对一个图，而不是走格子。那么需要遍历这个图
            //反之就很简单啦
            //_para != null

            _minValue = _para.Step;
            _maxValue = _para.Step * Math.Sqrt(2);
            Node.Weight = _para.Weight;
            _valueInit = HeursticInfo.StartNode.CostFunc;
            _df = (1 - _para.Weight) * _minValue;
        }

        private List<Node> GetPath(Node item)
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
            if (_para.NeedPathSimplifed)
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
    }
}
