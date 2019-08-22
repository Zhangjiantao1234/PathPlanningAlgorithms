using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using PlanningAlgorithmInterface.AlgorithmInterface;
using PlanningAlgorithmInterface.Define.Input;
using SceneElementDll.Basic;
using PlanningAlgorithmInterface.Define.Output;
using SceneElementDll.UAV;
namespace AStarOrigin
{
    /// <summary>
    /// A*算法核心步骤
    /// </summary>
    public class AStarOriginAlgorithmBase
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

        private List<Node> BuildPathForSingleUAVwithSingleStageInStatic(int iTaskIndex, int iStageIndex, AStarOriginAlgorithmParameter mPara, out List<Node> openlist)
        {
            #region 定义并实例化集合
            int toExtendNum = -1;//定义跳转到最小代价点在openList中的位置
            bool isReachTarget = false;   //是否到达目标 - 初始未到达，循环结束标志
            openlist = new List<Node>();//我觉得可以这样定义Open集和close集,这里是个局部变量
            List<Node> APath = new List<Node>();//按照算法寻找的点，比较多
            List<Node> FAPath = new List<Node>();//最终的路线，把Apath中的一部分不必要的点过滤掉
            #endregion

            #region 在每个阶段都先获取起始点和目标点，并进行处理
            var start = AlgoInput.UAVTask[iTaskIndex].Stages[iStageIndex].StartState.Location;
            var goal = AlgoInput.UAVTask[iTaskIndex].Stages[iStageIndex].TargetState.Location;
            Node startnode = new Node(start, goal, 0, null);//初始化开始节点
            #endregion

          /*  if (IsSafeLine(start, goal))//如果当前阶段起点与终点连线安全，那么最终的路线就是这俩点啊
            {
                FAPath.Add(startnode);
                Node finalnode = new Node(start, goal, goal, startnode);//初始化开始节点
                FAPath.Add(finalnode);
            }*/
            //else
           // {
                #region   从Open集中不断找寻代价最低的节点，直到找到目标点为止,并得到A*的路线
                openlist.Add(startnode); //将开始节点加到Open表,开始了

                while (!isReachTarget)
                {
                    toExtendNum = FindMin(openlist);

                    if (toExtendNum != -1)//如果找到了最小代价点，即Open集不为空，问题来了，如果Open集为空集怎么办
                    {
                        openlist[toExtendNum].sign = true;//加到Close集合中
                        if (IsSafeLine(openlist[toExtendNum].NodeLocation, goal) &&
                            FPoint3.DistanceBetweenTwoSpacePointsXY(openlist[toExtendNum].NodeLocation, goal) <= mPara.Step * Math.Sqrt(2))
                        //如果当前找到的点与目标点连线安全，就可以直接加入目标点了
                        {
                            Node goalnode = new Node(start, goal, goal, openlist[toExtendNum]);//将goal连接到了Close表中了
                            isReachTarget = true;
                            APath.Add(goalnode);//加入目标节点
                            Node temp = new Node();
                            temp = goalnode.ParentNode;
                            while (temp.ParentNode != null)
                            {
                                APath.Add(temp);
                                Node Anode = new Node();
                                Anode = temp;
                                temp = Anode.ParentNode;
                            }
                            APath.Add(startnode);//加入开始节点，至此路径中的点找齐了
                            APath.Reverse();//将序列翻转变为正序
                        }
                        else
                        {
                            openlist = FindAroundPoint(openlist[toExtendNum], openlist, start, goal);
                        }
                    }
                    else
                    {
                    APath = new List<Node>();
                    APath.Add(startnode);
                    Node finalnode = new Node(start, goal, goal, startnode);//初始化开始节点
                    APath.Add(finalnode);
                    return APath;
                    }
                }
                #endregion
                #region 计算最终简化A*的路线
                if (mPara.NeedPathSimplifed)
                {
                    int j = 0;
                    int i = APath.Count - 1;
                    FAPath.Add(startnode);
                    bool IsSimplify = true;
                    while (IsSimplify)
                    {
                        if (IsSafeLine(APath[j].NodeLocation, APath[i].NodeLocation))
                        {
                            if (i == APath.Count - 1)
                            {
                                IsSimplify = false;
                            }
                            FAPath.Add(APath[i]);
                            j = i;
                            i = APath.Count - 1;
                        }
                        else
                        {
                            i--;
                        }
                    }
                }
                else
                {
                   /* int j = 1;
                    int i = 0;
                    FAPath.Add(startnode);
                    bool IsSimplify = true;
                    while (IsSimplify)
                    {
                        if (!IsSafeLine(APath[i].NodeLocation, APath[j].NodeLocation))
                        {
                            FAPath.Add(APath[j-1]);
                            i= j-1; 
                        }
                        else
                        {
                            if (j == APath.Count - 1)
                            {
                                FAPath.Add(APath[APath.Count - 1]);
                                IsSimplify = false;
                            }
                            else
                            {
                                j++;
                            }
                        }
                    }*/

                    FAPath = APath;
                }
                #endregion
           // }
            return FAPath;
        }

        /// <summary>
        /// 可视化数据
        /// </summary>
        AStarVisualization Visualization = new AStarVisualization();

        /// <summary>
        /// 主要算法函数，求最后的所有阶段的路径集合
        /// </summary>
        /// <param name="iTaskIndex"></param>
        /// <returns></returns>
        protected MPath BuildPathForSingleUAV(int iTaskIndex)
        {

            //算法参数
            AStarOriginAlgorithmParameter mPara = AlgoParameter as AStarOriginAlgorithmParameter;

            #region 定义变量，需要再添加
            MPath mPath = new MPath();//初始化一条路径，最后得到的就是这个类中的一个list存放的路径序列
            mPath.Index = AlgoInput.UAVTask[iTaskIndex].Index;//得到当前无人机编号，就是这条路径的编号
            mPath.Waypoints = new List<MWaypoint>();
            int iWaypointIndex = 0;  //所有阶段的点的集合的索引
            #endregion

            #region ----------------------------对每一个阶段规划航路------------------------------------------
            for (int iStageIndex = 0; iStageIndex < AlgoInput.UAVTask[iTaskIndex].Stages.Count; iStageIndex++)
            {
                List<Node> openlist = null;
                var FAPath = BuildPathForSingleUAVwithSingleStageInStatic(iTaskIndex, iStageIndex, mPara, out openlist);
                
                #region  航迹可视化处理
                //为可视化输出保存
                Visualization.mPathForVisualizaition = openlist.Where(a => a.sign == true).ToList();
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
                #endregion

                #region 航迹保存
                //添加当前阶段航路到总航路(起始航路点为当前阶段, 此阶段的目标航路点为下一个阶段的起始航路点, 注意总航路的最后一个航路点属于最后一个阶段)
                for (int k = 0; k < FAPath.Count - 1; k++)
                {
                    mPath.Waypoints.Add(new MWaypoint(iWaypointIndex, FAPath[k].ConvertTreeNodeToUAVState(), iStageIndex));
                    iWaypointIndex = iWaypointIndex + 1;
                }
                if (iStageIndex == AlgoInput.UAVTask[iTaskIndex].Stages.Count - 1)//如果到了最后一个阶段，不要忘记把最后一个点加进来
                {
                    mPath.Waypoints.Add(new MWaypoint(iWaypointIndex, FAPath[FAPath.Count - 1].ConvertTreeNodeToUAVState(), AlgoInput.UAVTask[iTaskIndex].Stages.Count - 1));
                }
                #endregion

            }
            #endregion
            return mPath;
        }
        
        /// <summary>
        /// 一个函数，求Open集合中代价的最小值的位置
        /// </summary>
        /// <param name="Open">输入一个集合</param>
        /// <returns></returns>
        private int FindMin(List<Node> Open)
        {
            int toExtendNum = -1;
            double min_cost = double.MaxValue;
            for (int i = 0; i < Open.Count; i++)
            {
                if (Open[i].sign == false)
                {
                    double temp_cost = Open[i].ComputeCostForAStar;
                    if (temp_cost < min_cost)//(注意改成小于会怎么样)
                    {
                        min_cost = temp_cost;
                        toExtendNum = i;
                    }
                }
            }
            //       toExtendNum = -1;
            return toExtendNum;
        }
        
        /// <summary>
        /// 一个函数，求当前节点的周围的八个安全点，并放到Open列表中,核心函数
        /// </summary>
        /// <param name="Current">当前节点</param>
        /// <param name="Open">Open集</param>
        /// <param name="start">当前阶段起始点</param>
        /// <param name="goal">当前阶段终点</param>
        /// <returns></returns>
        private List<Node> FindAroundPoint(Node Current, List<Node> Open, FPoint3 start, FPoint3 goal)
        {
            double step = (AlgoParameter as AStarOriginAlgorithmParameter).Step;
            //  double step = 1;
            //获取坐标系x轴上下限
            double minX = AlgoInput.Scenario.FlightScene.Coordinate.MinX;
            double maxX = AlgoInput.Scenario.FlightScene.Coordinate.MaxX;
            //获取坐标系Y轴上下限
            double minY = AlgoInput.Scenario.FlightScene.Coordinate.MinY;
            double maxY = AlgoInput.Scenario.FlightScene.Coordinate.MaxY;
            FPoint3[] grid = new FPoint3[8];
            grid[0] = new FPoint3(step, step,0);//右上
            grid[1] = new FPoint3(0, step,0);//上
            grid[2] = new FPoint3(-step, step,0);//左上
            grid[3] = new FPoint3(-step, 0,0);//左
            grid[4] = new FPoint3(-step, -step,0);//左下
            grid[5] = new FPoint3(0, -step,0);//下
            grid[6] = new FPoint3(step, -step,0);//右下
            grid[7] = new FPoint3(step, 0,0);//右
            for (int i = 0; i < 8; i++)
            {
                FPoint3 newpoint = new FPoint3();//定义当前节点坐标
                //newpoint.X = Current.NodeLocation.X + grid[i].X;
                //newpoint.Y = Current.NodeLocation.Y + grid[i].Y;
                //newpoint.Z = Current.NodeLocation.Z;
                newpoint = Current.NodeLocation + grid[i];

                if (IsSafeLine(Current.NodeLocation, newpoint) && IsSafePoint(newpoint) == true && newpoint.X < maxX && newpoint.X > minX && newpoint.Y < maxY && newpoint.Y > minY)//如果此点安全且在范围内，就可以进行了
                {
                    bool find = false;//是否在点集中找到的标志位
                    for (int j = 0; j < Open.Count; j++)//先在Open表中进行for循环，看能不能找到这个点
                    {
                        if (Open[j].NodeLocation == newpoint)//直到从集合中找到这个点为止
                        {
                            find = true;
                            //if (Open[j].sign == false)      //表示这个点还在Open表中
                            //{
                            double distancetoroot = 1; //FPoint2.DistanceBetweenTwoPlanePoints(newpoint, Current.NodeLocation);//缺陷2
                                if ((Current.CostfromStart + distancetoroot) < Open[j].CostfromStart)//
                                {
                                    Open[j].ParentNode = Current;//更新父节点  //缺陷1
                                    Open[j].CostfromStart = Current.CostfromStart + distancetoroot; 
                                    Open[j].computeCostForAStar = Open[j].DistancetoGoal + Open[j].CostfromStart;//Current.CostfromStart + distancetoroot;//更新代价函数        
                                    if (Open[j].sign == true)
                                    {
                                        Open[j].sign = false;
                                    }
                                }                                        
                            break;//既然已经找到这个点，就没有必要找下去了，跳出第一个for循环
                        }
                    }//----------------------end for （j）-----------------------------------------//
                    if (find == false)//如果j到了这个程度，还没跳出这个for循环，说明找到的这个节点根本不在Open表中啊，可以直接加进去了
                    {
                        Node newNode = new Node(start, newpoint, goal, Current);//如果此点是安全的，而且在范围内，就初始化为一个节点
                        Open.Add(newNode);
                    }
                }
            }//------------------------------------------------end for(i)-----------------------------------------------//
            return Open;
        }
    }

}
