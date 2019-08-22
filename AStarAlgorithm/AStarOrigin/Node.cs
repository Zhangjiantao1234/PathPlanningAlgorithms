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
    /// 在地图上每个节点应该带有的属性
    /// </summary>
    class Node :IComparable
    {
        /// <summary>
        /// Open集与Close集的标志
        /// </summary>
        public bool sign { get; set; }

        /// <summary>
        /// 一个节点的坐标
        /// </summary>
        public FPoint3 NodeLocation { get; set; }

        /// <summary>
        /// 父节点，类型和他一样
        /// </summary>
        public Node ParentNode { get; set; }

        /// <summary>
        /// 当前节点与开始节点的距离（花费）
        /// </summary>
        public double CostfromStart { get; set; }
        /// <summary>
        /// 当前节点与目标点的距离
        /// </summary>
        public double DistancetoGoal { get; set; }
        /// <summary>
        /// 当前节点的总的代价，就是上三者之和
        /// </summary>
        public double computeCostForAStar;
        public double ComputeCostForAStar
        {
            get { return computeCostForAStar; }
            // get { return (DistancetoGoal); }
            set { computeCostForAStar = value; }
        }
        #region 好几个构造函数哎，感觉我只需要一个就够了啊
        /// <summary>
        /// 构造函数
        /// </summary>
        public Node()
        {
        }

        /// <summary>
        /// 构造函数 - 将参数传入节点内进行初始化
        /// </summary>
        /// <param name="iNodeIndex">节点编号</param>
        /// <param name="mFPoint3">点</param>
        /// <param name="flightDirection">飞行方向(</param>
        /// <param name="mParentNode">父节点</param>
        public Node(int iNodeIndex, FPoint3 mFPoint3, Node mParentNode) : base()
        {
            //将参数传入节点内进行初始化
            NodeLocation = new FPoint3(mFPoint3.X, mFPoint3.Y, mFPoint3.Z);

            ParentNode = mParentNode;
        }

        /// <summary>
        /// 构造函数 - 专为初始节点设置的构造函数
        /// </summary>
        /// <param name="mFPoint3">点</param>
        /// <param name="flightDirection">飞行方向(</param>
        /// <param name="mParentNode">父节点</param>
        public Node(FPoint3 mFPoint3, FPoint3 goalPara, double costfromStart, Node mParentNode)
            : base()
        {
            //将参数传入节点内进行初始化
            NodeLocation = new FPoint3(mFPoint3.X, mFPoint3.Y, mFPoint3.Z);
            ParentNode = mParentNode;
            CostfromStart = costfromStart;
            //DistancetoGoal = Math.Min(Math.Abs(NodeLocation.X - goalPara.X), (NodeLocation.Y - goalPara.Y)) * (Math.Sqrt(2) - 1)
              //  + Math.Max(Math.Abs(NodeLocation.X - goalPara.X), (NodeLocation.Y - goalPara.Y));
            DistancetoGoal= FPoint2.DistanceBetweenTwoPlanePoints(NodeLocation, goalPara);
        }

        /// <summary>
        /// 构造函数 - 将参数传入节点内进行初始化
        /// </summary>
        /// <param name="mX">X坐标</param>
        /// <param name="mY">Y坐标</param>
        /// <param name="mZ">Z坐标</param>
        /// <param name="mParentNode">父节点</param>
        public Node(double mX, double mY, double mZ, Node mParentNode) : base()
        {
            //将参数传入节点内进行初始化
            NodeLocation = new FPoint3(mX, mY, mZ);
            ParentNode = mParentNode;
        }
        /// <summary>
        /// 编写自己的构造函数
        /// </summary>
        /// <param name="startPara">起始点</param>
        /// <param name="pointNumPara">当前点坐标</param>
        /// <param name="goalPara">目标点坐标</param>
        /// <param name="m_Input">输入场景信息</param>
        /// <param name="mParentNode">父节点</param>
        public Node(FPoint3 startPara, FPoint3 pointNumPara, FPoint3 goalPara, Node mParentNode) : base()
        {
            NodeLocation = pointNumPara;
            ParentNode = mParentNode;
            CostfromStart = mParentNode.CostfromStart + FPoint2.DistanceBetweenTwoPlanePoints(NodeLocation, mParentNode.NodeLocation);
            //DistancetoGoal = FPoint3.DistanceBetweenTwoSpacePointsXY(NodeLocation, goalPara);
            DistancetoGoal = FPoint2.DistanceBetweenTwoPlanePoints(NodeLocation, goalPara);
           computeCostForAStar = CostfromStart + DistancetoGoal;
        }
        #endregion
        /// <summary>
        /// 将AStar节点转化为无人机状态,就是把这个节点坐标给无人机，也就四无人机的坐标
        /// </summary>
        /// <param name="mAStarNode">AStar节点</param>
        /// <returns>无人机状态</returns>
        public SEUAVState ConvertTreeNodeToUAVState()
        {
            SEUAVState mUAVState = new SEUAVState();
            mUAVState.PointLocation = this.NodeLocation;
            //mUAVState.FlightDirection = mAStarNode.NodeDirection;
            return mUAVState;
        }

        public int CompareTo(object obj)
        {
            int result;
            try
            {
                Node info = obj as Node;
                if (this.computeCostForAStar > info.computeCostForAStar)
                {
                    result = 0;
                }
                else
                    result = 1;
                return result;
            }
            catch (Exception ex) { throw new Exception(ex.Message); }
        }






    }
}
