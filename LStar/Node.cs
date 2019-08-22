using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.ConstrainedExecution;
using System.Text;
using System.Threading.Tasks;

using PlanningAlgorithmInterface.AlgorithmInterface;
using PlanningAlgorithmInterface.Define.Input;
using SceneElementDll.Basic;
using PlanningAlgorithmInterface.Define.Output;
using SceneElementDll.UAV;

/// <summary>
/// 参考论文：
/// L* Algorithm—A Linear Computational Complexity Graph Searching Algorithm for Path Planning
/// Journal of Intelligent & Robotic Systems
/// 参考伪代码:
/// https://github.com/adamniewola/Lstar
/// </summary>

namespace LStar
{
    /// <summary>
    /// 在地图上每个节点应该带有的属性
    /// </summary>
    public class Node : IComparable
    {
        /// <summary>
        /// ID号～
        /// </summary>
        public long Index { get; set; }

        /// <summary>
        /// 节点的坐标
        /// </summary>
        public FPoint3 NodeLocation { get; set; }

        /// <summary>
        /// 父节点
        /// </summary>
        public Node ParentNode { get; set; }

        /// <summary>
        /// 当前节点与开始节点的距离（花费）
        /// </summary>
        public double GValue { get; set; }
        /// <summary>
        /// 当前节点与目标点的距离
        /// </summary>
        public double HValue { get; set; }

        /// <summary>
        /// HValue 权重
        /// </summary>
        public static double Weight { get; set; }

        /// <summary>
        /// CostFunc
        /// </summary>
        public double CostFunc
        {
            get { return GValue + Weight * HValue; }
        }
        
        /// <summary>
        /// 构造函数
        /// </summary>
        private Node()
        {
            NodeLocation = null;
            ParentNode = null;
            GValue = 0;
            HValue = double.MaxValue;

            //不刷ID了，总会有临时的Node出来，不搞了不搞了
            Index = -1;//临时工
        }

        /// <summary>
        /// 构造函数 - 将参数传入节点内进行初始化
        /// </summary>
        /// <param name="mFPoint3">点</param>
        /// <param name="mParentNode">父节点</param>
        public Node( FPoint3 mFPoint3, Node mParentNode, LStarHeurstic mHeurstic = null)
        {
            ///mHeurstic不要写成Static，否则没办法多线程并行化处理～
            ///函数式编程的IDEA吧。
            ///
            NodeLocation = new FPoint3(mFPoint3.X, mFPoint3.Y, mFPoint3.Z);
            ParentNode = mParentNode;
            
            if(mHeurstic != null)
            {
                if (mParentNode == null)
                {
                    GValue = 0;
                    HValue = mHeurstic.HeuristicFunc(mHeurstic.StartNode, mHeurstic.TargetNode);
                }
                else
                {
                    ///刷出来俩宝贝。。。。。
                    GValue = mHeurstic.GValueFunction(this);
                    HValue = mHeurstic.HValueFunction(this);
                }
            }
            else
            {
                GValue = 0;
                HValue = double.MaxValue;
            }
        }
        


        /// <summary>
        /// 
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
            return CostFunc.CompareTo( (obj as Node)?.CostFunc);
        }

        [ReliabilityContract(Consistency.WillNotCorruptState, Cer.MayFail)]
        public bool Equals(Node value)
        {
            if ( (object)value == null || NodeLocation == null)
                return false;
            return NodeLocation.Equals(value.NodeLocation);
        }

        [ReliabilityContract(Consistency.WillNotCorruptState, Cer.MayFail)]
        public override bool Equals(object obj)
        {
            if (obj == null || (object)NodeLocation == null)
                return false;
            return NodeLocation.Equals(((Node)obj).NodeLocation);
        }
        
        public override int GetHashCode()
        {
            return NodeLocation.GetHashCode();
        }

        public static bool operator ==(Node p1, Node p2)
        {
            return object.Equals(p1, p2);
        }

        public static bool operator !=(Node p1, Node p2)
        {
                return !(p1 == p2);

        }

    }
}
