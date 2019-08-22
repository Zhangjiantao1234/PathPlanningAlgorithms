using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using PlanningAlgorithmInterface.AlgorithmInterface;
using SceneElementDll.Basic;
using PlanningAlgorithmInterface.Define.Input;
using PlanningAlgorithmInterface.Define.Output;

namespace JumpPointSearch
{
    public class JPSAlgorithmHelper 
    {
        public JPSAlgorithmHelper()
        {
            HeursticInfo =  new JPSHeurstic() {HeuristicFunc = HeuristicFunction.Euclidean };
        }

        /// <summary>
        /// 计算启发式获取的类；
        /// </summary>
        public JPSHeurstic HeursticInfo { get; set; } 

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

        protected bool IsPointOutrange(FPoint3 mPoint)
        {
            //var size = AlgoInput?.Scenario?.FlightScene?.Coordinate;
            var size = AlgoInput.Scenario.FlightScene.Coordinate;
            if (mPoint.X < size.MinX || mPoint.X > size.MaxX || mPoint.Y < size.MinY || mPoint.Y > size.MaxY)
                return true;
            else
                return false;
        }

    }

    public class JPSHeurstic
    {
        public JPSHeurstic()
        {
            StartNode = null;
            TargetNode = null;
            HeuristicFunc = null;
        }

        /// <summary>
        /// 处理过后的START Set
        /// 保证坐标落在格子上
        /// 初始状态 G = 0
        /// </summary>
        public Node StartNode { get; set; } 

        /// <summary>
        /// 处理过后的Target Set
        /// 保证坐标落在格子上
        /// </summary>
        public Node TargetNode { get; set; } 

        /// <summary>
        /// 启发式函数——定义
        /// </summary>
        public Func<Node, Node, double> HeuristicFunc { get; set; } 

        public double GValueFunction(Node CurrentNode)
        {
            return CurrentNode.ParentNode == null ?
                0 : CurrentNode.ParentNode.GValue + FPoint3.DistanceBetweenTwoSpacePointsXY(CurrentNode.NodeLocation, CurrentNode.ParentNode.NodeLocation);
        }

        public double HValueFunction(Node CurrentNode)
        {
            return HeuristicFunc(CurrentNode, TargetNode);
        }
    }

    public class HeuristicFunction
    {
        public enum HeuristicMode
        {
            MANHATTAN,
            EUCLIDEAN,
            CHEBYSHEV,
        };

        public static double Manhattan(Node lhs, Node rhs)
        {
            var error = (lhs.NodeLocation - rhs.NodeLocation);
            return Math.Abs(error.X) + Math.Abs(error.Y);
        }

        public static double Euclidean(Node lhs, Node rhs)
        {
            return FPoint3.DistanceBetweenTwoSpacePointsXY(lhs.NodeLocation, rhs.NodeLocation);
        }

        public static double Chebyshev(Node lhs, Node rhs)
        {
            var error = (lhs.NodeLocation - rhs.NodeLocation);
            return Math.Max(Math.Abs(error.X),Math.Abs(error.Y));
        }
    }



}
