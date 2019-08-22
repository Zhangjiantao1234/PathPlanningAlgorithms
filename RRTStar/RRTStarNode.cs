/**************************文档说明*******************************
 * Copyright(C), 2010, BUAA, 软件与控制研究室
 * 文件名称:    RRTNode.cs
 * 作者:        Liuwei
 * 版本:        1.0        
 * 创建日期:    2011.06.15
 * 完成日期:    2011
 * 文件描述:    RRT树的节点类。包括坐标、父节点和构造函数。
 * 调用关系:    
 * 其它:        无
 * 函数列表:    略
 * 
 * 修改历史:
 * 1.   修改日期:
 *      修改人:
 *      修改功能:
 * 2.   
*****************************************************************/
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
//
using PlanningAlgorithmInterface.Define.Input;

using SceneElementDll;
using SceneElementDll.Basic;
using SceneElementDll.UAV;
using PlanningAlgorithmInterface.Define.Input;

namespace RRTStar
{
    /// <summary>
    /// RRT节点类
    /// </summary>
    public partial class RrtStarNode
    {
        /// <summary>
        /// RRT树节点编号
        /// </summary>
        private int _nodeIndex = 0;
        
        /// <summary>
        /// 获取或设置RRT树节点编号
        /// </summary>
        public int NodeIndex
        {
          get => _nodeIndex;
            set => _nodeIndex = value;
        }

        /// <summary>
        /// RRT树节点坐标
        /// </summary>
        private FPoint3 _nodeLocation = new FPoint3(0, 0, 0);
        /// <summary>
        /// 获取或设置RRT树节点坐标
        /// </summary>
        public FPoint3 NodeLocation
        {
            get => _nodeLocation;
            set => _nodeLocation = value;
        }

        /// <summary>
        /// 该节点的父节点
        /// </summary>
        private RrtStarNode _parentNode = null;
        /// <summary>
        /// 获取或设置该节点的父节点
        /// </summary>
        public RrtStarNode ParentNode
        {
            get => _parentNode;
            set => _parentNode = value;
        }

        /// <summary>
        /// 确定该节点是否能够构成feasible航迹
        /// false 无法构成 true 可以构成
        /// </summary>
        private bool _isValid = false;
        /// <summary>
        /// 确定该节点是否能够构成feasible航迹
        /// false 无法构成 true 可以构成
        /// </summary>
        public bool IsValid
        {
            get => _isValid;
            set => _isValid = value;
        }

        /// <summary>
        /// 确定该节点是否被弃用
        /// </summary>
        private bool _isAbandoned = false;
        /// <summary>
        /// 确定该节点是否被弃用
        /// false 未被弃用
        /// true 被弃用，在一段时间后被删除
        /// </summary>
        public bool IsAbandoned
        {
            get => _isAbandoned;
            set => _isAbandoned = value;
        }



        /// <summary>
        /// 字节点列表（RRTStarFN 算法使用，其他的算法暂时不用）
        /// </summary>
        private List<RrtStarNode> _childNodes = new List<RrtStarNode>();

        /// <summary>
        /// 字节点列表（RRTStarFN 算法使用，其他的算法暂时不用）s
        /// </summary>
        public List<RrtStarNode> ChildNodes
        {
            get => _childNodes;
            set => _childNodes = value;
        }

        /// <summary>
        /// 计算该航路点的时间 - 毫秒
        /// </summary>
        private double _computationalTime = 0;
        /// <summary>
        /// 计算该航路点的时间 - 毫秒
        /// </summary>
        public double ComputationalTime
        {
            get => _computationalTime;
            set => _computationalTime = value;
        }

        /// <summary>
        /// 当前点的CostFunc值
        /// 该参数为RRT*所用
        /// </summary>
        private double _costFuncValue = -1;
        /// <summary>
        /// 当前点的CostFunc值
        /// 该参数为RRT*所用
        /// </summary>
        public double CostFuncValue
        {
            get => _costFuncValue;
            set => _costFuncValue = value;
        }


        private void Init4RrtStarNode(int iNodeIndex, FPoint3 mFPoint3, double costFunc, RrtStarNode mParentNode)
        {
            //将参数传入节点内进行初始化
            _nodeIndex = iNodeIndex;
            _nodeLocation.X = mFPoint3.X;
            _nodeLocation.Y = mFPoint3.Y;
            _nodeLocation.Z = mFPoint3.Z;
            _costFuncValue = costFunc;
            _parentNode = mParentNode;
        }

        /// <summary>
        /// 构造函数
        /// </summary>
        public RrtStarNode()
        {
        }

        /// <summary>
        /// 构造函数 - 将参数传入节点内进行初始化
        /// </summary>
        /// <param name="iNodeIndex">节点编号</param>
        /// <param name="mFPoint3">点</param>
        /// <param name="costFunc">当前节点目标函数值(</param>
        /// <param name="mParentNode">父节点</param>
        public RrtStarNode(int iNodeIndex, FPoint3 mFPoint3, double costFunc, RrtStarNode mParentNode)
        {
            Init4RrtStarNode(iNodeIndex, mFPoint3, costFunc, mParentNode);
        }

        
        /// <summary>
        /// 构造函数 - 将参数传入节点内进行初始化
        /// </summary>
        /// <param name="mFPoint3">点</param>
        /// <param name="flightDirection">飞行方向(</param>
        /// <param name="mParentNode">父节点</param>
        public RrtStarNode(FPoint3 mFPoint3, double costFunc, RrtStarNode mParentNode)
        {
            Init4RrtStarNode(0, mFPoint3, costFunc, mParentNode);
        }

        /// <summary>
        /// 构造函数 - 将参数传入节点内进行初始化
        /// </summary>
        /// <param name="mX">X坐标</param>
        /// <param name="mY">Y坐标</param>
        /// <param name="mZ">Z坐标</param>
        /// <param name="mParentNode">父节点</param>
        public RrtStarNode(double mX, double mY, double mZ, RrtStarNode mParentNode)
        {
            //将参数传入节点内进行初始化
            Init4RrtStarNode(0, new FPoint3(mX, mY, mZ), -2, mParentNode);
        }

        /// <summary>
        /// 将状态转换为节点
        /// </summary>
        /// <param name="mUavState">点的状态</param>
        /// <returns>树节点</returns>
        public static RrtStarNode ConvertUavStateToNode(MKeyState mUavState)
        {
            return (new RrtStarNode(mUavState.Location, -3, null));
        }

        /// <summary>
        /// 将状态和父节点组合转换为节点
        /// </summary>
        /// <param name="mUavState">点的状态</param>
        /// <param name="parentRrtNode">父节点</param>
        /// <returns>树节点</returns>
        public static RrtStarNode ConvertUavStateToNode(MKeyState mUavState, RrtStarNode parentRrtNode)
        {
            return (new RrtStarNode(mUavState.Location, -4 , parentRrtNode));
        }

        /// <summary>
        /// 将RRT节点转化为无人机状态
        /// </summary>
        /// <param name="mRrtNode">点的状态</param>
        /// <returns>树节点</returns>
        public static SEUAVState ConvertNodeToUavState(RrtStarNode mRrtNode)
        {
            SEUAVState mUavState = new SEUAVState();
            mUavState.PointLocation = mRrtNode.NodeLocation;
            mUavState.FlightDirection = 0;
            return mUavState;
        }
    }
}
