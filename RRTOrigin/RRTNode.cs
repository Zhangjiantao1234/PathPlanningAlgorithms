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

using SceneElementDll;
using SceneElementDll.Basic;
using SceneElementDll.UAV;
using PlanningAlgorithmInterface.Define.Input;

namespace RRTOrigin
{
    /// <summary>
    /// RRT节点类
    /// </summary>
    public class RRTNode
    {
        /// <summary>
        /// RRT树节点编号
        /// </summary>
        private int nodeIndex = 0;
        /// <summary>
        /// 获取或设置RRT树节点编号
        /// </summary>
        public int NodeIndex
        {
            get { return nodeIndex; }
            set { nodeIndex = value; }
        }

        /// <summary>
        /// RRT树节点坐标
        /// </summary>
        private FPoint3 nodeLocation = new FPoint3(0, 0, 0);
        /// <summary>
        /// 获取或设置RRT树节点坐标
        /// </summary>
        public FPoint3 NodeLocation
        {
            get { return nodeLocation; }
            set { nodeLocation = value; }
        }

        /// <summary>
        /// RRT树节点上的飞行方向
        /// </summary>
        private double nodeDirection = 0;
        /// <summary>
        /// 获取或设置RRT树节点上的飞行方向
        /// </summary>
        public double NodeDirection
        {
            get { return nodeDirection; }
            set { nodeDirection = value; }
        }

        /// <summary>
        /// 该节点的父节点
        /// </summary>
        private RRTNode parentNode = null;
        /// <summary>
        /// 获取或设置该节点的父节点
        /// </summary>
        public RRTNode ParentNode
        {
            get { return parentNode; }
            set { parentNode = value; }
        }

        /// <summary>
        /// 是否路径上的节点
        /// </summary>
        private byte isInRRTPath = 0;
        /// <summary>
        /// 获取或设置是否路径上的节点
        /// </summary>
        public byte IsInRRTPath
        {
            get { return isInRRTPath; }
            set { isInRRTPath = value; }
        }

        /// <summary>
        /// 是否有效节点 - 1:有效; 0:无效
        /// </summary>
        private int isValid = 1;
        /// <summary>
        /// 获取或设置是否有效节点
        /// </summary>
        public int IsValid
        {
            get { return isValid; }
            set { isValid = value; }
        }

        /// <summary>
        /// 计算该航路点的时间 - 毫秒
        /// </summary>
        protected double m_ComputationalTime = 0;
        /// <summary>
        /// 计算该航路点的时间 - 毫秒
        /// </summary>
        public double ComputationalTime
        {
            get { return m_ComputationalTime; }
            set { m_ComputationalTime = value; }
        }

        /// <summary>
        /// 构造函数
        /// </summary>
        public RRTNode()
        {
        }

        /// <summary>
        /// 构造函数 - 将参数传入节点内进行初始化
        /// </summary>
        /// <param name="iNodeIndex">节点编号</param>
        /// <param name="mFPoint3">点</param>
        /// <param name="flightDirection">飞行方向(</param>
        /// <param name="mParentNode">父节点</param>
        public RRTNode(int iNodeIndex, FPoint3 mFPoint3, double flightDirection, RRTNode mParentNode)
        {
            //将参数传入节点内进行初始化
            nodeIndex = iNodeIndex;
            nodeLocation.X = mFPoint3.X;
            nodeLocation.Y = mFPoint3.Y;
            nodeLocation.Z = mFPoint3.Z;
            nodeDirection = flightDirection;
            parentNode = mParentNode;
            isInRRTPath = 0;
            isValid = 1;
        }

        /// <summary>
        /// 构造函数 - 将参数传入节点内进行初始化
        /// </summary>
        /// <param name="mFPoint3">点</param>
        /// <param name="flightDirection">飞行方向(</param>
        /// <param name="mParentNode">父节点</param>
        public RRTNode(FPoint3 mFPoint3, double flightDirection, RRTNode mParentNode)
        {
            //将参数传入节点内进行初始化
            nodeIndex = 0;
            nodeLocation.X = mFPoint3.X;
            nodeLocation.Y = mFPoint3.Y;
            nodeLocation.Z = mFPoint3.Z;
            nodeDirection = flightDirection;
            parentNode = mParentNode;
            isInRRTPath = 0;
            isValid = 1;
        }

        /// <summary>
        /// 构造函数 - 将参数传入节点内进行初始化
        /// </summary>
        /// <param name="mX">X坐标</param>
        /// <param name="mY">Y坐标</param>
        /// <param name="mZ">Z坐标</param>
        /// <param name="mParentNode">父节点</param>
        public RRTNode(double mX, double mY, double mZ, RRTNode mParentNode)
        {
            //将参数传入节点内进行初始化
            nodeIndex = 0;
            nodeLocation.X = mX;
            nodeLocation.Y = mY;
            nodeLocation.Z = mZ;
            parentNode = mParentNode;
            isInRRTPath = 0;
            isValid = 1;
        }

        /// <summary>
        /// 将状态转换为节点
        /// </summary>
        /// <param name="mUAVState">点的状态</param>
        /// <returns>树节点</returns>
        public static RRTNode ConvertUAVStateToNode(MKeyState mUAVState)
        {
            return (new RRTNode(mUAVState.Location, mUAVState.Direction, null));
        }

        /// <summary>
        /// 将状态和父节点组合转换为节点
        /// </summary>
        /// <param name="mUAVState">点的状态</param>
        /// <param name="parentRRTNode">父节点</param>
        /// <returns>树节点</returns>
        public static RRTNode ConvertUAVStateToNode(MKeyState mUAVState, RRTNode parentRRTNode)
        {
            return (new RRTNode(mUAVState.Location, mUAVState.Direction, parentRRTNode));
        }

        /// <summary>
        /// 将RRT节点转化为无人机状态
        /// </summary>
        /// <param name="mRRTNode">点的状态</param>
        /// <returns>树节点</returns>
        public static SEUAVState ConvertNodeToUAVState(RRTNode mRRTNode)
        {
            SEUAVState mUAVState = new SEUAVState();
            mUAVState.PointLocation = mRRTNode.NodeLocation;
            mUAVState.FlightDirection = mRRTNode.NodeDirection;
            return mUAVState;
        }
    }
}
