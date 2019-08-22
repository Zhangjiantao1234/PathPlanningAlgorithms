/****************************************************文档说明************************************************
 * Copyright(C), 2010, BUAA, 软件与控制研究室
 * 文件名称:    RRTParameter.cs
 * 作者:        Liuwei
 * 版本:        1.0        
 * 创建日期:    2013.04.01, 16:02
 * 完成日期:    2013.04.02
 * 文件描述:    定义RRT算法参数
 *              
 * 调用关系:    由RRT算法创建并使用
 * 其它:        无
 * 函数列表:    无
 * 
 * 修改历史:
 * 1.   修改日期:   2013.06.25 16:08
 *      修改人:     刘伟
 *      修改功能:   增加User属性, 用于读取用户自定义算法参数
************************************************************************************************************/
using System;
using System.Collections.Generic;
using System.Linq;
using System.ComponentModel;
using System.Text;
using System.IO;
//
using SceneElementDll.Basic;
using SceneElementDll.Scene;
using ConfigDll;
using PlanningAlgorithmInterface.AlgorithmInterface;

namespace RRTOrigin
{
    /// <summary>
    /// RRT算法参数用户设置接口类
    /// </summary>
    public class RRTParameter : IParameter
    {
        /// <summary>
        /// 目标点到达误差
        /// </summary>
        public const double Error = 4;

        /// <summary>
        /// 航迹规划距离步长
        /// </summary>
        protected double m_Step = 10;
        /// <summary>
        /// 获取或设置航迹规划距离步长
        /// </summary>
        [CategoryAttribute("规划步长设置"), DisplayName("步长"), Browsable(true),
        Description("RRT树生长时采用的距离步长.")]
        public double Step
        {
            get { return m_Step; }
            set
            {
                m_Step = value; 
            }
        }

        /// <summary>
        /// 航迹规划步长类型
        /// </summary>
        protected PlanningStepType m_StepType = PlanningStepType.Constant;
        /// <summary>
        /// 获取或设置航迹规划距离步长
        /// </summary>
        [CategoryAttribute("规划步长设置"), DisplayName("规划步长类型"), Browsable(true),
        Description("RRT树生长时采用的步长模式.\r\n\"Constant\"为固定步长模式, \"Random\"为随机步长模式.")]
        //[TypeConverter(typeof(PlanningStepType))]
        public PlanningStepType StepType
        {
            get { return m_StepType; }
            set { m_StepType = value; }
        }

        /// <summary>
        /// 随机步长上限
        /// </summary>
        protected double m_RandomStepMax = 20;
        /// <summary>
        /// 获取或设置随机步长上限
        /// </summary>
        [CategoryAttribute("规划步长设置"), DisplayName("随机规划步长上限"), Browsable(true),
        Description("RRT树采用随机步长模式生长时随机步长的上限值.")]
        public double RandomStepMax
        {
            get { return m_RandomStepMax; }
            set
            {
                m_RandomStepMax = value;
            }
        }

        /// <summary>
        /// 随机步长下限
        /// </summary>
        protected double m_RandomStepMin = 2;
        /// <summary>
        /// 获取或设置随机步长下限
        /// </summary>
        [CategoryAttribute("规划步长设置"), DisplayName("随机规划步长下限"), Browsable(true),
        Description("RRT树采用随机步长模式生长时随机步长的下限值.")]
        public double RandomStepMin
        {
            get { return m_RandomStepMin; }
            set
            {
                m_RandomStepMin = (value > 0 ? value : 2);
            }
        }

        /// <summary>
        /// 最大生长节点数
        /// </summary>
        protected int m_MaxNodeNumber = 65535;
        /// <summary>
        /// 获取或设置最大生长节点数
        /// </summary>
        [CategoryAttribute("RRT扩展设置"), DisplayName("节点最大生长数"), Browsable(true),
        Description("RRT树最大生长节点数量, 超出该数量但仍找不到航迹时则无法找到到达目标的航迹.")]
        public int MaxNodeNumber
        {
            get { return m_MaxNodeNumber; }
            set { m_MaxNodeNumber = (value > 0 ? value : 65535); }
        }

        /// <summary>
        /// RRT节点扩展类型
        /// </summary>
        protected NodeSelectionType m_SelectionType = NodeSelectionType.Classical;
        /// <summary>
        /// 获取或设置RRT节点扩展类型
        /// </summary>
        [CategoryAttribute("RRT扩展设置"), DisplayName("节点生长类型"), Browsable(true),
        Description("RRT树生长时采用的节点扩展策略.\r\n\"Classical\"为经典扩展策略, \"Optimzed\"为优化扩展策略.")]
        public NodeSelectionType SelectionType
        {
            get { return m_SelectionType; }
            set { m_SelectionType = value; }
        }

        /// <summary>
        /// 阈值:取值为[0,1]. 取值越大,则选择目标点当作新点的概率越大
        /// </summary>
        protected double m_ChooseTargetThreshold = 0.9;
        /// <summary>
        /// 获取或设置阈值:取值为[0,1]. 取值越大,则选择目标点当作新点的概率越大
        /// </summary>
        [CategoryAttribute("RRT扩展设置"), DisplayName("向目标点生长概率"), Browsable(true),
        Description("RRT树采用优化扩展策略生长时, 将目标点作为新节点的概率.\r\n该值取值越大, 则选择目标点当作新点的概率越大, 路径越优, 但效率降低.")]
        public double ChooseTargetThreshold
        {
            get { return m_ChooseTargetThreshold; }
            set
            {
                m_ChooseTargetThreshold = value;
            }
        }

        /// <summary>
        /// 目标到达模式
        /// </summary>
        protected TargetReachMode m_ReachMode = TargetReachMode.Gradual;
        /// <summary>
        /// 获取或设置目标到达模式
        /// </summary>
        [CategoryAttribute("RRT扩展设置"), DisplayName("目标到达模式"), Browsable(true),
        Description("RRT树生长时采用的目标到达模式.\r\n\"Gradual\"为传统渐近到达模式, \"Direct\"为直接到达模式.")]
        public TargetReachMode ReachMode
        {
            get { return m_ReachMode; }
            set { m_ReachMode = value; }
        }

        /// <summary>
        /// 是否自动优化参数
        /// </summary>
        protected bool m_AutoOptimizeParameter = true;
        /// <summary>
        /// 获取或设置是否自动优化参数
        /// </summary>
        [CategoryAttribute("其它"), DisplayName("自动优化参数"), Browsable(true),
        Description("运行开始前根据环境自动优化参数")]
        public bool AutoOptimizeParameter
        {
            get { return m_AutoOptimizeParameter; }
            set { m_AutoOptimizeParameter = value; }
        }

        protected bool m_DijkstraOptimizeParameter = true;
        [CategoryAttribute("其它"), DisplayName("Dijkstra优化过程"), Browsable(true),
            Description("是否使用Dijkstra优化航迹结果")]
        public bool DijkstraOptimizeParameter
        {
            get { return m_DijkstraOptimizeParameter; }
            set { m_DijkstraOptimizeParameter = value; }
        }


        /// <summary>
        /// 飞行任务高度
        /// </summary>
        protected double m_MissionAltitude = 8;
        /// <summary>
        /// 获取或设置飞行任务高度
        /// </summary>
        [Browsable(false)]
        public double MissionAltitude
        {
            get { return m_MissionAltitude; }
            set { m_MissionAltitude = value; }
        }


        /// <summary>
        /// 获取算法参数默认值
        /// </summary>
        [Browsable(false)]
        public IParameter Default
        {
            get { return new RRTParameter(); }
        }

        /// <summary>
        /// 从算法参数配置文件获取算法参数值
        /// </summary>
        [Browsable(false)]
        public IParameter User
        {
            get 
            {
                //参数文件目录
                string sFileDir = System.AppDomain.CurrentDomain.BaseDirectory + @"PathPlanning\Method\Parameter\" +
                     typeof(RRTBase).ToString() + ".ini"; //参数文件地址
                RRTParameter mRRTParameter = (RRTParameter)this.Default;//初始为默认
                //如果有参数文件则从文件设置
                if (File.Exists(sFileDir))
                {
                    mRRTParameter.StepType = (PlanningStepType)Convert.ToInt32(
                        IniOperation.GetProfileString("PlanningStep", "StepType", "0", sFileDir));
                    mRRTParameter.Step = Convert.ToDouble(
                        IniOperation.GetProfileString("PlanningStep", "Step", "10", sFileDir));
                    mRRTParameter.RandomStepMax = Convert.ToDouble(
                        IniOperation.GetProfileString("PlanningStep", "RandomStepMax", "20", sFileDir));
                    mRRTParameter.RandomStepMin = Convert.ToDouble(
                        IniOperation.GetProfileString("PlanningStep", "RandomStepMin", "2", sFileDir));
                    mRRTParameter.SelectionType = (NodeSelectionType)Convert.ToInt32(
                        IniOperation.GetProfileString("Extension", "SelectionType", "0", sFileDir));
                    mRRTParameter.ChooseTargetThreshold = Convert.ToDouble(
                        IniOperation.GetProfileString("Extension", "ChooseTargetThreshold", "0.9", sFileDir));
                    mRRTParameter.MaxNodeNumber = Convert.ToInt32(
                        IniOperation.GetProfileString("Extension", "MaxNodeNumber", "65535", sFileDir));
                    mRRTParameter.ReachMode = (TargetReachMode)Convert.ToInt32(
                        IniOperation.GetProfileString("Extension", "ReachMode", "0", sFileDir));
                    mRRTParameter.AutoOptimizeParameter = 
                        IniOperation.GetProfileString("Others", "AutoOptimizeParameter", "1", sFileDir) == "1" ? true : false;
                    mRRTParameter.DijkstraOptimizeParameter =
                        IniOperation.GetProfileString("Others", "DijkstraOptimizeParameter", "1", sFileDir) == "1" ? true : false;
                }
                else
                {
                    //新创建文件
                    SetParameterFile();
                }
                //
                return mRRTParameter;
            }
        }

        /// <summary>
        /// 设置参数文件
        /// </summary>
        public void SetParameterFile()
        {
            //存储到文件
            string sFileDir = System.AppDomain.CurrentDomain.BaseDirectory + @"PathPlanning\Method\Parameter\" +
                     typeof(RRTBase).ToString() + ".ini"; //参数文件地址
            IniOperation.WriteProfileString("PlanningStep", "StepType", ((int)(m_StepType)).ToString(), sFileDir);
            IniOperation.WriteProfileString("PlanningStep", "Step", m_Step.ToString(), sFileDir);
            IniOperation.WriteProfileString("PlanningStep", "RandomStepMax", m_RandomStepMax.ToString(), sFileDir);
            IniOperation.WriteProfileString("PlanningStep", "RandomStepMin", m_RandomStepMin.ToString(), sFileDir);
            IniOperation.WriteProfileString("Extension", "SelectionType",((int)m_SelectionType).ToString(), sFileDir);
            IniOperation.WriteProfileString("Extension", "ChooseTargetThreshold", m_ChooseTargetThreshold.ToString(), sFileDir);
            IniOperation.WriteProfileString("Extension", "MaxNodeNumber", m_MaxNodeNumber.ToString(), sFileDir);
            IniOperation.WriteProfileString("Extension", "ReachMode", ((int)m_ReachMode).ToString(), sFileDir);
            IniOperation.WriteProfileString("Others", "AutoOptimizeParameter", (Convert.ToInt32(m_AutoOptimizeParameter)).ToString(), sFileDir);
            IniOperation.WriteProfileString("Others", "DijkstraOptimizeParameter", (Convert.ToInt32(DijkstraOptimizeParameter)).ToString(), sFileDir);
        }
    }

    /// <summary>
    /// 规划步长类型
    /// </summary>
    public enum PlanningStepType
    {
        /// <summary>
        /// 固定步长
        /// </summary>
        [Description("固定步长")]
        Constant = 0,
        /// <summary>
        /// 随机步长
        /// </summary>
        [Description("随机步长")]
        Random = 1
    }

    /// <summary>
    /// RRT节点扩展类型
    /// </summary>
    public enum NodeSelectionType
    {
        /// <summary>
        /// 经典节点扩展策略
        /// </summary>
        [Description("经典扩展策略")]
        Classical = 0,
        /// <summary>
        /// 优化节点扩展策略
        /// </summary>
        [Description("优化扩展策略")]
        Optimzed = 1
    }

    /// <summary>
    /// 目标到达模式
    /// </summary>
    public enum TargetReachMode
    {
        /// <summary>
        /// 经典节点扩展策略
        /// </summary>
        [Description("传统到达模式")]
        Gradual = 0,
        /// <summary>
        /// 优化节点扩展策略
        /// </summary>
        [Description("直接到达模式")]
        Direct = 1
    }
}
