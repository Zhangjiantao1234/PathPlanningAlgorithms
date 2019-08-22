/****************************************************文档说明************************************************
 * Copyright(C), 2010, BUAA, 软件与控制研究室
 * 文件名称:    RRTStarParameter.cs
 * 作者:        刘洋
 * 版本:        1.0        
 * 创建日期:    2017.07.17 15:20
 * 完成日期:    2017.07.17
 * 文件描述:    定义RRTStar算法参数
 *              
 * 调用关系:    由RRTStar算法创建并使用
 * 其它:        无
 * 函数列表:    无
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

namespace RRTStar
{
    /// <summary>
    /// RRT算法参数用户设置接口类
    /// </summary>
    public class RrtStarParameter : IParameter
    { 
        /// <summary>
        /// 目标点到达误差
        /// </summary>
        public const double Error = 4;

        /// <summary>
        /// 航迹规划距离步长
        /// </summary>
        protected double MStep = 10;
        /// <summary>
        /// 获取或设置航迹规划距离步长
        /// </summary>
        [CategoryAttribute("规划步长设置"), DisplayName("步长"), Browsable(true),
        Description("RRTStar树生长时采用的距离步长.")]
        public double Step
        {
            get { return MStep; }
            set
            {
                MStep = value; 
            }
        }

        protected double MNearRatio = 4;
        /// <summary>
        /// RRTStar生长搜寻near节点的搜索半径
        /// </summary>
        [CategoryAttribute("规划步长设置"), DisplayName("附近节点搜索半径比例"), Browsable(true),
        Description("RRTStar生长搜寻near节点的搜索半径.半径长度=步长*附近节点搜索半径比例 ")]
        public double FixedNearDistanceRatio
        {
            get { return MNearRatio; }
            set
            {
                MNearRatio = value;
            }
        }

        /// <summary>
        /// 航迹规划步长类型
        /// </summary>
        protected PlanningStepType MStepType = PlanningStepType.Constant;
        /// <summary>
        /// 获取或设置航迹规划距离步长
        /// </summary>
        [CategoryAttribute("规划步长设置"), DisplayName("规划步长类型"), Browsable(true),
        Description("RRT树生长时采用的步长模式.\r\n\"Constant\"为固定步长模式, \"Random\"为随机步长模式.")]
        //[TypeConverter(typeof(PlanningStepType))]
        public PlanningStepType StepType
        {
            get { return MStepType; }
            set { MStepType = value; }
        }

        /// <summary>
        /// 随机步长上限
        /// </summary>
        protected double MRandomStepMax = 20;
        /// <summary>
        /// 获取或设置随机步长上限
        /// </summary>
        [CategoryAttribute("规划步长设置"), DisplayName("随机规划步长上限"), Browsable(true),
        Description("RRT树采用随机步长模式生长时随机步长的上限值.")]
        public double RandomStepMax
        {
            get { return MRandomStepMax; }
            set
            {
                MRandomStepMax = value;
            }
        }

        /// <summary>
        /// 随机步长下限
        /// </summary>
        protected double MRandomStepMin = 2;
        /// <summary>
        /// 获取或设置随机步长下限
        /// </summary>
        [CategoryAttribute("规划步长设置"), DisplayName("随机规划步长下限"), Browsable(true),
        Description("RRT树采用随机步长模式生长时随机步长的下限值.")]
        public double RandomStepMin
        {
            get { return MRandomStepMin; }
            set
            {
                MRandomStepMin = (value > 0 ? value : 2);
            }
        }

        /// <summary>
        /// 最大生长节点数
        /// </summary>
        protected Int32 MMaxNodeNumber = 65536;
        /// <summary>
        /// 获取或设置最大生长节点数
        /// </summary>
        [CategoryAttribute("RRT扩展设置"), DisplayName("节点最大生长数"), Browsable(true),
        Description("RRT树最大生长节点数量, 超出该数量但仍找不到航迹时则无法找到到达目标的航迹.")]
        public int MaxNodeNumber
        {
            get { return MMaxNodeNumber; }
            set { MMaxNodeNumber = (value > 0 ? value : 65536); }
        }

        /// <summary>
        /// RRT节点扩展类型
        /// </summary>
        protected NodeSelectionType MSelectionType = NodeSelectionType.Classical;
        /// <summary>
        /// 获取或设置RRT节点扩展类型
        /// </summary>
        [CategoryAttribute("RRT扩展设置"), DisplayName("节点生长类型"), Browsable(true),
        Description("RRT树生长时采用的节点扩展策略.\r\n\"Classical\"为均匀随机采样策略, \"Bias\"为有偏随机采样策略.")]
        public NodeSelectionType SelectionType
        {
            get { return MSelectionType; }
            set { MSelectionType = value; }
        }

        /// <summary>
        /// 阈值:取值为[0,1]. 取值越大,则选择目标点当作新点的概率越大
        /// </summary>
        protected double MChooseTargetThreshold = 0.1;
        /// <summary>
        /// 获取或设置阈值:取值为[0,1]. 取值越大,则选择目标点当作新点的概率越大
        /// </summary>
        [CategoryAttribute("RRT扩展设置"), DisplayName("向目标点生长概率"), Browsable(true),
        Description("RRT树采用优化扩展策略生长时, 将目标点作为新节点的概率.\r\n该值取值越大, 则选择目标点当作新点的概率越大, 路径越优, 但效率降低.")]
        public double ChooseTargetThreshold
        {
            get { return MChooseTargetThreshold; }
            set
            {
                MChooseTargetThreshold = value;
            }
        }

        /// <summary>
        /// 目标到达模式
        /// </summary>
        protected TargetReachMode MReachMode = TargetReachMode.Gradual;
        /// <summary>
        /// 获取或设置目标到达模式
        /// </summary>
        [CategoryAttribute("RRT扩展设置"), DisplayName("目标到达模式"), Browsable(true),
        Description("RRT树生长时采用的目标到达模式.\r\n\"Gradual\"为传统渐近到达模式, \"Direct\"为直接到达模式.")]
        public TargetReachMode ReachMode
        {
            get { return MReachMode; }
            set { MReachMode = value; }
        }

        /// <summary>
        /// 是否自动优化参数
        /// </summary>
        protected bool MAutoOptimizeParameter = true;
        /// <summary>
        /// 获取或设置是否自动优化参数
        /// </summary>
        [CategoryAttribute("其它"), DisplayName("自动优化参数"), Browsable(true),
        Description("运行开始前根据环境自动优化参数")]
        public bool AutoOptimizeParameter
        {
            get { return MAutoOptimizeParameter; }
            set { MAutoOptimizeParameter = value; }
        }



        /// <summary>
        /// 是否绘制RRT-StarTree
        /// </summary>
        protected bool MIsDrawingTree = false;
        /// <summary>
        /// 是否绘制RRT-StarTree
        /// </summary>
        [CategoryAttribute("其它"), DisplayName("绘制Tree"), Browsable(true),
        Description("无")]
        public bool IsDrawingTree
        {
            get { return MIsDrawingTree; }
            set { MIsDrawingTree = value; }
        }





        /// <summary>
        /// 飞行任务高度
        /// </summary>
        protected double MMissionAltitude = 8;
        /// <summary>
        /// 获取或设置飞行任务高度
        /// </summary>
        [Browsable(false)]
        public double MissionAltitude
        {
            get { return MMissionAltitude; }
            set { MMissionAltitude = value; }
        }


        /// <summary>
        /// 获取算法参数默认值
        /// </summary>
        [Browsable(false)]
        public IParameter Default
        {
            get { return new RrtStarParameter(); }
        }

        #region DubinsMode

            /// <summary>
            /// 是否采用Dubins模型
            /// </summary>
            protected bool MIsDubinsMode = false;
            /// <summary>
            /// 是否采用Dubins模型
            /// </summary>
            [CategoryAttribute("其它"), DisplayName("是否采用Dubins模型"), Browsable(true),
            Description("无")]
            public bool IsDubinsMode
            {
                get { return MIsDubinsMode; }
                set { MIsDubinsMode = value; }
            }

            /// <summary>
            /// 设定Dubins模型下的转弯半径
            /// </summary>
            protected double MTurningRadius = 3;
            /// <summary>
            /// 设定Dubins模型下的转弯半径
            /// </summary>
            [CategoryAttribute("其它"), DisplayName("转弯半径（若Dubins模型生效）"), Browsable(true),
            Description("无")]
            public double TurningRadius
            {
                get { return MTurningRadius; }
                set { MTurningRadius = value; }
            }
        #endregion

        #region FixNodeMode

            /// <summary>
            /// 是否采用FixNode策略
            /// </summary>
            protected bool MIsFixNodeStrategy = false;
            /// <summary>
            /// 是否采用FixNode策略
            /// </summary>
            [CategoryAttribute("其它"), DisplayName("是否采用FixNode策略"), Browsable(true),
            Description("无")]
            public bool IsFixNodeStrategy
            {
                get { return MIsFixNodeStrategy; }
                set { MIsFixNodeStrategy = value; }
            }

            /// <summary>
            /// FixNode的node数量
            /// </summary>
            protected int MFixNodeNum = 2000;
            /// <summary>
            /// FixNode的node数量
            /// </summary>
            [CategoryAttribute("其它"), DisplayName("FixNode的node数量"), Browsable(true),
            Description("无")]
            public int FixNodeNum
            {
                get { return MFixNodeNum; }
                set 
                {
                    if (MFixNodeNum < MaxNodeNumber)
                        MFixNodeNum = value;
                    else
                        MFixNodeNum = MaxNodeNumber/2;
                }
            }


            #endregion

        #region NearNodesFilter
        /// <summary>
        /// Filter Trigger
        /// </summary>
        protected bool BNearNodesFilter = true;
        /// <summary>
        /// Filter Trigger
        /// </summary>
        [CategoryAttribute("NearNodesFilter"), DisplayName("NearNodesFilterTrigger"), Browsable(true),
        Description("Filter Trigger, True means filter available.")]
        public bool NearNodesFilter
        {
            get { return BNearNodesFilter; }
            set { BNearNodesFilter = value; }
        }

        /// <summary>
        /// Filter Trigger
        /// </summary>
        protected int MTriggerNum = 5;
        /// <summary>
        /// Filter Trigger
        /// </summary>
        [CategoryAttribute("NearNodesFilter"), DisplayName("TriggerNum"), Browsable(true),
        Description("Filter Trigger number, if near nodes number bigger than TriggerNum, then filter is available.")]
        public int TriggerNum
        {
            get { return MTriggerNum; }
            set { MTriggerNum = value; }
        }


        #endregion

        /// <summary>
        /// 从算法参数配置文件获取算法参数值
        /// </summary>
        [Browsable(false)]
        public IParameter User
        {
            get 
            {
                //参数文件目录
                string sFileDir = AppConfigOperation.GetConfigurationValue("ApplicationFullPath") + @"\PathPlanning\Method\Parameter\" +
                    ((int)EPathPlanningMethod.RRTStar).ToString() + ".ini";
                RrtStarParameter mRrtParameter = (RrtStarParameter)this.Default;//初始为默认
                //如果有参数文件则从文件设置
                if (File.Exists(sFileDir))
                {
                    mRrtParameter.StepType = (PlanningStepType)Convert.ToInt32(
                        IniOperation.GetProfileString("PlanningStep", "StepType", "0", sFileDir));
                    mRrtParameter.Step = Convert.ToDouble(
                        IniOperation.GetProfileString("PlanningStep", "Step", "10", sFileDir));
                    mRrtParameter.FixedNearDistanceRatio = Convert.ToDouble(
                        IniOperation.GetProfileString("PlanningStep", "FixedNearDistanceRatio", "3", sFileDir));
                    mRrtParameter.RandomStepMax = Convert.ToDouble(
                        IniOperation.GetProfileString("PlanningStep", "RandomStepMax", "20", sFileDir));
                    mRrtParameter.RandomStepMin = Convert.ToDouble(
                        IniOperation.GetProfileString("PlanningStep", "RandomStepMin", "2", sFileDir));
                    mRrtParameter.SelectionType = (NodeSelectionType)Convert.ToInt32(
                        IniOperation.GetProfileString("Extension", "SelectionType", "0", sFileDir));
                    mRrtParameter.ChooseTargetThreshold = Convert.ToDouble(
                        IniOperation.GetProfileString("Extension", "ChooseTargetThreshold", "0.1", sFileDir));
                    mRrtParameter.MaxNodeNumber = Convert.ToInt32(
                        IniOperation.GetProfileString("Extension", "MaxNodeNumber", "65535", sFileDir));
                    mRrtParameter.ReachMode = (TargetReachMode)Convert.ToInt32(
                        IniOperation.GetProfileString("Extension", "ReachMode", "0", sFileDir));
                    mRrtParameter.AutoOptimizeParameter = 
                        IniOperation.GetProfileString("Others", "AutoOptimizeParameter", "1", sFileDir) == "1" ? true : false;
                    mRrtParameter.IsDrawingTree =
                       IniOperation.GetProfileString("Others", "IsDrawingTree", "0", sFileDir) == "1" ? true : false;

                    mRrtParameter.IsDubinsMode =
                        IniOperation.GetProfileString("Others", "IsDubinsMode", "0", sFileDir) == "1" ? true : false;
                    mRrtParameter.TurningRadius = Convert.ToDouble(
                         IniOperation.GetProfileString("Others", "TurningRadius", "3", sFileDir));

                    #region NearNodesFilter
                    mRrtParameter.NearNodesFilter =
                        IniOperation.GetProfileString("NearNodesFilter", "NearNodesFilter", "1", sFileDir) == "1" ? true : false;
                    mRrtParameter.TriggerNum = Convert.ToInt32(
                        IniOperation.GetProfileString("NearNodesFilter", "TriggerNum", "5", sFileDir));
                    #endregion

                }
                else
                {
                    //新创建文件
                    SetParameterFile();
                }
                //
                return mRrtParameter;
            }
        }

        /// <summary>
        /// 设置参数文件
        /// </summary>
        public void SetParameterFile()
        {
            //存储到文件
            string sFileDir = AppConfigOperation.GetConfigurationValue("ApplicationFullPath") + @"\PathPlanning\Method\Parameter\" +
                ((int)EPathPlanningMethod.RRTStar).ToString() + ".ini";  //参数文件地址
            IniOperation.WriteProfileString("PlanningStep", "StepType", ((int)(MStepType)).ToString(), sFileDir);
            IniOperation.WriteProfileString("PlanningStep", "Step", MStep.ToString(), sFileDir);
            IniOperation.WriteProfileString("PlanningStep", "FixedNearDistanceRatio", FixedNearDistanceRatio.ToString(), sFileDir);
            IniOperation.WriteProfileString("PlanningStep", "RandomStepMax", MRandomStepMax.ToString(), sFileDir);
            IniOperation.WriteProfileString("PlanningStep", "RandomStepMin", MRandomStepMin.ToString(), sFileDir);
            IniOperation.WriteProfileString("Extension", "SelectionType",((int)MSelectionType).ToString(), sFileDir);
            IniOperation.WriteProfileString("Extension", "ChooseTargetThreshold", MChooseTargetThreshold.ToString(), sFileDir);
            IniOperation.WriteProfileString("Extension", "MaxNodeNumber", MMaxNodeNumber.ToString(), sFileDir);
            IniOperation.WriteProfileString("Extension", "ReachMode", ((int)MReachMode).ToString(), sFileDir);
            IniOperation.WriteProfileString("Others", "AutoOptimizeParameter", (Convert.ToInt32(MAutoOptimizeParameter)).ToString(), sFileDir);
            IniOperation.WriteProfileString("Others", "IsDrawingTree", (Convert.ToInt32(MIsDrawingTree)).ToString(), sFileDir);

            IniOperation.WriteProfileString("Others", "IsDubinsMode", (Convert.ToInt32(MIsDubinsMode)).ToString(), sFileDir);
            IniOperation.WriteProfileString("Others", "TurningRadius", TurningRadius.ToString(), sFileDir);

            #region NearNodesFilter
            IniOperation.WriteProfileString("NearNodesFilter", "NearNodesFilter",(Convert.ToInt32(NearNodesFilter)).ToString(), sFileDir);
            IniOperation.WriteProfileString("NearNodesFilter", "TriggerNum", TriggerNum.ToString(), sFileDir);
            #endregion

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
        /// 有偏随机点扩展策略
        /// </summary>
        [Description("有偏随机扩展策略")]
        Bias = 1
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
