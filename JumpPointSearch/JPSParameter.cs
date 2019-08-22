
using System;
using System.ComponentModel;
using ConfigDll;
using PlanningAlgorithmInterface.AlgorithmInterface;
using System.IO;

namespace JumpPointSearch
{
    /// <summary>
    /// JPS接口类
    /// </summary>
    public class JPSParameter : IParameter
    {
        /// <summary>
        /// 获取或设置是否自动优化参数
        /// </summary>
        [CategoryAttribute("其它"), DisplayName("自动优化参数"), Browsable(true),
        Description("运行开始前根据环境自动优化参数")]
        public bool AutoOptimizeParameter { get; set; }

        /// <summary>
        /// 测试参数
        /// </summary>
        [CategoryAttribute("参数设置"), DisplayName("步长"), Browsable(true),
        Description("仿真步长")]
        public double Step { get; set; }

        /// <summary>
        /// 路径是否会简化
        /// </summary>
        [CategoryAttribute("参数设置"), DisplayName("是否需要简化路径"), Browsable(true),
        Description("简化可联通的多余节点")]
        public bool NeedPathSimplifed { get; set; }

        /// <summary>
        /// 获取算法参数默认值
        /// </summary>
        [Browsable(false)]
        public IParameter Default
        {
            get { return new JPSParameter()
            {
                NeedPathSimplifed = false,
                AutoOptimizeParameter = false,
                Step = 1
            }; }
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
                    typeof(JPSAlgorithmHelper).ToString() + ".ini";
                var mParameter = (JPSParameter)this.Default;//初始为默认
                                                                                                       //如果有参数文件则从文件设置
                if (File.Exists(sFileDir))
                {
                    mParameter.AutoOptimizeParameter =
                        IniOperation.GetProfileString("Others", "AutoOptimizeParameter", "0", sFileDir) == "1" ? true : false;
                    mParameter.Step = Convert.ToDouble(
                        IniOperation.GetProfileString("ParameterSetting", "Step", "10", sFileDir));
                    mParameter.NeedPathSimplifed =
                        IniOperation.GetProfileString("ParameterSetting", "NeedPathSimplifed", "0", sFileDir) == "1" ? true : false;
                }
                else
                {
                    //新创建文件
                    SetParameterFile();
                }
                //
                return mParameter;
            }
        }

        /// <summary>
        /// 设置参数文件
        /// </summary>
        public void SetParameterFile()
        {
            //存储到文件
            string sFileDir = System.AppDomain.CurrentDomain.BaseDirectory + @"PathPlanning\Method\Parameter\" +
                     typeof(JPSAlgorithmHelper).ToString() + ".ini"; //参数文件地址

            IniOperation.WriteProfileString("Others", "AutoOptimizeParameter", (Convert.ToInt32(AutoOptimizeParameter)).ToString(), sFileDir);
            IniOperation.WriteProfileString("ParameterSetting", "Step", Step.ToString(), sFileDir);
            IniOperation.WriteProfileString("ParameterSetting", "NeedPathSimplifed", (Convert.ToInt32(NeedPathSimplifed)).ToString(), sFileDir);
        }

    }
}
