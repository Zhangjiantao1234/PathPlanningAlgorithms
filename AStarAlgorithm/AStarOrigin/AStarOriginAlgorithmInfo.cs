using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using SceneElementDll.Basic;
using PlanningAlgorithmInterface.AlgorithmInterface;
using ConfigDll;

namespace AStarOrigin
{
    /// <summary>
    /// 给出算法基本信息，算法名字，类型，参数类型，版本描述，版本号的信息
    /// </summary>
    public class AStarOriginAlgorithmInfo : IPlanningAlgoDllInfo
    {  
        public PlanningAlgoInfoBase AlgoInfo { get; set; } //获取算法名字，类型，参数类型，版本描述，版本号的信息

        public bool OnLoad()
        {
            if (AlgoInfo == null)
                AlgoInfo = new PlanningAlgoInfoBase()
                {
                    AlgorithmName = "重写A*算法",
                    VersionNum = "1.0",
                    VersionDescription = "一个空算法，做个测试而已，别较真～",
                    AlgorithmType = typeof(AStarOriginAlgorithm),
                    ParameterType = typeof(AStarOriginAlgorithmParameter)
                };

            return true;
        }
    }
}
