using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using SceneElementDll.Basic;
using PlanningAlgorithmInterface.AlgorithmInterface;
using ConfigDll;

namespace RRTStar
{
    /// <summary>
    /// 给出算法基本信息，算法名字，类型，参数类型，版本描述，版本号的信息
    /// </summary>
    public class RrtStarAlgorithmInfo : IPlanningAlgoDllInfo
    {  
        public PlanningAlgoInfoBase AlgoInfo { get; set; } //获取算法名字，类型，参数类型，版本描述，版本号的信息

        public bool OnLoad()
        {
            if (AlgoInfo == null)
                AlgoInfo = new PlanningAlgoInfoBase()
                {
                    AlgorithmName = "DUBIN曲线优化的RRT算法",
                    VersionNum = "1.0",
                    VersionDescription = "1阶可导（G1）的轨迹，显然不是G2轨迹，每次转弯以最小转弯半径满足飞行距离最短要求。",
                    AlgorithmType = typeof(MrrtStarCentralizedStatic),
                    ParameterType = typeof(RrtStarParameter)
                };

            return true;
        }
    }
}
