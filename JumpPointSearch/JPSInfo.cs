
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using SceneElementDll.Basic;
using PlanningAlgorithmInterface.AlgorithmInterface;


namespace JumpPointSearch
{
    public class JPSInfo : IPlanningAlgoDllInfo
    {
        public PlanningAlgoInfoBase AlgoInfo { get; set; } //获取算法名字，类型，参数类型，版本描述，版本号的信息

        public bool OnLoad()
        {
            if (AlgoInfo == null)
                AlgoInfo = new PlanningAlgoInfoBase()
                {
                    AlgorithmName = "JPS算法",
                    VersionNum = "1.0",
                    VersionDescription = "传说中吊吊吊炸天的JPS～不过没有任何其他优化哦～",
                    AlgorithmType = typeof(JPSAlgorithm),
                    ParameterType = typeof(JPSParameter)
                };

            return true;
        }
    }
}
