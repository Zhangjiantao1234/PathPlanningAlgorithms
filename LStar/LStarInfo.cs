
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using LStar;
using SceneElementDll.Basic;
using PlanningAlgorithmInterface.AlgorithmInterface;


namespace JumpPointSearch
{
    public class LStarInfo : IPlanningAlgoDllInfo
    {
        public PlanningAlgoInfoBase AlgoInfo { get; set; } //获取算法名字，类型，参数类型，版本描述，版本号的信息

        public bool OnLoad()
        {
            if (AlgoInfo == null)
                AlgoInfo = new PlanningAlgoInfoBase()
                {
                    AlgorithmName = "LStar算法",
                    VersionNum = "1.0",
                    VersionDescription = "传说中吊吊吊炸天的LStar～不过没有任何其他优化哦～",
                    AlgorithmType = typeof(LStarAlgorithm),
                    ParameterType = typeof(LStarParameter)
                };

            return true;
        }
    }
}
