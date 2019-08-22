using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
//
using PlanningAlgorithmInterface.Define.Input;
using PlanningAlgorithmInterface.Define.Output;
using PlanningAlgorithmInterface.AlgorithmInterface;


namespace RRTStar
{
    /// <summary>
    /// 集中/静态模式下基于RRT的航迹规划算法
    /// </summary>
    public partial class MrrtStarCentralizedStatic : MrrtStarBase, IPlanningAlgorithm
    {
        public MPath BuildPathForSingleUAVInStatic(int iTaskIndex)
        {
            return BuildPathForSingleUav(iTaskIndex);
        }

        public void InitParameter()
        {
            MRrtParameter = AlgoParameter as RrtStarParameter;

            if (MRrtParameter.AutoOptimizeParameter == true)
            {
                //自动化参数
            }
        }

        
    }
}
