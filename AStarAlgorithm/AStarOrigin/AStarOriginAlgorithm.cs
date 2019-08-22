using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SceneElementDll;
using PlanningAlgorithmInterface.AlgorithmInterface;
using PlanningAlgorithmInterface.Define.Input;
using SceneElementDll.Basic;
using PlanningAlgorithmInterface.Define.Output;


namespace AStarOrigin
{
    public class AStarOriginAlgorithm : AStarOriginAlgorithmBase, IPlanningAlgorithm
    {
        /// <summary>
        /// 初始化算法参数
        /// 隐含参数AlgoParameter
        /// 在Planning函数初始时执行（controlCenter）
        /// </summary>
        public void InitParameter()
        {
            //初始化算法参数
         //   var mParameter = TestAlgorithmParameter AlgoParameter;//这个参数就是从基类来的隐含参数，直接转化为TestAlgorithmParameter类

            //是否自动优化参数
           // if (mParameter.AutoOptimizeParameter)
            if (((AStarOriginAlgorithmParameter)AlgoParameter).AutoOptimizeParameter)
            {
                //做个例子吧，丢在这里
                /*
                m_RRTParameter.Step = Math.Max(m_Input.Scenario.FlightScene.Coordinate.MaxX - m_Input.Scenario.FlightScene.Coordinate.MinX,
                    m_Input.Scenario.FlightScene.Coordinate.MaxY - m_Input.Scenario.FlightScene.Coordinate.MinY) / 20;
                m_RRTParameter.RandomStepMin = Math.Max(m_Input.Scenario.FlightScene.Coordinate.MaxX - m_Input.Scenario.FlightScene.Coordinate.MinX,
                    m_Input.Scenario.FlightScene.Coordinate.MaxY - m_Input.Scenario.FlightScene.Coordinate.MinY) / 100;
                if (m_RRTParameter.RandomStepMin < 1)
                {
                    m_RRTParameter.RandomStepMin = 1;
                }
                m_RRTParameter.RandomStepMax = Math.Max(m_Input.Scenario.FlightScene.Coordinate.MaxX - m_Input.Scenario.FlightScene.Coordinate.MinX,
                    m_Input.Scenario.FlightScene.Coordinate.MaxY - m_Input.Scenario.FlightScene.Coordinate.MinY) / 10;
                */
            }
        }

        /// <summary>
        /// 静态算法执行函数
        /// </summary>
        /// <param name="iTaskIndex"></param>
        /// <returns></returns>
        public MPath BuildPathForSingleUAVInStatic(int iTaskIndex)
        {
            return BuildPathForSingleUAV(iTaskIndex);
        }


    }
}
