using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using PathPlaningDataVisualization;

namespace RRTOrigin
{
    class RrtOriginVisualization : IVisualizationConverter
    {
        public List<RRTNode> MPathForVisualizaition {get;set;}
        public List<MyTreeNode> MyTreeNodeConverter(object mData )
        {
            List<MyTreeNode> resultList = new List<MyTreeNode>();

            var mTreeNodeList = (mData as List<RRTNode>) ;

            foreach (var node in mTreeNodeList)
            {
                MyTreeNode tmp = new MyTreeNode();
                tmp.NodeLocation = node.NodeLocation;
                tmp.Direction = node.NodeDirection;
                tmp.CostFuncValue = 0;
                resultList.Add(tmp);
            }
            for (int i = 0; i < mTreeNodeList.Count; i++)
            {
                //场景600000071崩溃，慢慢查吧！明天查1

                if (mTreeNodeList[i].ParentNode != null)
                {
                    //Nani?!
                    //IndexOf失效?
                    //9.21.2018原始代码
                    //resultList[i].ParentNode = resultList[mTreeNodeList.IndexOf(mTreeNodeList[i].ParentNode)];

                    //Bug 已GET 由于错误的算法导致的结果（算法根本就没有解。。。。。）
                    //9.21.2018修复代码
                    resultList[i].ParentNode = resultList[mTreeNodeList[i].ParentNode.NodeIndex];
                }

                else
                    resultList[i].ParentNode = null;
            }
            return resultList;


        }


    }
}
