using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using PathPlaningDataVisualization;

namespace AStarOrigin
{
    class AStarVisualization : IVisualizationConverter
    {
        public List<Node> mPathForVisualizaition {get;set;}
        public List<MyTreeNode> MyTreeNodeConverter(object mData )
        {
            List<MyTreeNode> resultList = new List<MyTreeNode>();

            var mTreeNodeList = (mData as List<Node>) ;
            
            foreach (var node in mTreeNodeList)
            {
                MyTreeNode tmp = new MyTreeNode();
                tmp.NodeLocation = node.NodeLocation;

                tmp.CostFuncValue = 0;
                resultList.Add(tmp);
            }
            for (int i = 0; i < mTreeNodeList.Count; i++)
            {
                if (mTreeNodeList[i].ParentNode != null)
                    resultList[i].ParentNode = resultList[mTreeNodeList.IndexOf(mTreeNodeList[i].ParentNode)];
                else
                    resultList[i].ParentNode = null;
            }
            return resultList;
        }
    }
}
