using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using PathPlaningDataVisualization;

namespace RRTStar
{
    class RrtStarVisualization : IVisualizationConverter
    {
        public List<RrtStarNode> MPathForVisualizaition {get;set;}
        public List<MyTreeNode> MyTreeNodeConverter(object mData )
        {
            List<MyTreeNode> resultList = new List<MyTreeNode>();

            var mTreeNodeList = (mData as List<RrtStarNode>) ;

            //好吧，换成hashset之后这个逻辑就不对了。
            //你需要更换一个逻辑
            try
            {
                var startNode = mTreeNodeList.First(e => e.ParentNode == null);
                if (startNode != null)
                    UpdateTreeNode(startNode, ref resultList);
            }
            catch
            {

            }

            return resultList;
            
            
        }
        private void UpdateTreeNode(RrtStarNode currentNode , ref List<MyTreeNode> resultList)
        {
            
            MyTreeNode tmp = new MyTreeNode();
            tmp.NodeLocation = currentNode.NodeLocation;
            tmp.Direction = double.MinValue;
            tmp.CostFuncValue = currentNode.CostFuncValue;
            tmp.Index = currentNode.NodeIndex;

            if (currentNode.ParentNode == null)
                tmp.ParentNode = null;
            else
                tmp.ParentNode = resultList.First(e => e.Index == currentNode.ParentNode.NodeIndex);
            resultList.Add(tmp);

            foreach(var node in currentNode.ChildNodes)
                UpdateTreeNode(node, ref resultList);
        }

    }
}
