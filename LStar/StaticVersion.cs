using PlanningAlgorithmInterface.AlgorithmInterface;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SceneElementDll.Basic;

namespace LStar
{
    public partial class LStarAlgorithm
    {
 

        private List<Node> BuildPathForSingleUavStageInStatic(int iTaskIndex, out HashSet<Node> visitedSet)
        {
            //Dictionary key为 math.floor取整得到的值,可能有些下标是没有值的
            //可能 List 某些下标是空的。
            var openList  = new Dictionary<int, Stack<Node>>();
            //初始化visitedList
            visitedSet = HeursticInfo != null ? new HashSet<Node>() { HeursticInfo.StartNode } : null;

            if (_para == null || visitedSet == null)
                return null;

            //初始化 openList
            InitOpenList();
            //插入初始点
            InsertToBucket(HeursticInfo.StartNode,ref openList);
            //初始化当前的Bucket
            int currentBucketReadIdx = 1;
            //退出标志
            bool pathExist = false;

            //还有节点没探索
            while (openList.Any(a => a.Value.Count != 0))
            {
                //把桶里的所有节点便利一圈
                while (openList[currentBucketReadIdx].Count!=0)
                {
                    var currentNode = openList[currentBucketReadIdx].Pop();

                    if (currentNode == HeursticInfo.TargetNode)
                    {
                        pathExist = true;
                        break;
                    }

                    foreach (var neighborNode in AllNeighbors(currentNode))
                    {
                        var selectedNode = visitedSet.FirstOrDefault(a => neighborNode == a.NodeLocation);
                        var candidateNode = new Node(neighborNode, currentNode, HeursticInfo);
                        if (selectedNode == null) // not visited
                        {
                            InsertToBucket(candidateNode, ref openList);
                            visitedSet.Add(candidateNode);
                        }
                        else   // visited
                        {
                            if (candidateNode.GValue < selectedNode.GValue)
                            {
                                visitedSet.Remove(selectedNode);
                                foreach (var stack in openList.Values)
                                {
                                    if (stack.TryRemove(candidateNode))
                                        break;
                                }
                                InsertToBucket(candidateNode, ref openList);
                                visitedSet.Add(candidateNode);
                            }
                        }
                    }

                }
                //正常找到解就跳出去
                //下面的代码顺序不要变，否则可能会出BUG
                if (pathExist)
                    break;

                //空桶就扔了
                openList.Remove(currentBucketReadIdx);
                currentBucketReadIdx++;
                //跳过那些空的桶
                while (!openList.ContainsKey(currentBucketReadIdx) && currentBucketReadIdx <= openList.Keys.Max()) 
                    currentBucketReadIdx++;
                //溢出就跳出去
                if (currentBucketReadIdx > openList.Keys.Max())
                    break;
            }

            if (pathExist)
            {
                var targetNode = visitedSet.FirstOrDefault(a => HeursticInfo.TargetNode.NodeLocation == a.NodeLocation);
                return GetPath(targetNode);
            }
            else
            {
                return null;
            }
        }

        
        private void InsertToBucket(Node node, ref Dictionary<int, Stack<Node>> openList)
        {
            //定义INDEX
            node.Index = _nodeCounter++;
            //计算所属的Bucket
            int key = Convert.ToInt32(Math.Floor((node.CostFunc - _valueInit) / _df)) + 1;
            
            //如果是空的那就初始化一下内存
            if(!openList.ContainsKey(key))
                openList.Add(key,new Stack<Node>());
            
            //Push进去
            openList[key].Push(node);
        }



        private List<FPoint3> AllNeighbors(Node mCurrentNode)
        {
            var ResultNeighbors = new List<FPoint3>();
            var step = _para.Step;

            List<FPoint3> Directions = new List<FPoint3>
            {   new FPoint3(0, 1, 0), new FPoint3(1, 0, 0),
                new FPoint3(1, 1, 0), new FPoint3(0, -1, 0),
                new FPoint3(-1, 0, 0), new FPoint3(-1, -1, 0),
                new FPoint3(-1, 1, 0), new FPoint3(1, -1, 0) };

            foreach (var tmp in Directions)
            {
                var node = mCurrentNode.NodeLocation + tmp * step;
                if (IsSafePoint(node) && !IsOutRange(node))
                    ResultNeighbors.Add(mCurrentNode.NodeLocation + tmp * step);
            }
            return ResultNeighbors;
        }


        private bool IsOutRange(FPoint3 node)
        {
            if (node.X < 0 || node.X > this.AlgoInput.Scenario.FlightScene.Coordinate.MaxX ||
                node.Y < 0 || node.Y > this.AlgoInput.Scenario.FlightScene.Coordinate.MaxY)
            {
                return true;
            }
            else
            {
                return false;
            }
        }


    }




}
