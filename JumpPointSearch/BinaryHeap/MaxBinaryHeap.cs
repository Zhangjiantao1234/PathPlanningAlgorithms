using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MyGenerics
{
    class MaxBinaryHeap<T>:BinaryHeap<T>
        where T : IComparable
    {
        public MaxBinaryHeap() : base()
        {

        }

        protected override void InsertCondition(List<T> list, int lastElement, int parentElement)
        {
            while (lastElement > 0 && list[parentElement].CompareTo(list[lastElement]) < 0)
            {
                T temp = list[lastElement];
                list[lastElement] = list[parentElement];
                list[parentElement] = temp;

                lastElement = parentElement;
                parentElement = (lastElement - 1) / 2;
            }
        }
        protected override bool HeapifyCondition(List<T> list, int leftChild, int rightChild, ref int typeHeap, int i)
        {
            if (leftChild < list.Count && list[leftChild].CompareTo(list[typeHeap]) > 0)
            {
                typeHeap = leftChild;
            }
            if (rightChild < list.Count && list[rightChild].CompareTo(list[typeHeap]) > 0)
            {
                typeHeap = rightChild;
            }
            if (typeHeap == i)
            {
                return true;
            }
            return false;
        }

    }
}
