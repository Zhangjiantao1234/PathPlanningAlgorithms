using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MyGenerics
{
   abstract class BinaryHeap<T>
        where T : IComparable
    {
        public readonly List<T> ArgumentList;

        public int HeapSize
        {
            get
            {
                return this.ArgumentList.Count();
            }
        }
        public BinaryHeap()
        {
            ArgumentList = new List<T>();
        }

        public void Insert(T value)
        {
            ArgumentList.Add(value);
            int i = HeapSize - 1;
            int parent = (i - 1) / 2;
            InsertCondition(ArgumentList, i, parent);

        }
        protected abstract void InsertCondition(List<T> list, int lastElement, int parentElement);
        protected abstract bool HeapifyCondition(List<T> list, int leftChild, int rightChild, ref int typeHeap, int i);  
        

        public void Heapify(int i)
        {
            int leftChild;
            int rightChild;
            int typeHeap;

            for (;;)
            {
                leftChild = 2 * i + 1;
                rightChild = 2 * i + 2;
                typeHeap = i;
                if (HeapifyCondition(ArgumentList, leftChild, rightChild, ref typeHeap,i) == true) return;

                T temp = ArgumentList[i];
                ArgumentList[i] = ArgumentList[typeHeap];
                ArgumentList[typeHeap] = temp;
                i = typeHeap;

            }
        }


        public bool TryExtract(out T result)
        {
            if (HeapSize > 0)
            {
                result = ArgumentList[0];
                ArgumentList[0] = ArgumentList[HeapSize - 1];
                ArgumentList.RemoveAt(HeapSize - 1);
                return true;
            }
            else
            {
                result = default(T);
                return false;
            }
        }
        public T Extract()
        {
            T rootValue;
            TryExtract(out rootValue);
            Heapify(0);
            return rootValue;
        }

    }

}

