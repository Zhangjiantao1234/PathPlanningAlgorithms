using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LStar
{
    static class MyExtensions
    {
        public static bool TryRemove<T>(this Stack<T> stack, T item) 
        {
            if (item == null)
                throw new ArgumentNullException(nameof(item));
            if (stack == null)
                throw new ArgumentNullException(nameof(stack));
            if (!stack.Contains(item))
                return false;
            else
            {
                Stack<T> tmp = new Stack<T>();
                while(! stack.Peek().Equals(item))
                    tmp.Push(stack.Pop());
                //删掉该项
                stack.Pop();
                while (tmp.Count!=0)
                {
                    stack.Push(tmp.Pop());
                }

                return true;
            }
        }
    }
    
}
