namespace KrpcAutoPilot.Utils
{
    public class CircularBuffer<T>
    {
        public T[] Value { get; private set; }
        public bool Filled { get; private set; }

        private int size_;
        private int index_;

        public void Reset(int size)
        {
            Value = new T[size];
            size_ = size;
            index_ = 0;
            Filled = false;
        }
        public void Reset(int size, T init_value)
        {
            Value = new T[size];
            for (int i = 0; i < size; i++)
                Value[i] = init_value;
            size_ = size;
            index_ = 0;
            Filled = true;
        }
        public void Push(T value)
        {
            Value[index_] = value;
            index_++;
            if (index_ >= size_)
            {
                index_ = 0;
                Filled = true;
            }
        }

        public CircularBuffer(int size)
        {
            Reset(size);
        }
        public CircularBuffer(int size, T init_value)
        {
            Reset(size, init_value);
        }
    }
}
