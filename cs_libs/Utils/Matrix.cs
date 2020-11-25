namespace KrpcAutoPilot.Utils
{
    public class Matrix3d
    {
        private readonly double[] v_;

        public Matrix3d() : this(new double[][]{
                new double[]{ 0, 0, 0 },
                new double[]{ 0, 0, 0 },
                new double[]{ 0, 0, 0 },
            })
        {
        }

        public Matrix3d(double[][] m)
        {
            v_ = new double[]
            {
                m[0][0], m[0][1], m[0][2],
                m[1][0], m[1][1], m[1][2],
                m[2][0], m[2][1], m[2][2]
            };
        }

        public double this[int row, int col]
        {
            get => v_[row * 3 + col];
            set => v_[row * 3 + col] = value;
        }

        public static Matrix3d One
        {
            get => new Matrix3d(new double[][]
            {
                new double[]{ 1, 0, 0 },
                new double[]{ 0, 1, 0 },
                new double[]{ 0, 0, 1 }
            });
        }

        public static Vector3d operator *(Matrix3d left, Vector3d right)
        {
            return new Vector3d(
                left[0, 0] * right[0] + left[0, 1] * right[1] + left[0, 2] * right[2],
                left[1, 0] * right[0] + left[1, 1] * right[1] + left[1, 2] * right[2],
                left[2, 0] * right[0] + left[2, 1] * right[1] + left[2, 2] * right[2]);
        }
    }
}
