using System;

namespace KRPCLibs.Math
{
    public class Vector3d
    {
        public double X;
        public double Y;
        public double Z;

        public Vector3d(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }
        public Vector3d(Tuple<double, double, double> v)
        {
            X = v.Item1;
            Y = v.Item2;
            Z = v.Item3;
        }
        public Vector3d(Vector3d v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }

        public static Vector3d One {
            get { return new Vector3d(1.0, 1.0, 1.0); }
        }
        public static Vector3d Zero {
            get { return new Vector3d(0.0, 0.0, 0.0); }
        }
        public double Length()
        {
            return System.Math.Sqrt(X * X + Y * Y + Z * Z);
        }
        public double LengthSquared()
        {
            return X * X + Y * Y + Z * Z;
        }
        public static Vector3d operator +(Vector3d left, Vector3d right)
        {
            return new Vector3d(left.X + right.X, left.Y + right.Y, left.Z + right.Z);
        }
        public static Vector3d operator -(Vector3d left, Vector3d right)
        {
            return new Vector3d(left.X - right.X, left.Y - right.Y, left.Z - right.Z);
        }
        public static Vector3d operator -(Vector3d value)
        {
            return new Vector3d(-value.X, -value.Y, -value.Z);
        }
        public static Vector3d operator *(Vector3d left, double right)
        {
            return new Vector3d(left.X * right, left.Y * right, left.Z * right);
        }
        public static Vector3d operator *(double left, Vector3d right)
        {
            return new Vector3d(left * right.X, left * right.Y, left * right.Z);
        }
        public static double operator *(Vector3d left, Vector3d right)
        {
            return left.X * right.X + left.Y * right.Y + left.Z * right.Z;
        }
        public static Vector3d operator /(Vector3d left, double right)
        {
            return new Vector3d(left.X / right, left.Y / right, left.Z / right);
        }
        public static Vector3d operator /(double left, Vector3d right)
        {
            return new Vector3d(left / right.X, left / right.Y, left / right.Z);
        }
        public static Vector3d operator /(Vector3d left, Vector3d right)
        {
            return new Vector3d(left.X / right.X, left.Y / right.Y, left.Z / right.Z);
        }
        public static bool operator ==(Vector3d left, Vector3d right)
        {
            return left.X == right.X && left.Y == right.Y && left.Z == right.Z;
        }
        public static bool operator !=(Vector3d left, Vector3d right)
        {
            return left.X != right.X || left.Y != right.Y || left.Z != right.Z;
        }
    }
}
