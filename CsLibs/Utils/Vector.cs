using System;
using System.Collections.Generic;

namespace KrpcAutoPilot.Utils
{
    public class Vector3d
    {
        private readonly double[] v_ = new double[3];

        public double X
        {
            get => v_[0];
            set => v_[0] = value;
        }
        public double Y
        {
            get => v_[1];
            set => v_[1] = value;
        }
        public double Z
        {
            get => v_[2];
            set => v_[2] = value;
        }
        public double this[int i]
        {
            get => v_[i];
            set => v_[i] = value;
        }

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
        public Vector3d(double[] v)
        {
            X = v[0];
            Y = v[1];
            Z = v[2];
        }
        public Vector3d(Vector3d v)
        {
            X = v.X;
            Y = v.Y;
            Z = v.Z;
        }

        public static Vector3d One
        {
            get => new Vector3d(1.0, 1.0, 1.0);
        }
        public static Vector3d Zero
        {
            get => new Vector3d(0.0, 0.0, 0.0);
        }
        public static Vector3d Cross(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(
                v1.Y * v2.Z - v1.Z * v2.Y,
                v1.Z * v2.X - v1.X * v2.Z,
                v1.X * v2.Y - v1.Y * v2.X);
        }
        public double Length()
        {
            return System.Math.Sqrt(X * X + Y * Y + Z * Z);
        }
        public double LengthSquared()
        {
            return X * X + Y * Y + Z * Z;
        }
        public Vector3d Norm()
        {
            return new Vector3d(X, Y, Z) / Math.Max(1e-9, Length());
        }
        public Tuple<double, double, double> ToTuple()
        {
            return new Tuple<double, double, double>(X, Y, Z);
        }
        public Matrix3d RotationMatrix()
        {
            double theta = Length();
            if (theta < 1e-9)
                return Matrix3d.One;
            double cos_t = Math.Cos(theta);
            double sin_t = Math.Sin(theta);
            double cos_t2 = 1d - cos_t;
            Vector3d u = this / theta;
            double xx = u.X * u.X;
            double xy = u.X * u.Y;
            double xz = u.X * u.Z;
            double yy = u.Y * u.Y;
            double yz = u.Y * u.Z;
            double zz = u.Z * u.Z;
            double r00 = cos_t + xx * cos_t2;
            double r01 = xy * cos_t2 - u.Z * sin_t;
            double r02 = xz * cos_t2 + u.Y * sin_t;
            double r10 = xy * cos_t2 + u.Z * sin_t;
            double r11 = cos_t + yy * cos_t2;
            double r12 = yz * cos_t2 - u.X * sin_t;
            double r20 = xz * cos_t2 - u.Y * sin_t;
            double r21 = yz * cos_t2 + u.X * sin_t;
            double r22 = cos_t + zz * cos_t2;
            return new Matrix3d(new double[][]
            {
                new double[]{ r00, r01, r02 },
                new double[]{ r10, r11, r12 },
                new double[]{ r20, r21, r22 }
            });
        }
        public override string ToString()
        {
            return string.Format("{0}\t{1}\t{2}", X, Y, Z);
        }
        public string ToString(string format)
        {
            string format_str = "{0:" + format + "}\t{1:" + format + "}\t{2:" + format + "}";
            return string.Format(format_str, X, Y, Z);
        }
        public Vector3d Abs()
        {
            return new Vector3d(Math.Abs(X), Math.Abs(Y), Math.Abs(Z));
        }
        public double Max()
        {
            return Math.Max(X, Math.Max(Y, Z));
        }
        public double Min()
        {
            return Math.Min(X, Math.Min(Y, Z));
        }
        public void SelectMax(double v)
        {
            for (int i = 0; i < 3; i++)
                v_[i] = Math.Max(v_[i], v);
        }
        public void SelectMin(double v)
        {
            for (int i = 0; i < 3; i++)
                v_[i] = Math.Min(v_[i], v);
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
        public override bool Equals(object obj)
        {
            return obj is object && Equals(obj as Vector3d);
        }
        public bool Equals(Vector3d v)
        {
            return X == v.X && Y == v.Y && Z == v.Z;
        }
        public override int GetHashCode()
        {
            return X.GetHashCode() ^ Y.GetHashCode() ^ Z.GetHashCode();
        }
    }
    public class TupleV3d
    {
        public Vector3d Item1 { get; private set; }
        public Vector3d Item2 { get; private set; }
        public TupleV3d(Tuple<Tuple<double, double, double>, Tuple<double, double, double>> t)
        {
            Item1 = new Vector3d(t.Item1);
            Item2 = new Vector3d(t.Item2);
        }
        public TupleV3d(Vector3d item1, Vector3d item2)
        {
            Item1 = new Vector3d(item1);
            Item2 = new Vector3d(item2);
        }
        public Tuple<Tuple<double, double, double>, Tuple<double, double, double>> ToTuple()
        {
            return new Tuple<Tuple<double, double, double>, Tuple<double, double, double>>(
                Item1.ToTuple(), Item2.ToTuple());
        }
        public TupleV3d Abs()
        {
            return new TupleV3d(Item1.Abs(), Item2.Abs());
        }
        public double Max()
        {
            return Math.Max(Item1.Max(), Item2.Max());
        }
        public double Min()
        {
            return Math.Min(Item1.Min(), Item2.Min());
        }
        public void SelectMax(double v)
        {
            Item1.SelectMax(v);
            Item2.SelectMax(v);
        }
        public void SelectMin(double v)
        {
            Item1.SelectMin(v);
            Item2.SelectMin(v);
        }
    }
}
