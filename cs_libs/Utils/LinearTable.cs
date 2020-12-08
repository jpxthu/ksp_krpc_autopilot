using System;

namespace KrpcAutoPilot.Utils
{
    public class LinearTable
    {
        private double[] values_;
        private bool[] has_value_;
        private double x_min_;
        private double x_max_;
        private double x_step_;
        private uint n_;
        private bool has_action_;
        private Func<double, double> action_;

        public LinearTable(
            double min, double step, uint n,
            Func<double, double> action = null)
        {
            Reset(min, step, n, action);
        }

        public void Reset(
            double min, double step, uint n,
            Func<double, double> action = null)
        {
            ResetCache(min, step, n);
            if (action == null)
            {
                has_action_ = false;
            }
            else
            {
                has_action_ = true;
                action_ = action;
            }
        }

        public void ResetCache(double min, double step, uint n)
        {
            x_min_ = min;
            x_max_ = min + step * (n - 1);
            x_step_ = step;
            n_ = n;
            values_ = new double[n];
            has_value_ = new bool[n];
            for (uint i = 0; i < n; i++)
                has_value_[i] = false;
        }

        private double GetValue_(uint i, double x)
        {
            if (!has_action_ || has_value_[i])
                return values_[i];
            values_[i] = action_(x);
            has_value_[i] = true;
            return values_[i];
        }
        public double this[double x]
        {
            get
            {
                if (x < x_min_)
                    return GetValue_(0, x_min_);
                if (x >= x_max_)
                    return GetValue_(n_ - 1, x_max_);
                uint i1 = (uint)((x - x_min_) / x_step_);
                uint i2 = i1 + 1;
                double x1 = x_min_ + x_step_ * i1;
                double x2 = x_min_ + x_step_ * i2;
                return (GetValue_(i1, x1) * (x2 - x) + GetValue_(i2, x2) * (x - x1)) / x_step_;
            }
            set
            {
                double i = (x - x_min_) / x_step_;
                uint ii = (uint)i;
                if (i - ii > x_step_ / 100d)
                    throw new Exception();
                values_[ii] = value;
                has_value_[ii] = true;
            }
        }
    }
}
