using System;

namespace KrpcAutoPilot.Utils
{
    public class BiLinearTable
    {
        private LinearTable[] values_;
        private double x_min_;
        private double x_max_;
        private double x_step_;
        private uint n_;

        public BiLinearTable(
            double min1, double step1, uint n1,
            double min2, double step2, uint n2,
            Func<double, double, double> action = null)
        {
            Reset(min1, step1, n1, min2, step2, n2, action);
        }

        public void Reset(
            double min1, double step1, uint n1,
            double min2, double step2, uint n2,
            Func<double, double, double> action = null)
        {
            x_min_ = min1;
            x_max_ = min1 + step1 * (n1 - 1);
            x_step_ = step1;
            n_ = n1;
            values_ = new LinearTable[n1];
            if (action == null)
            {
                for (uint i = 0; i < n1; i++)
                    values_[i] = new LinearTable(min2, step2, n2);
            }
            else
            {
                for (uint i = 0; i < n1; i++)
                {
                    double x = x_min_ + i * x_step_;
                    values_[i] = new LinearTable(
                        min2, step2, n2,
                        (double x2) => action(x, x2));
                }
            }
        }

        public double this[double x1, double x2]
        {
            get
            {
                if (x1 < x_min_)
                    return values_[0][x2];
                if (x1 >= x_max_)
                    return values_[n_ - 1][x2];
                uint i1 = (uint)((x1 - x_min_) / x_step_);
                uint i2 = i1 + 1;
                double x11 = x_min_ + x_step_ * i1;
                double x12 = x_min_ + x_step_ * i2;
                return (values_[i1][x2] * (x12 - x1) + values_[i2][x2] * (x1 - x11)) / x_step_;
            }
            set
            {
                double i = (x1 - x_min_) / x_step_;
                uint ii = (uint)i;
                if (i - ii > x_step_ / 100d)
                    throw new Exception();
                values_[ii][x2] = value;
            }
        }
    }
}
