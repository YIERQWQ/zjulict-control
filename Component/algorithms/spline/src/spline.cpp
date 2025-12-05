#include "spline.h"
#include <cstdlib>
#include <cmath>
#include <new>

namespace spline
{
    CubicSpline::CubicSpline() {}

    CubicSpline::~CubicSpline()
    {
        clear();
    }

    void CubicSpline::clear()
    {
        if (x_)
            delete[] x_;
        if (a_)
            delete[] a_;
        if (b_)
            delete[] b_;
        if (c_)
            delete[] c_;
        if (d_)
            delete[] d_;

        x_ = a_ = b_ = c_ = d_ = nullptr;
        n_ = 0;
    }

    bool CubicSpline::setPoints(const float *x, const float *y, std::size_t n)
    {
        if (!x || !y || n < 2)
            return false;

        // check strictly increasing x
        for (std::size_t i = 1; i < n; ++i)
        {
            if (!(x[i] > x[i - 1]))
                return false;
        }

        // allocate
        clear();
        n_ = n;
        x_ = new (std::nothrow) float[n_];
        a_ = new (std::nothrow) float[n_];
        b_ = new (std::nothrow) float[n_];
        c_ = new (std::nothrow) float[n_];
        d_ = new (std::nothrow) float[n_];

        if (!x_ || !a_ || !b_ || !c_ || !d_)
        {
            clear();
            return false;
        }

        for (std::size_t i = 0; i < n_; ++i)
        {
            x_[i] = x[i];
            a_[i] = y[i];
        }

        if (n_ == 2)
        {
            // simple linear spline (degenerate cubic)
            float h = x_[1] - x_[0];
            b_[0] = (a_[1] - a_[0]) / h;
            c_[0] = 0.0f;
            d_[0] = 0.0f;
            return true;
        }

        // natural cubic spline: solve tridiagonal system for c
        // h[i] = x[i+1] - x[i]
        float *h = nullptr;
        float *alpha = nullptr;
        float *l = nullptr;
        float *mu = nullptr;
        float *z = nullptr;

        h = new (std::nothrow) float[n_ - 1];
        alpha = new (std::nothrow) float[n_ - 1];
        l = new (std::nothrow) float[n_];
        mu = new (std::nothrow) float[n_];
        z = new (std::nothrow) float[n_];

        if (!h || !alpha || !l || !mu || !z)
        {
            delete[] h; delete[] alpha; delete[] l; delete[] mu; delete[] z;
            clear();
            return false;
        }

        for (std::size_t i = 0; i < n_ - 1; ++i)
            h[i] = x_[i + 1] - x_[i];

        // compute alpha
        for (std::size_t i = 1; i < n_ - 1; ++i)
            alpha[i] = (3.0f / h[i]) * (a_[i + 1] - a_[i]) - (3.0f / h[i - 1]) * (a_[i] - a_[i - 1]);

        // boundary conditions: natural
        l[0] = 1.0f;
        mu[0] = 0.0f;
        z[0] = 0.0f;

        for (std::size_t i = 1; i < n_ - 1; ++i)
        {
            l[i] = 2.0f * (x_[i + 1] - x_[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n_ - 1] = 1.0f;
        z[n_ - 1] = 0.0f;
        c_[n_ - 1] = 0.0f;

        // back substitution
        for (std::size_t j = n_ - 1; j-- > 0;)
        {
            c_[j] = z[j] - mu[j] * c_[j + 1];
            b_[j] = (a_[j + 1] - a_[j]) / h[j] - h[j] * (c_[j + 1] + 2.0f * c_[j]) / 3.0f;
            d_[j] = (c_[j + 1] - c_[j]) / (3.0f * h[j]);
        }

        // last element b/d not used; keep zeros
        b_[n_ - 1] = 0.0f;
        d_[n_ - 1] = 0.0f;

        delete[] h;
        delete[] alpha;
        delete[] l;
        delete[] mu;
        delete[] z;

        return true;
    }

    std::size_t CubicSpline::findInterval(float x) const
    {
        if (n_ <= 1)
            return 0;
        // binary search
        std::size_t low = 0, high = n_ - 1;
        if (x <= x_[0])
            return 0;
        if (x >= x_[n_ - 1])
            return n_ - 2;

        while (high - low > 1)
        {
            std::size_t mid = (low + high) >> 1;
            if (x_[mid] > x)
                high = mid;
            else
                low = mid;
        }
        return low;
    }

    float CubicSpline::eval(float xq) const
    {
        if (n_ == 0 || !x_)
            return 0.0f;
        if (n_ == 1)
            return a_[0];

        std::size_t i = findInterval(xq);
        float dx = xq - x_[i];
        // S(x) = a[i] + b[i]*dx + c[i]*dx^2 + d[i]*dx^3
        return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
    }

} // namespace spline
