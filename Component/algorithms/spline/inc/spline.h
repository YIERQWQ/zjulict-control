#ifndef _SPLINE_H_
#define _SPLINE_H_

#include <cstddef>

namespace spline
{
    // Natural cubic spline for 1D interpolation (float precision)
    // Usage:
    //  CubicSpline s;
    //  s.setPoints(x_array, y_array, n);
    //  float y = s.eval(xq);

    class CubicSpline
    {
    public:
        CubicSpline();
        ~CubicSpline();

        // Set interpolation points. x must be strictly increasing and n>=2.
        // Returns true on success, false on invalid input or allocation failure.
        bool setPoints(const float *x, const float *y, std::size_t n);

        // Evaluate spline at xq. If xq outside [x0, x_{n-1}] it will extrapolate using the
        // end interval polynomial.
        float eval(float xq) const;

        // clear internal storage
        void clear();

        // number of points
        std::size_t size() const { return n_; }

    private:
        std::size_t n_ = 0;
        float *x_ = nullptr; // nodes
        float *a_ = nullptr; // a[i] = y[i]
        float *b_ = nullptr; // first derivative coeff
        float *c_ = nullptr; // second derivative coeff
        float *d_ = nullptr; // third term

        // find interval i such that x in [x_[i], x_[i+1]]
        std::size_t findInterval(float x) const;
    };

} // namespace spline

#endif
