/*
 * Created on Fri Nov 12 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#include <Eigen/Core>

#include <iostream>

template <int N> struct Jet {
 public:
  Jet() {}
  Jet(const double _a, const Eigen::Matrix<double, 1, N>& _v) {
    a = _a;
    v = _v;
  }
  Jet operator+(const Jet& rhs) const {
    return Jet(a + rhs.a, v + rhs.v);
  }
  Jet operator-(const Jet& rhs) const{
    return Jet(a - rhs.a, v - rhs.v);
  }
  Jet operator*(const Jet& rhs) const{
    return Jet(a * rhs.a, v * rhs.a + a * rhs.v);
  }
  Jet operator/(const Jet& rhs) const{
    return Jet(a / rhs.a, v / rhs.a - a * rhs.v / (rhs.a * rhs.a));
  }

  double a;
  Eigen::Matrix<double, 1, N> v;
};

// f(x, y) = x ^ 2 + xy
class simpleFunctor {
 public:
  simpleFunctor() {}

  template <typename T>
  bool operator() (const T* parameters, T* residuals) const {
    const T x = parameters[0];
    const T y = parameters[1];
    residuals[0] = x * x + x * y;
  }

 private:
  int y_ = 0; // any value is ok.
};


int main() {
  double working_point[2] = {1, 3};
  Jet<2> jets[2];
  for (int i = 0; i < 2; ++i) {
    jets[i].a = working_point[i];
    jets[i].v.setZero();
    jets[i].v[i] = 1.0;
  }
  
  Jet<2> res;
  simpleFunctor sf;
  sf(jets, &res);
  for (int i = 0; i < 2; ++i) {
    std::cout << res.v[i] << std::endl;
  }
}
