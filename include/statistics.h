#ifndef statistics_h__
#define statistics_h__

#include <algorithm>
#include <numeric>
#include <prims.h>

// template<class T>
// inline T sqr(const T& t) { return t*t; }

template<class II, class OI>
void calculate_difference(II b, II e, OI o)
{
  double prev=*b;
  while(++b!=e)
    *o++ = (*b - prev);
}

template<class II>
double calculate_mean(II b, II e)
{
  int n=int(std::distance(b,e));
  if (n<=0) return 0;
  auto sum=std::accumulate(b,e,0.0);
  return (1.0/n)*sum;
}

template<class II>
double calculate_variance(II b, II e, bool biased = true)
{
  if (b == e) return 0;
  unsigned n = unsigned(std::distance(b, e));
  double avg = calculate_mean(b, e);
  double sum=0;
  for (; b != e; ++b)
  {
    //sum+=sqr(*b - avg);
    double value = *b;
    double diff = value - avg;
    double sdiff = sqr(diff);
    sum += sdiff;
  }
  if (biased)
    return sum/n;
  return sum / (n-1);
}

template<class II>
double calculate_stdev(II b, II e, bool biased=true)
{
  return sqrt(calculate_variance(b, e, biased));
}

template<class II>
double calculate_variance_coefficient(II b, II e)
{
  if (b==e) return 0.0;
  double var=calculate_variance(b,e);
  double avg=std::accumulate(b,e,0.0)/std::distance(b,e);
  return var/avg;
}


template<class II>
inline void linear_regression(II xb, II xe, II yb, II ye, double& m, double& b)
{
  int n=std::distance(yb,ye);
  double xa=std::accumulate(xb,xe,0.0)/n;
  double ya=std::accumulate(yb,ye,0.0)/n;
  double s1=0,s2=0;
  II xit=xb;
  for(II it=yb;it!=ye;++it,++xit)
  {
    double dx=(*xit - xa);
    double dy=(*it - ya);
    s1+=dx*dy;
    s2+=dx*dx;
  }
  m=s1/s2;
  b=ya-m*xa;
}

template<class II>
inline void linear_regression(II yb, II ye, double& m, double& b)
{
  std::vector<double> xt;
  int x=0;
  for(II it=yb;it!=ye;++it,++x)
    xt.push_back(x);
  linear_regression(xt.begin(),xt.end(),yb,ye,m,b);
}

class RunningStatistics
{
  double m_Sum;
  double m_Sum2;
  int m_N;
public:
  RunningStatistics() : m_Sum(0), m_Sum2(0), m_N(0) {}

  RunningStatistics& operator() (double value)
  {
    m_Sum += value;
    m_Sum2 += sqr(value);
    m_N++;
    return *this;
  }

  double mean() const
  {
    if (m_N == 0) return 0;
    return m_Sum / m_N;
  }

  double variance() const
  {
    if (m_N == 0) return 0;
    return m_Sum2 / m_N - sqr(mean());
  }

  double stdev() const
  {
    if (m_N == 0) return 0;
    return sqrt(variance());
  }
};



#endif // statistics_h__
