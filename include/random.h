#ifndef random_h__
#define random_h__

#include <utils.h>

/*
#include <random>

inline double uniform_rand(int seed = 0)
{
  static std::mt19937 engine;
  static std::uniform_real<double> uniform(0,1);
  if (seed) engine.seed(seed);
  return 1.0-uniform(engine);
}

inline double uniform_rand_double(double range)
{
  return uniform_rand()*range;
}

inline int uniform_rand_int(int range)
{
  return int(uniform_rand() * range);
}

inline int uniform_rand_int(int min, int max)
{
  int range=max-min+1;
  return uniform_rand_int(range)+min;
}

inline double normal_rand()
{
  static std::mt19937 engine;
  static std::normal_distribution<double> norm_dist;
  return norm_dist(engine);
}

inline double normal_rand(double mu, double sigma)
{
  return normal_rand()*sigma+mu;
}
*/

#endif // random_h__
