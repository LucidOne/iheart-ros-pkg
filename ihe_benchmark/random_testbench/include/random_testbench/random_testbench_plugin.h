#ifndef RANDOM_TESTBENCH_RANDOM_TESTBENCH_PLUGIN_H_
#define RANDOM_TESTBENCH_RANDOM_TESTBENCH_PLUGIN_H_
#include <testbench/test_base.h>
#include "ros/ros.h"
#include <cmath>
#include <stdlib.h>

namespace test_plugins
{
  class Random : public test_base::Test
  {
    public:
      Random(){}

      void initialize()
      {
        random_seed += (unsigned int)time(0);
        srand(random_seed);
      }

      double run()
      {
        return (double)rand()/(double)RAND_MAX;
      }

    private:
      unsigned int random_seed;

  };
};
#endif
