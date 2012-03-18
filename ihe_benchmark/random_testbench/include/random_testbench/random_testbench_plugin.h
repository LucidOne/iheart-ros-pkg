#ifndef RANDOM_TESTBENCH_RANDOM_TESTBENCH_PLUGIN_H_
#define RANDOM_TESTBENCH_RANDOM_TESTBENCH_PLUGIN_H_
#include <testbench/test_base.h>
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
        srand((unsigned)time(0));
      }

      double run()
      {
        return (double)rand()/(double)RAND_MAX;
      }

  };
};
#endif
