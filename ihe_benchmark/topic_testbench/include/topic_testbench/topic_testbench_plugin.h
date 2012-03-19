#ifndef TOPIC_TESTBENCH_TOPIC_TESTBENCH_PLUGIN_H_
#define TOPIC_TESTBENCH_TOPIC_TESTBENCH_PLUGIN_H_
#include "ros/ros.h"
#include <testbench/test_base.h>
#include <cmath>
#include <time.h>

namespace test_plugins
{
  class Topic : public test_base::Test
  {
    public:
      Topic(){}
      void initialize(){} 

      double run();

    private:
      
  };
};
#endif
