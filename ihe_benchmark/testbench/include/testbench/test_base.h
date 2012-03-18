#ifndef TESTBENCH_TEST_BASE_H_
#define TESTBENCH_TEST_BASE_H_

namespace test_base
{
  class Test
  {
    public:
      virtual void initialize() = 0;
      virtual double run() = 0;
      virtual ~Test(){}

    protected:
      Test(){}
  };
};
#endif
