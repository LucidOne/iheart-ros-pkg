#ifndef PRIMES_TESTBENCH_PRIMES_TESTBENCH_PLUGIN_H_
#define PRIMES_TESTBENCH_PRIMES_TESTBENCH_PLUGIN_H_
#include "ros/ros.h"
#include <testbench/test_base.h>
#include <cmath>
#include <time.h>
#include <openssl/bn.h>


namespace test_plugins
{
  class Primes : public test_base::Test
  {
    public:
      Primes(){}

      void initialize()
      {
      }

      double run()
      {
        int i;
        char number[129];
        int p;
        double time;
        BIGNUM *big = BN_new();
        BIGNUM *r = BN_new();
        BN_CTX *ctx = BN_CTX_new();
        clock_t begin, end;
        begin = clock();
        for (i=0; i<0x10000; i++) {
          sprintf(number,"FF00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000%04X",i);
          BN_hex2bn(&big, number);
          p = BN_is_prime(big, BN_prime_checks, 0, ctx, NULL);
          if (p==1) {
            ROS_DEBUG("number: %s is_prime: %d\n",number,p);
            ROS_DEBUG("n: %s\n",BN_bn2dec(big));
          }
        }
        end = clock();
        time = (double)(end-begin)/(double)CLOCKS_PER_SEC;
        ROS_DEBUG("total   time = %6.6f\n", time);
        BN_free(big);
        BN_free(r);
        BN_CTX_free(ctx);
        if (time < 1) {
          return (double)1;
        } else {
          return (1.0/(1.0+log(time)/10.0));
        }
      }

    private:
      
  };
};
#endif
