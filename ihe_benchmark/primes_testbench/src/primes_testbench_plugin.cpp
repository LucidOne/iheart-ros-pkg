#include <pluginlib/class_list_macros.h>
#include <primes_testbench/primes_testbench_plugin.h>

PLUGINLIB_DECLARE_CLASS(testbench, test_primes, test_plugins::Primes, test_base::Test)
