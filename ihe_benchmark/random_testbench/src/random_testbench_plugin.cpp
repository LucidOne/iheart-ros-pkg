#include <pluginlib/class_list_macros.h>
#include <random_testbench/random_testbench_plugin.h>

PLUGINLIB_DECLARE_CLASS(testbench, test_random, test_plugins::Random, test_base::Test)
