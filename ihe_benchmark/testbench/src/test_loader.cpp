#include <pluginlib/class_loader.h>
#include <testbench/test_base.h>

int main(int argc, char** argv)
{
  pluginlib::ClassLoader<test_base::Test> test_loader("testbench", "test_base::Test");

  test_base::Test* random_test = NULL;
  test_base::Test* primes_test = NULL;

  try
  {
    random_test = test_loader.createClassInstance("testbench/test_random");
    random_test->initialize();

    primes_test = test_loader.createClassInstance("testbench/test_primes");
    primes_test->initialize();

    ROS_INFO("Random Test: %.6f", random_test->run());
    ROS_INFO("Primes Test: %.6f", primes_test->run());

  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  return 0;
}
