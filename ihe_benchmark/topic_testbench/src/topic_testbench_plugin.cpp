#include <ros/ros.h>
#include <ros/package.h>

#include <pluginlib/class_list_macros.h>
#include <topic_testbench/topic_testbench_plugin.h>

PLUGINLIB_DECLARE_CLASS(testbench, test_topic, test_plugins::Topic, test_base::Test)

namespace test_plugins
{
  double Topic::run()
  {
    int runtime = 30; // test run time
    char roslaunch_bin_path[FILENAME_MAX];
    pid_t launch_pid; // roslaunch pid
    char *argv[64];   // roslaunch argv

    ROS_INFO ("Testing Topic Update Frequency");
    std::string roslaunch_path = ros::package::getPath("roslaunch");
    sprintf (roslaunch_bin_path, "%s/bin/roslaunch", roslaunch_path.c_str ());
    ROS_INFO ("FOO: missing %s", roslaunch_bin_path);

    // roslaunch beginner_tutorials testbench.launch


/*
    // API for roslaunch is currently unstable
    // so fork() and execv() is used to start topic to be tested
    launch_pid = fork ();
    if (launch_pid == -1)
    {
      perror ("Error: fork() failed.");
      exit (-1);
    }
    else if (launch_pid == 0)
    {
      // roslaunch child process
      ROS_INFO("roslaunch: pid = %d\n", (int) getpid ());
      execv(data->rosbag_rec_path, argv);
      // _exit()
      _exit (-1);
    }
    else
    {
      // topic_testbench parent process
      ROS_DEBUG("topic_testbench: child pid = %d\n", (int) launch_pid);
      // monitor then kill
    }
*/
    return 0.5;
  }
}

