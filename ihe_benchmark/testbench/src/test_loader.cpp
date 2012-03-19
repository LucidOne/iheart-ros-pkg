#include <fstream>
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"
#include <pluginlib/class_loader.h>
#include <testbench/test_base.h>

struct Benchmark {
   std::string plugin;
   std::string name;
   int weight;
};

void operator >> (const YAML::Node& node, Benchmark& mark) {
  node["plugin"] >> mark.plugin;
  node["name"] >> mark.name;
  node["weight"] >> mark.weight;
}

int main(int argc, char** argv)
{
  double total_score = 0;
  pluginlib::ClassLoader<test_base::Test> test_loader("testbench", "test_base::Test");
  test_base::Test* test_plugin = NULL;

  std::string checklist_file = "checklist.yaml";
// private_nh.param("checklist", checklist_file, std::string("checklist.yaml"));
  std::ifstream checklist(checklist_file.c_str());
  if (checklist.fail()) {
    ROS_ERROR("Testbench could not open config file: %s", checklist_file.c_str());
    exit(-1);
  }

  YAML::Parser parser(checklist);
  if (!parser) {
    ROS_ERROR("Can not create parser");
    exit(-1);
  }

  YAML::Node doc;
  parser.GetNextDocument(doc);
  for(unsigned i=0;i<doc.size();i++) {
    Benchmark benchmark;
    double score;

    doc[i] >> benchmark; 
    try
    {
      test_plugin = test_loader.createClassInstance(benchmark.plugin);
      test_plugin->initialize();
      score = test_plugin->run();
      total_score += (double)benchmark.weight * score;
      ROS_INFO("%s: %.6f", benchmark.name.c_str(), score);
    }
    catch(pluginlib::PluginlibException& e)
    {
      ROS_ERROR("Error Loading Plugin: %s", e.what());
    }
  }
  ROS_INFO("TOTAL SCORE: %.6f", total_score);
  return 0;
}
