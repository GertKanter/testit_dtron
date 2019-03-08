/*
   Copyright (c) Gert Kanter
 */

#include <xtaproto.pb.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <cstring>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include "sp.h"
#include <tf/tf.h>
#include <iostream>
#include <fstream>

#include <move_base_msgs/MoveBaseAction.h>
#include <topological_navigation/GotoNodeAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>

#include <testit_msgs/Coverage.h>


struct SpreadMessage {
  int Type;
  char* Sender;
  char* Group;
  char* Msg;
};

class SpreadAdapter {
public:
  SpreadAdapter(const char* SpreadName, const char* UserName, boost::function<void (int, char*, char*, char*)>);
  SpreadAdapter(mailbox* spreadMailBox);
  int JoinGroup(const char * Name);
  void ReaderThread();
  SpreadMessage ReadMessage();
  int SendMessage(const char* channel, std::string data);
  mailbox* Mailbox();
  bool isConnActive();
private:
  boost::function<void (int, char*, char*, char*)> callback;
  bool connectionActive;
  mailbox* Mbox;
  char* User;
  char* Spread;
  char* PrivateGroup;
};


SpreadAdapter::SpreadAdapter(const char* SpreadName, const char* UserName, boost::function<void (int, char*, char*, char*)> callbackFunction) {
  int ret;
  Mbox = new mailbox;
  PrivateGroup = new char[80];
  callback = callbackFunction;
  ret = SP_connect(SpreadName, UserName, 0, 1, Mbox, PrivateGroup);
  if(ret < 0) {
    SP_error(ret);
    connectionActive = false;
  } else {
    connectionActive = true;
  }
}

bool SpreadAdapter::isConnActive() {
  return connectionActive;
}

mailbox* SpreadAdapter::Mailbox() {
  return Mbox;
}

SpreadAdapter::SpreadAdapter(mailbox* spreadMailBox) {
  Mbox = new mailbox;
  Mbox = spreadMailBox;
  connectionActive = true;
}

int SpreadAdapter::JoinGroup(const char* Name) {
  return SP_join(*Mbox, Name);
}


void SpreadAdapter::ReaderThread() {
  SpreadMessage spreadMessage;
  do {
    spreadMessage = SpreadAdapter::ReadMessage();
    if(spreadMessage.Type != -1) { 
      callback(spreadMessage.Type, spreadMessage.Sender, spreadMessage.Group, spreadMessage.Msg);
    }
  } while (spreadMessage.Type != -1);
}

int SpreadAdapter::SendMessage(const char* channel, std::string data) {
  if(!connectionActive) {
    printf("Error: Spread server is not running!\n");
    return -1;
  }
  int ret;
  ret = SP_multicast(*Mbox, AGREED_MESS, channel, 1, strlen(data.c_str()), data.c_str());
  return ret;
}

SpreadMessage SpreadAdapter::ReadMessage() {
  SpreadMessage spreadMessage;
  if(!connectionActive) {
    spreadMessage.Type = -1;
    char messageSender[] = "";
    char messageGroup[] = "";
    char spreadMsg[] = "Error: Spread server is not running!\n";
    spreadMessage.Sender = new char[MAX_GROUP_NAME];
    spreadMessage.Sender = messageSender;
    spreadMessage.Group = new char[MAX_GROUP_NAME];
    spreadMessage.Group = messageGroup;
    spreadMessage.Msg = new char[102400];
    spreadMessage.Msg = spreadMsg;
    return spreadMessage;
  }
  static char message[102400];
  char sender[MAX_GROUP_NAME];
  char target_groups[100][MAX_GROUP_NAME];
  int	num_groups, service_type, endian_mismatch, ret;
  membership_info memb_info;
  int16 mess_type;

  service_type = 0;
  ret = SP_receive( *Mbox, &service_type, sender, 100, &num_groups, target_groups, &mess_type, &endian_mismatch, sizeof(message), message );
  if(ret < 0) {
    SP_error(ret);
  }

  if(Is_regular_mess(service_type)){
    /* A regular message, sent by one of the processes */
    message[ret] = 0;
  } else if( Is_membership_mess(service_type)){
    /* A membership notification */
    ret = SP_get_memb_info(message, service_type, &memb_info);
    if (ret < 0) {
      printf("Bug: membership message does not have valid body\n");
      SP_error(ret);
    }
  }
  spreadMessage.Type = service_type;
  spreadMessage.Sender = new char[MAX_GROUP_NAME];
  spreadMessage.Sender = sender;
  spreadMessage.Group = new char[MAX_GROUP_NAME];
  spreadMessage.Group = target_groups[0];
  spreadMessage.Msg = new char[102400];
  spreadMessage.Msg = message;
  return spreadMessage;
}

namespace dtron_test_adapter {
  class TestAdapter {
    private:
      ros::NodeHandle nh_;
      SpreadAdapter spreadAdapter_;
      boost::function<void (std::string, std::map<std::string, int>)> receiveMessage_;
    public:
      TestAdapter(ros::NodeHandle nh, 
          const char* spread_host, 
          const char* spread_username, 
          std::vector<const char*> groups,
          boost::function<void (std::string, std::map<std::string, int>)> receiveMessage);
      void sendMessage(const char* group, std::map<std::string, int> args);
      void spreadMessageCallback(int type, char* sender, char* group, char* msg);
  };
}


namespace dtron_test_adapter {

  TestAdapter::TestAdapter(
      ros::NodeHandle nh,
      const char* spread_host,
      const char* spread_username,
      std::vector<const char*> groups,
      boost::function<void (std::string, std::map<std::string, int>)> receiveMessage) :
    nh_(nh),
    spreadAdapter_(spread_host, spread_username, boost::bind(&TestAdapter::spreadMessageCallback, this, _1, _2, _3, _4)),
    receiveMessage_(receiveMessage) {
      GOOGLE_PROTOBUF_VERIFY_VERSION;
      if(spreadAdapter_.isConnActive()) {
        ROS_INFO("Successfully connected to Spread server!");
        for (unsigned int i = 0; i < groups.size(); ++i) {
          spreadAdapter_.JoinGroup(groups[i]);
        }
        boost::thread spreadMessageReader(&SpreadAdapter::ReaderThread, &spreadAdapter_);
      } else {
        ROS_ERROR("Error: Spread server is not running!\n");
      }
    }

  void TestAdapter::sendMessage(const char* group, std::map<std::string, int> args) {
    Sync response;
    response.set_name(group);
    int i = 0;
    for (std::map<std::string, int>::const_iterator it = args.begin(); it != args.end(); ++it)
    {
      Variable* var = response.add_variables();
      ROS_INFO_STREAM("Setting name = " << it->first << "  value = " << it->second << " variables_size = " << response.variables_size());
      response.mutable_variables(i)->set_name(it->first);
      response.mutable_variables(i)->set_value(it->second);
      i++;
    }
    std::string data = "";
    response.SerializeToString(&data);
    ROS_INFO_STREAM("data size = " << data.size());
    spreadAdapter_.SendMessage(group, data);
  }

  void TestAdapter::spreadMessageCallback(int type, char* sender, char* group, char* msg) {
    Sync sync;
    sync.ParseFromString(msg);
    if ((sync.name() != "") && sender != NULL) {
      printf("[Google protocol buffers]: Channel: '%s', Sender: '%s'\n", sync.name().c_str(), sender);
      std::map<std::string, int> mapOfVariables;
      for (unsigned int i = 0; i < sync.variables_size(); ++i) {
        mapOfVariables.insert(std::pair<std::string, int>(sync.variables(i).name(), sync.variables(i).value()));
      }
      receiveMessage_(sync.name(), mapOfVariables);
    } else {
      if (sender == NULL) {
        printf("Received incorrectly formed message: %s\n", msg);
      }
    }
  }
}

class Adapter {
private:
  dtron_test_adapter::TestAdapter* testAdapter_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_movebase_;
  actionlib::SimpleActionClient<topological_navigation::GotoNodeAction> ac_topological_;
  std::map<std::string, std::string> node_map_;
  ros::NodeHandle nh_;
  std::vector<std::string> sync_input_;
  std::string sync_output_;
  std::string robot_name_;
  std::string object_detector_topic_;
  ros::Subscriber object_detector_sub_;
  std::string coverage_format_;
  std::string coverage_output_;
  double coverage_trace_start_timestamp_;
  bool object_detected_;
  ros::ServiceClient sut_coverage_client_;
public:
  Adapter(ros::NodeHandle nh,
      std::string goal_topic,
      std::vector<std::string> sync_input,
      std::string sync_output,
      std::string waypoint_goal_topic,
      std::string robot_name,
      std::string object_detector_topic,
      std::string coverage_format,
      std::string coverage_output) :
    ac_movebase_(goal_topic, true),
    ac_topological_(waypoint_goal_topic, true),
    nh_(nh),
    sync_input_(sync_input),
    sync_output_(sync_output),
    robot_name_(robot_name),
    object_detector_topic_(object_detector_topic),
    object_detected_(false),
    coverage_format_(coverage_format),
    coverage_output_(coverage_output),
    coverage_trace_start_timestamp_(ros::WallTime::now().toSec())
    {
      sut_coverage_client_ = nh_.serviceClient<testit_msgs::Coverage>("/testit/flush_coverage");
      ROS_INFO("ROBOT NAME IN ADAPTER %s", robot_name_.c_str());
      nh.getParam("/test_adapter/node_map", node_map_);
      ROS_WARN("Loaded node map with %lu nodes!", node_map_.size());
      //if (navigation_mode_ != "waypoint") {
        //ROS_INFO("Connecting to move_base action server @ %s", goal_topic.c_str());
        /*ac_movebase_.waitForServer(ros::Duration(1.0));
        if (!ac_movebase_.isServerConnected()) {
          ROS_ERROR("Unable to connect to move_base action server!");
        }*/
      //} else {
        //ROS_INFO("Connecting to topological_navigation action server @ %s", waypoint_goal_topic.c_str());
        /*ac_topological_.waitForServer(ros::Duration(4.0));
        if (!ac_topological_.isServerConnected()) {
          ROS_ERROR("Unable to connect to topological_navigation action server!");
        }*/
      //}
      // Subscribe to topics
      if (object_detector_topic_ != "")
        object_detector_sub_ = nh_.subscribe(object_detector_topic_, 10, &Adapter::objectDetectorCallback, this);
      ROS_INFO("Adapter is ready for use!");
    }

  void objectDetectorCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data)
      object_detected_ = true;
    else
      object_detected_ = false;
  }

  void setTestAdapter(dtron_test_adapter::TestAdapter testAdapter) {
    testAdapter_ = &testAdapter;
  }

  bool flushCoverage(std::string name, std::map<std::string, int> args, std::string event) {
    if ((name.find("goto") != std::string::npos) ||
        (name.find("moveto") != std::string::npos) ||
        (name.find("object_detect") != std::string::npos)
       )
    {
      ROS_DEBUG_STREAM("Flushing coverage information");
      // Open coverage results file
      std::ofstream coverage_file;
      coverage_file.open ((coverage_output_ + "/testit_coverage.log").c_str(), std::ios::app);
      testit_msgs::Coverage coverage_results;
      if (sut_coverage_client_.call(coverage_results))
      {
        ROS_DEBUG("Flush call success");
        if (coverage_format_ == "yaml" || coverage_format_ == "")
        {
	  double timestamp = (ros::Time::now()).toSec();
	  for (int i = 0; i < coverage_results.response.coverage.size(); ++i) {
	    ROS_INFO_STREAM("FILE " << coverage_results.response.coverage[i].filename << "  TOTAL LINES " << coverage_results.response.coverage[i].lines.size());
	    coverage_file << "- traceStartTimestamp: " << std::setprecision(18) << coverage_trace_start_timestamp_ << "\n"
                          << "  timestamp: " << timestamp << "\n"
	                  << "  event: " << event << "\n"
                          << "  name: " << name << "\n"
			  << "  state: [";
	    std::map<std::string, int>::iterator it;
	    bool add_separator = false;
	    for (it = args.begin(); it != args.end(); it++) {
              if (add_separator)
	        coverage_file << ", ";
	      coverage_file << "'" << it->first << "': " << it->second;
	      add_separator = true;
	    }
	    coverage_file << "]\n"
	                  << "  file: " << coverage_results.response.coverage[i].filename << "\n"
			  << "  lines: [";
	    for (int j = 0; j < coverage_results.response.coverage[i].lines.size(); ++j) {
	      coverage_file << coverage_results.response.coverage[i].lines[j];
	      if (j+1 < coverage_results.response.coverage[i].lines.size())
	        coverage_file << ", ";
	    }
	    coverage_file << "]\n"
	                  << "  sum: "
			  << coverage_results.response.coverage[i].lines.size()
			  << "\n";
	    }
        }
        else
          ROS_ERROR("Unknown coverage log format!");
      coverage_file.close();
      }
      else
      {
        ROS_ERROR("Flush call failed!");
      }
    }
    return true;
  }

  void receiveMessage(std::string name, std::map<std::string, int> args) {
    ROS_INFO_STREAM("Received a message with name '" << name << "'");
    flushCoverage(name, args, "PRE");
    if (name.find("goto") != std::string::npos)
    {
      // "goto" sync
      ROS_DEBUG("goto sync handler!");
      std::string waypoint = boost::lexical_cast<std::string>(args["waypoint"]);
      int mode = 1;
      if (args.count("mode") == 1)
        mode = args["mode"];
      std::string node_name = node_map_[waypoint];
      std::string robot_name = robot_name_;
      if(node_name != "")
      {
        actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::PENDING);
        std::map<std::string, int> vars;
        if (mode == 1) // topological_navigation mode
        {
          topological_navigation::GotoNodeGoal goal;
          goal.target = node_name;
          ROS_INFO("Goal node = %s", node_name.c_str());
          if (ac_topological_.isServerConnected())
          {
            ac_topological_.sendGoal(goal);
            ac_topological_.waitForResult();
            state = ac_topological_.getState();
          }
          else {
            while (!ac_topological_.isServerConnected()) {
              ROS_ERROR("Topological_navigation action server not connected! Waiting for server (10 s)...");
              ac_topological_.waitForServer(ros::Duration(10, 0));
            }
            receiveMessage(name, args); // Call self to retry
            return;
          }
        }
        else
        {
          move_base_msgs::MoveBaseGoal goal;
          double x, y;
          nh_.getParam("/test_adapter/nodes/" + node_name + "/x", x);
          nh_.getParam("/test_adapter/nodes/" + node_name + "/y", y);
          goal.target_pose.header.frame_id = "/map";
          goal.target_pose.pose.position.x = x;
          goal.target_pose.pose.position.y = y;
          goal.target_pose.pose.position.z = 0;
          goal.target_pose.pose.orientation.x = 0;
          goal.target_pose.pose.orientation.y = 0;
          goal.target_pose.pose.orientation.z = 0;
          goal.target_pose.pose.orientation.w = 1;
          ROS_INFO("Goal. Node %s, x: %.3f  y: %.3f", node_name.c_str(), x, y);
          if (ac_movebase_.isServerConnected())
          {
            ac_movebase_.sendGoal(goal);
            ac_movebase_.waitForResult();
            state = ac_movebase_.getState();
          }
          else {
            while (!ac_movebase_.isServerConnected()) {
              ROS_ERROR("Move_base action server not connected! Waiting for server (10 s)...");
              ac_movebase_.waitForServer(ros::Duration(10, 0));
            }
            receiveMessage(name, args); // Call self to retry
            return;
          }
        }
        ROS_INFO("Action finished with state = %s", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          vars["value"] = 1; // 1 == SUCCESS
        }
        else
        {
          vars["value"] = -1; // -1 == FAIL
          ROS_WARN("Action result was not SUCCEEDED!");
        }
        testAdapter_->sendMessage(sync_output_.c_str(), vars);
      }
      else
      {
        ROS_ERROR_STREAM("Unknown goal waypoint (" << waypoint << ")! Sync ignored!");
      }
    }
    else if (name.find("moveto") != std::string::npos)
    {
      // "moveto" sync
      ROS_DEBUG("moveto sync handler!");
      int x = boost::lexical_cast<int>(args["x"]);
      int y = boost::lexical_cast<int>(args["y"]);
      int a = boost::lexical_cast<int>(args["a"]);
      move_base_msgs::MoveBaseGoal goal;
      double dx, dy, da;
      dx = ((double)x) / 10.0;
      dy = ((double)y) / 10.0;
      da = ((double)a) / 10.0;
      goal.target_pose.header.frame_id = "/map";
      goal.target_pose.pose.position.x = dx;
      goal.target_pose.pose.position.y = dy;
      goal.target_pose.pose.position.z = 0;
      tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, da);
      tf::quaternionTFToMsg(q, goal.target_pose.pose.orientation);
      ROS_INFO("Goal x: %.2f  y: %.2f  a: %.2f", dx, dy, da);
      if (ac_movebase_.isServerConnected())
      {
        ac_movebase_.sendGoal(goal);
        ac_movebase_.waitForResult();
        actionlib::SimpleClientGoalState state = ac_movebase_.getState();
        ROS_INFO("Action finished with state = %s", state.toString().c_str());
        std::map<std::string, int> vars;
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          vars["value"] = 1; // 1 == SUCCESS
        }
        else
        {
          vars["value"] = -1; // -1 == FAIL
          ROS_ERROR("Action result was not SUCCEEDED!");
        }
        testAdapter_->sendMessage(sync_output_.c_str(), vars);
      }
      else
      {
        ROS_ERROR("Move_base action server not connected!");
      }
    }
    else if (name.find("object_detect") != std::string::npos)
    {
      // "object_detect" sync
      ROS_INFO("object_detect sync handler!");
      std::map<std::string, int> vars;
      if (object_detected_)
        vars["value"] = 2; // detected == 2
      else
        vars["value"] = 1; // not detected == 1
      testAdapter_->sendMessage(sync_output_.c_str(), vars);
    }
    flushCoverage(name, args, "POST");
    ROS_INFO("Finished message processing.");
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_adapter");
  ros::NodeHandle nh;
  if (argc <= 1)
  {
    ROS_ERROR("Robot name not defined (must be passed as the first argument)");
    return 1;
  }
  std::string robot_name = argv[1];
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  std::string coverage_format;
  nh.param<std::string>("/test_adapter/coverage/format", coverage_format, "yaml");
  std::string coverage_output;
  nh.param<std::string>("/test_adapter/coverage/output", coverage_output, "");
  if (coverage_output == "")
  {
    ROS_WARN("Coverage results output directory not defined (using working directory)!");
  }

  std::vector<const char*> groups;
  std::vector<std::string> sync_input;
  nh.getParam("/test_adapter/"+robot_name+"/dtron/sync/input", sync_input);
  for (std::vector<std::string>::const_iterator it = sync_input.begin(); it != sync_input.end(); ++it)
  {
    groups.push_back(it->c_str());
  }
  std::string sync_output;
  nh.getParam("/test_adapter/"+robot_name+"/dtron/sync/output", sync_output);
  groups.push_back(sync_output.c_str());
  std::string goal_topic;
  nh.getParam("/test_adapter/"+robot_name+"/goal", goal_topic);
  std::string waypoint_goal_topic;
  nh.getParam("/test_adapter/"+robot_name+"/topological_goal", waypoint_goal_topic);
  std::string object_detector_topic;
  nh.param<std::string>("/test_adapter/"+robot_name+"/object_detector", object_detector_topic, "object_detector");
  std::string ip;
  std::string port;
  std::string username;
  nh.getParam("/test_adapter/"+robot_name+"/spread/ip", ip);
  nh.getParam("/test_adapter/"+robot_name+"/spread/port", port);
  nh.getParam("/test_adapter/"+robot_name+"/spread/username", username);
  if (ip == "" || port == "" || username == "")
  {
    ROS_ERROR("Spread parameters not configured, unable to launch adapter!");
    return 1;
  }
  Adapter adapter(nh, goal_topic, sync_input, sync_output, waypoint_goal_topic, robot_name, object_detector_topic, coverage_format, coverage_output);
  dtron_test_adapter::TestAdapter testAdapter(nh,  (port + "@" + ip).c_str(), username.c_str(), groups, boost::bind(&Adapter::receiveMessage, &adapter, _1, _2));
  adapter.setTestAdapter(testAdapter);
  ROS_INFO("Test adapter running...");

  ros::Rate r(20);
  while (nh.ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
