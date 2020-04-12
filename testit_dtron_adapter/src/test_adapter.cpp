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
#include <sys/stat.h>
#include <cstdlib>

#include <std_msgs/Bool.h>
#include "testit_dtron_adapter/HandleSpreadMessage.h"

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
  const char* spreadName;
  const char* userName;
  char* User;
  char* Spread;
  char* PrivateGroup;
};


SpreadAdapter::SpreadAdapter(const char* SpreadName, const char* UserName, boost::function<void (int, char*, char*, char*)> callbackFunction) {
  int ret;
  spreadName = SpreadName;
  userName = userName;
  Mbox = new mailbox;
  PrivateGroup = new char[80];
  callback = callbackFunction;
  ret = SP_connect(spreadName, userName, 0, 1, Mbox, PrivateGroup);
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
    std::cout << spreadMessage.Type << ";; " << spreadMessage.Sender << ";; " << spreadMessage.Group << ";; " << spreadMessage.Msg << std::endl;
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
    if (ret == -18) {
      std::cout << "Trying to reconnect" << std::endl;
      SP_disconnect(*Mbox);
      int conn_ret = SP_connect(spreadName, userName, 0, 1, Mbox, PrivateGroup);
      if(conn_ret < 0) {
        std::cout << "Succeeded reconnect" << std::endl;
        SP_error(ret);
        connectionActive = false;
      } else {
        std::cout << "Failed reconnect" << std::endl;
        connectionActive = true;
      }
    }
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
    printf("Received spread message: '%s', group: '%s', sender: '%s'", msg, group, sender);
    try {
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
    catch (...) {
      printf("Something went wrong receiving message from spread");
    };
  }
}

class Adapter {
private:
  dtron_test_adapter::TestAdapter* testAdapter_;
  ros::ServiceClient handle_spread_message_client_;
  ros::NodeHandle nh_;
  std::vector<std::string> sync_input_;
  std::vector<std::string> sync_output_;
  std::string robot_name_;
public:
  Adapter(ros::NodeHandle nh,
      std::vector<std::string> sync_input,
      std::vector<std::string> sync_output,
      std::string robot_name) :
    nh_(nh),
    sync_input_(sync_input),
    sync_output_(sync_output),
    robot_name_(robot_name)
    {
      handle_spread_message_client_ = nh_.serviceClient<testit_dtron_adapter::HandleSpreadMessage>("/testit/dtron_adapter/handle_spread_message");
      ROS_INFO("ROBOT NAME IN ADAPTER %s", robot_name_.c_str());
    }

  void setTestAdapter(dtron_test_adapter::TestAdapter testAdapter) {
    testAdapter_ = &testAdapter;
  }

  std::string spreadMessageToYamlString(std::string name, std::map<std::string, int> args) {
    std::string paramString = "name: " + name;
    for (const auto &value : args) {
      paramString += "\n";
      paramString += value.first + ": " + std::to_string(value.second);
    }
    return paramString;
  }

  void receiveMessage(std::string name, std::map<std::string, int> args) {
    ROS_INFO_STREAM("Received a message with name '" << name << "'");
    std::vector<std::string>::iterator it = std::find(sync_input_.begin(), sync_input_.end(), name);
    int index = std::distance(sync_input_.begin(), it);
    std::string sync_output = sync_output_[index];
    std::map<std::string, int> vars;
    testit_dtron_adapter::HandleSpreadMessage srv;
    srv.request.input = spreadMessageToYamlString(name, args);
    bool call_success = handle_spread_message_client_.call(srv);
    if (call_success && (bool)srv.response.response) {
      vars["value"] = 1;
    } else {
      vars["value"] = -1;
    }
    testAdapter_->sendMessage(sync_output.c_str(), vars);
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

  std::vector<const char*> groups;
  std::vector<std::string> sync_input;
  nh.getParam("/test_adapter/"+robot_name+"/dtron/sync/input", sync_input);
  for (std::vector<std::string>::const_iterator it = sync_input.begin(); it != sync_input.end(); ++it)
  {
    groups.push_back(it->c_str());
  }
  std::vector<std::string> sync_output;
  nh.getParam("/test_adapter/"+robot_name+"/dtron/sync/output", sync_output);
  for (std::vector<std::string>::const_iterator it = sync_output.begin(); it != sync_output.end(); ++it)
  {
    groups.push_back(it->c_str());
  }
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
  Adapter adapter(nh, sync_input, sync_output, robot_name);
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
