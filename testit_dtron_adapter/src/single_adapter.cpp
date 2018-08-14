#include <xtaproto.pb.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <cstring>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include "sp.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <topological_navigation/GotoNodeAction.h>
#include <actionlib/client/simple_action_client.h>


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
		std::string data = "";
		response.SerializeToString(&data);
		spreadAdapter_.SendMessage(group, data);
	}

	void TestAdapter::spreadMessageCallback(int type, char* sender, char* group, char* msg) {
		Sync sync;
		sync.ParseFromString(msg);
		if ((sync.name() != "") && (strstr(sender, "#SUT") == NULL)) {
			printf("[Google protocol buffers]: Channel: '%s', Sender: '%s'\n", sync.name().c_str(), sender);
			//printf("[Google protocol buffers]: Channel: '%s', Sender: '%s', VariableName: '%s' VariableValue: '%d'\n", sync.name().c_str(), sender, sync.variables(0).name().c_str(), sync.variables(0).value());
			std::map<std::string, int> mapOfVariables;
			for (unsigned int i = 0; i < sync.variables_size(); ++i) {
			   mapOfVariables.insert(std::pair<std::string, int>(sync.variables(i).name(), sync.variables(i).value()));
			}
			receiveMessage_(sync.name(), mapOfVariables);
			} else {
				printf("Received incorrectly formed message from %s in %s: %s\n", sender, group, msg);
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
		std::string sync_input_;
		std::string sync_output_;
		std::string navigation_mode_;
	public:
		Adapter(ros::NodeHandle nh,
				std::string goal_topic,
				std::string sync_input,
				std::string sync_output,
				std::string navigation_mode,
				std::string waypoint_goal_topic) :
		ac_movebase_(goal_topic, true),
		ac_topological_(waypoint_goal_topic, true),
		nh_(nh),
		sync_input_(sync_input),
		sync_output_(sync_output),
		navigation_mode_(navigation_mode) {
			nh.getParam("test_adapter/node_map", node_map_);
			ROS_WARN("Loaded node map with %lu nodes!", node_map_.size());
			if (navigation_mode_ != "waypoint") {
				ROS_INFO("Connecting to move_base action server @ %s", goal_topic.c_str());
				ac_movebase_.waitForServer(ros::Duration(8.0));
				if (!ac_movebase_.isServerConnected()) {
					ROS_ERROR("Unable to connect to move_base action server!");
				} 
			} else {
					ROS_INFO("Connecting to topological_navigation action server @ %s", waypoint_goal_topic.c_str());
					ac_topological_.waitForServer(ros::Duration(8.0));
					if (!ac_topological_.isServerConnected()) {
						ROS_ERROR("Unable to connect to topological_navigation action server!");
					}
			}
			ROS_INFO("Adapter is ready for use!");
		}

	void setTestAdapter(dtron_test_adapter::TestAdapter testAdapter) {
		testAdapter_ = &testAdapter;
	}

	void receiveMessage(std::string name, std::map<std::string, int> args) {
		ROS_INFO("Received a message - %s!", name.c_str());
		std::string state = boost::lexical_cast<std::string>(args["state"]);
		std::string node_name = node_map_[state];

		if (navigation_mode_ != "waypoint") {
			move_base_msgs::MoveBaseGoal goal;
			double x, y;
			nh_.getParam("test_adapter/nodes/" + node_name + "/x", x);
			nh_.getParam("test_adapter/nodes/" + node_name + "/y", y);
			//std::cout <<node_name << " X: " << x << "Y: " << y << "\n";
			goal.target_pose.header.frame_id = "/map";
			goal.target_pose.pose.position.x = x;
			goal.target_pose.pose.position.y = y;
			goal.target_pose.pose.position.z = 0;
			goal.target_pose.pose.orientation.x = 0;
			goal.target_pose.pose.orientation.y = 0;
			goal.target_pose.pose.orientation.z = 0;
			goal.target_pose.pose.orientation.w = 1;
			ROS_INFO("Goal. Node %s, x: %.3f  y: %.3f", node_name.c_str(), x, y);
			if (ac_movebase_.isServerConnected()) {
				ac_movebase_.sendGoal(goal);
				ac_movebase_.waitForResult();
				actionlib::SimpleClientGoalState state = ac_movebase_.getState();
				ROS_INFO("Action finished: %s", state.toString().c_str());
				if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
					testAdapter_->sendMessage(sync_output_.c_str(), std::map<std::string, int>());
				} else {
					ROS_ERROR("Action result was not SUCCEEDED!");
				}
			} else {
				ROS_ERROR("Action server not connected!");
 			}
		} else {
			topological_navigation::GotoNodeGoal goal;
			goal.target = node_name;
			ROS_INFO("Goal. Node: %s", node_name.c_str());
			//std::cout << node_name << "\n";
			if (ac_topological_.isServerConnected()) {
				ac_topological_.sendGoal(goal);
				ac_topological_.waitForResult();
				actionlib::SimpleClientGoalState state = ac_topological_.getState();
				ROS_INFO("Action finished: %s", state.toString().c_str());
				if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
					testAdapter_->sendMessage(sync_output_.c_str(), std::map<std::string, int>());
				} else {
					ROS_ERROR("Action result was not SUCCEEDED!");
				}
			} else {
              ROS_ERROR("Action server not connected!");
			}
		}
	}
};
int main(int argc, char** argv) {
		ros::init(argc, argv, "dtron_test_adapter");
		ros::NodeHandle nh;
		GOOGLE_PROTOBUF_VERIFY_VERSION;
		std::vector<const char*> groups;
		std::string sync_input;
		nh.getParam("test_adapter/dtron/sync/input", sync_input);
		groups.push_back(sync_input.c_str());
		std::string sync_output;
		nh.getParam("test_adapter/dtron/sync/output", sync_output);
		groups.push_back(sync_output.c_str());
		std::string goal_topic;
		nh.getParam("test_adapter/goal", goal_topic);
		std::string waypoint_goal_topic;
		nh.getParam("test_adapter/waypoint_goal", waypoint_goal_topic);
		std::string ip;
		std::string port;
		std::string username;
		nh.getParam("test_adapter/spread/ip", ip);
		nh.getParam("test_adapter/spread/port", port);
		nh.getParam("test_adapter/spread/username", username);
		std::string navigation_mode;
		nh.getParam("test_adapter/navigation_mode", navigation_mode);
		Adapter adapter(nh, goal_topic, sync_input, sync_output, navigation_mode, waypoint_goal_topic);
		dtron_test_adapter::TestAdapter testAdapter(nh,  (port + "@" + ip).c_str(), username.c_str(), groups, boost::bind(&Adapter::receiveMessage, &adapter, _1, _2));
		adapter.setTestAdapter(testAdapter);
		ROS_INFO("Test adapter is running!");
		ros::Rate r(20);
		while (nh.ok()) {
			ros::spinOnce();
			r.sleep();
		}
		return 0;
}
