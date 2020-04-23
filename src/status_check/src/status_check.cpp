#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <status_check/parametersConfig.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <math.h>


class StatusCheckNode
{
	status_check::parametersConfig param;

	ros::NodeHandle nh;
	dynamic_reconfigure::Server<status_check::parametersConfig> config_server;

	ros::Publisher status_pub;
	ros::Subscriber dist_sub;

	void config_callback(status_check::parametersConfig &config, uint32_t level)
	{
		if (level == 0)
		{
			param.crash_limit = config.crash_limit;
			param.safe_limit = (param.crash_limit <= config.safe_limit) ? config.safe_limit : param.crash_limit;
		} else
		{
			param.safe_limit = config.safe_limit;
			param.crash_limit = (param.safe_limit >= config.crash_limit) ? config.crash_limit : param.safe_limit;
		}

		if (param.crash_limit != config.crash_limit || param.safe_limit != config.safe_limit)
		{
			ROS_WARN("Invalid parameters combination");
		}

		ROS_INFO("Reconfigure parameters: {crash_limit: %f, safe_limit: %f}", param.crash_limit, param.safe_limit);
	}

	void dist_callback(const std_msgs::Float64::ConstPtr &dist)
	{
		// ignores message if it is NaN
		if (isnan(dist->data)) return;

		std_msgs::String msg;

		if (dist->data < param.crash_limit)
			msg.data = "crash";

		else if (dist->data < param.safe_limit)
			msg.data = "unsafe";

		else
			msg.data = "safe";

		status_pub.publish(msg);

		ROS_INFO("Status is %s", msg.data.c_str());
	}

public:
	StatusCheckNode()
	{
		ros::NodeHandle nhl("~");

		nhl.getParam("safe_limit", param.safe_limit);
		nhl.getParam("crash_limit", param.crash_limit);

		config_server.setConfigDefault(param);

		config_server.setCallback([this](auto &&PH1, auto &&PH2) { return config_callback(PH1, PH2); });

		status_pub = nh.advertise<std_msgs::String>("status", 10);

		dist_sub = nh.subscribe("dist", 1, &StatusCheckNode::dist_callback, this);

		ROS_INFO("Status check node ready!");
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "status_check");

	StatusCheckNode node;

	ros::spin();
	return 0;
}
