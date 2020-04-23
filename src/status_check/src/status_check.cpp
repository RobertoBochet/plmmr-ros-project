#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <status_check/parametersConfig.h>


class StatusCheckNode
{
	status_check::parametersConfig param;

	ros::NodeHandle nh;
	dynamic_reconfigure::Server<status_check::parametersConfig> config_server;

	void config_callback(status_check::parametersConfig &config, uint32_t level)
	{
		if (level == 0)
		{
			param.crash_limit = config.crash_limit;
			param.safe_limit = (param.crash_limit <= config.safe_limit) ? config.safe_limit : param.crash_limit;
		} else if (level == 1)
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

public:
	StatusCheckNode()
	{
		ros::NodeHandle nhl("~");

		nhl.getParam("safe_limit", param.safe_limit);
		nhl.getParam("crash_limit", param.crash_limit);

		config_server.setConfigDefault(param);

		config_server.setCallback([this](auto &&PH1, auto &&PH2) { return config_callback(PH1, PH2); });
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "status_check");

	StatusCheckNode node;

	ros::spin();
	return 0;
}
