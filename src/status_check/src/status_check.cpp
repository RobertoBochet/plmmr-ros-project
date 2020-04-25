#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <status_check/parametersConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>

#include <distance_service/DistanceCalculator.h>

#include <status_check/Status.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <cmath>


class StatusCheckNode
{
public:
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> SyncApproxPolicy;
	typedef message_filters::Synchronizer<SyncApproxPolicy> Sync;

	status_check::parametersConfig param;

	ros::NodeHandle nh;

	dynamic_reconfigure::Server<status_check::parametersConfig> config_server;

	ros::Publisher status_pub;

	message_filters::Subscriber<nav_msgs::Odometry> sub_odom_front;
	message_filters::Subscriber<nav_msgs::Odometry> sub_odom_obs;

	std::shared_ptr<Sync> sync_ptr;

	ros::ServiceClient distance_calculator;


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

	void callback(const nav_msgs::Odometry::ConstPtr &front, const nav_msgs::Odometry::ConstPtr &obs)
	{
		status_check::Status s;

		// handles the GPS signal loss
		if (front->header.frame_id == "front_lost" || obs->header.frame_id == "obs_lost")
		{
			s.distance = NAN;
			s.status = "unavailable";

			status_pub.publish(s);

			ROS_INFO("The distance is unavailable");
			return;
		}

		// prepares the odometrical data for the distance service
		distance_service::DistanceCalculator srv;
		srv.request.a = front->pose.pose.position;
		srv.request.b = obs->pose.pose.position;

		// checks if the service worked
		if (!distance_calculator.call(srv))
		{
			ROS_ERROR("The distance service failed");
			return;
		}

		// publishes status topic's data
		s.distance = srv.response.dist;

		if (s.distance < param.crash_limit)
			s.status = "crash";

		else if (s.distance < param.safe_limit)
			s.status = "unsafe";

		else
			s.status = "safe";

		status_pub.publish(s);

		ROS_INFO("The distance is %fm and the status is %s", s.distance, s.status.c_str());
	}

	inline void sub_setup()
	{
		sub_odom_front.subscribe(nh, "odom/front", 1);
		sub_odom_obs.subscribe(nh, "odom/obs", 1);

		sync_ptr = std::make_shared<Sync>(SyncApproxPolicy(10), sub_odom_front, sub_odom_obs);

		sync_ptr->registerCallback(&StatusCheckNode::callback, this);
	}

public:

	StatusCheckNode()
	{
		config_server.setCallback([this](auto &&PH1, auto &&PH2) { return config_callback(PH1, PH2); });

		distance_calculator = nh.serviceClient<distance_service::DistanceCalculator>("distance_calculator");

		status_pub = nh.advertise<status_check::Status>("status", 10);

		sub_setup();

		ROS_INFO("Status check node ready!");
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "status_check");

	StatusCheckNode node;

	//node.sub_odom_front.subscribe(node.nh, "odom/front", 1);
	//node.sub_odom_obs.subscribe(node.nh, "odom/obs", 1);

	//node.sync.reset(new StatusCheckNode::Sync(StatusCheckNode::MySyncPolicy(10), node.sub_odom_front, node.sub_odom_obs));

	ros::spin();
	return 0;
}
