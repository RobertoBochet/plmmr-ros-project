#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>
#include <tf/transform_broadcaster.h>
#include <tuple>


class RawDataHandlerNode
{
	typedef message_filters::Subscriber<sensor_msgs::NavSatFix> sub_t;
	typedef std::shared_ptr<sub_t> sub_ptr_t;
	typedef message_filters::TimeSequencer<sensor_msgs::NavSatFix> seq_t;
	typedef std::shared_ptr<seq_t> seq_ptr_t;

	bool debug = false;

	ros::NodeHandle nh;

	ros::Publisher odom_pub;
	ros::Publisher odom_debug_pub;

	tf::TransformBroadcaster br;

	sub_ptr_t sub;
	seq_ptr_t seq;

	std::string topic, name, name_lost, frame_id;
	double init_lat, init_lon, init_alt;


	void callback(const sensor_msgs::NavSatFixConstPtr &msg)
	{

		ROS_INFO("Input position: [%f, %f, %f]", msg->latitude, msg->longitude, msg->altitude);

		// handles gps signal loss
		if (msg->latitude == 0 && msg->longitude == 0 && msg->altitude == 0)
		{
			send_om(0, 0, 0, ros::Time::now(), true);

			ROS_WARN("GPS signal of %s is lost", name.c_str());
			return;
		}

		double x, y, z;
		std::tie(x, y, z) = lla_to_enu(msg->latitude, msg->longitude, msg->altitude);

		ROS_INFO("ENU position: [%f, %f, %f]", x, y, z);

		auto t = ros::Time::now();

		send_om(x, y, z, t);
		send_tf(x, y, z, t);
	}

	void send_om(double x, double y, double z, ros::Time t, bool lost = false)
	{
		nav_msgs::Odometry o;

		o.pose.pose.position.x = x;
		o.pose.pose.position.y = y;
		o.pose.pose.position.z = z;

		o.header.frame_id = lost ? name_lost : name;
		o.child_frame_id = frame_id;
		o.header.stamp = t;

		odom_pub.publish(o);

		if (debug)
		{
			o.pose.pose.position.x = x / 100;
			o.pose.pose.position.y = y / 100;
			o.pose.pose.position.z = z / 100;

			odom_debug_pub.publish(o);
		}
	}

	void send_tf(double x, double y, double z, ros::Time t)
	{
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(x / 100, y / 100, z / 100));
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, t, frame_id, name));
	}

	std::tuple<double, double, double> lla_to_enu(double lat, double lon, double alt)
	{
		double a = 6378137;
		double b = 6356752.3142;
		double f = (a - b) / a;
		double e_sq = f * (2 - f);
		float deg_to_rad = 0.0174533;

		//lla to ecef
		float lamb = deg_to_rad * (lat);
		float phi = deg_to_rad * (lon);
		float s = sin(lamb);
		float N = a / sqrt(1 - e_sq * s * s);

		float sin_lambda = sin(lamb);
		float cos_lambda = cos(lamb);
		float sin_phi = sin(phi);
		float cos_phi = cos(phi);

		float x = (alt + N) * cos_lambda * cos_phi;
		float y = (alt + N) * cos_lambda * sin_phi;
		float z = (alt + (1 - e_sq) * N) * sin_lambda;

		ROS_DEBUG("ECEF position: [%f, %f, %f]", x, y, z);

		// ecef to enu

		lamb = deg_to_rad * (init_lat);
		phi = deg_to_rad * (init_lon);
		s = sin(lamb);
		N = a / sqrt(1 - e_sq * s * s);

		sin_lambda = sin(lamb);
		cos_lambda = cos(lamb);
		sin_phi = sin(phi);
		cos_phi = cos(phi);

		float x0 = (init_alt + N) * cos_lambda * cos_phi;
		float y0 = (init_alt + N) * cos_lambda * sin_phi;
		float z0 = (init_alt + (1 - e_sq) * N) * sin_lambda;

		float xd = x - x0;
		float yd = y - y0;
		float zd = z - z0;

		float xEast = -sin_phi * xd + cos_phi * yd;
		float yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
		float zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;


		return std::make_tuple(xEast, yNorth, zUp);
	}

public:
	RawDataHandlerNode()
	{
		ros::NodeHandle nhl("~");

		nhl.getParam("topic", topic);
		nhl.getParam("name", name);
		nhl.getParam("frame_id", frame_id);
		nh.getParam("init_lat", init_lat);
		nh.getParam("init_lon", init_lon);
		nh.getParam("init_alt", init_alt);
		nh.getParam("debug", debug);

		name_lost = name + "_lost";

		ROS_INFO("\nTopic:\t\t%s\nName:\t\t%s\nFrame id:\t%s", topic.c_str(), name.c_str(), frame_id.c_str());
		ROS_INFO("Init point:\t[%f, %f, %f]", init_lat, init_lon, init_alt);

		odom_pub = nh.advertise<nav_msgs::Odometry>("odom/" + name, 10);
		odom_debug_pub = nh.advertise<nav_msgs::Odometry>("odom/debug/" + name, 10);

		sub = std::make_shared<sub_t>(nh, topic, 1);
		seq = std::make_shared<seq_t>(*sub, ros::Duration(0.1), ros::Duration(0.01), 10);
		seq->registerCallback([this](auto &&PH1) { callback(PH1); });

		ROS_INFO("Raw data handler node ready!");
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "raw_data_handler");

	RawDataHandlerNode node;

	ros::spin();
	return 0;
}
