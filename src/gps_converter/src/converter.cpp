#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tuple>


class TfSubPub
{
	tf::TransformBroadcaster br;
	ros::Subscriber sub;
	std::string topic, name, frame_id;
	double init_lat, init_lon, init_alt;

public:
	TfSubPub()
	{
		ros::NodeHandle("~").getParam("topic", topic);
		ros::NodeHandle("~").getParam("name", name);
		ros::NodeHandle("~").getParam("frame_id", frame_id);
		ros::NodeHandle().getParam("init_lat", init_lat);
		ros::NodeHandle().getParam("init_lon", init_lon);
		ros::NodeHandle().getParam("init_alt", init_alt);

		ROS_INFO("\nTopic:\t\t%s\nName:\t\t%s\nFrame id:\t%s", topic.c_str(), name.c_str(), frame_id.c_str());
		ROS_INFO("Init point:\t[%f, %f, %f]", init_lat, init_lon, init_alt);

		sub = ros::NodeHandle().subscribe(topic, 1000, &TfSubPub::callback, this);
	}

	void callback(const sensor_msgs::NavSatFixConstPtr &msg)
	{
		ROS_INFO("Input position: [%f, %f, %f]", msg->latitude, msg->longitude, msg->altitude);

		double x, y, z;
		std::tie(x, y, z) = lla_to_enu(msg->latitude, msg->longitude, msg->altitude);

		ROS_INFO("ENU position: [%f, %f, %f]", x, y, z);


		tf::Transform transform;
		transform.setOrigin(tf::Vector3(x / 100, y / 100, z / 100));
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, name));
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
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "gps_converter");

	TfSubPub tf_sub_pub;

	ros::spin();
	return 0;
}