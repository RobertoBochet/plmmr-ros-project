#include <ros/ros.h>
#include <distance_service/DistanceCalculator.h>
#include <cmath>

bool calc(distance_service::DistanceCalculator::Request &req, distance_service::DistanceCalculator::Response &res)
{
	res.dist = sqrt(pow(req.a.x - req.b.x, 2) + pow(req.a.y - req.b.y, 2) + pow(req.a.z - req.b.z, 2));

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "distance_service");

	auto s = ros::NodeHandle().advertiseService("distance_calculator", calc);

	ros::spin();

	return 0;
}
