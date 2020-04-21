#include <ros/ros.h>
#include <distance_service/DistanceService.h>
#include <cmath>

bool calc(distance_service::DistanceService::Request &req, distance_service::DistanceService::Response &res)
{
	res.dist = sqrt(pow(req.a.x - req.b.x, 2) + pow(req.a.y - req.b.y, 2) + pow(req.a.z - req.b.z, 2));

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "distance_service");

	auto s = ros::NodeHandle().advertiseService("calc_distance", calc);

	ros::spin();

	return 0;
}
