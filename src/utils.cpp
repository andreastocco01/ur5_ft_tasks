#include <geometry_msgs/Vector3.h>

geometry_msgs::Vector3 mean(std::vector<geometry_msgs::Vector3> &vec) {
    geometry_msgs::Vector3 mean;
    mean.x = mean.y = mean.z = 0.0;
    for(int i = 0; i < vec.size(); i++) {
        mean.x += std::abs(vec[i].x);
        mean.y += std::abs(vec[i].y);
        mean.z += std::abs(vec[i].z);
    }

    mean.x /= vec.size();
    mean.y /= vec.size();
    mean.z /= vec.size();

    return mean;
}

geometry_msgs::Vector3 standard_deviation(std::vector<geometry_msgs::Vector3> &vec, geometry_msgs::Vector3 mean) {
    geometry_msgs::Vector3 sd;
    sd.x = sd.y = sd.z = 0.0;
    for (int i = 0; i < vec.size(); i++) {
        sd.x += pow(std::abs(vec[i].x) - mean.x, 2);
        sd.y += pow(std::abs(vec[i].y) - mean.y, 2);
        sd.z += pow(std::abs(vec[i].z) - mean.z, 2);
    }

    sd.x = sqrt(sd.x / vec.size());
    sd.y = sqrt(sd.y / vec.size());
    sd.z = sqrt(sd.z / vec.size());

    return sd;
}