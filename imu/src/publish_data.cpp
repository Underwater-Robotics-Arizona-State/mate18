#include <stdio.h>
#include <sstream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"

std::array<double, 10> dx_array;
uint8_t x_index = 0;
std::array<double, 10> dy_array;
uint8_t y_index = 0;
std::array<double, 10> dz_array;
uint8_t z_index = 0;

std::array<double, 3> previous_accels;

double threshold = 0.10;

double get_average(std::array<double, 10>& array) {
    double new_average = 0;
    for (double value : array) {
        new_average += value;
    }

    new_average /= array.size();

    return new_average;
}

std::string num_to_dir(uint8_t direction) {
    std::string result = "";

    switch (direction) {
        case 0:
            result = "x";
            break;
        case 1:
            result = "y";
            break;
        case 2:
            result = "z";
            break;
    }

    return result;
}

void print_detected_jerks() {
    bool print = false;

    std::array<double, 3> averages;
    averages[0] = get_average(dx_array);
    averages[1] = get_average(dy_array);
    averages[2] = get_average(dz_array);
    
    for (uint8_t i = 0; i < 3; i++) {
        if (fabs(averages[i] - previous_accels[i]) > threshold) {
            ROS_INFO("Detected change in %s acceleration!", num_to_dir(i).c_str());
            print = true;
        }

        previous_accels[i] = averages[i];
    }
    
    if (print) {
        std::cout << "\n\n";
    }
}

void insert(double value, std::array<double, 10>& array, uint8_t& index) {
    array[index] = value;
    index = (index + 1) % 10;
}

void callback(const sensor_msgs::Imu& msg) {
    insert(msg.linear_acceleration.x, dx_array, x_index);
    insert(msg.linear_acceleration.y, dy_array, y_index);
    insert(msg.linear_acceleration.z, dz_array, z_index);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_publisher_node");

    ros::NodeHandle nh;
    ros::Subscriber subscriber = nh.subscribe("imu/data", 10, callback);

    ros::Rate r(10);

    dx_array.fill(0);
    dy_array.fill(0);
    dz_array.fill(0);
    previous_accels.fill(0);

    while(ros::ok()) {
        ros::spinOnce();
        print_detected_jerks();
        r.sleep();
    }

    return 0;
}
