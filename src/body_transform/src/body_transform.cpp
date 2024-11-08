#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "body_init_to_body_transform");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;

    tf::StampedTransform T_body_lidar;         // 机器人本体到激光雷达的固定变换
    tf::StampedTransform T_lidar_init_lidar;   // lidar_init 到 lidar 的实时变换
    tf::Transform T_body_init_body;            // 最终 body_init 到 body 的实时变换
    tf::Transform T_lidar_init_body_init;      // lidar_init 到 body_init 的变换

    // 初始化 body_init 到 lidar_init 的初始变换
    bool init_transform_set = false;
    tf::Transform T_body_init_lidar_init;
    ROS_WARN("body_transform Node started successfully!");
    ros::Rate rate(20.0); // 设置频率 10 Hz
    while (nh.ok()) {
        try {
            // 1. 获取 body 到 lidar 的固定变换
            listener.lookupTransform("pelvis", "mid360_link", ros::Time(0), T_body_lidar);

            // 2. 获取 lidar_init 到 lidar 的实时变换
            listener.lookupTransform("camera_init", "body", ros::Time(0), T_lidar_init_lidar);

            // 3. 初始化 body_init 到 lidar_init 的初始变换
            if (!init_transform_set) {
                // T_body_init_lidar_init = T_body_lidar.inverse();
                T_body_init_lidar_init = T_body_lidar;
                init_transform_set = true;
                ROS_INFO("Initial transform from body_init to lidar_init is set.");
            }

            // 4. 计算 body_init 到 body 的变换
            tf::Transform T_lidar_body = T_body_lidar.inverse();
            T_body_init_body = T_body_init_lidar_init * T_lidar_init_lidar * T_lidar_body;

            // 5. 计算 lidar_init 到 body_init 的变换
            T_lidar_init_body_init = T_body_init_lidar_init.inverse();

            // 6. 发布 lidar_init 到 body_init 的变换
            broadcaster.sendTransform(
                tf::StampedTransform(T_lidar_init_body_init, ros::Time::now(), "camera_init", "map"));

            // 7. 发布 body_init 到 body 的变换
            broadcaster.sendTransform(
                tf::StampedTransform(T_body_init_body, ros::Time::now(), "map", "pelvis"));

        } catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }
    return 0;
}
