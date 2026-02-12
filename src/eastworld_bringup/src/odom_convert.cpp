#include "msf_component.hpp"

namespace uosm
{
    namespace perception
    {
        MultiSensorFusion::MultiSensorFusion(const rclcpp::NodeOptions &options)
            : Node("msf_node", options),
              window_size_(10),
              buffer_max_size_(1),
              static_transform_cached_(false),
              odom_frame_("odom"),
              base_frame_("base_link"),
              camera_frame_("zed_camera_link")
        {
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), "  Multi Sensor Fusion Component ");
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
            RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
            RCLCPP_INFO(get_logger(), "********************************");

            // Declare parameters
            declare_parameter("odom_frame", "odom");
            declare_parameter("base_frame", "base_link");
            declare_parameter("camera_frame", "zed_camera_link");
            declare_parameter("window_size", 10);
            declare_parameter("publish_rate", 30.0f);
            declare_parameter("buffer_max_size", 10);
            declare_parameter("broadcast_tf", true);

            float publish_rate = 30.0f;
            get_parameter("odom_frame", odom_frame_);
            get_parameter("base_frame", base_frame_);
            get_parameter("camera_frame", camera_frame_);
            get_parameter("window_size", window_size_);
            get_parameter("publish_rate", publish_rate);
            get_parameter("buffer_max_size", buffer_max_size_);
            get_parameter("broadcast_tf", broadcast_tf_);

            // Initialize the window for covariance values
            covariance_window_.resize(window_size_, 0.0);
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            
            // Initialize TF broadcaster
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            // Cache static transform after a short delay to let tf tree populate
            cache_timer_ = create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&MultiSensorFusion::cache_static_transform, this));

            // Subscription with highest priority QoS for lowest latency
            auto qos = rclcpp::QoS(1)
                           .reliability(rclcpp::ReliabilityPolicy::Reliable)
                           .durability(rclcpp::DurabilityPolicy::Volatile)
                           .history(rclcpp::HistoryPolicy::KeepLast);

            vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "/vio/odom", qos,
                [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    handle_vio_callback(msg);
                });

            fused_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/fused/odom", qos);

            odom_publish_timer_ = create_wall_timer(
                std::chrono::duration<double>(1.0 / publish_rate),
                std::bind(&MultiSensorFusion::publish_fused_odom, this));
        }

        void MultiSensorFusion::cache_static_transform()
        {
            try
            {
                // Look up static transform and cache it
                auto tf_msg = tf_buffer_->lookupTransform(
                    base_frame_,
                    camera_frame_,
                    tf2::TimePointZero);

                // Convert to Eigen transform for better performance
                cached_transform_eigen_ = transform_msg_to_eigen(tf_msg).inverse();
                static_transform_cached_ = true;
                cache_timer_->cancel(); // Stop the timer once we have the transform

                RCLCPP_INFO(get_logger(), "Cached static transform from %s to %s",
                            camera_frame_.c_str(), base_frame_.c_str());
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(get_logger(), "Failed to lookup static transform: %s", ex.what());
            }
        }

        Eigen::Isometry3d MultiSensorFusion::transform_msg_to_eigen(
            const geometry_msgs::msg::TransformStamped& tf_msg)
        {
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            
            // Set translation
            transform.translation() = Eigen::Vector3d(
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z
            );
            
            // Set rotation
            Eigen::Quaterniond q(
                tf_msg.transform.rotation.w,
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z
            );
            transform.linear() = q.toRotationMatrix();
            
            return transform;
        }

        geometry_msgs::msg::Pose MultiSensorFusion::transform_pose_eigen(
            const geometry_msgs::msg::Pose &pose_in,
            const Eigen::Isometry3d &transform)
        {
            // Convert input pose (camera pose in odom frame) to Eigen
            Eigen::Isometry3d camera_pose = Eigen::Isometry3d::Identity();
            camera_pose.translation() = Eigen::Vector3d(
                pose_in.position.x, pose_in.position.y, pose_in.position.z);

            Eigen::Quaterniond orientation(
                pose_in.orientation.w, pose_in.orientation.x,
                pose_in.orientation.y, pose_in.orientation.z);
            camera_pose.linear() = orientation.toRotationMatrix();

            // Transform: T_odom_baselink = T_odom_camera * T_camera_baselink
            Eigen::Isometry3d baselink_pose = camera_pose * transform;

            // Convert back to ROS message
            geometry_msgs::msg::Pose pose_out;
            pose_out.position.x = baselink_pose.translation().x();
            pose_out.position.y = baselink_pose.translation().y();
            pose_out.position.z = baselink_pose.translation().z();

            Eigen::Quaterniond result_quat(baselink_pose.linear());
            pose_out.orientation.w = result_quat.w();
            pose_out.orientation.x = result_quat.x();
            pose_out.orientation.y = result_quat.y();
            pose_out.orientation.z = result_quat.z();

            return pose_out;
        }

        geometry_msgs::msg::Twist MultiSensorFusion::transform_twist_eigen(
            const geometry_msgs::msg::Twist& twist_in,
            const Eigen::Isometry3d& transform)
        {
            // Extract rotation matrix from transform
            Eigen::Matrix3d rotation = transform.linear();
            
            // Transform linear velocity
            Eigen::Vector3d linear_velocity(twist_in.linear.x, twist_in.linear.y, twist_in.linear.z);
            Eigen::Vector3d transformed_linear = rotation * linear_velocity;
            
            // Transform angular velocity
            Eigen::Vector3d angular_velocity(twist_in.angular.x, twist_in.angular.y, twist_in.angular.z);
            Eigen::Vector3d transformed_angular = rotation * angular_velocity;
            
            // Convert back to ROS message
            geometry_msgs::msg::Twist twist_out;
            twist_out.linear.x = transformed_linear.x();
            twist_out.linear.y = transformed_linear.y();
            twist_out.linear.z = transformed_linear.z();
            
            twist_out.angular.x = transformed_angular.x();
            twist_out.angular.y = transformed_angular.y();
            twist_out.angular.z = transformed_angular.z();
            
            return twist_out;
        }

        void MultiSensorFusion::broadcast_odom_tf(const nav_msgs::msg::Odometry& odom_msg)
        {
            if (!broadcast_tf_) {
                return;
            }

            geometry_msgs::msg::TransformStamped transform_stamped;
            
            // Header
            transform_stamped.header.stamp = odom_msg.header.stamp;
            transform_stamped.header.frame_id = odom_frame_;
            transform_stamped.child_frame_id = base_frame_;
            
            // Copy pose to transform
            transform_stamped.transform.translation.x = odom_msg.pose.pose.position.x;
            transform_stamped.transform.translation.y = odom_msg.pose.pose.position.y;
            transform_stamped.transform.translation.z = odom_msg.pose.pose.position.z;
            
            transform_stamped.transform.rotation.x = odom_msg.pose.pose.orientation.x;
            transform_stamped.transform.rotation.y = odom_msg.pose.pose.orientation.y;
            transform_stamped.transform.rotation.z = odom_msg.pose.pose.orientation.z;
            transform_stamped.transform.rotation.w = odom_msg.pose.pose.orientation.w;
            
            // Broadcast the transform
            tf_broadcaster_->sendTransform(transform_stamped);
        }
        void MultiSensorFusion::handle_vio_callback(const nav_msgs::msg::Odometry::SharedPtr &msg)
        {
            Eigen::Isometry3d transform_eigen;

            if (!static_transform_cached_)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "Static transform not cached yet, cannot process VIO message");
                return;
            }

            transform_eigen = cached_transform_eigen_;

            auto transformed_msg = std::make_unique<nav_msgs::msg::Odometry>();
            transformed_msg->header.stamp = msg->header.stamp;
            transformed_msg->header.frame_id = odom_frame_;
            transformed_msg->child_frame_id = base_frame_;
            
            // Transform pose and twist using Eigen
            transformed_msg->pose.pose = transform_pose_eigen(msg->pose.pose, transform_eigen);
            transformed_msg->pose.covariance = msg->pose.covariance;
            
            transformed_msg->twist.twist = transform_twist_eigen(msg->twist.twist, transform_eigen);
            transformed_msg->twist.covariance = msg->twist.covariance;

            last_callback_time = msg->header.stamp;
            {
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                odom_buffer_.push_back(std::move(transformed_msg));
                while (odom_buffer_.size() > buffer_max_size_)
                {
                    odom_buffer_.pop_front();
                }
            }
        }

        void MultiSensorFusion::publish_fused_odom()
        {
            std::unique_ptr<nav_msgs::msg::Odometry> msg_to_publish = nullptr;
            {
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                if (!odom_buffer_.empty())
                {
                    // Get the most recent message from the buffer
                    msg_to_publish = std::move(odom_buffer_.back());
                    odom_buffer_.clear(); // Clear the buffer after taking the most recent message
                }
            }

            if (msg_to_publish)
            {
                // Broadcast TF transform before publishing odometry
                broadcast_odom_tf(*msg_to_publish);
                
                // Publish the most recent odometry message
                fused_odom_pub_->publish(std::move(msg_to_publish));
            }
            else
            {
                RCLCPP_DEBUG(get_logger(), "No odometry message available to publish");
            }
        }

    } // namespace perception
} // namespace uosm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uosm::perception::MultiSensorFusion)
