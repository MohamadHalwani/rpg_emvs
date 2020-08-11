#include <mapper_emvs/data_loading.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <glog/logging.h>

namespace data_loading {

void parse_rosbag(const std::string &rosbag,
                  std::vector<dvs_msgs::Event>& events_,
                  std::map<ros::Time, geometry_utils::Transformation>& poses_,
                  sensor_msgs::CameraInfo& camera_info_msg,
                  const std::string& event_topic,
                  const std::string& camera_info_topic,
                  const std::string& pose_topic,
                  const double tmin,
                  const double tmax)
{
  std::vector<std::string> topics;
  std::string my_event_topic = "/dvs_corner_events_soft";
  std::string my_info_topic = "/dvs/camera_info";
  std::string my_pose_topic = "/UR_driver_pose";

  topics.push_back(my_event_topic);
  topics.push_back(my_info_topic);
  topics.push_back(my_pose_topic);

  poses_.clear();
  events_.clear();

  std::cout << "topics: " << topics[0] << topics[1] << topics[2] << std::endl;

  rosbag::Bag  bag(rosbag, rosbag::bagmode::Read);
  // rosbag::View view(bag, rosbag::TopicQuery(topics));
  rosbag::View view(bag);

  bool continue_looping_through_bag = true;
  bool got_initial_stamp = false;
  ros::Time initial_timestamp;
  int iterator = 0;
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if(!continue_looping_through_bag)
    {
      break;
    }

    const std::string& topic_name = m.getTopic();
    VLOG(2) << topic_name;
    //std::cout << "Topic name:" << topic_name << std::endl;
    
    // Events
    
    if (topic_name == topics[0])
    {
      if (iterator<4)
      {
        iterator++;
        continue;
      }
      std::cout << "event msg" << std::endl;
      dvs_msgs::EventArray::ConstPtr msg = m.instantiate<dvs_msgs::EventArray>();
      if (msg != NULL)
      {
        if(msg->events.empty())
        {
          continue;
        }
        const ros::Time& stamp = msg->events[0].ts;
        std::cout << "stamp = " << stamp << std::endl;
        if(!got_initial_stamp)
        {
          initial_timestamp = stamp;
          LOG(INFO) << "initial stamp: " << stamp;
          got_initial_stamp = true;
        }

        for (size_t i = 0; i < msg->events.size(); ++i)
        {
          const double rel_stamp = (msg->events[i].ts - initial_timestamp).toSec();
          std::cout << "initial_timestamp = " << initial_timestamp<< std::endl;
          std::cout << "rel_stamp = " << rel_stamp << std::endl;
          std::cout << "Event timestamp = " << msg->events[i].ts << std::endl;
          if(rel_stamp < tmin)
          {
            continue;
          }
          if(rel_stamp > tmax)
          {
            std::cout << "rel_stamp_in_if_statement= " << rel_stamp << std::endl;
            std::cout << "Event timestamp in if statement= " << msg->events[i].ts << std::endl;
            continue_looping_through_bag = false;
          }

          dvs_msgs::Event ev_modified(msg->events[i]);
          ev_modified.ts = ros::Time(ev_modified.ts.toSec() - initial_timestamp.toSec());
          events_.push_back(ev_modified);
        }
      }
    }
    
    

    // Camera Info
    if (topic_name == topics[1])
    {
      std::cout << "camera_info_msg" << std::endl;
      camera_info_msg = *(m.instantiate<sensor_msgs::CameraInfo>());
    }

    // Pose
    if (topic_name == topics[2])
    {
      std::cout << "pose msg" << std::endl;
      const geometry_msgs::PoseStamped pose_msg
          = *(m.instantiate<geometry_msgs::PoseStamped>());
      const ros::Time& stamp = pose_msg.header.stamp;
      if(!got_initial_stamp)
      {
        initial_timestamp = stamp;
        LOG(INFO) << "Initial stamp: " << stamp;
        got_initial_stamp = true;
      }
      
      const double rel_stamp = (stamp - initial_timestamp).toSec();
      if(rel_stamp < tmin)
      {
        continue;
      }
      if(rel_stamp > tmax)
      {
        continue_looping_through_bag = false;
      }
      const Eigen::Vector3d position(pose_msg.pose.position.x,
                                     pose_msg.pose.position.y,
                                     pose_msg.pose.position.z);
      const Eigen::Quaterniond quat(pose_msg.pose.orientation.w,
                                    pose_msg.pose.orientation.x,
                                    pose_msg.pose.orientation.y,
                                    pose_msg.pose.orientation.z);
      geometry_utils::Transformation T(position, quat);
      poses_.insert(std::pair<ros::Time, geometry_utils::Transformation>(ros::Time(pose_msg.header.stamp.toSec() - initial_timestamp.toSec()), T));
    }
  }

  // Sort events by increasing timestamps
  std::sort(events_.begin(), events_.end(),
            [](const dvs_msgs::Event& a, const dvs_msgs::Event& b) -> bool
  {
    return a.ts < b.ts;
  });
}

} // namespace data_loading
