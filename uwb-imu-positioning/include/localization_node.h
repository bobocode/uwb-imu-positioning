#ifndef _LOCALIZATION_NODE_H_
#define _LOCALIZATION_NODE_H_

#include "uwb.h"
#include "uwb_init_loc.h"
#include "utility.h"
#include "parameter.h"

class Localization_Node{
public:
    explicit Localization_Node(ros::NodeHandle& nh);
    ~Localization_Node(){
    }

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_ndb_sub;
    ros::Subscriber m_imu_info_sub;
    ros::Publisher m_uwb_odometry_pub;
    ros::Timer m_print_timer;

    double m_last_imu_t;

    double m_slam_fps;
    bool m_is_fix_fps;
    bool m_is_initialize_with_ceres;
    double m_td;
    double m_current_time;

    int m_mobile_id;
    boost::shared_ptr<UWB_Mobile> m_p_mobile;
    geometry_msgs::Pose m_position;
    geometry_msgs::Twist m_velocity;

    std::vector<int> m_anchor_list;
    std::map<int, boost::shared_ptr<UWB_Anchor> > m_anchor_map;

    std::queue<sensor_msgs::ImuConstPtr> m_imu_buf;
    std::queue<common_msgs::UWB_FullNeighborDatabase::ConstPtr> m_ndb_buf;
    std::mutex m_buf;
    std::mutex m_process;
    std::condition_variable m_con;


public:
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, common_msgs::UWB_FullNeighborDatabase::ConstPtr>> getMeasurements();
    void createUWBMobile(const int mobile_id);
    void imuInfoCallback(const sensor_msgs::ImuConstPtr &imu_msg);
    void ndbCallback(const common_msgs::UWB_FullNeighborDatabase::ConstPtr& msg);
    void printLocalizationCallback(const ros::TimerEvent& event);
    bool getValidNeighborDatabse(const common_msgs::UWB_FullNeighborDatabase &ndb);
    void solveSLAM(const ros::TimerEvent& event);
    void process();
    void pubOdometry(double header);
};


#endif