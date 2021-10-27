#include "localization_node.h"

Localization_Node::Localization_Node(ros::NodeHandle& nh):
    m_nh(nh){
    // load param
    m_nh.param<double>("slam_fps", m_slam_fps, 0);
    m_nh.param<double>("td", m_td, 0.3);

    if(m_slam_fps<20){
        m_is_fix_fps = false;
    } else {
        m_is_fix_fps = true;
    }
    m_nh.param<bool>("is_initialize_with_ceres", m_is_initialize_with_ceres, true);
    m_last_imu_t = 0;
    m_current_time = -1;

    // create UWB_Mobile
    if( m_nh.getParam(std::string("/uav_id"), m_mobile_id ) ) {
        ROS_INFO("Mobile ID (/uav_id): %d", m_mobile_id);
    } else if( m_nh.getParam(std::string("mobile_id"), m_mobile_id ) ){
        ROS_INFO("Mobile ID (yaml): %d", m_mobile_id);
    } else {
        ROS_ERROR("Mobile ID not found!");
        return;
    }
    createUWBMobile(m_mobile_id);
    ROS_INFO("UWB_Mobile created.");

    // initialize range info callback
    m_ndb_sub = m_nh.subscribe("/time_domain/NDB",
                                   10,
                                   &Localization_Node::ndbCallback,
                                   this);

    m_imu_info_sub = m_nh.subscribe("/nax/imu",1000, &Localization_Node::imuInfoCallback,this);

    m_uwb_odometry_pub = m_nh.advertise<nav_msgs::Odometry>("odometry", 1000, this);
    
    //m_print_timer = m_nh.createTimer(ros::Duration(1), &Localization_Node::printLocalizationCallback, this);
}


void Localization_Node::imuInfoCallback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if(imu_msg->header.stamp.toSec() <= m_last_imu_t)
    {
        ROS_WARN("imu message in disorder");
        return;
    }

    m_last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    m_imu_buf.push(imu_msg);
    m_buf.unlock();
    m_con.notify_one();

}

void Localization_Node::ndbCallback(const common_msgs::UWB_FullNeighborDatabase::ConstPtr& msg)
{
       common_msgs::UWB_FullNeighborDatabase m_ndb = *msg;

       if(getValidNeighborDatabse(m_ndb))
       {
           m_buf.lock();
           m_ndb_buf.push(msg);
           m_buf.unlock();
           m_con.notify_one();
       }

}

bool Localization_Node::getValidNeighborDatabse(const common_msgs::UWB_FullNeighborDatabase &ndb)
{
    int valid_anchor_counter = 0;

    for(int i=0;i<ndb.numNeighborEntries;++i)
    {
        geometry_msgs::PointStamped anchor_position;
        std::cout<<ndb.neighbors.at(i).nodeId<<", "<<ndb.neighbors.at(i).rangeMm<<std::endl;
        if(m_p_mobile->getAnchorPosition(ndb.neighbors.at(i).nodeId, anchor_position)
                && ndb.neighbors.at(i).rangeMm/1000.0>0.3)
        {
            valid_anchor_counter += 1;
        }
        ROS_INFO("[Ceres Init] Total ndb anchor: %d", valid_anchor_counter);

    }

    if(valid_anchor_counter < m_p_mobile->m_least_anchor_num)
    {
        return false;
    }else
    {
        return true;
    }

}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, common_msgs::UWB_FullNeighborDatabase::ConstPtr>> Localization_Node::getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, common_msgs::UWB_FullNeighborDatabase::ConstPtr>> measurements;
    while(true)
    {
        if(m_imu_buf.empty() || m_ndb_buf.empty())
        {
            return measurements;
        }

        if(!(m_imu_buf.back()->header.stamp.toSec() > m_ndb_buf.front()->header.stamp.toSec() + m_td))
        {
            return measurements;
        }

        if(!(m_imu_buf.front()->header.stamp.toSec() < m_ndb_buf.front()->header.stamp.toSec()+ m_td))
        {
            ROS_WARN("throw ndb, only should happen at the beginning");
            m_ndb_buf.pop();
            continue;
        }

        common_msgs::UWB_FullNeighborDatabase::ConstPtr ndb_msg = m_ndb_buf.front();
        m_ndb_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (m_imu_buf.front()->header.stamp.toSec() < ndb_msg->header.stamp.toSec() + m_td)
        {
            IMUs.emplace_back(m_imu_buf.front());
            m_imu_buf.pop();
        }
        IMUs.emplace_back(m_imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two uwb range");
        measurements.emplace_back(IMUs, ndb_msg);

    }

    return measurements;

}

void Localization_Node::process()
{
    while(true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, common_msgs::UWB_FullNeighborDatabase::ConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        m_con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();

        m_process.lock();
        for(auto &measurement : measurements)
        {
            common_msgs::UWB_FullNeighborDatabase::ConstPtr ndb_msg = measurement.second;

            // initialize by zero or Ceres
            UWB_Loc_Init initializer(*ndb_msg, m_p_mobile);
            if(!initializer.initializeMobileByCeres())
            {
               ROS_ERROR("UWB is not initialized!");
               return;
            }

            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            double ndb_t = ndb_msg->header.stamp.toSec() + m_td;
    
            for(auto &imu_msg: measurement.first)
            {
                double t = imu_msg->header.stamp.toSec();
                
                if(t <= ndb_t)
                {
                    if(m_current_time < 0)
                    {
                        m_current_time = t;
                    }

                    double dt = t - m_current_time;
                    ROS_ASSERT(dt >= 0);

                    m_current_time = t;

                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;

                    m_p_mobile->processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));

                }else
                {
                    double dt_1 = ndb_t - m_current_time;
                    double dt_2 = t-ndb_t;

                    m_current_time = ndb_t;

                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;

                    m_p_mobile->processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));

                }

            }

            if(m_p_mobile->filter3DUpdate())
            {
                pubOdometry(ndb_t);

            }

        }

        m_process.unlock();

    }

}

void Localization_Node::pubOdometry(double header)
{
    m_p_mobile->printPosition();
    m_p_mobile->printVelocity();
    m_p_mobile->printOrientation();

    geometry_msgs::PointStamped position = m_p_mobile->getPosition();
    geometry_msgs::PointStamped velocity = m_p_mobile->getVelocity();
    Eigen::Quaterniond orientation = m_p_mobile->getOrientation();


    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(header);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = position.point.x;
    odometry.pose.pose.position.y = position.point.y;
    odometry.pose.pose.position.z = position.point.z;

    odometry.pose.pose.orientation.w = orientation.w();
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();

    odometry.twist.twist.linear.x = velocity.point.x;
    odometry.twist.twist.linear.y = velocity.point.y;
    odometry.twist.twist.linear.z = velocity.point.z;

    m_uwb_odometry_pub.publish(odometry);

}

void Localization_Node::createUWBMobile(const int mobile_id){
    // get anchor list from parameter server
    std::string param_key("anchor_list");
    if(!m_nh.getParam(param_key, m_anchor_list)){
        ROS_ERROR("Can't find anchor list param.");
        return;
    }

    // get anchor position, build anchor_map
    for(int i=0;i<m_anchor_list.size();++i){
        int anchor_id = m_anchor_list.at(i);

        param_key = std::string("anchor_") + num2str(anchor_id);
        std::vector<double> position;
        if(m_nh.getParam(param_key, position) && position.size()==3){
            // has valid position
            // set anchor, variance set to constant 1
            boost::shared_ptr<UWB_Anchor> p_anchor(
                        new UWB_Anchor( anchor_id,
                                               position.at(0), position.at(1), position.at(2),
                                               1, 1, 1 ) );
            m_anchor_map.insert(std::pair<int, boost::shared_ptr<UWB_Anchor> >(anchor_id, p_anchor) );
        }
    }

    // create pointer
    m_p_mobile = boost::shared_ptr<UWB_Mobile>(new UWB_Mobile(mobile_id, m_anchor_map, m_nh) );
}


int main(int argc, char **argv){

    //initialize ros
    ros::init(argc, argv, "uwb_odom");
    ros::NodeHandle nh("~");

    Localization_Node loc_node(nh);

    std::thread loc_thread(&Localization_Node::process, &loc_node);

    ros::spin();

    //google::InitGoogleLogging(argv[0]);

    return 0;
}
