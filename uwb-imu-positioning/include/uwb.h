#ifndef _UWB_H_
#define _UWB_H_

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>
#include <map>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "common_msgs/UWB_FullRangeInfo.h"
#include "common_msgs/UWB_FullNeighborDatabase.h"
#include "common_msgs/UWB_DataInfo.h"
#include "common_msgs/UWB_SendData.h"
#include "common_msgs/UWB_EchoedRangeInfo.h"
#include "common_msgs/NavigationState.h"
#include "pose_local_parameterization.h"
#include "basic_function.h"
#include "utility.h"
#include "imu_factor.h"
#include "uwb_position_factor.h"
#include "uwb_range_factor.h"


typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;


// -----------------------------
struct comp{
    template <typename T>
    inline bool operator()(const T& a, const T& b) const{
        return (a<b);
    }
};


template <typename T>
double robustAverage(std::vector<T> array){
    // the paramter is copied, so that the original data will not be affected.
    const int cut_length = 2;

    if(array.size()<=2*cut_length){
        return 0;
    }

    double sum = 0;
    std::sort(array.begin(), array.end(), comp());
    for(int i=cut_length;i<array.size()-cut_length;++i){
        sum+=array.at(i);
    }

    return sum / (array.size()-2*cut_length);

}

template <typename T>
bool medianAndVariance(std::vector<T> array, double& median, double& variance){
    if(array.size()==0){
        return false;
    }

    // get median
    std::sort(array.begin(), array.end(), comp());
    median = array.at(std::floor(array.size()/2));

    // get mean
    double sum = 0;
    for(int i=0;i<array.size();++i){
        sum+=array.at(i);
    }
    double mean = sum / array.size();


    // get variance
    double variance_sum = 0;
    for(int i=0;i<array.size();++i){
        double res = array.at(i)-mean;
        variance_sum += res*res;
    }
    variance = variance_sum / array.size();

    return true;
}

// -----------------------------


class UWB_Node{
public:
    explicit UWB_Node(const int node_id);

protected:
    int m_node_id;

    geometry_msgs::PointStamped m_position;
    geometry_msgs::PointStamped m_velocity;
    Eigen::Quaterniond m_orientation;



public:
    inline int getNodeId() const{
        return m_node_id;
    }
    inline geometry_msgs::PointStamped getPosition() const{
        return m_position;
    }
    inline geometry_msgs::PointStamped* getPositionPtr(){
        return &m_position;
    }
    inline geometry_msgs::PointStamped getVelocity() const{
        return m_velocity;
    }

    inline Eigen::Quaterniond getOrientation() const{
        return m_orientation;
    }

    inline void setOrientation(const double x, const double y, const double z, const double w)
    {
        m_orientation.w() = w;
        m_orientation.x() = x;
        m_orientation.y() = y;
        m_orientation.z() = z;
    }

    inline void setPosition(const double x, const double y, const double z){
        m_position.header.stamp = ros::Time::now();

        m_position.point.x = x;
        m_position.point.y = y;
        m_position.point.z = z;
    }

    inline void setVelocity(const double vx, const double vy, const double vz){
        m_velocity.header.stamp = ros::Time::now();

        m_velocity.point.x = vx;
        m_velocity.point.y = vy;
        m_velocity.point.z = vz;
    }



};

class UWB_Anchor : public UWB_Node{
public:
    explicit UWB_Anchor(const int node_id,
                        const double x, const double y, const double z,
                        const double var_x, const double var_y, const double var_z);

public:
    std::map<int, std::vector<double>> m_range_map;

    void insertRangeInfo(const common_msgs::UWB_EchoedRangeInfo& msg);
    void insertRangeInfo(const common_msgs::UWB_FullRangeInfo& msg);
    double getAverageRangeToNode(const int anchor_id);
    bool getMedianAndVarianceToNode(const int anchor_id, double& median, double& variance);

    inline void printPosition(){
        std::cout<<m_node_id<<": "<<m_position.point.x<<", "<<m_position.point.y<<", "<<m_position.point.z<<std::endl;
    }

};


class UWB_Mobile : public UWB_Node{
public:
    explicit UWB_Mobile(const int node_id,
                        const std::map<int, boost::shared_ptr<UWB_Anchor> > anchor_map,
                        ros::NodeHandle& nh);

private:
    // stores last correct/accepted measurements
    std::map<int, common_msgs::UWB_FullRangeInfo> m_ranges_to_anchors;
    ros::Time m_last_range_time;

    Eigen::Vector3d m_last_position;
    Eigen::Vector3d m_last_velocity;
    Eigen::Matrix3d m_last_rotation;

    Eigen::Vector3d m_pred_position;
    Eigen::Vector3d m_pred_velocity;
    Eigen::Matrix3d m_pred_rotation;


    // provided by parameter server
    std::map<int, boost::shared_ptr<UWB_Anchor> > m_anchor_map;
    Eigen::Vector3d m_g;

    Eigen::Vector3d m_acceleration_bias;
    Eigen::Vector3d m_angular_bias;
    Eigen::Matrix3d m_rotation;

    std::vector<double> m_td_buf;
    std::vector<Eigen::Vector3d> m_linear_acceleration_buf;
    std::vector<Eigen::Vector3d> m_angular_velocity_buf;

    Eigen::Vector3d acc_0, gyr_0;
    

    //imu
    IntegrationBase *tmp_pre_integration;
    IntegrationBase *pre_integration;
    bool m_first_imu;

    // parameters
    ros::NodeHandle m_nh;
    std::string m_filter_type;


public:
    // reading statistics, to get effective frequency
    std::map<int, int> m_anchor_reading_counter;
    std::map<int, int> m_anchor_successful_reading_counter;
    int m_least_anchor_num;

public:

    inline void setLastPosition(const Eigen::Vector3d position)
    {
        m_last_position = position;
        m_pred_position = position;
    }

    inline void setLastVelocity(const Eigen::Vector3d velocity)
    {
        m_last_velocity = velocity;
        m_pred_velocity = velocity;
    }

    inline void setLastRotation(const Eigen::Matrix3d rotation)
    {
        m_last_rotation = rotation;
        m_pred_rotation = rotation;
    }


    inline void setAngularBias(const Eigen::Vector3d angular_bias){
        m_angular_bias = angular_bias; 
    }

    inline Eigen::Vector3d getAngularBias() const
    {
        return m_angular_bias;

    }

    inline void setAccelerationBias(const Eigen::Vector3d acceleration_bias){
        m_acceleration_bias = acceleration_bias;
        
    }

    inline Eigen::Vector3d getAccelerationBias() const
    {
        return m_acceleration_bias;

    }

    inline common_msgs::UWB_FullRangeInfo getRangeInfoToAnchor(const int anchor_id) const{
        return m_ranges_to_anchors.find(anchor_id)->second;
    }
    inline double getRangeMeterToAnchor(const int anchor_id) const{
        return double(m_ranges_to_anchors.find(anchor_id)->second.precisionRangeMm) / 1000.0;
    }

    inline void setRangeInfoToAnchor(const common_msgs::UWB_FullRangeInfo& range_info){
        m_ranges_to_anchors[range_info.responderId] = range_info;
    }

    inline bool getAnchorPosition(const int anchor_id, geometry_msgs::PointStamped& anchor_position) const {
        if(m_anchor_map.find(anchor_id)!=m_anchor_map.end()){
            anchor_position = m_anchor_map.find(anchor_id)->second->getPosition();
            return true;
        } else {
            return false;
        }
    }
    inline int getAnchorNumber() const{
        return m_anchor_map.size();
    }

    
    void fullInitialize(const common_msgs::UWB_FullNeighborDatabase& ndb, const geometry_msgs::PointStamped& position);
    double getAverageRangeErrEst();
    
    // dynamic innovation threshold, to avoid filter divergence
    void getInnovation();
    void processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    bool filter3DUpdate();
    bool tightlyFGOUpdate();
    bool looselyFGOUpdate();
    void doubleToVector();
    void vectorToDouble();
    double para_pose[2][7];
    double para_speed_bias[2][9];
    

    // print
    inline void printPosition(){
        std::cout<<m_node_id<<":"<<std::endl;

        std::cout<<"pose: "<<m_position.point.x<<", "<<m_position.point.y<<", "<<m_position.point.z <<std::endl;
    }

    inline void printOrientation()
    {
        std::cout << "orientation: \n" << m_orientation.toRotationMatrix() << std::endl;

    }

    inline void printVelocity(){

            std::cout<<"velo: "<<m_velocity.point.x<<", "<<m_velocity.point.y<<", "<<m_velocity.point.z  <<std::endl;
        
    }

    inline void printAccelerationBias()
    {
        
            std::cout << "acc bias: " << m_acceleration_bias.x() << ", " << m_acceleration_bias.y() <<", " << m_acceleration_bias.z() << std::endl;
            std::cout << "angular bias: " << m_angular_bias.x() << ", " << m_angular_bias.y() << ", " << m_angular_bias.z() << std::endl;
    }
    
};





#endif