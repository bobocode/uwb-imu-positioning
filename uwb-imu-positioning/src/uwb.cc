#include "uwb.h"

UWB_Node::UWB_Node(const int node_id):m_node_id(node_id){}

// ---------------------------------------------
UWB_Anchor::UWB_Anchor(const int node_id,
                              const double x, const double y, const double z,
                              const double var_x, const double var_y, const double var_z)
    :UWB_Node(node_id)
{
    setPosition(x,y,z);
}

void UWB_Anchor::insertRangeInfo(const common_msgs::UWB_EchoedRangeInfo &msg){
    // only insert range info that is requested by myself.

    int requester_id = msg.requesterId;
    int responder_id = msg.responderId;
    double range = double(msg.precisionRangeMm) / 1000.0;

    if(m_node_id==requester_id){
        // if haven't seen the responder yet, create it
        if(m_range_map.find(responder_id)==m_range_map.end()){
            m_range_map.insert(std::pair<int, std::vector<double> >(responder_id, std::vector<double>()));
            // add the range measurement into the vector
            m_range_map.find(responder_id)->second.push_back(range);
        } else {
            // add the range measurement into the vector
            m_range_map.find(responder_id)->second.push_back(range);
        }
    }
}

void UWB_Anchor::insertRangeInfo(const common_msgs::UWB_FullRangeInfo& msg){
    // only insert range info that is requested by myself.

    int requester_id = msg.nodeId;
    int responder_id = msg.responderId;
    double range = double(msg.precisionRangeMm) / 1000.0;

    if(m_node_id==requester_id){
        // if haven't seen the responder yet, create it
        if(m_range_map.find(responder_id)==m_range_map.end()){
            m_range_map.insert(std::pair<int, std::vector<double> >(responder_id, std::vector<double>()));
            // add the range measurement into the vector
            m_range_map.find(responder_id)->second.push_back(range);
        } else {
            // add the range measurement into the vector
            m_range_map.find(responder_id)->second.push_back(range);
        }
    }
}

double UWB_Anchor::getAverageRangeToNode(const int anchor_id){
    // check whether the anchor_id has been seen
    std::map<int, std::vector<double> >::iterator it = m_range_map.find(anchor_id);
    if(it==m_range_map.end()){
        return 0;
    } else {
        // check the size of the vector
        if(it->second.size()==0){
            return 0;
        } else {
            return robustAverage<double>(it->second);
        }
    }
}

bool UWB_Anchor::getMedianAndVarianceToNode(const int anchor_id, double &median, double &variance){
    std::map<int, std::vector<double> >::iterator it = m_range_map.find(anchor_id);
    if(it==m_range_map.end()){
        return false;
    } else {
        if(it->second.size()==0){
            return false;
        } else {
            medianAndVariance<double>(it->second, median, variance);
            return true;
        }
    }
}


UWB_Mobile::UWB_Mobile(const int node_id,
                              const std::map<int, boost::shared_ptr<UWB_Anchor> > anchor_map,
                              ros::NodeHandle& nh)
    :UWB_Node(node_id),
      m_anchor_map(anchor_map),
      m_nh(nh)
{
    m_g.setZero();
    m_nh.param<double>("G", m_g.z(), 9.8);

    m_first_imu = false;

}

void UWB_Mobile::fullInitialize(const common_msgs::UWB_FullNeighborDatabase& ndb, const geometry_msgs::PointStamped& position){
    setPosition(position.point.x, position.point.y, position.point.z);
    setVelocity(0,0,0);
    setAccelerationBias(Eigen::Vector3d(0,0,0));
    setAngularBias(Eigen::Vector3d(0,0,0));

    m_ranges_to_anchors.clear();

    common_msgs::UWB_FullRangeInfo zero_range_info;
    for(int i=0;i<ndb.numNeighborEntries;++i){
        int anchor_id = ndb.neighbors.at(i).nodeId;

        zero_range_info.header.stamp = ndb.header.stamp;
        zero_range_info.responderId = anchor_id;
        zero_range_info.precisionRangeMm = ndb.neighbors.at(i).rangeMm;
        zero_range_info.precisionRangeErrEst = ndb.neighbors.at(i).rangeErrorEstimate;

        // avoid invalid ndb entry
        if(zero_range_info.precisionRangeMm>50)
        {
            m_ranges_to_anchors.insert(std::make_pair(anchor_id,zero_range_info));
        }
    }

    m_last_range_time = ros::Time::now();
}

double UWB_Mobile::getAverageRangeErrEst()
{
    double sum_range_err = 0.0;
    std::map<int, common_msgs::UWB_FullRangeInfo>::iterator iter;
    for(iter=m_ranges_to_anchors.begin();iter!=m_ranges_to_anchors.end();++iter)
    {
        sum_range_err += iter->second.precisionRangeErrEst;
    }

    return sum_range_err/m_ranges_to_anchors.size();
}


void UWB_Mobile::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if(!m_first_imu)
    {
        m_first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if(!pre_integration)
    {
        pre_integration = new IntegrationBase{acc_0, gyr_0, m_acceleration_bias,m_angular_bias};
    }

    pre_integration->push_back(dt, linear_acceleration,angular_velocity);
    m_td_buf.push_back(dt);
    m_linear_acceleration_buf.push_back(linear_acceleration);
    m_angular_velocity_buf.push_back(angular_velocity);

    Eigen::Vector3d un_acc_0 = m_pred_rotation * (acc_0 - m_acceleration_bias) -m_g;
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - m_angular_bias;
    m_pred_rotation *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
    Eigen::Vector3d un_acc_1 = m_pred_rotation * (linear_acceleration - m_acceleration_bias) - m_g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    m_pred_position += dt * m_pred_velocity + 0.5 * dt * dt * un_acc;
    m_pred_velocity += dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;

}

bool UWB_Mobile::filter3DUpdate()
{
    if(std::string("tightly")==m_filter_type)
    {
        return tightlyFGOUpdate();

    } else if(std::string("loosely")==m_filter_type)
    {
        return looselyFGOUpdate();

    }else
    {   
        ROS_WARN("Unknown filter type, use tightly-FGO now.");
        return tightlyFGOUpdate();
    } 

}

void UWB_Mobile::doubleToVector()
{
    setLastPosition(Eigen::Vector3d(para_pose[1][0],para_pose[1][1],para_pose[1][2]));
    setLastVelocity(Eigen::Vector3d(para_speed_bias[1][0],para_speed_bias[1][1],para_speed_bias[1][2]));
    setLastRotation(Eigen::Quaterniond(para_pose[1][6],para_pose[1][3],para_pose[1][4],para_pose[1][5]).toRotationMatrix());

    setPosition(para_pose[1][0],para_pose[1][1],para_pose[1][2]);
    setVelocity(para_speed_bias[1][0],para_speed_bias[1][1],para_speed_bias[1][2]);
    setOrientation(para_pose[1][3], para_pose[1][4], para_pose[1][5], para_pose[1][6]);

    m_acceleration_bias = Eigen::Vector3d(para_speed_bias[1][3],para_speed_bias[1][4],para_speed_bias[1][5]);
    m_angular_bias = Eigen::Vector3d(para_speed_bias[1][6],para_speed_bias[1][7], para_speed_bias[1][8]);

}

void UWB_Mobile::vectorToDouble()
{
    //last
    para_pose[0][0] = m_last_position.x();
    para_pose[0][1] = m_last_position.y();
    para_pose[0][2] = m_last_position.z();
    Eigen::Quaterniond q{m_last_rotation};

    para_pose[0][3] = q.x();
    para_pose[0][4] = q.y();
    para_pose[0][5] = q.z();
    para_pose[0][6] = q.w();

    para_speed_bias[0][0] = m_last_velocity.x();
    para_speed_bias[0][1] = m_last_velocity.y();
    para_speed_bias[0][2] = m_last_velocity.z();

    para_speed_bias[0][3] = m_acceleration_bias.x();
    para_speed_bias[0][4] = m_acceleration_bias.y();
    para_speed_bias[0][5] = m_acceleration_bias.z();

    para_speed_bias[0][6] = m_angular_bias.x();
    para_speed_bias[0][7] = m_angular_bias.y();
    para_speed_bias[0][8] = m_angular_bias.z();

    //current pred
    para_pose[1][0] = m_pred_position.x();
    para_pose[1][1] = m_pred_position.y();
    para_pose[1][2] = m_pred_position.z();
    Eigen::Quaterniond q1{m_pred_rotation};

    para_pose[1][3] = q1.x();
    para_pose[1][4] = q1.y();
    para_pose[1][5] = q1.z();
    para_pose[1][6] = q1.w();

    para_speed_bias[1][0] = m_pred_velocity.x();
    para_speed_bias[1][1] = m_pred_velocity.y();
    para_speed_bias[1][2] = m_pred_velocity.z();

    para_speed_bias[1][3] = m_acceleration_bias.x();
    para_speed_bias[1][4] = m_acceleration_bias.y();
    para_speed_bias[1][5] = m_acceleration_bias.z();

    para_speed_bias[1][6] = m_angular_bias.x();
    para_speed_bias[1][7] = m_angular_bias.y();
    para_speed_bias[1][8] = m_angular_bias.z();
}

bool UWB_Mobile::tightlyFGOUpdate()
{
    vectorToDouble();
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);

    for(int i =0; i < 2; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_pose[i], 7, local_parameterization);
        problem.AddParameterBlock(para_speed_bias[i],9);
    }

    problem.SetParameterBlockConstant(para_pose[0]);
    problem.SetParameterBlockConstant(para_speed_bias[0]);

    if(pre_integration->sum_dt > 10.0)
    {
        return false;
    }

    IMUFactor* imu_factor = new IMUFactor(pre_integration);
    problem.AddResidualBlock(imu_factor, NULL, para_pose[0], para_speed_bias[0], para_pose[1], para_speed_bias[1]);

    std::map<int, common_msgs::UWB_FullRangeInfo>::iterator iter;
    for(iter=m_ranges_to_anchors.begin();iter!=m_ranges_to_anchors.end();++iter)
    {
        int anchor_id = iter->first;
        geometry_msgs::PointStamped anchor_position;
        if(getAnchorPosition(anchor_id, anchor_position))
        {
            ceres::CostFunction* cost_function = UWBRangeFactor::Create(anchor_position.point.x,
                                                                          anchor_position.point.y,
                                                                          anchor_position.point.z,
                                                                          iter->second.precisionRangeMm/1000.0,
                                                                          iter->second.precisionRangeErrEst/1000.0*5);

            problem.AddResidualBlock(cost_function, loss_function, &(para_pose[1][0]), &(para_pose[1][1]),&(para_pose[1][2]));
        }
        
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 10000;
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout<<"\n"<<summary.BriefReport()<<"\n";
    //    std::cout << "Initial cost: " << summary.initial_cost << "  Final cost: "<< summary.final_cost << '\n';
    doubleToVector();

    return summary.IsSolutionUsable();
    //return true;

}

bool UWB_Mobile::looselyFGOUpdate()
{
    vectorToDouble();
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);

    for(int i =0; i < 2; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_pose[i], 7, local_parameterization);
        problem.AddParameterBlock(para_speed_bias[i],9);
    }

    problem.SetParameterBlockConstant(para_pose[0]);
    problem.SetParameterBlockConstant(para_speed_bias[0]);

    if(pre_integration->sum_dt > 10.0)
    {
        return false;
    }

    IMUFactor* imu_factor = new IMUFactor(pre_integration);
    problem.AddResidualBlock(imu_factor, NULL, para_pose[0], para_speed_bias[0], para_pose[1], para_speed_bias[1]);

    ceres::CostFunction* cost_function = UWBPositionFactor::Create(m_position.point.x,
                                                                    m_position.point.y,
                                                                    m_position.point.z,
                                                                    getAverageRangeErrEst()/1000.0 * 5);

    problem.AddResidualBlock(cost_function, loss_function, &(para_pose[1][0]), &(para_pose[1][1]),&(para_pose[1][2]));

    
    ceres::Solver::Options options;
    options.max_num_iterations = 10000;
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout<<"\n"<<summary.BriefReport()<<"\n";
    //    std::cout << "Initial cost: " << summary.initial_cost << "  Final cost: "<< summary.final_cost << '\n';
    doubleToVector();

    return summary.IsSolutionUsable();
    //return true;

}
