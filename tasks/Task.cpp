/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

//#define DEBUG_PRINTS 1

using namespace gp_odometry;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::delta_pose_samplesCallback(const base::Time &ts, const ::base::samples::BodyState &delta_pose_samples_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<<"[GP_ODOMETRY DELTA_POSE_SAMPLES] Received time-stamp: "<<delta_pose_samples_sample.time.toMicroseconds()<<"\n";
    #endif

    this->angular_velocity_samples.push_back(delta_pose_samples_sample.angular_velocity());

    if(this->angular_velocity_samples.size() > this->gp_number_samples)
    {
        this->angular_velocity_samples.pop_front();
    }

    if ((this->joints_samples.size() == this->gp_number_samples)
            &&(this->orientation_samples.size() == this->gp_number_samples)
            &&(this->angular_velocity_samples.size() == this->gp_number_samples))
    {

        this->input_vector = this->meanSamples();
        double x_value = this->gp_x.predict(this->input_vector);

        #ifdef DEBUG_PRINTS
        std::cout<<"[GP_ODOMETRY DELTA_POSE_SAMPLES] GP Input vector: \n";
        for (std::vector<double>::const_iterator i = this->input_vector.begin(); i != this->input_vector.end(); ++i)
        {
            std::cout << *i << ' ';
        }
        std::cout<<"\n";
        std::cout<<"[GP_ODOMETRY DELTA_POSE_SAMPLES] X-Output: "<<x_value<<std::endl;
        #endif

        /** Get the new delta pose **/
        this->delta_pose = delta_pose_samples_sample;

        /** On-line covariance **/
        std::vector<double> sigma_pose(3, 0.00);
        sigma_pose[0] = std::abs(this->delta_pose.velocity.vel[0] - x_value)*this->_delta_pose_samples_period.value();
        this->sigma_poses.push_back(sigma_pose);
        Eigen::Matrix<double, 3, 3> cov_position;
        cov_position = this->delta_pose.cov_pose().bottomRightCorner<3,3>();
        this->onlineCovariance (cov_position);
        this->delta_pose.cov_pose().bottomRightCorner<3,3>() = cov_position;
        this->delta_pose.cov_velocity().bottomRightCorner<3,3>() = cov_position / this->_delta_pose_samples_period.value();

        /** Port out the delta pose **/
        this->_delta_pose_samples_out.write(this->delta_pose);
    }

    return;
}

void Task::joints_samplesCallback(const base::Time &ts, const ::base::samples::Joints &joints_samples_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<<"[GP_ODOMETRY JOINT_SAMPLES] Received time-stamp: "<<joints_samples_sample.time.toMicroseconds()<<"\n";
    std::cout<<"[GP_ODOMETRY ORIENTATION_SAMPLES] orientation_samples.size(): "<<this->joints_samples.size()<<"\n";
    #endif

    this->joints_samples.push_back(joints_samples_sample);

    if(this->joints_samples.size() > this->gp_number_samples)
    {
        this->joints_samples.pop_front();
    }

}

void Task::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout<<"[GP_ODOMETRY ORIENTATION_SAMPLES] Received time-stamp: "<<orientation_samples_sample.time.toMicroseconds()<<"\n";
    std::cout<<"[GP_ODOMETRY ORIENTATION_SAMPLES] orientation_samples.size(): "<<this->orientation_samples.size()<<"\n";
    #endif

    this->orientation_samples.push_back(orientation_samples_sample);

    if(this->orientation_samples.size() > this->gp_number_samples)
    {
        this->orientation_samples.pop_front();
    }

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /************************/
    /** Read configuration **/
    /************************/
    double input_port_samples_per_seconds = (1.0)/_delta_pose_samples_period.value();
    this->gp_number_samples = (1.0/_gaussian_process_predict_frequency.value()) * input_port_samples_per_seconds;
    this->gp_counter_samples = 0;
    this->position_joint_names = _gaussian_process_position_joint_names.value();
    this->speed_joint_names = _gaussian_process_speed_joint_names.value();
    RTT::log(RTT::Warning)<<"[GP_ODOMETRY] Gaussian Process Frequency[Hertz]: ";
    RTT::log(RTT::Warning)<<_gaussian_process_predict_frequency.value()<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[GP_ODOMETRY] Gaussian Process Number samples to average: ";
    RTT::log(RTT::Warning)<<this->gp_number_samples<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[GP_ODOMETRY] Position Joints Names:\n";
    for (std::vector<std::string>::const_iterator it = this->position_joint_names.begin(); it != this->position_joint_names.end(); ++it)
    {
        RTT::log(RTT::Warning)<< *it;
        RTT::log(RTT::Warning)<< ' ';
    }
    RTT::log(RTT::Warning)<<RTT::endlog();;

    RTT::log(RTT::Warning)<<"[GP_ODOMETRY] Speed Joints Names:\n";
    for (std::vector<std::string>::const_iterator it = this->speed_joint_names.begin(); it != this->speed_joint_names.end(); ++it)
    {
        RTT::log(RTT::Warning)<< *it;
        RTT::log(RTT::Warning)<< ' ';
    }
    RTT::log(RTT::Warning)<<RTT::endlog();;

    /*************************************************/
    /** Create and configure the Gaussian processes **/
    /*************************************************/
    this->gp_x.init(_gaussian_process_x_axis_file.value());
    std::vector<double> kernel_params = this->gp_x.kernel_();

    RTT::log(RTT::Warning)<<"[GP_ODOMETRY] Gaussian Process Model for X-axis with Kernel parameters: [ ";
    for (std::vector<double>::const_iterator it = kernel_params.begin(); it != kernel_params.end(); ++it)
    {
        RTT::log(RTT::Warning)<< *it;
        RTT::log(RTT::Warning)<< ' ';
    }
    RTT::log(RTT::Warning)<<"]"<<RTT::endlog();

    /** Sigma weights **/
    this->sigma_weights.resize(this->gp_number_samples);
    this->cubicWeights(this->sigma_weights);

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    this->angular_velocity_samples.clear();
    this->joints_samples.clear();
    this->orientation_samples.clear();

    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

std::vector<double> Task::meanSamples()
{
    ::base::Vector3d angular_velocity(0.00, 0.00, 0.00);
    Eigen::Quaterniond orientation_quaternion;
    std::vector<double> position_joints, speed_joints;
    std::vector<double> samples_mean;

    #ifdef DEBUG_PRINTS
    std::cout<<"[GP_ODOMETRY MEAN_SAMPLES] \n";
    #endif

    /** ********* **/
    /**  Joints   **/
    /** ********* **/

    /** Compute the mean values for joints position **/
    for(std::vector<std::string>::const_iterator it = this->position_joint_names.begin();
                                                 it != this->position_joint_names.end(); ++it)
    {
        double position_mean = 0.00;

        for (std::list< ::base::samples::Joints >::const_iterator ir = this->joints_samples.begin();
                                                                  ir != this->joints_samples.end(); ++ir)
        {
            position_mean += (*ir)[*it].position;
        }

        position_mean /= static_cast<double>(this->joints_samples.size());
        position_joints.push_back(position_mean);
    }

    /** Compute the mean values for joints speed **/
    for(std::vector<std::string>::const_iterator it = this->speed_joint_names.begin();
                                                 it != this->speed_joint_names.end(); ++it)
    {
        double speed_mean = 0.00;

        for (std::list< ::base::samples::Joints >::const_iterator ir = this->joints_samples.begin();
                                                                  ir != this->joints_samples.end(); ++ir)
        {
            speed_mean += (*ir)[*it].speed;
        }

        speed_mean /= static_cast<double>(this->joints_samples.size());
        speed_joints.push_back(speed_mean);
    }

    /** ******************* **/
    /** Orientation samples **/
    /** ******************* **/
    double w = 0.00;
    double x = 0.00;
    double y = 0.00;
    double z = 0.00;

    for (std::list< ::base::samples::RigidBodyState >::iterator it=this->orientation_samples.begin(); it != this->orientation_samples.end(); ++it)
    {
        w += (*it).orientation.w();
        x += (*it).orientation.x();
        y += (*it).orientation.y();
        z += (*it).orientation.z();
    }

    w = w/static_cast<double>(this->orientation_samples.size());
    x = x/static_cast<double>(this->orientation_samples.size());
    y = y/static_cast<double>(this->orientation_samples.size());
    z = z/static_cast<double>(this->orientation_samples.size());
    orientation_quaternion = Eigen::Quaterniond(w, x, y, z);
    orientation_quaternion.normalize();


    /** ******************* **/
    /** Angular Velocity    **/
    /** ******************* **/
    for(std::list< ::base::Vector3d >::const_iterator it = this->angular_velocity_samples.begin();
                                                 it != this->angular_velocity_samples.end(); ++it)
    {
        angular_velocity[0] += (*it)[0];
        angular_velocity[1] += (*it)[1];
        angular_velocity[2] += (*it)[2];
    }

    angular_velocity[0] /= static_cast<double>(this->angular_velocity_samples.size());
    angular_velocity[1] /= static_cast<double>(this->angular_velocity_samples.size());
    angular_velocity[2] /= static_cast<double>(this->angular_velocity_samples.size());

    /** ******************* **/
    /** Mean samples vector **/
    /** ******************* **/

    /** Speed Joints **/
    for (register unsigned int i=0; i<speed_joints.size(); ++i)
    {
        samples_mean.push_back(speed_joints[i]);
    }

    /** Position Joints **/
    for (register unsigned int i=0; i<position_joints.size(); ++i)
    {
        samples_mean.push_back(position_joints[i]);
    }

    /** Inertial sensor information **/
    for (register int i=0; i<angular_velocity.size(); ++i)
    {
        samples_mean.push_back(angular_velocity[i]);
    }

    /** Orientation information **/
    ::base::Vector3d euler = base::getEuler(orientation_quaternion);
    samples_mean.push_back(euler[2]);//Roll
    samples_mean.push_back(euler[1]);//Pitch

    return samples_mean;
}

void Task::onlineCovariance (Eigen::Matrix<double, 3, 3>&  covariance)
{

    Eigen::Vector3d variance(0.00, 0.00, 0.00);
    Eigen::Vector3d sigma;
    //sigma[1] = covariance(1,1); sigma[2] = covariance(2,2);

    if (this->sigma_poses.size() < 2.0)
    {

        std::vector<double> sigma_pose = this->sigma_poses.front();
        sigma = Eigen::Map< const Eigen::Vector3d >(&(sigma_pose[0]), sigma_pose.size());
        variance =  sigma;
    }
    else
    {
        for(std::list< std::vector<double> >::const_iterator it = this->sigma_poses.begin();
                                                 it != this->sigma_poses.end(); ++it)
        {
            sigma = Eigen::Map< const Eigen::Vector3d >(&((*it)[0]), (*it).size());
            variance +=  sigma;
        }

        variance /= this->sigma_poses.size();
    }

    std::cout<<"covariance:\n"<<covariance<<"\n";

    /** Compute the variance = std^2*/
    variance[0] = variance[0] * variance[0];
    variance[1] = variance[1] * variance[1];
    variance[2] = variance[2] * variance[2];

    covariance.diagonal() = variance;
    ::base::guaranteeSPD< Eigen::Matrix<double, 3, 3> >(covariance);

    std::cout<<"covariance:\n"<<covariance<<"\n";

    this->sigma_poses.pop_front();

    return;
}

void Task::cubicWeights(std::vector<double>& weights)
{
    double alpha = 0.8;

    if (weights.size() > 0)
    {
        /** reverse order because the newest samples is in the back **/
        weights[weights.size()-1] = std::pow(weights.size(), 3);
        for (register unsigned int i =  weights.size()-2; i > 0; --i)
        {
            weights[i] = std::pow(i+1, 3);
        }
    }

    /** Normalize the weights **/
    double sum_weights = 0.00;
    for(std::vector<double>::const_iterator it = weights.begin(); it != weights.end(); ++it)
    {
        sum_weights += (*it);
    }

    for(std::vector<double>::iterator it = weights.begin(); it != weights.end(); ++it)
    {
        (*it) = (*it)/sum_weights;
    }

    return;
}
