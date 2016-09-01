/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "GpyTask.hpp"

//#define DEBUG_PRINTS 1

using namespace gp_odometry;

GpyTask::GpyTask(std::string const& name)
    : GpyTaskBase(name)
{
}

GpyTask::GpyTask(std::string const& name, RTT::ExecutionEngine* engine)
    : GpyTaskBase(name, engine)
{
}

GpyTask::~GpyTask()
{
}

void GpyTask::delta_pose_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_sample)
{
    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[GP_ODOMETRY DELTA_POSE_SAMPLES] Received time-stamp: "<<delta_pose_samples_sample.time.toMicroseconds()<<RTT::endlog();
    #endif

    /** Prediction is required **/
    if ((this->joints_samples.size() == this->gp_number_samples)
            &&(this->orientation_samples.size() == this->gp_number_samples)
            &&(this->inertial_samples.size() == this->gp_number_samples))
    {
        /** Input vector **/
        std::vector<double> input_vector = this->meanSamples();

        /** Predict **/
        this->pred_mean = gp.predict("m", input_vector, pred_var);

        /** Reset the input **/
        this->joints_samples.clear();
        this->orientation_samples.clear();
        this->inertial_samples.clear();

        #ifdef DEBUG_PRINTS
        std::cout<<"[GP_ODOMETRY DELTA_POSE_SAMPLES] PRED_MEAN: ";
        for (std::vector<double>::const_iterator it = this->pred_mean.begin(); it != this->pred_mean.end(); ++it)
        {
            std::cout << *it << ' ';
        }
        std::cout<<"\n";
        #endif
    }

    /** Residual as uncertainty in delta position **/
    ::base::samples::RigidBodyState delta_pose = delta_pose_samples_sample;
    ::base::Vector3d prediction = Eigen::Map<const Eigen::Vector3d> (&(this->pred_mean[0]), this->pred_mean.size());
    delta_pose.cov_position = static_cast<base::Matrix3d>(prediction.asDiagonal());

    /** Port out the delta pose **/
    _delta_pose_samples_out.write(delta_pose);
}

void GpyTask::inertial_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample)
{
    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[GP_ODOMETRY INERTIAL_SAMPLES] Received time-stamp: "<<inertial_samples_sample.time.toMicroseconds()<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[GP_ODOMETRY INERTIAL_SAMPLES] inertial_samples.size(): "<<this->inertial_samples.size()<<RTT::endlog();
    #endif

    this->inertial_samples.push_back(inertial_samples_sample);

    if(this->inertial_samples.size() > this->gp_number_samples)
    {
        this->inertial_samples.pop_front();
    }

}

void GpyTask::joints_samplesCallback(const base::Time &ts, const ::base::samples::Joints &joints_samples_sample)
{
    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[GP_ODOMETRY JOINT_SAMPLES] Received time-stamp: "<<joints_samples_sample.time.toMicroseconds()<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[GP_ODOMETRY JOINT_SAMPLES] joints_samples.size(): "<<this->joints_samples.size()<<RTT::endlog();
    #endif

    this->joints_samples.push_back(joints_samples_sample);

    if(this->joints_samples.size() > this->gp_number_samples)
    {
        this->joints_samples.pop_front();
    }


}

void GpyTask::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[GP_ODOMETRY ORIENTATION_SAMPLES] Received time-stamp: "<<orientation_samples_sample.time.toMicroseconds()<<RTT::endlog();
    RTT::log(RTT::Warning)<<"[GP_ODOMETRY ORIENTATION_SAMPLES] orientation_samples.size(): "<<this->orientation_samples.size()<<RTT::endlog();
    #endif

    base::Vector3d euler_orientation;
    this->orientationToEuler (orientation_samples_sample.orientation, euler_orientation);

    this->orientation_samples.push_back(euler_orientation);

    if(this->orientation_samples.size() > this->gp_number_samples)
    {
        this->orientation_samples.pop_front();
    }

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See GpyTask.hpp for more detailed
// documentation about them.

bool GpyTask::configureHook()
{
    if (! GpyTaskBase::configureHook())
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

    /*************************************************/
    /** Create and configure the Gaussian processes **/
    /*************************************************/
    this->gp.init(_path_to_init.value());
    this->gp.load(_gaussian_process_file.value(), "m");
    std::vector<std::string> param_names = gp.parameterNames("m.model");
    RTT::log(RTT::Warning)<<"[GP_ODOMETRY] Loaded Gaussian Process with parameters: ";
    for (std::vector<std::string>::const_iterator it = param_names.begin(); it != param_names.end(); ++it)
    {
        RTT::log(RTT::Warning)<< *it;
        RTT::log(RTT::Warning)<< ' ';
    }
    RTT::log(RTT::Warning)<<RTT::endlog();

    /** Configure the prediction variables **/
    this->pred_mean.resize(3);
    this->pred_var.resize(1);

    /*****************************/
    /** Clear storage variables **/
    /*****************************/
    this->joints_samples.clear();
    this->orientation_samples.clear();
    this->inertial_samples.clear();

    return true;
}
bool GpyTask::startHook()
{
    if (! GpyTaskBase::startHook())
        return false;
    return true;
}
void GpyTask::updateHook()
{
    GpyTaskBase::updateHook();
}
void GpyTask::errorHook()
{
    GpyTaskBase::errorHook();
}
void GpyTask::stopHook()
{
    GpyTaskBase::stopHook();

    /** Clear the input **/
    this->joints_samples.clear();
    this->orientation_samples.clear();
    this->inertial_samples.clear();
}
void GpyTask::cleanupHook()
{
    GpyTaskBase::cleanupHook();
}

std::vector<double> GpyTask::meanSamples()
{
    std::vector<double> position_joints, speed_joints;

    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[GP_ODOMETRY MEAN_SAMPLES] ["<<this->orientation_samples.size()<<"] ["<<this->inertial_samples.size()<<"] ["<<this->joints_samples.size()<<"]"<<RTT::endlog();
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
    ::base::Vector3d euler; euler.setZero();
    for (std::list< ::base::Vector3d >::iterator it=this->orientation_samples.begin(); it != this->orientation_samples.end(); ++it)
    {
        euler += (*it);
    }

    euler = euler/static_cast<double>(this->orientation_samples.size());

    /** ******************* **/
    /** Inertial Values     **/
    /** ******************* **/
    std::vector<double> inertial_values (this->inertial_samples.front().acc.size() +
            this->inertial_samples.front().gyro.size());

    for(std::list< ::base::samples::IMUSensors >::const_iterator it = this->inertial_samples.begin();
                                                 it != this->inertial_samples.end(); ++it)
    {
        inertial_values[0] += (*it).acc[0];
        inertial_values[1] += (*it).acc[1];
        inertial_values[2] += (*it).acc[2];

        inertial_values[3] += (*it).gyro[0];
        inertial_values[4] += (*it).gyro[1];
        inertial_values[5] += (*it).gyro[2];
    }

    for (std::vector<double>::iterator it = inertial_values.begin();
            it != inertial_values.end(); ++it)
    {
        *it /= static_cast<double>(this->inertial_samples.size());

    }

    /** ******************* **/
    /** Mean samples vector **/
    /** ******************* **/
    std::vector<double> samples_mean;

    /** Orientation **/
    samples_mean.push_back(euler[2]);//Roll
    samples_mean.push_back(euler[1]);//Pitch

    /** Position Joints **/
    for (register size_t i=0; i<position_joints.size(); ++i)
    {
        samples_mean.push_back(position_joints[i]);
    }

    /** Speed Joints **/
    for (register size_t i=0; i<speed_joints.size(); ++i)
    {
        samples_mean.push_back(speed_joints[i]);
    }

    /** Inertial sensor information (acceleration and gyro)**/
    for (register size_t i=0; i<inertial_values.size(); ++i)
    {
        samples_mean.push_back(inertial_values[i]);
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"[GP_ODOMETRY MEAN_SAMPLES] samples_mean of size: "<< samples_mean.size() <<"\n";

    for (std::vector<double>::const_iterator i = samples_mean.begin(); i != samples_mean.end(); ++i)
    {
        std::cout << *i << ' ';
    }
    std::cout<<"\n";
    #endif

    return samples_mean;
}

void GpyTask::orientationToEuler(const ::base::Orientation &orient, ::base::Vector3d &euler)
{

    euler = ::base::getEuler(orient);

    return;
}
