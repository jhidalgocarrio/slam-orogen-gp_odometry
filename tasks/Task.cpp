/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

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
    this->input_vector = this->meanSamples();
    this->gp_x.predict(this->input_vector);

    return;
}

void Task::joints_samplesCallback(const base::Time &ts, const ::base::samples::Joints &joints_samples_sample)
{
    this->joints_samples.push_back(joints_samples_sample);

    if(this->joints_samples.size() > this->gp_number_samples)
    {
        this->joints_samples.pop_front();
    }

}

void Task::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
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
    this->gp_number_samples = 1.0/_gaussian_process_predict_frequency.value();
    this->gp_counter_samples = 0;

    /*************************************************/
    /** Create and configure the Gaussian processes **/
    /*************************************************/
    this->gp_x.init(_gaussian_process_x_axis_file.value());
    std::vector<double> kernel_params = this->gp_x.kernel_();

    RTT::log(RTT::Warning)<<"[GP_ODOMETRY] Gaussian Process Model with Kernel parameters:"<<RTT::endlog();
    for (std::vector<double>::const_iterator i = kernel_params.begin(); i != kernel_params.end(); ++i)
    {
        RTT::log(RTT::Warning)<< *i <<' '<<RTT::endlog();
    }
    RTT::log(RTT::Warning)<<RTT::endlog();

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
    std::vector<double> mean_samples;

    return mean_samples;
}
