/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "GpyTask.hpp"

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
    throw std::runtime_error("Aggregator callback for delta_pose_samples not implemented");
}

void GpyTask::inertial_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &inertial_samples_sample)
{
    throw std::runtime_error("Aggregator callback for inertial_samples not implemented");
}

void GpyTask::joints_samplesCallback(const base::Time &ts, const ::base::samples::Joints &joints_samples_sample)
{
    throw std::runtime_error("Aggregator callback for joints_samples not implemented");
}

void GpyTask::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    throw std::runtime_error("Aggregator callback for orientation_samples not implemented");
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See GpyTask.hpp for more detailed
// documentation about them.

bool GpyTask::configureHook()
{
    if (! GpyTaskBase::configureHook())
        return false;
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
}
void GpyTask::cleanupHook()
{
    GpyTaskBase::cleanupHook();
}
