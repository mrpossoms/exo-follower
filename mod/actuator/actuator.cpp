#include <exo/exo.hpp>

#if defined(__APPLE__) || defined(__linux__)
#include <exo/unix.hpp>

#include <sdf/sdf.hh>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#endif

#include "msg/motors.h"

using namespace exo;
using namespace gazebo;

struct GAZEBO_VISIBLE actuator : public exo::Mod,
                                  public gazebo::ModelPlugin
{
    event::ConnectionPtr update_connection;
    physics::ModelPtr model;

    exo::msgs::Motors motors;
    physics::JointPtr wheels[2];

    std::mutex mutex;
    std::thread* inlet_thread;

    unix::Net::In inlet;

    actuator() : exo::Mod("MOD_NAME"),
                  inlet(3000)
    {
        motors.speeds[0] = 0;
        motors.speeds[1] = 0;
    }

    ~actuator()
    {
        // TODO
    }

    bool msg_compatible(exo::msg::Hdr& h)
    {
        return h == exo::msgs::Motors::hdr();
    }


    exo::Result msg_received(exo::msg::Hdr& h, exo::msg::Inlet& in)
    {
        std::lock_guard<std::mutex> lock(mutex);

        if (h == exo::msgs::Motors::hdr())
        {
            exo::msg::Payload<sizeof(motors)> payload(in);

            payload.get<exo::msgs::Motors>(motors);
       }

        return exo::Result::OK;
    }


    exo::Result enter(exo::Context ctx)
    {
        // TODO
        return exo::Result::OK;
    }


    exo::Result update()
    {
        std::lock_guard<std::mutex> lock(mutex);

#ifndef __DEBUG__
        wheels[0]->SetVelocity(0, motors.speeds[0] / 64.f);
        wheels[1]->SetVelocity(0, motors.speeds[1] / 64.f);
#else
        exo::Log::info(1, std::to_string(motors.speeds[0]) + ":" + std::to_string(motors.speeds[1]));
#endif

        return exo::Result::OK;
    }

    exo::Result exit()
    {
        // TODO
        return exo::Result::OK;
    }

    // Documentation Inherited.
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        model = _model;

        exo::Log::instance(new exo::unix::Log::Stderr(0), 0);
        exo::Log::info(1, "Log up");

        update_connection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&actuator::update, this));

        // find_sensor(model);

        { // find wheel joints
            auto joints = model->GetJoints();
            if (joints.size() < 2u)
                return;

            physics::Joint_V rev_joints;
            for (const auto &j : joints)
            {
                if (j->GetMsgType() == gazebo::msgs::Joint::REVOLUTE)
                {
                    rev_joints.push_back(j);
                }
            }

            if (rev_joints.size() < 2u)
                return;

            wheels[0] = rev_joints[0];
            wheels[1] = rev_joints[1];
        }

        inlet_thread = new std::thread([&]{
            while(true)
            {
                exo::msg::Hdr hdr;
                auto res = inlet >> hdr;

                if (res == exo::Result::OK)
                {
                    // we've gotten a message, does the module
                    // understand this message type?
                    if (msg_compatible(hdr))
                    {
                        // it does, process it
                        msg_received(hdr, inlet);
                    }
                    else
                    {
                        // it doesn't, discard the payload.
                        inlet.flush(hdr.payload_length);
                    }
                }
            }
        });
    }

    // Documentation Inherited.
    virtual void Init()
    {

    }

    // Documentation Unherited.
    virtual void Reset()
    {
        std::lock_guard<std::mutex> lock(mutex);
    }

};

GZ_REGISTER_MODEL_PLUGIN(actuator)
