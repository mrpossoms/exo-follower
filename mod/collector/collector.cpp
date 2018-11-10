#include <exo/exo.hpp>

//#ifdef __unix__
#include <exo/unix.hpp>

#include <sdf/sdf.hh>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
//#endif

#include "msg/sensor.h"

using namespace exo;
using namespace gazebo;

struct GAZEBO_VISIBLE collector : public exo::Mod,
                                  public gazebo::ModelPlugin
{
    event::ConnectionPtr update_connection;
    physics::ModelPtr model;

    // physics::JointPtr wheels[2];
    // double wheel_separation;

    std::mutex mutex;
    event::ConnectionPtr new_depth_frame_conn;
    rendering::DepthCameraPtr depth_camera;
    float *depth_buffer = NULL;

    gazebo::msgs::Image image_msg;

    // unix::Net::In inlet;
    unix::Net::Out outlet;

    collector() : exo::Mod("MOD_NAME"),
                  // inlet(1337),
                  outlet("localhost", 1337)
    {
        // TODO
    }

    ~collector()
    {
        // TODO
    }

    bool msg_compatible(exo::msg::Hdr& h)
    {
        // TODO

        return false;
    }

    exo::Result msg_received(exo::msg::Hdr& h, exo::msg::Inlet& in)
    {
        // TODO
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

        update_connection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&collector::update, this));

        find_sensor(model);
    }

    // Documentation Inherited.
    virtual void Init()
    {

    }

    // Documentation Unherited.
    virtual void Reset()
    {
        std::lock_guard<std::mutex> lock(mutex);
        image_msg.Clear();
    }

    bool find_sensor(const physics::ModelPtr &_model)
    {
        // loop through links to find depth sensor
        for (const auto l : _model->GetLinks())
        {
            for (unsigned int i = 0; i < l->GetSensorCount(); ++i)
            {
                std::string sensorName = l->GetSensorName(i);
                sensors::SensorPtr sensor = sensors::get_sensor(sensorName);
                if (!sensor)
                    continue;

                if (sensor->Type() == "depth")
                {
                    sensors::DepthCameraSensorPtr depthSensor =
                    std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);
                    if (depthSensor)
                    {
                        rendering::DepthCameraPtr camera = depthSensor->DepthCamera();
                        if (camera)
                        {
                            depth_camera = camera;
                            new_depth_frame_conn = depth_camera->ConnectNewDepthFrame(
                                std::bind(&collector::on_new_depth_frame, this,
                                std::placeholders::_1, std::placeholders::_2,
                                std::placeholders::_3, std::placeholders::_4,
                                std::placeholders::_5)
                            );
                            return true;
                        }
                    }
                }
            }
        }

        // recursively look for sensor in nested models
        for (const auto &m : _model->NestedModels())
        {
            if (this->find_sensor(m))
                return true;
        }

      return false;
    }

    void on_new_depth_frame(const float *_image,
        const unsigned int _width, const unsigned int _height,
        const unsigned int /*_depth*/, const std::string &/*_format*/)
    {
        std::lock_guard<std::mutex> lock(mutex);

        unsigned int depth_buffer_size = _width * _height * sizeof(float);
        if (_width != image_msg.width() || _height != image_msg.height())
        {
            if (depth_buffer != NULL)
                delete [] depth_buffer;
            depth_buffer = new float[depth_buffer_size];
            image_msg.set_width(_width);
            image_msg.set_height(_height);
        }

        // copy the depth buffer into the sensors message
        exo::msgs::Sensors sensors;
        memcpy(sensors.depth, _image, depth_buffer_size);

        // create the header, pack the payload
        msg::Payload<sizeof(exo::msgs::Sensors) + sizeof(exo::msg::Hdr)> payload;
        payload << exo::msgs::Sensors::hdr();
        payload.put<exo::msgs::Sensors>(sensors);

        // stuff it into the outlet
        if ((outlet << payload.buffer()) == Result::OK)
        {
            // outlet << payload.buffer();
            // gzdbg << "Wrote frame: " << std::to_string(_width) << "x" << std::to_string(_height) << std::endl;
        }
    }
};

GZ_REGISTER_MODEL_PLUGIN(collector)
