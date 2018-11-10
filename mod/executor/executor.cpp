#include <exo/exo.hpp>

#if defined(__APPLE__) || defined(__linux__)
#include <exo/unix.hpp>
#endif

#include "msg/sensor.h"
#include "msg/motors.h"

struct executor : public exo::Mod
{
    exo::msgs::Motors motors;
    exo::msgs::Sensors last_sensor_readings;

    exo::msg::Inlet& inlet;
    exo::msg::Outlet& outlet;

    executor(exo::msg::Inlet& inlet, exo::msg::Outlet& outlet) :
        exo::Mod("MOD_NAME"),
        inlet(inlet),
        outlet(outlet)
    {

    }

    ~executor()
    {
        // TODO
    }

    bool msg_compatible(exo::msg::Hdr& h)
    {
        return h == exo::msgs::Sensors::hdr();
    }

    exo::Result msg_received(exo::msg::Hdr& h, exo::msg::Inlet& in)
    {
        if (h == exo::msgs::Sensors::hdr())
        {
            exo::msg::Payload<sizeof(exo::msgs::Sensors)> payload(in);
            payload.get<exo::msgs::Sensors>(last_sensor_readings);
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
        float* row = last_sensor_readings.depth + (128 * 64);
        int idx = 64;
        float idx_min = row[idx];

        for (auto i = 128; i--;)
        {
            if (row[i] < idx_min && row[i] > 1.5)
            {
                idx = i;
                idx_min = row[i];
            }
        }

        exo::Log::info(1, "idx_min:" + std::to_string(idx_min) + " idx:" + std::to_string(idx));

        if (idx_min > 1.5)
        {
            float p = idx / 128.f;
            exo::Log::info(1, "p:" + std::to_string(p));
            motors.speeds[0] = (p) * -128;
            motors.speeds[1] = (1 - p) * -128;
        }
        else
        {
           motors.speeds[0] = motors.speeds[1] = 0;
        }

        exo::Log::info(1, std::to_string(motors.speeds[0]) + ":" + std::to_string(motors.speeds[1]) + "\n");

        auto hdr = exo::msgs::Motors::hdr();
        exo::msg::Payload<sizeof(motors) + sizeof(hdr)> payload;
        payload.put<exo::msg::Hdr>(hdr);
        payload.put<exo::msgs::Motors>(motors);
        if ((outlet << payload.buffer()) == exo::Result::OK)
        {
            exo::Log::info(1, "wrote motor commands");
        }

        return exo::Result::OK;
    }

    exo::Result exit()
    {
        // TODO
        return exo::Result::OK;
    }
};




#if defined(__APPLE__) || defined(__linux__)

int main(int argc, char* argv[])
{
    // Inlet and outlet setup
    exo::unix::Net::In  inlet(1337);
    exo::unix::Net::Out outlet("localhost", 3000);

    executor mod(inlet, outlet);
    exo::Result res;
    int log_verbosity = 0;

    // Log level handling
    exo::unix::CLI::parser(argc, argv)
    .optional<int>("-v", [&](int verbosity) {
        log_verbosity = verbosity;
    });

    // instantiate stderr logger
    exo::Log::instance(new exo::unix::Log::Stderr(log_verbosity), log_verbosity);

    res = mod.enter({ argc, argv });

    if (res != exo::Result::OK)
    {
        exo::Log::error(0, "executor: enter() failed");
        return 1;
    }

    exo::Log::good(2, "Module running");

    // process messages and perform updates until
    // mod.update() no longer returns exo::Result::OK
    do
    {
        exo::msg::Hdr hdr;
        auto res = inlet >> hdr;

        if (res == exo::Result::OK)
        {
            // we've gotten a message, does the module
            // understand this message type?
            if (mod.msg_compatible(hdr))
            {
                // it does, process it
                mod.msg_received(hdr, inlet);
            }
            else
            {
                // it doesn't, discard the payload.
                inlet.flush(hdr.payload_length);
            }
        }

    } while (mod.update() == exo::Result::OK);

    // tear down
    mod.exit();

    // clean up the logger
    auto li = exo::Log::instance();
    if (li != nullptr) delete li;

    return 0;
}
#endif
