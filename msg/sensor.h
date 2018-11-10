#pragma once

namespace exo
{
    namespace msgs
    {
        struct Sensors : public msg::Msg
        {
            float depth[128 * 128];

            static msg::Hdr hdr()
            {
                return { 1, SENSOR_MAGIC, sizeof(Sensors) };
            }
        };
    }
}
