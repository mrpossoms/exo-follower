#pragma once

namespace exo
{
    namespace msgs
    {
        struct Motors : public msg::Msg
        {
            int8_t speeds[2];

            static msg::Hdr hdr()
            {
                return { 2, MOTORS_MAGIC, sizeof(Motors) };
            }
        };
    }
}
