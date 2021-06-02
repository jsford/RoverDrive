#pragma once
#include <fmt/format.h>

namespace pr {

template <typename ...Params>
void log_debug(Params&&... params)
{
    fmt::print(std::forward<Params>(params)...);
}

template <typename ...Params>
void log_info(Params&&... params)
{
    fmt::print(std::forward<Params>(params)...);
}

template <typename ...Params>
void log_warn(Params&&... params)
{
    fmt::print(std::forward<Params>(params)...);
}

template <typename ...Params>
void log_error(Params&&... params)
{
    fmt::print(std::forward<Params>(params)...);
}

} // namespace pr
