#pragma once

#include <algorithm>

namespace pr {

template <typename T>
T clamp(const T& t, const T& tmin, const T& tmax) {
  return std::max<T>(std::min<T>(t, tmax), tmin);
}

}
