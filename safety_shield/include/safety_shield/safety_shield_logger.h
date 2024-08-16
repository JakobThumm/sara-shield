// safety_shield_logger.h
#ifndef SAFETY_SHIELD_LOGGER_H
#define SAFETY_SHIELD_LOGGER_H

#include <string>


#ifdef ENABLE_LOGGING
#include <spdlog/spdlog.h>
#endif

namespace safety_shield_logger {

#ifdef ENABLE_LOGGING

template <typename... Args>
inline void info(const std::string& format, Args&&... args) {
    spdlog::info(format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void debug(const std::string& format, Args&&... args) {
    spdlog::debug(format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void warn(const std::string& format, Args&&... args) {
    spdlog::warn(format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void error(const std::string& format, Args&&... args) {
    spdlog::error(format, std::forward<Args>(args)...);
}

#else

template <typename... Args>
inline void info(const std::string& format, Args&&... args) {
    // Do nothing
}

template <typename... Args>
inline void debug(const std::string& format, Args&&... args) {
    // Do nothing
}

template <typename... Args>
inline void warn(const std::string& format, Args&&... args) {
    // Do nothing
}

template <typename... Args>
inline void error(const std::string& format, Args&&... args) {
    // Do nothing
}

#endif // ENABLE_LOGGING

} // namespace safety_shield_logger

#endif // SAFETY_SHIELD_LOGGER_H
