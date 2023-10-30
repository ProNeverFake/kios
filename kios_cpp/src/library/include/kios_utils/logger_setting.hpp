#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

namespace kios
{
    inline std::shared_ptr<spdlog::logger> set_logger(std::string tag, std::string verbosity = "debug")
    {
        spdlog::level::level_enum info_level;
        if (verbosity == "trace")
        {
            info_level = spdlog::level::trace;
        }
        else if (verbosity == "debug")
        {
            info_level = spdlog::level::debug;
        }
        else if (verbosity == "info")
        {
            info_level = spdlog::level::info;
        }
        else
        {
            info_level = spdlog::level::info;
        }

        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(info_level);
        console_sink->set_pattern("[kios][" + tag + "][%^%l%$] %v");

        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/kios_" + tag + ".txt", true);
        file_sink->set_level(spdlog::level::debug);

        auto logger = std::shared_ptr<spdlog::logger>(new spdlog::logger("kios", {console_sink, file_sink}));
        logger->set_level(info_level);
        // spdlog::set_default_logger(logger);
        logger->info("spdlog: " + tag + " logger has been initialized.");
        return logger;
    }
} // namespace kios