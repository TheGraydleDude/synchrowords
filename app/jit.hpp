#pragma once
#include <synchrolib/synchrolib.hpp>
#include <jitlib/jitlib.hpp>
#include <app/io.hpp>
#include <external/json.hpp>
#include <algorithm>
#include <filesystem>
#include <optional>
#include <string>


class Jit {
public:
  using Path = std::filesystem::path;
  using AlgoResult = synchrolib::AlgoResult;

  static bool run(const IO::json& config, uint n, uint k, const std::string& aut, const std::string& build_suffix, AlgoResult& result) {
    try {
      if (config["algorithms"].empty()) {
        Logger::error() << "Need at least one algorithm";
        std::exit(4);
      }
      auto algorithms = get_algorithm_names(config);

      auto config_hash = std::to_string(std::hash<std::string>{}(config.dump()));
      auto build_name = "synchrolib_" + std::to_string(n) + "_" + std::to_string(k) + "_" + config_hash;
      if (!build_suffix.empty()) {
        build_name += "_" + build_suffix;
      }
      Path libroot = Path("build") / build_name;

      try {
        Timer jitlib_timer("jit");

        JitLib jitlib;
        if (std::filesystem::is_directory(libroot)) {
          // Logger::info() << "Loading precompiled library";
          jitlib.set_dir_path(libroot);
          jitlib.load("libsynchro.so");
        } else {
          Logger::info() << "Recompiling for N = " << n << ", K = " << k;
          std::filesystem::create_directory(libroot);
          auto subst_map = get_subst_map(config, n, k);
          jitlib
            .substitute(std::vector<std::pair<Path, Path>>{
                {"synchrolib", "synchrolib"},
                {"external", "external"},
                {"jit/makefile", "makefile"},
                {"jit/makefile_jit", "makefile_jit"},
                {"jit/makefile_jit_gpu", "makefile_jit_gpu"},
                {"jit/jitmain.cpp", "jitmain.cpp"},
                {"jit/jitdefines.hpp", "jitdefines.hpp"}
              }, libroot, subst_map)
            .compile(config.value("gpu", false) ? "jit_gpu" : "jit", Logger::get_log_level() >= Logger::LogLevel::VERBOSE)
            .load("libsynchro.so");
        }
        jitlib_timer.stop();

        jitlib.run<const std::string&, const std::vector<std::string>&, AlgoResult&, Logger::LogLevel>("run", aut, algorithms, result, Logger::get_log_level());

      } catch (JitLib::JitLibException& ex) {
        Logger::error() << "JitLibException: " << ex.what();
        std::filesystem::remove_all(libroot);
        std::exit(4);
      } catch (nlohmann::detail::exception& json_error) {
        Logger::error() << "Config exception: " << json_error.what();
        std::filesystem::remove_all(libroot);
        std::exit(4);
      } catch (std::exception& err) {
        Logger::error() << err.what();
        std::filesystem::remove_all(libroot);
        std::exit(4);
      } catch (...) {
        Logger::error() << "Unknown exception";
        std::filesystem::remove_all(libroot);
        std::exit(4);
      }

      return result.algorithms_run.size() == algorithms.size();
    } catch (nlohmann::detail::exception& json_error) {
      Logger::error() << "Config exception: " << json_error.what();
      std::exit(4);
    }
  }

private:
  using Logger = synchrolib::Logger;
  using Timer = synchrolib::Timer;

  class JitLibLogger : public Logger::LoggerStream {
  public:
    JitLibLogger() : Logger::LoggerStream(Logger::LogLevel::VERBOSE, "jitlib") {}
  };

  using JitLib = jitlib::JitLib<JitLibLogger>;

  static std::unordered_map<std::string, std::string> get_subst_map(const IO::json& config, uint n, uint k) {
    std::unordered_map<std::string, std::string> subst_map;
    for (auto& algo : config["algorithms"]) {
      auto name = algo["name"].get<std::string>();
      auto algo_config = synchrolib::make_algo_config(name);
      for (auto pr : algo_config->get_substs(algo["config"])) {
        subst_map.insert(pr);
      }
    }

    subst_map["$DEFINES$"] = get_global_defines(config, n, k);

    return subst_map;
  }

  static std::string get_global_defines(
      const IO::json& config, uint n, uint k) {
    std::string defines = synchrolib::make_define("AUT_N", std::to_string(n)) +
        synchrolib::make_define("AUT_K", std::to_string(k));

    defines += synchrolib::make_define(
        "UPPER_BOUND", synchrolib::get_str_int(config, "upper_bound", DEFAULT_UPPER_BOUND));

    defines += synchrolib::make_define("GPU", config.value("gpu", false));
    defines += synchrolib::make_define("GPU_MEMORY", synchrolib::get_str_int(config, "gpu_max_memory_mb", "1024 * 2"));

    int64_t threads = config.value("threads", 1);
    if (threads < 1 || threads > 64) {
      throw std::runtime_error("threads must belong to range [1, 64]");
    }
    defines += synchrolib::make_define("THREADS", threads);
    
    defines += synchrolib::make_define("JIT");

    auto algorithms = get_algorithm_names(config);
    std::sort(algorithms.begin(), algorithms.end());
    algorithms.resize(std::distance(algorithms.begin(), std::unique(algorithms.begin(), algorithms.end())));

    for (auto name : algorithms) {
      std::transform(name.begin(), name.end(), name.begin(), ::toupper);
      defines += synchrolib::make_define("COMPILE_" + name);
    }

    return defines;
  }

  static std::vector<std::string> get_algorithm_names(const IO::json& config) {
    std::vector<std::string> names;
    for (auto& algo : config["algorithms"]) {
      auto name = algo["name"].get<std::string>();
      names.push_back(name);
    }
    return names;
  }

  static inline const std::string DEFAULT_UPPER_BOUND = "1ULL * AUT_N * AUT_N * AUT_N / 6";
};
