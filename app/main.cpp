#include <synchrolib/utils/logger.hpp>
#include <app/args.hpp>
#include <app/io.hpp>
#include <app/jit.hpp>
#include <string>


using Logger = synchrolib::Logger;

int main(int argc, char** argv) {
  auto args = CmdArgs::parse_arguments(argc, argv);

  if (args.verbose) {
    Logger::set_log_level(Logger::LogLevel::VERBOSE);
  } else if (args.quiet) {
    Logger::set_log_level(Logger::LogLevel::WARNING);
  } else if (args.debug) {
    Logger::set_log_level(Logger::LogLevel::DEBUG);
  } 

  // HERE: we generate the automata using algo!
  auto auts_encoded = IO::generate_automata(8,2);
  auto config = IO::read_config(args.config_path);

  size_t skip = 0;
  if (args.output_path) {
    std::ofstream os;
    if (args.cont) {
      std::ifstream is(*args.output_path);
      if (is) {
        skip = IO::count_nonempty_lines(is);
      }
      os = std::ofstream(*args.output_path, std::ios_base::app);
    } else {
      os = std::ofstream(*args.output_path);
    }

    if (!os) {
      Logger::error() << "Error while opening the output file";
      std::exit(1);
    }
    IO::set_output(std::move(os));
  }

  if (skip) {
    Logger::info() << "Skipping " << skip << " automata";
  }

  size_t index = 0;
  for (const auto& aut : auts_encoded) {
    if (index < skip) {
      index++;
      continue;
    }

    Jit::AlgoResult result;
    uint cur_n = aut.N;
    uint cur_k = aut.K;
    std::string cur_aut = aut.str;

    while (!Jit::run(config, cur_n, cur_k, cur_aut, args.build_suffix, result)) {
      if (result.reduce && !result.reduce->done) {
        cur_n = result.reduce->aut.N;
        cur_k = result.reduce->aut.K;
        cur_aut = result.reduce->aut.code();

        result.reduce->done = true;
      }
    }

    if (result.non_synchro) {
      Logger::info() << "NON SYNCHRO";
    } else {
      // Logger::info() << "Minimum synchronizing word length: ["
      //                 << result.mlsw_lower_bound << ", "
      //                 << result.mlsw_upper_bound << "]";
    }
    IO::push_result(result, index++);
  }
  IO::print_result();

  return 0;
}
