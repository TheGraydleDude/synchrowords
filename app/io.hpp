#pragma once
#include <synchrolib/utils/logger.hpp>
#include <synchrolib/algorithm/algorithm.hpp>
#include <external/json.hpp>
#include <filesystem>
#include <fstream>
#include <optional>
#include <cctype>
#include <algorithm>
#include <sstream>
#include <string>


class IO {
public:
  using json = nlohmann::json;
  using AlgoResult = synchrolib::AlgoResult;
  using Path = std::filesystem::path;

  static json read_config(Path path) {
    std::ifstream ss(path);
    json ret;
    try {
      ss >> ret;
    } catch (nlohmann::detail::exception& ex) {
      Logger::error() << ex.what();
      std::exit(2);
    }

    Logger::verbose() << "Config:\n" << ret.dump(4);
    return ret;
  }

  struct EncodedAutomaton {
    uint N, K;
    std::string str;

    void validate() {
      std::stringstream ss(str);
      for (uint i = 0; i < N * K; ++i) {
        uint cur;
        if (!(ss >> cur)) {
          Logger::error() << "Expected " << N * K << " integers, found " << i;
          std::exit(3);
        }
        if (cur >= N) {
          Logger::error() << "Expected integer in range [0, " << N - 1 << "], found " << cur;
          std::exit(3);
        }
      }
    }
  };

  /// Generate canonical DFAs (BFS-canonical) with n states and alphabet size k.
/// Returns a vector<EncodedAutomaton>.
 static std::vector<EncodedAutomaton> generate_automata(uint n, uint k)
{
  if (n == 0 || k == 0) {
    Logger::error() << "n and k must be > 0";
    std::exit(3);
  }

  // transitions[state][symbol], -1 = unset
  std::vector<std::vector<int>> transitions(n, std::vector<int>(k, -1));

  std::vector<EncodedAutomaton> result;
  result.reserve(1024);

  std::uint64_t counter = 0;

  auto serialize_current = [&]() -> std::string {
    std::string s;
    s.reserve(static_cast<size_t>(n) * k * 2);
    for (uint st = 0; st < n; ++st) {
      for (uint sym = 0; sym < k; ++sym) {
        if (!s.empty()) s.push_back(' ');
        s += std::to_string(transitions[st][sym]);
      }
    }
    return s;
  };

  auto push_current = [&]() {
    EncodedAutomaton a;
    a.N = n;
    a.K = k;
    a.str = serialize_current();
    a.validate(); // will exit(3) on error like your original code
    result.push_back(std::move(a));
    ++counter;
  };

  // recursive generator: set_state_symbol_rec(state_idx, sym_idx, seen)
  std::function<void(uint, uint, uint)> rec;
  rec = [&](uint state_idx, uint sym_idx, uint seen) -> void {
    // finished all states
    if (state_idx == n) {
      if (seen == n) push_current();
      return;
    }

    // move to next state if we've done all symbols
    if (sym_idx == k) {
      rec(state_idx + 1, 0, seen);
      return;
    }

    // if first state, make it a sink state (all transitions to 0)
    if (state_idx == 0) {
      for (uint s = 0; s < k; ++s) transitions[0][s] = 0;
      // Python used seen+1 here
      rec(1, 0, seen + 1);
      return;
    } else {
      // check whether we already have a downward transition (to a lower-numbered state)
      bool has_trans_going_down = false;
      for (uint s = 0; s < sym_idx; ++s) {
        int val = transitions[state_idx][s];
        if (val != -1 && static_cast<uint>(val) < state_idx) {
          has_trans_going_down = true;
          break;
        }
      }

      // max target is either the next unseen state (seen) or n-1
      uint max_new_target = std::min(seen, n - 1u);

      // if there's no downward transition yet and this is the last symbol, force downward max
      if (!has_trans_going_down && sym_idx == k - 1) {
        // ensure state_idx > 0 here
        max_new_target = state_idx - 1;
      }

      for (uint target = 0; target <= max_new_target; ++target) {
        uint introduced = (target == seen && seen < n) ? 1u : 0u;
        transitions[state_idx][sym_idx] = static_cast<int>(target);
        uint new_seen = seen + introduced;
        rec(state_idx, sym_idx + 1, new_seen);
      }

      // reset as Python does (None / -1)
      transitions[state_idx][sym_idx] = -1;
    }
  };

  auto t0 = std::chrono::high_resolution_clock::now();
  // start with state 0 discovered (seen = 1)
  rec(0, 0, 1);
  auto t1 = std::chrono::high_resolution_clock::now();

  double elapsed = std::chrono::duration<double>(t1 - t0).count();
  Logger::info() << "Read " << result.size() << " automata";
  Logger::info() << "Total enumerated (canonical under BFS): " << counter;
  Logger::info() << "Total runtime: " << std::fixed << std::setprecision(6) << elapsed << " seconds";

  return result;
}

  static void set_output(std::ofstream stream) {
    output = std::move(stream);
  }

  static size_t count_nonempty_lines(std::ifstream& stream) {
    size_t ret = 0;
    std::string s;
    while (std::getline(stream, s)) {
      if (!std::all_of(s.begin(), s.end(), ::isspace)) {
        ret++;
      }
    }
    return ret;
  }

  static void push_result(const AlgoResult& result, size_t index) {
    if (!output) {
      if (result.word) {
        Logger::info() << "Found synchronizing word of length "
                        << result.word->size()
                        << " (use the -o flag to save it)";
      }
      return;
    }

    *output << index << ": ";

    if (result.non_synchro) {
      *output << "NON SYNCHRO\n";
      output->flush();
      return;
    }

    *output << "[" << result.mlsw_lower_bound << ", "
            << result.mlsw_upper_bound << "]";

    *output << " (";
    bool first = true;
    for (const auto& [name, time] : result.algorithms_run) {
      if (!first) {
        *output << ", ";
      }
      *output << "(" << name << ", " << time << ")";
      first = false;
    }
    *output << ")";

    if (result.word) {
      Logger::info() << "Saving synchronizing word of length "
                      << result.word->size();

      *output << " {";
      for (size_t i = 0; i < result.word->size(); ++i) {
        if (i != 0) {
          *output << " ";
        }
        *output << (*result.word)[i];
      }
      *output << "}";
    }

    *output << "\n";
    output->flush();
  }

private:
  using Logger = synchrolib::Logger;

  inline static std::optional<std::ofstream> output;

  IO() {}
};
