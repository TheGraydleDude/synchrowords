#pragma once
#include <algorithm>
#include <synchrolib/algorithm/algorithm.hpp>
#include <synchrolib/data_structures/automaton.hpp>
#include <synchrolib/data_structures/subset.hpp>
#include <synchrolib/utils/connectivity.hpp>
#include <synchrolib/utils/vector.hpp>
#include <synchrolib/utils/general.hpp>
#include <synchrolib/utils/logger.hpp>
#include <cassert>
#include <cmath>
#include <cstring>
#include <numeric>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

#ifndef __INTELLISENSE__
$BRUTE_DEF$
#endif

namespace synchrolib {

template <uint N, uint K>
class Brute : public Algorithm<N, K> {
public:
  void run(AlgoData<N, K>& data) override {
    auto log_name = Logger::set_log_name("brute");

    if (data.result.non_synchro) {
      return;
    }

    if (data.result.mlsw_lower_bound == data.result.mlsw_upper_bound) {
      return;
    }

    if (data.result.reduce && data.result.reduce->done) {
      Logger::error()
          << "Algorithm is not compatible with inputs after reduction";
      return;
    }

    if (MAX_N > 32) {
      Logger::error() << "max_n can not be greater than 32, exiting...";
      return;
    }

    if (N > MAX_N) {
      Logger::info() << "N > " << MAX_N << ", exiting...";
      return;
    }

    std::optional<uint> mlsw;
    if (N == 1) {
      mlsw = 0;
    } else {
      mlsw = bfs(data.aut);
    }

    if (!mlsw) {
      data.result.non_synchro = true;
      Logger::warning() << "Automaton is non-synchronizing";
      return;
    }
    assert(*mlsw <= data.result.mlsw_upper_bound);
    assert(*mlsw >= data.result.mlsw_lower_bound);

    data.result.mlsw_upper_bound = data.result.mlsw_lower_bound = *mlsw;
    // Logger::info() << "mlsw: " << *mlsw;
  }

private:
  uint apply(const Automaton<N, K>& aut, const uint k, uint mask) {
    uint ret = 0;
    uint n = 0;
    while (mask) {
      int shift = __builtin_ctz(mask);
      mask >>= shift;
      n += shift;
      ret |= (1 << aut[n][k]);
      mask ^= 1;
    }
    return ret;
  }

  std::optional<uint> bfs(const Automaton<N, K>& aut) {
    uint full = 0;
    for (uint i = 0; i < N; ++i) {
      full |= (static_cast<uint>(1) << i);
    }

    const uint8l INF = std::numeric_limits<uint8l>::max();

    uint shift = N;  // prevents -Wshift-count-overflow warnings
    FastVector<uint8l> dist(static_cast<size_t>(1) << shift, INF);
    dist[full] = 0;

    std::queue<uint> queue;
    queue.push(full);

    while (!queue.empty()) {
      uint mask = queue.front();
      queue.pop();

      for (uint k = 0; k < K; ++k) {
        uint v = apply(aut, k, mask);

        if (dist[v] == INF) {
          dist[v] = dist[mask] + 1;
          queue.push(v);

          if (__builtin_popcount(v) == 1) {
            return dist[v];
          }
        }
      }
    }

    return std::nullopt;
  }
};

}  // namespace synchrolib

#ifndef __INTELLISENSE__
$BRUTE_UNDEF$
#endif
