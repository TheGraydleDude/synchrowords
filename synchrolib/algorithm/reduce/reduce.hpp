#pragma once
#include <algorithm>
#include <synchrolib/algorithm/algorithm.hpp>
#include <synchrolib/data_structures/automaton.hpp>
#include <synchrolib/data_structures/preprocessed_transition.hpp>
#include <synchrolib/data_structures/subset.hpp>
#include <synchrolib/utils/vector.hpp>
#include <synchrolib/data_structures/subsets_implicit_trie.hpp>
#include <synchrolib/utils/connectivity.hpp>
#include <synchrolib/utils/distribution.hpp>
#include <synchrolib/utils/general.hpp>
#include <synchrolib/utils/logger.hpp>
#include <cassert>
#include <cmath>
#include <cstring>
#include <exception>
#include <map>
#include <numeric>
#include <optional>
#include <utility>

#ifndef __INTELLISENSE__
$REDUCE_DEF$
#endif

namespace synchrolib {

template <uint N, uint K>
class Reduce : public Algorithm<N, K> {
public:
  void run(AlgoData<N, K>& data) override {
    auto log_name = Logger::set_log_name("reduce");

    if (data.result.reduce && data.result.reduce->done) {
      Logger::error()
          << "Algorithm is not compatible with inputs after reduction";
      return;
    }

    if (data.result.non_synchro) {
      return;
    }

    if (data.result.mlsw_lower_bound == data.result.mlsw_upper_bound) {
      return;
    }

    if (N < MIN_N) {
      return;
    }

    mlsw = 0;
    max_mlsw = data.result.mlsw_upper_bound - 1;

    singletons.resize(N);
    for (uint i = 0; i < N; i++) {
      singletons[i] = Subset<N>::Singleton(i);
    }

    list_bfs.push_back(Subset<N>::Complete());

    bool found = process_short_bfs(data.aut);
    if (!found && mlsw == max_mlsw) {
      mlsw++;
      found = true;
    }

    if (found) {
      // Logger::info() << "mlsw: " << mlsw;
      assert(data.result.mlsw_lower_bound <= mlsw && mlsw <= data.result.mlsw_upper_bound);
      data.result.mlsw_lower_bound = data.result.mlsw_upper_bound = mlsw;
      return;
    }

    if (!try_to_reduce(data.aut)) {
      Logger::info() << "Skipping reduction";
      return;
    }

    Logger::info() << "Reduced to N = " << aut_reduced.N << " in " << mlsw
                   << " bfs steps";

    if (data.result.mlsw_lower_bound > mlsw) {
      data.result.mlsw_lower_bound -= mlsw;
    } else {
      data.result.mlsw_lower_bound = 0;
    }
    assert(mlsw < data.result.mlsw_upper_bound);
    data.result.mlsw_upper_bound -= mlsw;

    data.result.reduce = ReduceData();
    data.result.reduce->aut = aut_reduced;
    data.result.reduce->list_bfs = to_vector(list_bfs);
    data.result.reduce->bfs_steps = mlsw;
    data.result.reduce->done = false;
  }

private:
  uint64 mlsw;
  uint64 max_mlsw;

  VarAutomaton aut_reduced;
  FastVector<Subset<N>> list_bfs;
  FastVector<Subset<N>> list_bfs_visited;

  FastVector<Subset<N>> singletons;

  bool process_short_bfs(const Automaton<N, K>& aut) {
    while (mlsw < max_mlsw) {
      if (list_bfs.size() > LIST_SIZE_THRESHOLD) { // TODO: maybe change to list_bfs_visited
        return false;
      }

      mlsw++;
      if (bfs_step(aut)) {
        return true;
      }
    }

    return false;
  }

  bool try_to_reduce(const Automaton<N, K>& aut) {
    auto reachable = get_reachable_states(aut);
    uint rn = 0;
    for (uint i = 0; i < N; ++i) {
      if (reachable[i]) {
        rn++;
      }
    }

    if (N == rn || !rn) {
      return false;
    }

    FastVector<uint> map(N);
    uint num = 0;
    for (uint n = 0; n < N; n++) {
      if (reachable[n]) {
        map[n] = num++;
      }
    }

    aut_reduced = VarAutomaton(rn, K);
    for (uint n = 0; n < N; n++) {
      if (reachable[n]) {
        for (uint k = 0; k < K; k++) {
          aut_reduced[map[n]][k] = map[aut[n][k]];
        }
      }
    }

    for (uint i = 0; i < list_bfs.size(); i++) {
      auto s = Subset<N>::Empty();
      for (uint n = 0; n < N; n++) {
        if (list_bfs[i].is_set(n)) {
          s.set(map[n]);
        }
      }
      list_bfs[i] = s;
    }

    return true;
  }

  FastVector<bool> get_reachable_states(const Automaton<N, K>& aut) {
    FastVector<bool> ret(N);
    std::stack<uint> stack;
    for (const auto& sub : list_bfs) {
      for (uint n = 0; n < N; n++) {
        if (sub.is_set(n) && !ret[n]) {
          ret[n] = true;
          stack.push(n);
        }
      }
    }

    while (!stack.empty()) {
      auto n = stack.top();
      stack.pop();

      for (uint k = 0; k < K; k++) {
        uint p = aut[n][k];
        if (!ret[p]) {
          ret[p] = true;
          stack.push(p);
        }
      }
    }

    return ret;
  }

  bool bfs_step(const Automaton<N, K>& aut) {
    const uint size = list_bfs.size() * K;
    FastVector<Subset<N>> list_next(size);

    for (uint i = 0; i < list_bfs.size(); i++) {
      for (uint k = 0; k < K; k++) {
        list_bfs[i].template apply<N, K>(aut, k, list_next[i * K + k]);
      }
    }

    list_bfs = std::move(list_next);

    SubsetsImplicitTrie<N, false, THREADS>::reduce(list_bfs_visited, list_bfs);
    SubsetsImplicitTrie<N, true, THREADS>::reduce(list_bfs);

    list_bfs_visited.reserve(list_bfs_visited.size() + list_bfs.size());
    for (const auto& sub : list_bfs) {
      list_bfs_visited.push_back(sub);
    }

    for (const auto& sub : list_bfs) {
      if (sub.size() == 1)
        return true;
    }
    return false;
    //auto ret = SubsetsImplicitTrie<N, false, THREADS>::check_contains_subset(list_bfs, singletons);
    //return std::any_of(ret.begin(), ret.end(), [](bool x) { return x; });
  }

  static FastVector<FastVector<bool>> to_vector(
      const FastVector<Subset<N>>& subs) {
    FastVector<FastVector<bool>> ret(subs.size(), FastVector<bool>(N));
    for (size_t i = 0; i < subs.size(); ++i) {
      for (uint n = 0; n < N; ++n) {
        ret[i][n] = subs[i].is_set(n);
      }
    }
    return ret;
  }
};

}  // namespace synchrolib

#ifndef __INTELLISENSE__
$REDUCE_UNDEF$
#endif
