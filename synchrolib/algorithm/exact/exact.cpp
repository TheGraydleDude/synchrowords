#include <jitdefines.hpp>
#ifdef COMPILE_EXACT

#include "exact.hpp"

#include <algorithm>
#include <synchrolib/algorithm/algorithm.hpp>
#include <synchrolib/algorithm/exact/utils.hpp>
#include <synchrolib/algorithm/exact/meet_in_the_middle.hpp>
#include <synchrolib/algorithm/exact/dfs.hpp>
#include <synchrolib/data_structures/automaton.hpp>
#include <synchrolib/data_structures/preprocessed_transition.hpp>
#include <synchrolib/data_structures/subset.hpp>
#include <synchrolib/utils/vector.hpp>
#include <synchrolib/data_structures/subsets_trie.hpp>
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
$EXACT_DEF$
#endif

namespace synchrolib {

template <uint N, uint K>
void Exact<N, K>::run(AlgoData<N, K>& data) {
  auto log_name = Logger::set_log_name("exact");

  if (DFS_SHORTCUT && !DFS) {
    Logger::error() << "dfs_shortcut: true depends on dfs: true";
    return;
  }

  if (data.result.non_synchro || data.result.mlsw_lower_bound == data.result.mlsw_upper_bound) {
    return;
  }

  if (N == 1) {
    // Logger::info() << "mlsw: " << 0;
    data.result.mlsw_upper_bound = data.result.mlsw_lower_bound = 0;
    return;
  }

  reset_threshold = 0;

  calculate_max_memory();

  initialize_bfs_lists(data);
  initialize_invbfs_lists(data);
  set_automaton_and_reorder(data);

  preprocess_transitions();

  bool found = run_meet_in_the_middle(data.result.mlsw_upper_bound - 1);

#if DFS
  if (!found) {
    if (run_dfs(data.result.mlsw_upper_bound - 1)) {
      reset_threshold++;
      found = true;
    }
  }
#endif

  if (!found) {
    reset_threshold++;
    if (reset_threshold == data.result.mlsw_upper_bound) {
      found = true;
    }
  }

  if (data.result.reduce && data.result.reduce->done) {
    reset_threshold += data.result.reduce->bfs_steps;
    data.result.mlsw_lower_bound += data.result.reduce->bfs_steps;
    data.result.mlsw_upper_bound += data.result.reduce->bfs_steps;
  }

  data.result.mlsw_lower_bound = std::max(data.result.mlsw_lower_bound, reset_threshold);
  if (found) {
    data.result.mlsw_upper_bound = reset_threshold;
  }

  // Logger::info() << (found ? "mlsw: " : "mlsw >= ") << reset_threshold;

  assert(data.result.mlsw_lower_bound <= data.result.mlsw_upper_bound);
}


template<uint N, uint K>
size_t Exact<N, K>::get_memory_usage() const {
  return synchrolib::get_memory_usage(list_bfs) +
      synchrolib::get_memory_usage(list_invbfs) +
      synchrolib::get_memory_usage(ptrans) +
      synchrolib::get_memory_usage(invptrans);
}

template<uint N, uint K>
FastVector<uint> Exact<N, K>::get_automaton_order(const Automaton<N, K>& aut, const InverseAutomaton<N, K>& invaut) {
  FastVector<double> pi(N), rpi(N);
  FastVector<bool> sink;
  get_automaton_reachable_states(
      aut, get_automaton_sink_component_vertex(aut), sink);
  VarAutomaton raut = get_automaton_reduced(aut, sink);
  get_automaton_stationary_distribution(raut, rpi);
  uint count = 0;
  for (uint n = 0; n < N; n++) {
    if (sink[n])
      pi[n] = rpi[count++];
    else {
      pi[n] = 1.0;
      for (uint k = 0; k < K; k++)
        pi[n] += invaut.end(n, k) - invaut.begin(n, k);
    }
  }
  return get_permutation_from_weights_ascending(pi);
}

template<uint N, uint K>
void Exact<N, K>::initialize_bfs_lists(const AlgoData<N, K>& data) {
  if (data.result.reduce && data.result.reduce->done) {
    list_bfs.reserve(data.result.reduce->list_bfs.size());

    for (auto vec : data.result.reduce->list_bfs) {
      auto sub = Subset<N>::Empty();
      for (uint i = 0; i < vec.size(); ++i) {
        if (vec[i]) {
          sub.set(i);
        }
      }
      list_bfs.push_back(sub);
    }
  } else {
    list_bfs.push_back(Subset<N>::Complete());
  }
}

template<uint N, uint K>
void Exact<N, K>::initialize_invbfs_lists(const AlgoData<N, K>& data) {
  FastVector<bool> sink;
  get_automaton_reachable_states(
      data.aut, get_automaton_sink_component_vertex<N, K>(data.aut), sink);

  for (uint n = 0; n < N; n++) {
    if (sink[n]) {
      for (uint k = 0; k < K; k++) {
        if (data.invaut.end(n, k) - data.invaut.begin(n, k) > 1) {
          list_invbfs.push_back(Subset<N>::Singleton(n));
          break;
        }
      }
    }
  }
}

template<uint N, uint K>
void Exact<N, K>::calculate_max_memory() {
  max_memory = static_cast<size_t>(MAX_MEMORY) * 1024 * 1024;
  if (max_memory < MEMORY_RESERVE) {
    Logger::error() << "Algorithm needs at least "
                    << get_megabytes(MEMORY_RESERVE) << " memory";
    return;
  }
  max_memory = max_memory - MEMORY_RESERVE;
}

template<uint N, uint K>
void Exact<N, K>::set_automaton_and_reorder(const AlgoData<N, K>& data) {
  auto order = get_automaton_order(data.aut, data.invaut);
  aut = Automaton<N, K>::permutation(data.aut, order);
  invaut = InverseAutomaton(aut);

  for (auto& sub : list_bfs) sub = Subset<N>::Permutation(sub, order);
  for (auto& sub : list_invbfs) sub = Subset<N>::Permutation(sub, order);
}

template<uint N, uint K>
void Exact<N, K>::preprocess_transitions() {
  for (uint k = 0; k < K; k++) {
    ptrans[k] = PreprocessedTransition<N, K>(aut, k);
    invptrans[k] = PreprocessedTransition<N, K>(invaut, k);
  }
}

template<uint N, uint K>
bool Exact<N, K>::run_meet_in_the_middle(uint64 max_reset_threshold) {
  MeetInTheMiddle<N, K> mitm(aut, invaut, ptrans, invptrans, reset_threshold, list_bfs, list_invbfs, max_reset_threshold, max_memory);
  return mitm.run();
}

template<uint N, uint K>
bool Exact<N, K>::run_dfs(uint64 max_reset_threshold) {
  Dfs<N, K> dfs(aut, invaut, ptrans, invptrans, reset_threshold, list_bfs, list_invbfs, max_reset_threshold, max_memory);
  return dfs.run();
}

template class Exact<AUT_N, AUT_K>;

}  // namespace synchrolib

#ifndef __INTELLISENSE__
$EXACT_UNDEF$
#endif

#endif
