#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL1_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL1_H

#include "../utils/type_conversion.h"
#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../mechanics/compute_trajectory.h"
#include "../bits/solver_types.h"


namespace dte3607::physengine::solver_dev::level1
{
  template <typename Data_T, typename Params_T>
  void computeCache(Data_T& data, Params_T const& params)
  {
    auto const proc_kernel = [&params](auto& data) {
      auto const& [F, timestep]       = params;
      auto& [pos, vel, out_a, out_ds] = data;

      auto [ds, a] = mechanics::computeLinearTrajectory(vel, F, timestep);

      out_ds = ds;
      out_a  = a;

      // Move to different process
      pos += out_ds;
      vel += out_a;
    };
    std::ranges::for_each(data, proc_kernel);
  }

  template <concepts::SolverFixture Fixture_T>
  void solve(Fixture_T& scenario, types::NanoSeconds timestep)
  {
    solver_types::Params params;
    params.F        = scenario.m_forces;
    params.timestep = timestep;
    computeCache(scenario.m_backend->m_cache_data, params);

    for (auto const& id : scenario.m_backend->m_rb_cache) {
      scenario.translateParent(
        id.second, scenario.m_backend->m_cache_data[id.first].out_ds);
      scenario.addAcceleration(
        id.second, scenario.m_backend->m_cache_data[id.first].out_a);
    }
  }

}   // namespace dte3607::physengine::solver_dev::level1


#endif   // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL1_H
