#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H

#include "../utils/type_conversion.h"
#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../mechanics/compute_trajectory.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"
#include "../bits/solver_types.h"

namespace dte3607::physengine::solver_dev::level2
{
  /*
    template <typename Data_T, typename Params_T>
    void computeCache(Data_T& data, Params_T const& params)
    {
      auto const proc_kernel = [&params](auto& data) {
        auto const& [F, timestep, t_0]            = params;
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

    template <typename Data_T, typename Params_T>
    void initTimepoints(Data_T& data, Params_T& params)
    {

      auto current_time = types::HighResolutionClock::now();
      params.t_0        = current_time;

      for (auto& sphere : data) {
        sphere.t_c = current_time;
      }
    }

    template <typename SphereData_T, typename FPlaneData_T,
              typename IntersectionData_T>
    void initIntersectionCache(SphereData_T&       sphere_data,
                               FPlaneData_T&       fplane_data,
                               IntersectionData_T& intersection_data)
    {
      for (auto sphere : sphere_data) {
        for (auto fplane : fplane_data) {
          intersection_data.emplace_back(sphere, fplane, (false, 0.0));
        }
      }
    }


    // Algorithm 4
    template <typename Data_T, typename Params_T>
    void computeIntersections(Data_T& intersection_data, Params_T const& params)
    {
      for (auto& intersection : intersection_data) {
        auto const& sphere = intersection.sphere;
        auto const& plane  = intersection.plane;
        auto&       status = intersection.status;

        auto x = mechanics::detectCollisionSphereFixedPlane(
          sphere.t_c, intersection.sphere.p, intersection.sphere.r,
          intersection.sphere.v, plane.p, plane.n, params.F, params.t_0,
          params.timestep);
        status.is_collision = x.has_value();
        // status.col_tp = (x.has_value()) ? params.t_0 + params.timestep * x :
        // NULL;
      }
    }

  */
  template <concepts::SolverFixtureLevel2 Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
  {
    /*
  solver_types::Params params;
  params.F        = scenario.m_forces;
  params.timestep = timestep;

  initTimepoints(scenario.m_backend.m_sphere_data, params);
  initIntersectionCache(scenario.m_backend.m_sphere_data,
                        scenario.m_backend.m_fplane_data,
                        scenario.m_backend.m_intersection_data);

  computeIntersections(scenario.m_backend.m_intersection_data, params);


  computeCache(scenario.m_backend.m_cache_data, params);

  for (auto const& [id, cache] : scenario.m_backend.m_id_to_cache) {
    scenario.translateParent(id,
                             scenario.m_backend.m_cache_data[cache].out_ds);
    scenario.addAcceleration(id,
                             scenario.m_backend.m_cache_data[cache].out_a);
  }
  */
  }

}   // namespace dte3607::physengine::solver_dev::level2


#endif   // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL2_H
