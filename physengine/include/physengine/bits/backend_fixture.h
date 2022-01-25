#ifndef DTE3607_PHYSENGINE_BACKEND_FIXTURES_H
#define DTE3607_PHYSENGINE_BACKEND_FIXTURES_H

#include <unordered_map>

// DTE-3607 PhysEngine
#include <physengine/bits/types.h>
#include <physengine/bits/solver_types.h>

namespace dte3607::physengine::backend
{
  namespace types        = dte3607::physengine::types;
  namespace solver_types = dte3607::physengine::solver_types;

  // Solver types
  using CacheProcDataBlock        = solver_types::CacheProcDataBlock;
  using SphereGeomDataBlock       = solver_types::SphereGeomDataBlock;
  using InfPlaneGeomDataBlock     = solver_types::InfPlaneGeomDataBlock;
  using IntersectDetProcDataBlock = solver_types::IntersectDetProcDataBlock;
  using CacheProcData             = std::vector<CacheProcDataBlock>;
  using SphereGeomData            = std::vector<SphereGeomDataBlock>;
  using InfPlaneGeomData          = std::vector<InfPlaneGeomDataBlock>;
  using IntersectDetProcData      = std::vector<IntersectDetProcDataBlock>;
  using Vector3                   = types::Vector3;

  struct BackendFixture {
    CacheProcData        m_cache_data;
    SphereGeomData       m_sphere_data;
    InfPlaneGeomData     m_fplane_data;
    IntersectDetProcData m_intersection_data;
    // std::unordered_map<size_t, size_t> m_sphere_to_id;
    // std::unordered_map<size_t, size_t> m_fplane_to_id;
    std::unordered_map<size_t, size_t> m_id_to_sphere;
    std::unordered_map<size_t, size_t> m_id_to_fplane;
    std::unordered_map<size_t, size_t> m_id_to_cache;
  };

  void initSphereCache(BackendFixture& bf, size_t rbi, double radius,
                       Vector3 position, Vector3 velocity)
  {
    // Cache data
    bf.m_cache_data.emplace_back(position, velocity, Vector3{0, 0, 0},
                                 Vector3{0, 0, 0});
    // Sphere geometry data
    bf.m_sphere_data.emplace_back(position, radius, velocity,
                                  types::HighResolutionClock::now());
    // ID
    bf.m_id_to_sphere.emplace(rbi, bf.m_sphere_data.size() - 1);
    bf.m_id_to_cache.emplace(rbi, bf.m_cache_data.size() - 1);
  }

  void initFPlaneCache(BackendFixture& bf, size_t rbi, Vector3 position,
                       Vector3 normal)
  {
    // Fixed plane geometry data
    bf.m_fplane_data.emplace_back(position, normal);
    // ID
    bf.m_id_to_fplane.emplace(rbi, bf.m_fplane_data.size() - 1);
  }

  /*
  template <typename Fixture_T>
  void initComputationalWorld(Fixture_T const& f, BackendFixture& bf)
  {
    size_t id = 0ul;

    for (auto const& rb : f.m_rigid_bodies) {
      auto current_id         = id++;
      auto constexpr NonFixed = Fixture_T::RBMode::NonFixed;

      if (rb->mode() not_eq NonFixed) {
        bf.m_cache_data.emplace_back(rb->globalFramePosition(), rb->velocity(),
                                     Vector3{0, 0, 0}, Vector3{0, 0, 0});

        bf.m_rb_cache.emplace(bf.m_cache_data.size() - 1, current_id);

        bf.m_sphere_data.emplace_back(rb->globalFramePosition(),
                                      rb->shape().radius(), rb->velocity());
        bf.m_spheres(bf.m_sphere_data.size() - 1, current_id);
      }
      else {
        bf.m_fplane_data.emplace_back(rb->globalFramePosition(), rb->normal());

        bf.m_fplanes(bf.m_fplane_data.size() - 1, current_id);
      }
    }
  }
  */
}   // namespace dte3607::physengine::backend

#endif   // DTE3607_PHYSENGINE_BACKEND_FIXTURES_H
