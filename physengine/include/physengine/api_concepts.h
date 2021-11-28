#ifndef DTE3607_PHYSENGINE_FIXTURE_API_CONCEPTS_H
#define DTE3607_PHYSENGINE_FIXTURE_API_CONCEPTS_H


#include "bits/types.h"

// stl
#include <concepts>


namespace dte3607::physengine::api::concepts
{

  template <typename Fixture_T>
  concept SolverFixtureLevel0 =

    // getters (functions must be marked with const)
    requires(Fixture_T const& fixture, size_t rid)
  {

    // Types
    typename Fixture_T::RigidBody;
    typename Fixture_T::RBShape;


    // Fixture Info Query
    {
      fixture.externalForces()
      } -> std::convertible_to<typename Fixture_T::Forces>;

    {
      fixture.noRigidBodies()
      } -> std::convertible_to<size_t>;


    // Rigid Body Info Query
    {
      fixture.globalFramePosition(rid)
      } -> std::convertible_to<typename Fixture_T::Point3>;

    {
      fixture.globalVelocity(rid)
      } -> std::convertible_to<typename Fixture_T::Vector3>;

    {
      fixture.mode(rid)
      } -> std::convertible_to<typename Fixture_T::RBMode>;
  }

  and

    // setters
    requires(Fixture_T fixture, size_t rid, typename Fixture_T::Vector3 vec)
  {

    // Rigid Body Actions
    {
      fixture.translateParent(rid, vec)
      } -> std::same_as<void>;

    {
      fixture.addAcceleration(rid, vec)
      } -> std::convertible_to<void>;
  }

  ;


  template <typename Fixture_T>
  concept SolverFixtureLevel1 = SolverFixtureLevel0<Fixture_T>;


}   // namespace dte3607::physengine::api::concepts


#endif   // DTE3607_PHYSENGINE_FIXTURE_API_CONCEPTS_H
