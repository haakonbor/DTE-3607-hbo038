#ifndef DTE3607_PHYSENGINE_FIXTURE_CONCEPTS_H
#define DTE3607_PHYSENGINE_FIXTURE_CONCEPTS_H


#include "types.h"

// stl
#include <concepts>


namespace dte3607::physengine::concepts
{


  template <typename Fixture_T>
  concept SolverFixtureLevel0 =

    //    SolverFixtureTypes<Fixture_T>

    // Types
    requires
  {
    // Basic types
    typename Fixture_T::ValueType;
    typename Fixture_T::Point3;
    typename Fixture_T::Vector3;
  }

  and

    // getters (functions must be marked with const)
    requires(Fixture_T const& fixture, size_t rid)
  {
    // Query the number of rigid bodies
    {
      fixture.noRigidBodies()
      } -> std::convertible_to<size_t>;

    // Query the global frame of rigid body with ID
    {
      fixture.globalFramePosition(rid)
      } -> std::convertible_to<types::Point3>;
  }

  and

    // setters
    requires(Fixture_T fixture, types::ValueType value, types::Vector3 vector)
  {
    {
      fixture.setGravity(vector)
      } -> std::same_as<void>;

    // Fixture construction
    {
      fixture.createSphere(value, vector, vector, value)
      } -> std::same_as<void>;
  }

  ;


  template <typename Fixture_T>
  concept SolverFixtureLevel1 = SolverFixtureLevel0<Fixture_T>;



  template <typename Fixture_T>
  concept SolverFixtureLevel2 = SolverFixtureLevel1<Fixture_T> and

    // getters (functions must be marked with const)
    requires(Fixture_T const& fixture, size_t rid)
  {


    // Fixture Info Query
    {
      fixture.externalForces()
      } -> std::convertible_to<types::Vector3>;

    {
      fixture.noRigidBodies()
      } -> std::convertible_to<size_t>;

  {
    fixture.nonFixedSpheres()
    } -> std::same_as<std::vector<size_t>>;

  {
    fixture.fixedInfPlanes()
    } -> std::same_as<std::vector<size_t>>;

  {
    fixture.rbSphereRadius(rid)
    } -> std::convertible_to<types::ValueType>;

  {
    fixture.rbPlaneNormal(rid)
    } -> std::convertible_to<types::Vector3>;


    // Rigid Body Info Query
    {
      fixture.globalFramePosition(rid)
      } -> std::convertible_to<types::Point3>;
  }

  and

    // setters
    requires(Fixture_T fixture, types::ValueType value, types::Vector3 vector)
  {

    // Fixture construction
    {
      fixture.createFixedInfPlane(vector, vector, value)
      } -> std::same_as<void>;
  }

  ;


  template <typename Fixture_T>
  concept SolverFixtureLevel3 = SolverFixtureLevel2<Fixture_T> and

    // getters (functions must be marked with const)
    requires(Fixture_T const& fixture, size_t rid)
  {
    // Fixture Info Query
    {
      fixture.mode(rid)
      } -> std::convertible_to<types::RBMode>;

    {
      fixture.state(rid)
      } -> std::convertible_to<types::RBState>;
  }

  ;




  template <typename Fixture_T>
  concept SolverFixtureLevel4 = SolverFixtureLevel2<Fixture_T>;




  // The requirements on the concept used for the sover API

  template <typename Fixture_T>
  concept SolverFixture = SolverFixtureLevel0<Fixture_T>;





}   // namespace dte3607::physengine::api::concepts


#endif   // DTE3607_PHYSENGINE_FIXTURE_CONCEPTS_H
