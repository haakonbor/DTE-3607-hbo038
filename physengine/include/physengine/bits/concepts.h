#ifndef DTE3607_PHYSENGINE_FIXTURE_CONCEPTS_H
#define DTE3607_PHYSENGINE_FIXTURE_CONCEPTS_H


#include "types.h"

// stl
#include <concepts>


namespace dte3607::physengine::concepts
{

//  template <typename Fixture_T>
//  concept SolverFixtureTypes = requires
//  {
//    // Basic types
//    typename Fixture_T::ValueType;
//    typename Fixture_T::Point3;
//    typename Fixture_T::Point3H;
//    typename Fixture_T::Vector3;
//    typename Fixture_T::Vector3H;
//    typename Fixture_T::Timepoint;

//    // Environment types
//    typename Fixture_T::Forces;

//    // Rigid body types
//    typename Fixture_T::RBMode;
//    typename Fixture_T::RBState;
//  };


  template <typename Fixture_T>
  concept SolverFixtureTypes =

      requires
  {
    // Basic types
    typename Fixture_T::ValueType;
    typename Fixture_T::Point3;
    typename Fixture_T::Vector3;
  }
      ;

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
      } -> std::convertible_to<typename types::Point3>;
  }

  and

    // setters
    requires(Fixture_T fixture, types::ValueType value,
             typename Fixture_T::Vector3 vector)
  {
    {
      fixture.setGravity(vector)
      } -> std::same_as<void>;

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

    //

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
  concept SolverFixtureLevel3 = SolverFixtureLevel2<Fixture_T>

//    and

//    requires
//  {
//    {};
//  }

  ;



  template <typename Fixture_T>
  concept SolverFixtureLevel4 = SolverFixtureLevel3<Fixture_T>;




  // The requirements on the concept used for the sover API

  template <typename Fixture_T>
  concept SolverFixture = SolverFixtureLevel0<Fixture_T>;





}   // namespace dte3607::physengine::api::concepts


#endif   // DTE3607_PHYSENGINE_FIXTURE_CONCEPTS_H
