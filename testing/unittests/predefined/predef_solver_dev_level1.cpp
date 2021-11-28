#include <physengine/solvers/solver_dev_level1.h>
#include <physengine/bits/types_ext.h>

// gtest
#include <gtest/gtest.h>   // googletest header file

// stl
#include <vector>
#include <chrono>
#include <memory>


// Safely enable namespaces
using namespace dte3607::physengine;
using namespace std::chrono_literals;




/////////////////
/// \brief TEST
///
///

struct SolverDevStep1_Fixture001 : ::testing::Test {

  using TestFixture = types_ext::FixtureOOP;
  std::unique_ptr<TestFixture> m_fixture;

  using ::testing::Test::Test;

  void SetUp() final
  {
    // Create Fixture
    m_fixture          = std::make_unique<TestFixture>();

    // Set external forces
    m_fixture->m_forces = types::Vector3{0, 0, 0};

    // make plane: normal, translation
    m_fixture->createFixedInfPlane({-1, 0, 0}, {10, 0, 0});

    // make sphere: radius, velocity, translation
    m_fixture->createSphere(1.0, {100, 0, 0}, {0, 0, 0});
  }
  void TearDown() final { m_fixture.release(); }
};



TEST_F(SolverDevStep1_Fixture001, Test001)
{
  solver_dev::level1::solve(*m_fixture, 1s);

  auto no_rbs = m_fixture->noRigidBodies();
  for( auto i = 0; i < no_rbs; ++i )
  {

    // Ask for global frame position of object nr. i
    auto const pos = m_fixture->globalFramePosition(i);
    EXPECT_TRUE(pos[0] < 10);
  }
}



