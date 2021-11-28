#ifndef DTE3607_PHYSENGINE_TYPES_EXT_H
#define DTE3607_PHYSENGINE_TYPES_EXT_H

#include "types.h"

namespace dte3607::physengine::types_ext
{
  namespace rb_oop {

    struct SpaceObjectBase {

      // Concept requirements
      using ValueType   = types::Point3::ElementType;
      using Point3      = types::Point3;
      using Point3H     = types::Point3H;
      using Vector3     = types::Vector3;
      using Vector3H    = types::Vector3H;
      using SpaceObject = types::ProjectiveSpaceObject;
      using Timepoint   = types::HighResolutionTP;


      // Common properties
      SpaceObject m_object;

      // Lin. alg. prop. access
      virtual SpaceObject&       spaceObject() { return m_object; }
      virtual SpaceObject const& spaceObject() const { return m_object; }

      typename SpaceObject::ASFrameH pFrame() const
      {
        return spaceObject().pSpaceFrameParent();
      }

      typename SpaceObject::Frame vFrame() const
      {
        return spaceObject().vSpaceFrameParent();
      }

      Point3 globalFramePosition() const { return m_object.frameOriginParent(); }
    };


    namespace rb_shapes
    {
      enum class ShapeType { Sphere, InfPlane };

      struct Shape : SpaceObjectBase {
        Shape(ShapeType p_shape_type) : m_shape_type{p_shape_type} {}
        ShapeType m_shape_type;
      };


      struct Sphere : Shape {
        Sphere(types::Vector3::ElementType const& radius = 1.)
          : Shape(ShapeType::Sphere), m_radius{radius}
        {
        }

        // Sphere prop. access
        Point3    point() const { return spaceObject().frameOriginParent(); }
        ValueType radius() const { return m_radius; }

      private:
        ValueType m_radius{1.0};
      };

      struct InfPlane : Shape {
        InfPlane(types::Vector3 const& n = {.0, 1., .0})
          : Shape(ShapeType::InfPlane), m_n{n}
        {
        }

        // Plane prop. access
        Point3  point() const { return spaceObject().frameOriginParent(); }
        Vector3 normal() const { return spaceObject().vSpaceFrameParent() * m_n; }

      private:
        Vector3 m_n{0, 1, 0};
      };
    }   // namespace rb_shapes



    struct RigidBodyPart : SpaceObjectBase {

      using Shape = rb_shapes::Shape;

      // Class type constants
      static constexpr ValueType frictionCoefMin{0.0};
      static constexpr ValueType frictionCoefMax{1.0};

      ValueType frictionCoef() const { return m_friction_coef; }
      void      setFrictionCoef(ValueType const& friction_coef)
      {
        m_friction_coef
          = std::clamp(friction_coef, frictionCoefMin, frictionCoefMax);
      }

      ValueType m_mass{1.0};

      Shape* shape() { return m_shape; }
      Shape* m_shape;

    private:
      ValueType m_friction_coef{0.0};   // 0 == no friction
    };

    struct RigidBody : SpaceObjectBase {

      RigidBody(const std::string& name = "") : m_name{name} {}

      // Concept requirements
      using ValueType        = types::Point3::ElementType;
      using Point3           = types::Point3;
      using Point3H          = types::Point3H;
      using Vector3          = types::Vector3;
      using Vector3H         = types::Vector3H;
      using SpaceObjectFrame = types::ProjectiveSpaceObject;
      using Timepoint        = types::HighResolutionTP;


      // Mechanics property access
      Vector3 velocity() const { return m_velocity; }

      /** a; in the "parent" spacial frame */
      void addAcceleration(types::Vector3 const& a) { m_velocity += a; }

      /** v; in the "parent" spacial frame */
      void setVelocity(types::Vector3 const& v) { m_velocity = v; }


      // Mode
      enum class Mode { NonFixed, Fixed, Hmm };
      Mode mode() const { return m_mode; }
      void setMode(Mode p_mode) { m_mode = p_mode; }

      // States
      enum class State { Free, Resting, Sliding, Rolling };
      State state() const { return m_state; }
      void  setState(State p_state) { m_state = p_state; }

    private:
      Timepoint m_timepoint;
      Vector3   m_velocity{0, 0, 0};
      Mode      m_mode{Mode::NonFixed};
      State     m_state{State::Free};


    public:
      std::string m_name;

      using Part    = RigidBodyPart;
      using PartPtr = std::unique_ptr<RigidBodyPart>;
      using Parts   = std::vector<PartPtr>;

      Parts m_parts;
    };

  }   // namespace rb_oop


  struct FixtureOOP {

    // Types [Concept requirements]
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces       = types::Vector3;

    using RigidBody    = rb_oop::RigidBody;
    using RigidBodyPtr = std::unique_ptr<RigidBody>;
    using RigidBodies  = std::vector<RigidBodyPtr>;

    using RBShape    = rb_oop::rb_shapes::Shape;
    using RBShapePtr = std::unique_ptr<RBShape>;
    using RBShapes   = std::vector<RBShapePtr>;

    using RBPart = rb_oop::RigidBodyPart;
    using RBMode = RigidBody::Mode;

    // Members
    RigidBodies m_rigid_bodies;
    RBShapes    m_rb_shapes;

    Forces m_forces;


    /*** API concept required methods ***/



    // Global properties
    size_t noRigidBodies() const {
      return m_rigid_bodies.size();
    }

    Forces externalForces() const { return m_forces; }



    // RB properties
    types::Point3 globalFramePosition(size_t rid) const {

      return m_rigid_bodies[rid]->globalFramePosition();
    }

    types::Vector3 globalVelocity(size_t rid) const {

      return m_rigid_bodies[rid]->velocity();
    }

    RigidBody::Mode mode(size_t rid) const {

      return m_rigid_bodies[rid]->mode();
    }

    // Modes and states




    void translateParent(size_t rid, Vector3 lin_trajectory)
    {
      m_rigid_bodies[rid]->m_object.translateParent(lin_trajectory);
    }

    void addAcceleration(size_t rid, Vector3 accel)
    {
      m_rigid_bodies[rid]->addAcceleration(accel);
    }





    /*** Custom API methods ***/
    void createSphere(ValueType radius = 1., Vector3 velocity = {0, 0, 0},
                      Vector3   translation   = {0, 0, 0},
                      ValueType friction_coef = 1.)
    {
      // Create Rigid body
      m_rigid_bodies.emplace_back(std::make_unique<RigidBody>());
      auto& rb = m_rigid_bodies.back();
      rb->m_object.translateParent(translation);
      rb->setMode(RigidBody::Mode::NonFixed);

      // Create Shape
      m_rb_shapes.emplace_back(
        std::make_unique<rb_oop::rb_shapes::Sphere>(radius));
      auto* sphere = m_rb_shapes.back().get();

      // Create RB body part and add sphere as shape to body part
      rb->m_parts.emplace_back(std::make_unique<RBPart>());
      auto& rbp = rb->m_parts.back();
      rbp->m_shape = sphere;
      rbp->setFrictionCoef(friction_coef);
    }

    void createFixedInfPlane(Vector3   normal        = {0, 0, 1},
                             Vector3   translation   = {0, 0, 0},
                             ValueType friction_coef = 1.)
    {
      // Create Rigid body
      m_rigid_bodies.emplace_back(std::make_unique<RigidBody>());
      auto& rb = m_rigid_bodies.back();
      rb->m_object.translateParent(translation);
      rb->setMode(RigidBody::Mode::Fixed);

      // Create Shape
      m_rb_shapes.emplace_back(
        std::make_unique<rb_oop::rb_shapes::InfPlane>(normal));
      auto* plane = m_rb_shapes.back().get();

      // Create RB body part and add sphere as shape to body part
      rb->m_parts.emplace_back(std::make_unique<RBPart>());
      auto& rbp = rb->m_parts.back();
      rbp->m_shape = plane;
      rbp->setFrictionCoef(friction_coef);
    }
  };

}   // namespace dte3607::physengine::types

#endif // DTE3607_PHYSENGINE_TYPES_EXT_H
