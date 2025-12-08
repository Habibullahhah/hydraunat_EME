#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
  class SimpleHydrodynamics : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;
      this->link = _model->GetLink("loco_base_frame"); 

      // --- CONFIGURATION ---
      // Reverting to the "Goldilocks" neutral volume.
      this->volume = 0.002235; 
      
      this->fluid_density = 997.0;
      this->g = 9.81;

      // --- DRAG (The "Water Brakes") ---
      // High drag prevents the "rocket" effect and stabilizes the robot.
      this->drag_linear = ignition::math::Vector3d(20.0, 20.0, 20.0); 
      this->drag_angular = ignition::math::Vector3d(5.0, 5.0, 5.0); 

      if (!this->link) {
          std::cerr << "[Hydrodynamics] Error: Link 'loco_base_frame' not found!" << std::endl;
      } else {
          std::cout << "[Hydrodynamics] LoCO Loaded. Mass=" 
                    << this->link->GetInertial()->Mass() 
                    << "kg. Volume=" << this->volume << std::endl;
      }

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SimpleHydrodynamics::OnUpdate, this));
    }

    void OnUpdate()
    {
      if (!this->link) return;

      // 1. BUOYANCY (ALWAYS ON)
      double buoyancy_mag = this->fluid_density * this->g * this->volume;
      ignition::math::Vector3d buoyancy_force(0, 0, buoyancy_mag);
      
      // Apply slightly above COM for self-righting stability
      ignition::math::Vector3d center_of_buoyancy(0, 0, 0.02); 
      this->link->AddForceAtRelativePosition(buoyancy_force, center_of_buoyancy);

      // 2. DRAG
      ignition::math::Vector3d lin_vel = this->link->WorldLinearVel();
      ignition::math::Vector3d ang_vel = this->link->WorldAngularVel();
      ignition::math::Pose3d pose = this->link->WorldPose();
      ignition::math::Vector3d lin_vel_body = pose.Rot().RotateVectorReverse(lin_vel);
      
      ignition::math::Vector3d drag_force_body;
      drag_force_body.X() = -this->drag_linear.X() * lin_vel_body.X();
      drag_force_body.Y() = -this->drag_linear.Y() * lin_vel_body.Y();
      drag_force_body.Z() = -this->drag_linear.Z() * lin_vel_body.Z();

      this->link->AddRelativeForce(drag_force_body);
      this->link->AddRelativeTorque(-this->drag_angular * ang_vel); 
    }

  private:
    physics::ModelPtr model;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;
    double volume, fluid_density, g;
    ignition::math::Vector3d drag_linear, drag_angular;
  };

  GZ_REGISTER_MODEL_PLUGIN(SimpleHydrodynamics)
}
