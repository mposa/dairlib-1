#include <gflags/gflags.h>
#include "drake/lcm/drake_lcm.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "common/find_resource.h"
#include "multibody/multibody_utils.h"

namespace dairlib {
namespace {

using drake::AbstractValue;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::math::RigidTransform;
using drake::multibody::Parser;
using drake::multibody::ContactResults;
using drake::systems::Context;
using Eigen::VectorXd;
using std::cout;
using std::endl;

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm;

  // Initializing plant with a discrete timestep
  drake::systems::DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant =
      *builder.AddSystem<MultibodyPlant<double>>(1.0);
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  std::string full_name =
      FindResourceOrThrow("examples/PointPlane/PointMass.urdf");
  Parser parser(&plant, &scene_graph);
  parser.AddModelFromFile(full_name);

  plant.WeldFrames(
      plant.world_frame(), plant.GetFrameByName("base"),
      drake::math::RigidTransform<double>(
          Eigen::Vector3d::Zero()).GetAsIsometry3());

  multibody::addFlatTerrain(&plant, &scene_graph, 1.0, 1.0);

  plant.Finalize();

  builder.Connect(
    plant.get_geometry_poses_output_port(),
    scene_graph.get_source_pose_port(plant.get_source_id().value()));

  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  auto diagram = builder.Build();

  VectorXd x(6);
  x << 1.1, -.5, -2.0, 0, 0, 0;  // Third element is height

  auto diagram_context = diagram->CreateDefaultContext();
  Context<double>& context = diagram->GetMutableSubsystemContext(plant,
      diagram_context.get());

  plant.SetPositionsAndVelocities(&context, x);

  // Evaluate via contact_results_output_port
  const ContactResults<double>& contact_results =
    plant.get_contact_results_output_port().Eval<AbstractValue>(context).
        get_value<ContactResults<double>>();

  // Number of active contacts
  cout << "num_contacts: " << contact_results.num_contacts() << endl;


  // Evaluate via ComputeSignedDistancePairwiseClosestPoints, should detect
  // possible contacts (not just active ones)
  const auto& query_port = plant.get_geometry_query_input_port();
  const auto& query_object = query_port.Eval<
      drake::geometry::QueryObject<double>>(context);
  const std::vector<drake::geometry::SignedDistancePair<double>>
      signed_distance_pairs =
          query_object.ComputeSignedDistancePairwiseClosestPoints();

  return 0;
}

}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
