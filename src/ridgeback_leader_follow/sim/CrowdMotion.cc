// F1-only simulator motion and observed actor transforms. No robot commands.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/pose_v.pb.h>

namespace leader_follow {
struct Waypoint { double t, x, y; };
struct Person {
  std::string name;
  double gaitSpeed{.5}, cycle{2.}, idleCycle{0.}, idleOffset{0.}, walkOffset{0.}, initialHeading{0.};
  std::string clip;
  std::vector<Waypoint> path;
  gz::sim::Entity entity{gz::sim::kNullEntity};
  gz::math::Pose3d Sample(double t, double &distance) const {
    distance = 0.;
    double yaw = initialHeading;
    auto end = path.back();
    for (size_t i = 1; i < path.size(); ++i) {
      auto a = path[i-1], b = path[i];
      double dx = b.x-a.x, dy = b.y-a.y;
      double f = std::clamp((t-a.t)/(b.t-a.t), 0., 1.);
      distance += f * std::hypot(dx,dy);
      if (dx != 0. || dy != 0.) yaw = std::atan2(dy,dx);
      if (t < b.t) return {a.x+f*dx,a.y+f*dy,0.,0.,0.,yaw};
    }
    return {end.x,end.y,0.,0.,0.,yaw};
  }
  // Matches PersonPath.moving(): standing segments and the path end are idle.
  bool Moving(double t) const {
    for (size_t i = 1; i < path.size(); ++i)
      if (path[i-1].t <= t && t < path[i].t)
        return path[i].x != path[i-1].x || path[i].y != path[i-1].y;
    return false;
  }
};

class CrowdMotion : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate,
                    public gz::sim::ISystemPostUpdate {
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  std::vector<Person> people;
  std::atomic<bool> requested{false};
  bool started{false};
  // Metres travelled per animation second; 0.5 is the baseline Cesium Man walk.
  double epoch{0.}, last{-1.}, duration{20.}, gaitSpeed{.5};

  bool Start(const gz::msgs::Empty &, gz::msgs::Boolean &response) {
    // Idempotent: repeated readiness calls never restart the scenario.
    requested = true;
    response.set_data(true);
    return true;
  }
 public:
  void Configure(const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    auto config = sdf->Clone();
    duration = config->Get<double>("duration");
    if (config->HasElement("gait_speed"))
      gaitSpeed = config->Get<double>("gait_speed");
    for (auto p = config->GetElement("person"); p;
         p = p->GetNextElement("person")) {
      Person person;
      person.name = p->Get<std::string>("name");
      person.gaitSpeed = p->HasElement("gait_speed") ? p->Get<double>("gait_speed") : gaitSpeed;
      if (p->HasElement("cycle")) person.cycle = p->Get<double>("cycle");
      if (p->HasElement("idle_cycle")) person.idleCycle = p->Get<double>("idle_cycle");
      if (p->HasElement("idle_offset")) person.idleOffset = p->Get<double>("idle_offset");
      if (p->HasElement("walk_offset")) person.walkOffset = p->Get<double>("walk_offset");
      if (p->HasElement("initial_heading")) person.initialHeading = p->Get<double>("initial_heading");
      for (auto w = p->GetElement("waypoint"); w;
           w = w->GetNextElement("waypoint")) {
        auto v = w->Get<gz::math::Vector3d>();
        person.path.push_back({v.X(),v.Y(),v.Z()});
      }
      people.push_back(person);
    }
    publisher = node.Advertise<gz::msgs::Pose_V>("/leader_follow/ground_truth");
    node.Advertise("/leader_follow/start", &CrowdMotion::Start, this);
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    double now = std::chrono::duration<double>(info.simTime).count();
    if (requested && !started) { started = true; epoch = now; last = now; }
    double elapsed = started ? std::min(now-epoch,duration) : 0.;
    for (auto &person : people) {
      if (person.entity == gz::sim::kNullEntity)
        person.entity = ecm.EntityByComponents(gz::sim::components::Name(person.name));
      if (person.entity == gz::sim::kNullEntity) continue;
      double distance;
      auto pose = person.Sample(elapsed,distance);
      ecm.SetComponentData<gz::sim::components::TrajectoryPose>(person.entity,pose);
      // Idle loop while standing (when the actor has one); gait follows distance.
      bool idle = person.idleCycle > 0. && !person.Moving(elapsed);
      std::string clip = idle ? "idle" : "walk";
      ecm.SetComponentData<gz::sim::components::AnimationName>(person.entity,clip);
      if (clip != person.clip) {
        person.clip = clip;
        ecm.SetChanged(person.entity,gz::sim::components::AnimationName::typeId,
                       gz::sim::ComponentState::OneTimeChange);
      }
      double seconds = idle ? std::fmod(elapsed+person.idleOffset,person.idleCycle)
                            : std::fmod(distance/person.gaitSpeed+person.walkOffset,person.cycle);
      auto phase = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(seconds));
      auto animation = ecm.Component<gz::sim::components::AnimationTime>(person.entity);
      if (animation) *animation = gz::sim::components::AnimationTime(phase);
      else ecm.CreateComponent(person.entity,gz::sim::components::AnimationTime(phase));
      ecm.SetChanged(person.entity,gz::sim::components::AnimationTime::typeId,
                     gz::sim::ComponentState::OneTimeChange);
      ecm.SetChanged(person.entity,gz::sim::components::TrajectoryPose::typeId,
                     gz::sim::ComponentState::OneTimeChange);
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override {
    if (!started || info.paused) return;
    double now = std::chrono::duration<double>(info.simTime).count();
    if (now-last < .05-1e-8) return;
    last = now;
    gz::msgs::Pose_V message;
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    message.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
    message.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
    auto meta = message.mutable_header()->add_data();
    meta->set_key("scenario_time");
    std::ostringstream elapsedText;
    elapsedText << std::setprecision(17) << now-epoch;
    meta->add_value(elapsedText.str());
    for (auto &person : people) {
      if (person.entity == gz::sim::kNullEntity) continue;
      auto trajectory = ecm.Component<gz::sim::components::TrajectoryPose>(person.entity);
      if (!trajectory) continue;
      // Exactly the transform consumed by RenderUtil for actor visuals:
      // actor origin world pose * observed TrajectoryPose component.
      auto observed = gz::sim::worldPose(person.entity,ecm) * trajectory->Data();
      auto output = message.add_pose();
      output->set_name(person.name);
      output->mutable_position()->set_x(observed.X());
      output->mutable_position()->set_y(observed.Y());
      output->mutable_position()->set_z(observed.Z());
      output->mutable_orientation()->set_w(observed.Rot().W());
      output->mutable_orientation()->set_x(observed.Rot().X());
      output->mutable_orientation()->set_y(observed.Rot().Y());
      output->mutable_orientation()->set_z(observed.Rot().Z());
    }
    publisher.Publish(message);
  }
};
}
GZ_ADD_PLUGIN(leader_follow::CrowdMotion,gz::sim::System,
              leader_follow::CrowdMotion::ISystemConfigure,
              leader_follow::CrowdMotion::ISystemPreUpdate,
              leader_follow::CrowdMotion::ISystemPostUpdate)
