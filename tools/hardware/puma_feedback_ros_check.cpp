// Isolated ROS transport + compiled Puma plugin check. No CAN or robot commands.
#include <clearpath_hardware_interfaces/puma/hardware.hpp>
#include <cassert>
#include <thread>
#include <iostream>
#include <fstream>
#include <cstdlib>
using namespace clearpath_hardware_interfaces;
using Ret=hardware_interface::return_type;
using Life=hardware_interface::CallbackReturn;
struct Fixture : PumaHardware {
  Fixture() {
    wheel_joints_={{"fl",0},{"fr",1},{"rl",2},{"rr",3}};
    hw_commands_.assign(4,0);hw_states_position_.assign(4,0);
    hw_states_position_offset_.assign(4,0);hw_states_velocity_.assign(4,0);
    node_=std::make_shared<PumaHardwareInterface>("isolated_puma_fixture");
  }
  void command(double v){hw_commands_.assign(4,v);}
  double measured(){return hw_states_velocity_[0];}
};
int main(int argc,char**argv){
  assert(std::getenv("ROS_DOMAIN_ID") && std::string(std::getenv("ROS_DOMAIN_ID"))=="214");
  assert(std::getenv("DYNAMO_FEEDBACK_CANDIDATE_LIBRARY"));
  std::ifstream maps("/proc/self/maps");std::string line;bool correct_library=false;
  while(std::getline(maps,line))if(line.find("libpuma_hardware.so")!=std::string::npos){
    assert(line.find(std::getenv("DYNAMO_FEEDBACK_CANDIDATE_LIBRARY"))!=std::string::npos);correct_library=true;
  }
  assert(correct_library);
  rclcpp::init(argc,argv);
  auto peer=std::make_shared<rclcpp::Node>("isolated_fake_wheels");
  auto publisher=peer->create_publisher<clearpath_motor_msgs::msg::PumaMultiFeedback>("platform/motors/feedback",rclcpp::SensorDataQoS());
  std::vector<sensor_msgs::msg::JointState> commands;
  auto subscriber=peer->create_subscription<sensor_msgs::msg::JointState>("platform/motors/cmd",rclcpp::SensorDataQoS(),[&](sensor_msgs::msg::JointState::SharedPtr m){commands.push_back(*m);});
  auto publish=[&](double speed){
    clearpath_motor_msgs::msg::PumaMultiFeedback msg;msg.header.stamp=peer->now();
    const char*names[]={"fl","fr","rl","rr"};unsigned ids[]={5,4,2,3};
    for(int i=0;i<4;i++){clearpath_motor_msgs::msg::PumaFeedback w;w.device_name=names[i];w.device_number=ids[i];w.speed=speed;w.travel=1;msg.drivers_feedback.push_back(w);}
    publisher->publish(msg);
  };
  auto wait=[&](double seconds){auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);while(std::chrono::steady_clock::now()<end){rclcpp::spin_some(peer);std::this_thread::sleep_for(std::chrono::milliseconds(5));}};
  Fixture plugin;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  while(publisher->get_subscription_count()==0 && std::chrono::steady_clock::now()<deadline)wait(.01);
  assert(publisher->get_subscription_count()==1);
  publish(0);assert(plugin.on_activate(rclcpp_lifecycle::State())==Life::SUCCESS);wait(.02);
  assert(!commands.empty());for(double v:commands.back().velocity)assert(v==0);
  plugin.command(.4);assert(plugin.write(peer->now(),rclcpp::Duration::from_seconds(.05))==Ret::OK);wait(.03);
  assert(commands.back().velocity[0]==.4);
  publish(.5);wait(.02);assert(plugin.read(peer->now(),rclcpp::Duration::from_seconds(.05))==Ret::OK);
  assert(plugin.measured()==.5);
  // Stop publishing feedback; compiled write path must inhibit without another read.
  wait(.27);plugin.command(.4);assert(plugin.write(peer->now(),rclcpp::Duration::from_seconds(.05))==Ret::ERROR);wait(.02);
  for(double v:commands.back().velocity)assert(v==0);assert(plugin.measured()==.5);
  publish(0);wait(.02);assert(plugin.read(peer->now(),rclcpp::Duration::from_seconds(.05))==Ret::ERROR);
  // Explicit stationary activation can recover, keeping command storage zero.
  publish(0);assert(plugin.on_activate(rclcpp_lifecycle::State())==Life::SUCCESS);wait(.02);
  assert(plugin.on_deactivate(rclcpp_lifecycle::State())==Life::SUCCESS);wait(.02);
  for(double v:commands.back().velocity)assert(v==0);
  std::cout<<"Compiled plugin / real ROS transport checks passed; controller-manager and physical braking untested.\n";
  rclcpp::shutdown();return 0;
}
