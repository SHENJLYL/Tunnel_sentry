#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include "chassis_controllers/kinematics.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace sentry {
struct Drive { double x=0, y=0, yaw=0, stamp=-1; };
struct Gimbal { double yaw=0, pitch=0; };
struct Orientation { double yaw=0, stamp=-1; };

class SentryController : public controller_interface::ControllerInterface {
  using CallbackReturn = controller_interface::CallbackReturn;
  const std::array<std::string,12> names_{
    "front_right_steering_joint", "front_right_wheel_joint",
    "rear_right_steering_joint", "rear_right_wheel_joint",
    "rear_left_steering_joint", "rear_left_wheel_joint",
    "front_left_steering_joint", "front_left_wheel_joint",
    "gimbal_yaw_joint", "gimbal_pitch_joint", "lift_joint", "lift_stage_joint"};
  std::array<size_t,12> cmd_index_{}, pos_index_{}, vel_index_{};
  std::array<double,4> integral_{};
  realtime_tools::RealtimeBuffer<Drive> drive_;
  realtime_tools::RealtimeBuffer<Gimbal> gimbal_;
  realtime_tools::RealtimeBuffer<double> lift_;
  realtime_tools::RealtimeBuffer<Orientation> imu_;
  realtime_tools::RealtimeBuffer<bool> world_mode_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drive_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr height_sub_, lift_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr world_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::UInt8>> limits_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> efforts_pub_;
  double radius_, span_, pitch_mass_, pitch_com_, pitch_min_, pitch_max_;
  double timeout_, max_v_, max_w_, wheel_kp_, wheel_ki_, wheel_limit_;
  double steer_kp_, steer_kd_, steer_limit_, yaw_kp_, yaw_kd_, yaw_limit_;
  double pitch_kp_, pitch_kd_, pitch_limit_, pitch_speed_;
  double folded_height_, lift_stroke_, lift_speed_, lift_mass_, lift_kp_, lift_kd_, lift_limit_;
  double pitch_ref_=0, lift_ref_=0, vx_=0, vy_=0, wz_=0;
  unsigned counter_=0;

  static double toward(double current, double goal, double step) {
    return current + std::clamp(goal-current, -step, step);
  }


public:
  CallbackReturn on_init() override {
    drive_.writeFromNonRT(Drive{}); gimbal_.writeFromNonRT(Gimbal{}); lift_.writeFromNonRT(0.0);
    imu_.writeFromNonRT(Orientation{}); world_mode_.writeFromNonRT(false);
    for (const auto &p : std::vector<std::pair<std::string,double>>{
      {"wheel_radius",.065},{"half_spacing",.2273},{"folded_height",.23925},
      {"lift_stroke",.16075},{"lift_speed",.06},{"lift_mass",13.75663013135},
      {"lift_kp",6000},{"lift_kd",550},{"lift_force",300},
      {"pitch_mass",1.82471587885},{"pitch_com_x",.133070260132},
      {"pitch_min",0},{"pitch_max",.52},{"command_timeout",.5},
      {"max_linear_speed",1.0},{"max_yaw_rate",1.5},{"wheel_kp",.12},{"wheel_ki",.5},{"wheel_torque",5},
      {"steering_kp",20},{"steering_kd",.35},{"steering_torque",8},
      {"yaw_kp",35},{"yaw_kd",5},{"yaw_torque",12},
      {"pitch_kp",25},{"pitch_kd",2.5},{"pitch_torque",8},
      {"pitch_speed",.25}}) {
      auto_declare<double>(p.first,p.second);
    }
    return CallbackReturn::SUCCESS;
  }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL,{}};
    for (const auto &name:names_) c.names.push_back(name+"/effort");
    return c;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL,{}};
    for (const auto &n:names_) { c.names.push_back(n+"/position"); c.names.push_back(n+"/velocity"); }
    return c;
  }
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    auto node=get_node();
    auto p=[&](const char *name){return node->get_parameter(name).as_double();};
    radius_=p("wheel_radius"); span_=p("half_spacing");
    pitch_mass_=p("pitch_mass"); pitch_com_=p("pitch_com_x");
    pitch_min_=p("pitch_min"); pitch_max_=p("pitch_max"); timeout_=p("command_timeout");
    max_v_=p("max_linear_speed"); max_w_=p("max_yaw_rate");
    wheel_kp_=p("wheel_kp"); wheel_ki_=p("wheel_ki"); wheel_limit_=p("wheel_torque");
    steer_kp_=p("steering_kp"); steer_kd_=p("steering_kd"); steer_limit_=p("steering_torque");
    yaw_kp_=p("yaw_kp"); yaw_kd_=p("yaw_kd"); yaw_limit_=p("yaw_torque");
    pitch_kp_=p("pitch_kp"); pitch_kd_=p("pitch_kd"); pitch_limit_=p("pitch_torque");
    pitch_speed_=p("pitch_speed"); folded_height_=p("folded_height");
    lift_stroke_=p("lift_stroke"); lift_speed_=p("lift_speed"); lift_mass_=p("lift_mass");
    lift_kp_=p("lift_kp"); lift_kd_=p("lift_kd"); lift_limit_=p("lift_force");
    if (!(radius_>0 && span_>0 && timeout_>0 && max_v_>0 && max_w_>0 &&
          pitch_mass_>0 && pitch_min_<pitch_max_ && pitch_speed_>0 && folded_height_>0 &&
          lift_stroke_>0 && lift_speed_>0 && lift_mass_>0 && lift_kp_>0 && lift_kd_>0 && lift_limit_>lift_mass_*9.81)) {
      RCLCPP_ERROR(node->get_logger(),"Check geometry, mass, travel and force parameters.");
      return CallbackReturn::ERROR;
    }
    drive_sub_=node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",1,[this](const geometry_msgs::msg::Twist &m){
      if (std::isfinite(m.linear.x)&&std::isfinite(m.linear.y)&&std::isfinite(m.angular.z))
        drive_.writeFromNonRT(Drive{m.linear.x,m.linear.y,m.angular.z,get_node()->now().seconds()});
    });
    gimbal_sub_=node->create_subscription<geometry_msgs::msg::Vector3>("gimbal_cmd",1,[this](const geometry_msgs::msg::Vector3 &m){
      if (std::isfinite(m.x)&&std::isfinite(m.y)) gimbal_.writeFromNonRT(Gimbal{wrap(m.x),std::clamp(m.y,pitch_min_,pitch_max_)});
    });
    height_sub_=node->create_subscription<std_msgs::msg::Float64>("height_cmd",1,[this](const std_msgs::msg::Float64 &m){
      if(std::isfinite(m.data)) lift_.writeFromNonRT(std::clamp(m.data-folded_height_,0.0,lift_stroke_));
    });
    lift_sub_=node->create_subscription<std_msgs::msg::Float64>("lift_cmd",1,[this](const std_msgs::msg::Float64 &m){
      if(std::isfinite(m.data)) lift_.writeFromNonRT(std::clamp(m.data,0.0,lift_stroke_));
    });
    world_sub_=node->create_subscription<std_msgs::msg::Bool>("gimbal_world_mode",1,[this](const std_msgs::msg::Bool &m){world_mode_.writeFromNonRT(m.data);});
    imu_sub_=node->create_subscription<sensor_msgs::msg::Imu>("imu",rclcpp::SensorDataQoS(),[this](const sensor_msgs::msg::Imu &m){
      const auto &q=m.orientation;
      const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if (std::isfinite(norm)&&norm>.5&&norm<1.5) {
        const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        imu_.writeFromNonRT(Orientation{yaw,get_node()->now().seconds()});
      }
    });
    limits_pub_=std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::UInt8>>(node->create_publisher<std_msgs::msg::UInt8>("height_limits",1));
    efforts_pub_=std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(node->create_publisher<std_msgs::msg::Float64MultiArray>("effort_command",1));
    efforts_pub_->msg_.data.resize(17,0.0);
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    for (size_t j=0;j<names_.size();++j) {
      auto find=[&](auto &interfaces,const std::string &suffix) {
        for (size_t i=0;i<interfaces.size();++i) if (interfaces[i].get_name()==names_[j]+suffix) return i;
        return interfaces.size();
      };
      cmd_index_[j]=find(command_interfaces_,"/effort");
      pos_index_[j]=find(state_interfaces_,"/position"); vel_index_[j]=find(state_interfaces_,"/velocity");
      if (cmd_index_[j]==command_interfaces_.size()||pos_index_[j]==state_interfaces_.size()||vel_index_[j]==state_interfaces_.size()) return CallbackReturn::ERROR;
    }
    integral_.fill(0); vx_=vy_=wz_=0; counter_=0;
    pitch_ref_=state_interfaces_[pos_index_[9]].get_optional().value_or(0.0);
    lift_ref_=state_interfaces_[pos_index_[10]].get_optional().value_or(0.0);
    lift_.writeFromNonRT(std::clamp(lift_ref_,0.0,lift_stroke_));
    gimbal_.writeFromNonRT(Gimbal{
      state_interfaces_[pos_index_[8]].get_optional().value_or(0.0),
      state_interfaces_[pos_index_[9]].get_optional().value_or(0.0)});
    drive_.writeFromNonRT(Drive{});
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    for(auto &c:command_interfaces_) (void)c.set_value(0.0);
    return CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &time,const rclcpp::Duration &period) override {
    const double dt=period.seconds();
    if (!(dt>0 && dt<.1)) return controller_interface::return_type::OK;
    std::array<double,12> q{},v{};
    std::array<double,12> effort{};
    for(size_t j=0;j<q.size();++j) {
      q[j]=state_interfaces_[pos_index_[j]].get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
      v[j]=state_interfaces_[vel_index_[j]].get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
      if (!(std::isfinite(q[j])&&std::isfinite(v[j]))) {
        for(auto &c:command_interfaces_) (void)c.set_value(0.0);
        return controller_interface::return_type::ERROR;
      }
    }
    Drive d=*drive_.readFromRT();
    const double received_stamp=d.stamp;
    if(d.stamp<0 || time.seconds()-d.stamp>timeout_ || time.seconds()+.1<d.stamp) d=Drive{};
    const double factor=std::min(1.0,max_v_/std::max(1e-6,std::hypot(d.x,d.y)));
    vx_=toward(vx_,d.x*factor,1.0*dt); vy_=toward(vy_,d.y*factor,1.0*dt);
    wz_=toward(wz_,std::clamp(d.yaw,-max_w_,max_w_),2.0*dt);
    const std::array<double,4> x{span_,-span_,-span_,span_},y{-span_,-span_,span_,span_};
    const std::array<double,4> mounts{0,-pi/2,pi,pi/2};
    for(size_t i=0;i<4;++i) {
      const size_t s=2*i,w=s+1;
      const auto target=solve_wheel(vx_,vy_,wz_,x[i],y[i],mounts[i],q[s],radius_);
      effort[s]=std::clamp(steer_kp_*wrap(target.steering-q[s])-steer_kd_*v[s],-steer_limit_,steer_limit_);
      const double error=target.velocity-v[w];
      const double candidate=std::clamp(integral_[i]+wheel_ki_*error*dt,-wheel_limit_,wheel_limit_);
      const double raw=wheel_kp_*error+candidate;
      if(std::abs(raw)<wheel_limit_ || error*raw<0) integral_[i]=candidate;
      effort[w]=std::clamp(wheel_kp_*error+integral_[i],-wheel_limit_,wheel_limit_);
    }
    Gimbal g=*gimbal_.readFromRT();
    if(*world_mode_.readFromRT()) {
      const auto orientation=*imu_.readFromRT();
      if(orientation.stamp>=0 && time.seconds()-orientation.stamp<.2) g.yaw=wrap(g.yaw-orientation.yaw);
      else g.yaw=q[8];
    }
    effort[8]=std::clamp(yaw_kp_*wrap(g.yaw-q[8])-yaw_kd_*v[8],-yaw_limit_,yaw_limit_);
    pitch_ref_=toward(pitch_ref_,g.pitch,pitch_speed_*dt);
    effort[9]=std::clamp(pitch_kp_*(pitch_ref_-q[9])-pitch_kd_*v[9]+pitch_mass_*9.81*pitch_com_*std::cos(q[9]),-pitch_limit_,pitch_limit_);
    lift_ref_=toward(lift_ref_,*lift_.readFromRT(),lift_speed_*dt);
    effort[10]=std::clamp(lift_kp_*(lift_ref_-q[10])-lift_kd_*v[10]+lift_mass_*9.81,-lift_limit_,lift_limit_);
    // Equivalent half-travel drive for the middle sleeve; its force is reported separately.
    effort[11]=std::clamp(1500.0*(.5*q[10]-q[11])+45.0*(.5*v[10]-v[11])+.35*9.81,-40.0,40.0);
    for(size_t j=0;j<effort.size();++j) if(!command_interfaces_[cmd_index_[j]].set_value(effort[j])) return controller_interface::return_type::ERROR;
    if(++counter_%10==0 && limits_pub_->trylock()) {
      limits_pub_->msg_.data=(q[10]<.001?1:0)|(q[10]>lift_stroke_-.001?2:0);
      limits_pub_->unlockAndPublish();
    }
    if(counter_%10==0 && efforts_pub_->trylock()) {
      auto &data=efforts_pub_->msg_.data;
      data[0]=time.seconds(); data[1]=received_stamp; data[2]=vx_; data[3]=vy_; data[4]=wz_;
      std::copy(effort.begin(),effort.end(),data.begin()+5);
      efforts_pub_->unlockAndPublish();
    }
    return controller_interface::return_type::OK;
  }
};
}
PLUGINLIB_EXPORT_CLASS(sentry::SentryController,controller_interface::ControllerInterface)
