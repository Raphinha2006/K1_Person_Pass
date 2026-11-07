#include "person_pass/controller.hpp"
#include <booster_interface/message_utils.hpp>
#include <booster_interface/msg/low_state.hpp>
#include <booster_interface/msg/odometer.hpp>
#include <booster_interface/msg/raw_bytes_msg.hpp>
#include <booster_msgs/msg/rpc_req_msg.hpp>
#include <vision_interface/msg/detections.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <random>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
namespace pp = person_pass;
using Json = nlohmann::json;
static double monotonic() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static std::string requestUuid() {
    static std::mt19937 generator(std::random_device{}());
    static const char hex[]="0123456789abcdef";
    std::uniform_int_distribution<int> digit(0,15);
    std::string id(36,'-');
    for (int i=0;i<36;++i) if (i!=8 && i!=13 && i!=18 && i!=23) id[i]=hex[digit(generator)];
    id[14]='4'; id[19]=hex[8+(digit(generator)%4)];
    return id;
}

class PersonPassNode : public rclcpp::Node {
public:
    PersonPassNode() : Node("person_pass_node"), config_(loadConfig()), controller_(config_) {
        dry_run_=fixed("dry_run",true);
        confidence_=positive("confidence_threshold",50.0);
        if (confidence_>100) throw std::invalid_argument("Confidence must be in [0,100]");
        ball_timeout_=positive("ball_timeout",1.0);
        person_timeout_=positive("person_timeout",2.5);
        frame_timeout_=positive("frame_timeout",.5);
        recovery_timeout_=positive("recovery_timeout",2.5);
        getup_version_=fixed<int>("getup_version",1);
        if (getup_version_<0 || getup_version_>1) throw std::invalid_argument("getup_version must be 0 or 1");
        odom_scale_=positive("odom_scale",.8);
        command_=create_publisher<booster_msgs::msg::RpcReqMsg>(
            fixed<std::string>("command_topic","/LocoApiTopicReq"),10);
        preview_=create_publisher<std_msgs::msg::String>("/person_pass/preview",10);
        status_=create_publisher<std_msgs::msg::String>("/person_pass/status",rclcpp::QoS(1).transient_local());
        const auto qos=rclcpp::SensorDataQoS();
        recovery_sub_=create_subscription<booster_interface::msg::RawBytesMsg>(
            fixed<std::string>("recovery_topic","/fall_down_recovery_state"),qos,
            [this](const booster_interface::msg::RawBytesMsg& m) {
                if (m.msg.size()<3 || m.msg[0]>3 || m.msg[1]>1) return;
                recovery_=m.msg[0]; available_=m.msg[1]!=0; planner_=m.msg[2]; robot_time_=monotonic();
            });
        odom_sub_=create_subscription<booster_interface::msg::Odometer>(
            fixed<std::string>("odometry_topic","/odometer_state"),qos,
            [this](const booster_interface::msg::Odometer& m) {
                if (!std::isfinite(m.x) || !std::isfinite(m.y) || !std::isfinite(m.theta)) return;
                pose_={m.x*odom_scale_,m.y*odom_scale_,m.theta}; odom_time_=monotonic();
            });
        head_sub_=create_subscription<booster_interface::msg::LowState>(
            fixed<std::string>("low_state_topic","/low_state"),qos,
            [this](const booster_interface::msg::LowState& m) {
                if (m.motor_state_serial.size()<2 || !std::isfinite(m.motor_state_serial[0].q)
                    || !std::isfinite(m.motor_state_serial[1].q)) return;
                head_time_=monotonic();
            });
        vision_sub_=create_subscription<vision_interface::msg::Detections>(
            fixed<std::string>("detections_topic","/booster_soccer/detection"),qos,
            [this](const vision_interface::msg::Detections& m) { detections(m); });
        enable_service_=create_service<std_srvs::srv::SetBool>("/person_pass/enable",
            [this](std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                controller_.enable(request->data);
                if (!request->data) halt();
                response->success=true;
                response->message=request->data ? "Enabled; one attempt per false->true transition" : "Stopped";
            });
        controller_.enable(fixed("auto_start",true));
        timer_=create_wall_timer(50ms,[this] { tick(); });
        RCLCPP_INFO(get_logger(),"Independent person pass; dry_run=%s; no referee or field localization",
                    dry_run_ ? "true" : "false");
    }
    void halt() { request(booster_interface::CreateMoveMsg(0,0,0)); }
private:
    template<class T> T fixed(const std::string& key,T value) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only=true;
        return declare_parameter<T>(key,value,descriptor);
    }
    double positive(const std::string& key,double value) {
        value=fixed(key,value);
        if (!std::isfinite(value) || value<=0) throw std::invalid_argument("Invalid parameter: "+key);
        return value;
    }
    pp::Config loadConfig() {
        pp::Config c;
        c.vx_limit=positive("vx_limit",c.vx_limit); c.vy_limit=positive("vy_limit",c.vy_limit);
        c.turn_limit=positive("turn_limit",c.turn_limit);
        c.min_vx=positive("min_vx",c.min_vx); c.min_vy=positive("min_vy",c.min_vy);
        c.min_turn=positive("min_turn",c.min_turn);
        c.stand_off=positive("stand_off",c.stand_off); c.alignment=positive("alignment",c.alignment);
        c.kick_speed=positive("kick_speed",c.kick_speed); c.kick_duration=positive("kick_duration",c.kick_duration);
        c.robot_timeout=positive("robot_timeout",c.robot_timeout);
        c.startup_timeout=positive("startup_timeout",c.startup_timeout);
        return c;
    }
    static bool fresh(double stamp,double now,double timeout) {
        return stamp>=0 && now>=stamp && now-stamp<=timeout;
    }
    struct Seen { pp::Point odom; double stamp=-1; };
    void detections(const vision_interface::msg::Detections& m) {
        const double t=monotonic();
        const double age=(now()-rclcpp::Time(m.header.stamp,get_clock()->get_clock_type())).seconds();
        if (!std::isfinite(age) || age<0 || age>frame_timeout_ || !fresh(odom_time_,t,config_.robot_timeout)) return;
        double best=-1, nearest=-1; pp::Point person,ball;
        obstacles_.clear(); obstacle_time_=t-age;
        for (const auto& o : m.detected_objects) {
            if (!std::isfinite(o.confidence) || o.confidence<confidence_
                || o.position_projection.size()<2) continue;
            const pp::Point p{o.position_projection[0],o.position_projection[1]};
            if (!pp::finite(p) || p.x<-.5 || pp::length(p)>20.0) continue;
            if (o.label=="Ball" && o.confidence>best) { best=o.confidence; ball=p; }
            if (o.label=="Person" && (nearest<0 || pp::length(p)<nearest)) {
                nearest=pp::length(p); person=p;
            }
            if (o.label=="Person" || o.label=="Opponent") obstacles_.push_back(pp::toOdom(p,pose_));
        }
        if (best>=0) ball_={pp::toOdom(ball,pose_),t-age};
        if (nearest>=0) person_={pp::toOdom(person,pose_),t-age};
    }
    void request(const booster_interface::msg::BoosterApiReqMsg& api) {
        booster_msgs::msg::RpcReqMsg m;
        m.uuid=requestUuid();
        m.header=Json{{"api_id",api.api_id}}.dump(); m.body=api.body;
        if (dry_run_) {
            std_msgs::msg::String s;
            s.data=Json{{"header",Json::parse(m.header)},{"body",m.body}}.dump(); preview_->publish(s);
        } else command_->publish(m);
    }
    void exitAutoKick() {
        request(booster_interface::ConstructMsg(booster::robot::b1::LocoApiId::kVisualKick,
                                               Json{{"start",false},{"version",2}}.dump()));
    }
    void head(double t,const pp::Input& i,pp::Stage stage) {
        if (t-head_command_time_<.2 || !fresh(head_time_,t,config_.robot_timeout)) return;
        if (stage!=pp::Stage::WaitTargets && stage!=pp::Stage::Blocked && stage!=pp::Stage::Chase
            && stage!=pp::Stage::Adjust && stage!=pp::Stage::Align && stage!=pp::Stage::Kick) return;
        double yaw=0, pitch=.35;
        const bool ball=fresh(ball_.stamp,t,ball_timeout_), person=fresh(person_.stamp,t,person_timeout_);
        if (ball && person) {
            const pp::Point target=(stage==pp::Stage::Kick || static_cast<int>(t/.5)%2==0) ? i.ball : i.person;
            yaw=std::atan2(target.y,target.x); pitch=std::atan2(.8,pp::length(target));
        } else {
            const int step=static_cast<int>(t/.7)%3;
            yaw=step==0 ? -1.0 : (step==1 ? 0 : 1.0);
            pitch=ball ? .3 : (person ? .9 : (static_cast<int>(t/2.1)%2 ? .3 : .9));
        }
        request(booster_interface::CreateRotateHeadMsg(std::clamp(pitch,.25,1.1),std::clamp(yaw,-1.0,1.0)));
        head_command_time_=t;
    }
    void tick() {
        const double t=monotonic();
        pp::Input i;
        i.now=t; i.robot_fresh=fresh(robot_time_,t,recovery_timeout_);
        i.recovery=recovery_; i.recovery_available=available_; i.planner=planner_;
        i.ball=pp::toRobot(ball_.odom,pose_); i.person=pp::toRobot(person_.odom,pose_);
        i.scene_valid=fresh(odom_time_,t,config_.robot_timeout) && fresh(head_time_,t,config_.robot_timeout)
            && fresh(ball_.stamp,t,ball_timeout_) && fresh(person_.stamp,t,person_timeout_);
        if (fresh(obstacle_time_,t,frame_timeout_)) for (auto p : obstacles_) {
            p=pp::toRobot(p,pose_);
            if (p.x>0 && pp::length(p)<.6) i.blocked=true;
        }
        const auto out=controller_.tick(i);
        if (out.send_velocity) request(booster_interface::CreateMoveMsg(out.vx,out.vy,out.turn));
        if (out.stage==pp::Stage::Settle && last_stage_!=pp::Stage::Settle) exitAutoKick();
        if (out.action==pp::Action::Prepare)
            request(booster_interface::CreateChangeModeMsg(booster::robot::RobotMode::kPrepare));
        if (out.action==pp::Action::Soccer) {
            request(booster_interface::CreateChangeModeMsg(booster::robot::RobotMode::kSoccer));
            exitAutoKick();
        }
        if (out.action==pp::Action::GetUp)
            request(booster_interface::ConstructMsg(booster::robot::b1::LocoApiId::kGetUp,
                                                   Json{{"version",getup_version_}}.dump()));
        head(t,i,out.stage);
        if (out.stage!=last_stage_ || t-status_time_>=1) {
            std_msgs::msg::String msg;
            msg.data=Json{{"state",pp::name(out.stage)},{"dry_run",dry_run_},
                {"recovery",recovery_},{"recovery_available",available_},{"planner",planner_},
                {"getup_version",getup_version_},{"robot_fresh",i.robot_fresh},
                {"odom_fresh",fresh(odom_time_,t,config_.robot_timeout)},
                {"head_fresh",fresh(head_time_,t,config_.robot_timeout)},
                {"ball_fresh",fresh(ball_.stamp,t,ball_timeout_)},
                {"person_fresh",fresh(person_.stamp,t,person_timeout_)}}.dump();
            status_->publish(msg); status_time_=t;
            if (out.stage!=last_stage_) RCLCPP_INFO(get_logger(),"%s",msg.data.c_str());
            last_stage_=out.stage;
        }
    }
    pp::Config config_;
    pp::Controller controller_;
    bool dry_run_=true, available_=false;
    int recovery_=0, planner_=0, getup_version_=1;
    double confidence_=50, ball_timeout_=1, person_timeout_=2.5, frame_timeout_=.5, odom_scale_=.8;
    double recovery_timeout_=2.5;
    double robot_time_=-1, odom_time_=-1, head_time_=-1, obstacle_time_=-1;
    double head_command_time_=-1, status_time_=-1;
    pp::Pose pose_;
    Seen ball_,person_;
    std::vector<pp::Point> obstacles_;
    pp::Stage last_stage_=pp::Stage::Fault;
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr command_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr preview_,status_;
    rclcpp::Subscription<booster_interface::msg::RawBytesMsg>::SharedPtr recovery_sub_;
    rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr odom_sub_;
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr head_sub_;
    rclcpp::Subscription<vision_interface::msg::Detections>::SharedPtr vision_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

static volatile std::sig_atomic_t running=1;
static void stopSignal(int) { running=0; }
int main(int argc,char** argv) {
    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT,stopSignal); std::signal(SIGTERM,stopSignal);
    std::shared_ptr<PersonPassNode> node;
    int result=0;
    try {
        node=std::make_shared<PersonPassNode>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        while (running && rclcpp::ok()) { executor.spin_some(); std::this_thread::sleep_for(10ms); }
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; result=1; }
    if (node && rclcpp::ok()) { node->halt(); std::this_thread::sleep_for(100ms); }
    rclcpp::shutdown();
    return result;
}
