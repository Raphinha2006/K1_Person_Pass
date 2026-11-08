#include "person_pass/controller.hpp"
#include "person_pass/search.hpp"
#include <booster_interface/message_utils.hpp>
#include <booster_interface/msg/low_state.hpp>
#include <booster_interface/msg/odometer.hpp>
#include <booster_interface/msg/raw_bytes_msg.hpp>
#include <booster_msgs/msg/rpc_req_msg.hpp>
#include <booster_msgs/msg/rpc_resp_msg.hpp>
#include <vision_interface/msg/detections.hpp>
#include <brain/msg/kick.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
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
        ball_memory_timeout_=positive("ball_memory_timeout",2.0);
        person_memory_timeout_=positive("person_memory_timeout",4.0);
        if (ball_memory_timeout_<ball_timeout_ || person_memory_timeout_<person_timeout_)
            throw std::invalid_argument("Memory periods must cover observation freshness periods");
        person_range_=positive("person_max_range",10.0);
        frame_timeout_=positive("frame_timeout",.5);
        recovery_timeout_=positive("recovery_timeout",2.5);
        getup_version_=fixed<int>("getup_version",1);
        if (getup_version_<0 || getup_version_>1) throw std::invalid_argument("getup_version must be 0 or 1");
        kick_version_=fixed<int>("visual_kick_version",1);
        if (kick_version_<0 || kick_version_>1) throw std::invalid_argument("visual_kick_version must be 0 or 1");
        // The firmware's visual-kick "power" is a 1..10 strength, not metres.
        // Scale it with the ball-to-receiver distance; gain/offset are tuned on
        // the real robot because rolling friction differs from the nominal range.
        kick_power_gain_=fixed("kick_power_gain",1.0);
        if (kick_power_gain_<0) throw std::invalid_argument("kick_power_gain must be non-negative");
        kick_power_offset_=fixed("kick_power_offset",0.0);
        kick_power_min_=positive("kick_power_min",1.0);
        kick_power_max_=positive("kick_power_max",10.0);
        if (kick_power_min_<1.0 || kick_power_max_>10.0 || kick_power_min_>kick_power_max_)
            throw std::invalid_argument("Kick power must stay inside the firmware range 1.0..10.0");
        odom_scale_=positive("odom_scale",.8);
        command_=create_publisher<booster_msgs::msg::RpcReqMsg>(
            fixed<std::string>("command_topic","/LocoApiTopicReq"),10);
        preview_=create_publisher<std_msgs::msg::String>("/person_pass/preview",10);
        kick_target_=create_publisher<brain::msg::Kick>(
            dry_run_ ? "/person_pass/kick_preview" : "/kick_ball",10);
        status_=create_publisher<std_msgs::msg::String>("/person_pass/status",rclcpp::QoS(1).transient_local());
        const auto qos=rclcpp::SensorDataQoS();
        response_sub_=create_subscription<booster_msgs::msg::RpcRespMsg>(
            "/LocoApiTopicResp",10,[this](const booster_msgs::msg::RpcRespMsg& m) {
                if (m.uuid!=mode_uuid_ && m.uuid!=soccer_uuid_ && m.uuid!=kick_start_uuid_
                    && m.uuid!=kick_stop_uuid_) return;
                const auto h=Json::parse(m.header,nullptr,false);
                if (h.is_discarded() || !h.is_object()) return;
                const auto status=h.value("status",-1);
                if (m.uuid==mode_uuid_) {
                    const auto body=Json::parse(m.body,nullptr,false);
                    mode_value_=body.is_object() ? body.value("mode",-1) : -1;
                    mode_known_=status==0 && mode_value_>=0;
                    if (mode_value_==4) soccer_ready_=true;
                    RCLCPP_INFO(get_logger(),"GET_MODE_RESPONSE status=%d mode=%d",status,mode_value_);
                    return;
                }
                if (m.uuid==soccer_uuid_) {
                    soccer_ready_=status==0;
                    // 501: already switched, or the robot is in a sub-mode that
                    // refuses the change. Re-read the mode instead of retrying blindly.
                    if (status!=0) mode_known_=false;
                    RCLCPP_INFO(get_logger(),"SOCCER_MODE_RESPONSE status=%d",status);
                    return;
                }
                const bool start=m.uuid==kick_start_uuid_;
                RCLCPP_INFO(get_logger(),"VISUAL_KICK_RESPONSE start=%s status=%d",
                            start ? "true" : "false",status);
                if (start && status!=0) { controller_.enable(false); halt(); }
            });
        recovery_sub_=create_subscription<booster_interface::msg::RawBytesMsg>(
            fixed<std::string>("recovery_topic","/fall_down_recovery_state"),qos,
            [this](const booster_interface::msg::RawBytesMsg& m) {
                if (m.msg.size()<3 || m.msg[0]>3 || m.msg[1]>1) return;
                if (visual_kick_active_ && planner_!=m.msg[2])
                    RCLCPP_INFO(get_logger(),"VISUAL_KICK_FEEDBACK planner=%u recovery=%u",m.msg[2],m.msg[0]);
                recovery_=m.msg[0]; available_=m.msg[1]!=0; planner_=m.msg[2]; robot_time_=monotonic();
                if (recovery_!=0) soccer_ready_=false;
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
                search_.reset();
                if (!request->data) halt();
                response->success=true;
                response->message=request->data ? "Enabled; one attempt per false->true transition" : "Stopped";
            });
        mode_uuid_=request(booster_interface::ConstructMsg(booster::robot::b1::LocoApiId::kGetMode,"{}"));
        mode_query_time_=monotonic();
        controller_.enable(fixed("auto_start",true));
        timer_=create_wall_timer(50ms,[this] { tick(); });
        RCLCPP_INFO(get_logger(),"Independent person pass; dry_run=%s; no referee or field localization",
                    dry_run_ ? "true" : "false");
    }
    void halt() {
        if (visual_kick_active_) visualKick(false);
        request(booster_interface::CreateMoveMsg(0,0,0));
    }
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
        c.kick_duration=positive("kick_duration",c.kick_duration);
        c.kick_cooldown=positive("kick_cooldown",c.kick_cooldown);
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
            if (o.label=="Person" && pp::length(p)<=person_range_
                && (nearest<0 || pp::length(p)<nearest)) {
                nearest=pp::length(p); person=p;
            }
            if (o.label=="Person" || o.label=="Opponent") obstacles_.push_back(pp::toOdom(p,pose_));
        }
        if (best>=0) ball_={pp::toOdom(ball,pose_),t-age};
        if (nearest>=0) {
            const double stamp=t-age;
            pp::Point odom=pp::toOdom(person,pose_);
            if (person_.stamp>=0 && stamp-person_.stamp<=.5) {
                const double jump=pp::length({odom.x-person_.odom.x,odom.y-person_.odom.y});
                if (jump>1.5 && ++person_rejects_<3) {
                    // Rejected detections must not renew observation or memory.
                } else {
                    const double a=.3;
                    odom.x=person_.odom.x+a*(odom.x-person_.odom.x);
                    odom.y=person_.odom.y+a*(odom.y-person_.odom.y);
                    person_={odom,stamp}; person_rejects_=0;
                }
            } else {
                person_={odom,stamp}; person_rejects_=0;
            }
        }
    }
    std::string request(const booster_interface::msg::BoosterApiReqMsg& api) {
        booster_msgs::msg::RpcReqMsg m;
        m.uuid=requestUuid();
        m.header=Json{{"api_id",api.api_id}}.dump(); m.body=api.body;
        if (dry_run_) {
            std_msgs::msg::String s;
            s.data=Json{{"uuid",m.uuid},{"header",Json::parse(m.header)},{"body",m.body}}.dump(); preview_->publish(s);
        } else command_->publish(m);
        return m.uuid;
    }
    void visualKick(bool start) {
        const auto id=request(booster_interface::ConstructMsg(booster::robot::b1::LocoApiId::kVisualKick,
                                               Json{{"start",start},{"version",kick_version_}}.dump()));
        (start ? kick_start_uuid_ : kick_stop_uuid_)=id;
        visual_kick_active_=start;
        RCLCPP_INFO(get_logger(),"VISUAL_KICK_REQUEST start=%s version=%d dry_run=%s",
                    start ? "true" : "false",kick_version_,dry_run_ ? "true" : "false");
    }
    void kickTarget(const pp::Input& i) {
        brain::msg::Kick m;
        m.header.stamp=now();
        m.x=i.ball.x; m.y=i.ball.y;
        // Refresh the receiver every frame: the native skill can take seconds to
        // swing, so a latched target would aim at where the person used to be.
        m.goal_x=i.person.x; m.goal_y=i.person.y;
        m.dir=std::atan2(i.person.y-i.ball.y,i.person.x-i.ball.x);
        // Local odometry is the reference; field localization is not required.
        m.robot_theta_to_field=pose_.yaw;
        m.power=kick_power_;
        kick_target_->publish(m);
    }
    void head(double t,const pp::Input& i,pp::Stage stage,const pp::SearchOutput& search) {
        if (t-head_command_time_<.2 || !fresh(head_time_,t,config_.robot_timeout)) return;
        if (search.head) {
            request(booster_interface::CreateRotateHeadMsg(search.pitch,search.yaw));
            head_command_time_=t;
            return;
        }
        if (stage!=pp::Stage::WaitTargets && stage!=pp::Stage::Blocked
            && stage!=pp::Stage::Chase && stage!=pp::Stage::Adjust
            && stage!=pp::Stage::Align && stage!=pp::Stage::Kick) return;
        double yaw=0, pitch=.35;
        const bool ball=fresh(ball_.stamp,t,ball_timeout_);
        const bool person=fresh(person_.stamp,t,person_timeout_);
        if (ball || person) {
            // Keep both targets in view: staring at the ball loses the receiver
            // standing to the side, which then blocks the pass handoff.
            const pp::Point target=(ball && person)
                ? ((stage==pp::Stage::Chase || stage==pp::Stage::Adjust
                    || static_cast<int>(t/.5)%2==0) ? i.ball : i.person)
                : (ball ? i.ball : i.person);
            yaw=std::atan2(target.y,target.x); pitch=std::atan2(.8,pp::length(target));
        } else {
            const int step=static_cast<int>(t/.7)%3;
            yaw=step==0 ? -1.0 : (step==1 ? 0 : 1.0);
            pitch=static_cast<int>(t/2.1)%2 ? .3 : .9;
        }
        request(booster_interface::CreateRotateHeadMsg(std::clamp(pitch,.25,1.1),std::clamp(yaw,-1.0,1.0)));
        head_command_time_=t;
    }
    void tick() {
        const double t=monotonic();
        pp::Input i;
        i.now=t; i.robot_fresh=fresh(robot_time_,t,recovery_timeout_);
        i.recovery=recovery_; i.recovery_available=available_; i.planner=planner_;
        i.soccer_ready=soccer_ready_;
        i.ball=pp::toRobot(ball_.odom,pose_); i.person=pp::toRobot(person_.odom,pose_);
        i.ball_fresh=fresh(ball_.stamp,t,ball_timeout_);
        i.person_fresh=fresh(person_.stamp,t,person_timeout_);
        const bool ball_memory=fresh(ball_.stamp,t,ball_memory_timeout_);
        const bool person_memory=fresh(person_.stamp,t,person_memory_timeout_);
        i.scene_valid=fresh(odom_time_,t,config_.robot_timeout) && fresh(head_time_,t,config_.robot_timeout)
            && ball_memory && person_memory;
        if (fresh(obstacle_time_,t,frame_timeout_)) for (auto p : obstacles_) {
            p=pp::toRobot(p,pose_);
            if (p.x>0 && pp::length(p)<.6) i.blocked=true;
        }
        if (!soccer_ready_ && (t-mode_query_time_>1.0)) {
            mode_uuid_=request(booster_interface::ConstructMsg(booster::robot::b1::LocoApiId::kGetMode,"{}"));
            mode_query_time_=t;
        }
        auto out=controller_.tick(i);
        const bool search_allowed=(out.stage==pp::Stage::WaitTargets || out.stage==pp::Stage::Blocked
            || out.stage==pp::Stage::Chase || out.stage==pp::Stage::Adjust)
            && i.robot_fresh && i.recovery==0 && i.soccer_ready
            && fresh(odom_time_,t,config_.robot_timeout) && fresh(head_time_,t,config_.robot_timeout);
        const auto search=search_.tick(t,search_allowed,
            i.ball_fresh,i.person_fresh,
            ball_.stamp>=0 ? i.ball : pp::Point{},person_.stamp>=0 ? i.person : pp::Point{},
            !i.blocked && !controller_.coolingDown(t),config_.turn_limit,
            ball_memory,person_memory,out.stage==pp::Stage::WaitTargets && !controller_.coolingDown(t));
        // Stop on reacquisition before allowing approach on the following tick.
        // Kick never receives search Move commands; its native skill owns motion.
        if (search.head || (search.stop && out.stage!=pp::Stage::Kick)) {
            out.send_velocity=true; out.vx=out.vy=0;
            out.turn=search.stop ? 0 : search.turn;
        }
        if (visual_kick_active_ && out.stage!=pp::Stage::Kick) visualKick(false);
        if (out.send_velocity) request(booster_interface::CreateMoveMsg(out.vx,out.vy,out.turn));
        if (out.stage==pp::Stage::Settle && last_stage_!=pp::Stage::Settle) visualKick(false);
        if (out.action==pp::Action::StartKick) {
            kick_ball_start_=pp::toOdom(i.ball,pose_);
            kick_target_distance_=pp::length({i.person.x-i.ball.x,i.person.y-i.ball.y});
            kick_travel_=0; kick_track_=true;
            // Proportional range controller: power maps to the requested kick
            // range, so scale it with the ball-to-receiver distance. The 1.2
            // gain comes from the field measurement "power 6 lands at ~5 m".
            kick_power_=std::clamp(kick_power_offset_+kick_power_gain_*kick_target_distance_,
                                   kick_power_min_,kick_power_max_);
            RCLCPP_INFO(get_logger(),"KICK_POWER distance=%.2f power=%.2f gain=%.2f",
                        kick_target_distance_,kick_power_,kick_power_gain_);
        }
        if (kick_track_ && out.stage==pp::Stage::Kick) {
            if (fresh(ball_.stamp,t,ball_timeout_))
                kick_travel_=std::max(kick_travel_,pp::length({ball_.odom.x-kick_ball_start_.x,
                                                               ball_.odom.y-kick_ball_start_.y}));
        }
        if (last_stage_==pp::Stage::Kick && out.stage!=pp::Stage::Kick && kick_track_) {
            kick_track_=false;
            RCLCPP_INFO(get_logger(),"KICK_TRAVEL target=%.2f measured=%.2f power=%.2f",
                        kick_target_distance_,kick_travel_,kick_power_);
        }
        if (out.stage==pp::Stage::Kick) kickTarget(i);
        if (out.action==pp::Action::StartKick) visualKick(true);
        if (out.action==pp::Action::Prepare) {
            soccer_ready_=false;
            request(booster_interface::CreateChangeModeMsg(booster::robot::RobotMode::kPrepare));
        }
        if (out.action==pp::Action::Soccer) {
            if (mode_known_ && mode_value_==4) soccer_ready_=true;
            else soccer_uuid_=request(booster_interface::CreateChangeModeMsg(booster::robot::RobotMode::kSoccer));
        }
        if (out.action==pp::Action::GetUp)
            request(booster_interface::ConstructMsg(booster::robot::b1::LocoApiId::kGetUp,
                                                   Json{{"version",getup_version_}}.dump()));
        head(t,i,out.stage,search);
        if (out.stage!=last_stage_ || t-status_time_>=1) {
            std_msgs::msg::String msg;
            msg.data=Json{{"state",pp::name(out.stage)},{"dry_run",dry_run_},
                {"recovery",recovery_},{"recovery_available",available_},{"planner",planner_},
                {"getup_version",getup_version_},{"robot_fresh",i.robot_fresh},
                {"odom_fresh",fresh(odom_time_,t,config_.robot_timeout)},
                {"head_fresh",fresh(head_time_,t,config_.robot_timeout)},
                {"ball_fresh",fresh(ball_.stamp,t,ball_timeout_)},
                {"person_fresh",fresh(person_.stamp,t,person_timeout_)},
                {"search_target",search.target==pp::SearchTarget::Ball ? "ball"
                    : (search.target==pp::SearchTarget::Person ? "person" : "none")},
                {"search_turn",search.head ? out.turn : 0.0},
                {"ball_age",ball_.stamp>=0 ? t-ball_.stamp : -1.0},
                {"person_age",person_.stamp>=0 ? t-person_.stamp : -1.0},
                {"ball_memory",ball_memory},{"person_memory",person_memory},
                {"search_reason",search.target==pp::SearchTarget::Ball ? "ball_not_fresh"
                    : (search.target==pp::SearchTarget::Person
                        ? (person_memory ? "refresh_before_kick" : "person_memory_expired") : "none")},
                {"ball_x",i.ball.x},{"ball_y",i.ball.y},
                {"person_x",i.person.x},{"person_y",i.person.y},
                {"kick_power",kick_power_}}.dump();
            status_->publish(msg); status_time_=t;
            if (out.stage!=last_stage_) RCLCPP_INFO(get_logger(),"%s",msg.data.c_str());
            last_stage_=out.stage;
        }
    }
    pp::Config config_;
    pp::Controller controller_;
    pp::TargetSearch search_;
    bool dry_run_=true, available_=false, visual_kick_active_=false, soccer_ready_=false;
    bool mode_known_=false;
    int mode_value_=-1;
    int recovery_=0, planner_=0, getup_version_=1, kick_version_=1;
    pp::Point kick_ball_start_{};
    bool kick_track_=false;
    int person_rejects_=0;
    double kick_target_distance_=0, kick_travel_=0;
    double kick_power_=1.5, kick_power_gain_=1.0, kick_power_offset_=0.0;
    double kick_power_min_=1.0, kick_power_max_=10.0;
    std::string kick_start_uuid_,kick_stop_uuid_,soccer_uuid_,mode_uuid_;
    double confidence_=50, ball_timeout_=1, person_timeout_=2.5, person_range_=10.0;
    double frame_timeout_=.5, odom_scale_=.8;
    double ball_memory_timeout_=2, person_memory_timeout_=4;
    double recovery_timeout_=2.5;
    double robot_time_=-1, odom_time_=-1, head_time_=-1, obstacle_time_=-1;
    double head_command_time_=-1, status_time_=-1, mode_query_time_=-1;
    pp::Pose pose_;
    Seen ball_,person_;
    std::vector<pp::Point> obstacles_;
    pp::Stage last_stage_=pp::Stage::Fault;
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr command_;
    rclcpp::Publisher<brain::msg::Kick>::SharedPtr kick_target_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr preview_,status_;
    rclcpp::Subscription<booster_interface::msg::RawBytesMsg>::SharedPtr recovery_sub_;
    rclcpp::Subscription<booster_msgs::msg::RpcRespMsg>::SharedPtr response_sub_;
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
