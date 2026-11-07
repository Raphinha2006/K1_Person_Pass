#include <rclcpp/rclcpp.hpp>
#include <booster_interface/msg/raw_bytes_msg.hpp>
#include <booster_interface/msg/low_state.hpp>
#include <booster_interface/msg/odometer.hpp>
#include <booster_msgs/msg/rpc_req_msg.hpp>
#include <vision_interface/msg/detections.hpp>
#include <std_msgs/msg/string.hpp>
#include <booster/third_party/nlohmann_json/json.hpp>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>
extern char** environ;
using Json=nlohmann::json;
using Clock=std::chrono::steady_clock;
static void check(bool condition,const char* message) {
    if (!condition) throw std::runtime_error(message);
}
static void stopChild(pid_t child) {
    if (child<=0) return;
    kill(child,SIGINT);
    for (int k=0;k<100;++k) {
        if (waitpid(child,nullptr,WNOHANG)==child) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    kill(child,SIGKILL); waitpid(child,nullptr,0);
}
static void run(const std::string& binary,bool preview_only) {
    setenv("ROS_DOMAIN_ID","83",1);
    std::vector<std::string> args={binary,"--ros-args","-p",
        std::string("dry_run:=")+(preview_only ? "true" : "false"),"-p","kick_duration:=4.0","-r",
        "/LocoApiTopicReq:=/person_pass_test/requests"};
    std::vector<char*> argv; for (auto& s : args) argv.push_back(s.data()); argv.push_back(nullptr);
    pid_t child=-1;
    check(posix_spawn(&child,binary.c_str(),nullptr,nullptr,argv.data(),environ)==0,"Spawn test node");
    try {
        rclcpp::init(0,nullptr);
        auto node=std::make_shared<rclcpp::Node>("person_pass_adapter_test");
        auto recovery=node->create_publisher<booster_interface::msg::RawBytesMsg>("/fall_down_recovery_state",10);
        auto odom=node->create_publisher<booster_interface::msg::Odometer>("/odometer_state",10);
        auto head=node->create_publisher<booster_interface::msg::LowState>("/low_state",10);
        auto vision=node->create_publisher<vision_interface::msg::Detections>("/booster_soccer/detection",10);
        int hardware_messages=0, getups=0, soccer=0, nonzero=0, auto_kick_exits=0;
        double last_vx=0,last_vy=0,last_turn=0;
        std::string stage;
        auto record=[&](const Json& header,const std::string& text) {
            const int api=header.at("api_id");
            Json body=text.empty() ? Json() : Json::parse(text);
            if (api==2008) ++getups;
            if (api==2000 && body.at("mode")==4) ++soccer;
            if (api==2038) {
                check(body.at("start")==false && body.at("version")==2,"Only exit automatic visual kick");
                ++auto_kick_exits;
            }
            if (api==2001) {
                last_vx=body.at("vx"); last_vy=body.at("vy"); last_turn=body.at("vyaw");
                check(std::isfinite(last_vx) && std::abs(last_vx)<=.40001,"Forward speed limit");
                check(std::isfinite(last_vy) && std::abs(last_vy)<=.30001,"Lateral speed limit");
                check(std::isfinite(last_turn) && std::abs(last_turn)<=.60001,"Turn speed limit");
                if (std::abs(last_vx)+std::abs(last_vy)+std::abs(last_turn)>1e-5) ++nonzero;
            }
        };
        auto rpc=node->create_subscription<booster_msgs::msg::RpcReqMsg>("/person_pass_test/requests",100,
            [&](const booster_msgs::msg::RpcReqMsg& m) {
                ++hardware_messages;
                check(m.uuid.size()==36 && m.uuid[14]=='4',"Canonical request UUID");
                record(Json::parse(m.header),m.body);
            });
        auto preview=node->create_subscription<std_msgs::msg::String>("/person_pass/preview",100,
            [&](const std_msgs::msg::String& m) { auto j=Json::parse(m.data); record(j.at("header"),j.at("body")); });
        auto status=node->create_subscription<std_msgs::msg::String>("/person_pass/status",rclcpp::QoS(1).transient_local(),
            [&](const std_msgs::msg::String& m) { stage=Json::parse(m.data).at("state"); });
        auto deadline=Clock::now()+std::chrono::seconds(8);
        while ((recovery->get_subscription_count()==0 || preview->get_publisher_count()==0
                || rpc->get_publisher_count()==0) && Clock::now()<deadline) {
            rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        check(recovery->get_subscription_count()>0 && rpc->get_publisher_count()>0,"DDS discovery");
        auto sample=[&](int state,int planner,double seconds,bool observations=true,bool feedback=true) {
            const auto end=Clock::now()+std::chrono::duration<double>(seconds);
            while (Clock::now()<end) {
                if (feedback) {
                    booster_interface::msg::RawBytesMsg r; r.msg={static_cast<uint8_t>(state),1,static_cast<uint8_t>(planner)};
                    recovery->publish(r);
                }
                booster_interface::msg::Odometer o; odom->publish(o);
                booster_interface::msg::LowState h; h.motor_state_serial.resize(2); head->publish(h);
                vision_interface::msg::Detections d; d.header.stamp=node->now();
                if (observations) for (int k=0;k<2;++k) {
                    vision_interface::msg::DetectedObject object;
                    object.label=k ? "Person" : "Ball"; object.confidence=90;
                    object.position_projection={k ? 2.5F : .5F,0.F,0.F};
                    d.detected_objects.push_back(object);
                }
                vision->publish(d); rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
            }
        };
        sample(2,1,1.2); check(getups>0,"Get-up request serialization");
        const int moving_before=nonzero;
        sample(3,1,.4); check(nonzero==moving_before,"No walking during get-up");
        sample(0,2,1.2); check(soccer>0,"Soccer mode serialization");
        const int ready_planner=preview_only ? 20 : 8;
        const int exits_before=auto_kick_exits;
        sample(0,ready_planner,1.8); check(nonzero>moving_before,"Aligned pass must produce a movement request");
        check(auto_kick_exits>exits_before,"Exit previous visual kick even when already in a ready mode");
        sample(0,ready_planner,1.2,false);
        check(stage=="DONE" && last_vx==0 && last_vy==0 && last_turn==0,"Expired ball must finish and stop");
        const int finished_count=nonzero;
        sample(0,ready_planner,.5); check(nonzero==finished_count,"No second pass after completion");
        sample(0,ready_planner,1.2,true,false);
        check(stage=="WAIT_ROBOT" && last_vx==0,"Expired robot feedback must stop");
        check(preview_only ? hardware_messages==0 : hardware_messages>0,"Dry-run interception / remapped output");
        stopChild(child); child=-1;
        rclcpp::shutdown();
        std::cout << "PASS ROS adapter: " << (preview_only ? "dry-run intercepts every RPC" : "RPC output remapped to isolated test topic") << '\n';
    } catch (...) {
        stopChild(child); if (rclcpp::ok()) rclcpp::shutdown(); throw;
    }
}
int main(int argc,char** argv) {
    try {
        check(argc==2,"Expected node executable path");
        run(argv[1],true); run(argv[1],false); return 0;
    } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
