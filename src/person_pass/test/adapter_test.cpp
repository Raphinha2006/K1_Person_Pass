#include <rclcpp/rclcpp.hpp>
#include <booster_interface/msg/raw_bytes_msg.hpp>
#include <booster_interface/msg/low_state.hpp>
#include <booster_interface/msg/odometer.hpp>
#include <booster_msgs/msg/rpc_req_msg.hpp>
#include <booster_msgs/msg/rpc_resp_msg.hpp>
#include <vision_interface/msg/detections.hpp>
#include <brain/msg/kick.hpp>
#include <std_srvs/srv/set_bool.hpp>
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
        std::string("dry_run:=")+(preview_only ? "true" : "false"),"-p","kick_duration:=4.0","-p","kick_cooldown:=60.0","-r",
        "/LocoApiTopicReq:=/person_pass_test/requests","-r","/kick_ball:=/person_pass_test/kick_ball",
        "-r","/LocoApiTopicResp:=/person_pass_test/responses"};
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
        auto responses=node->create_publisher<booster_msgs::msg::RpcRespMsg>("/person_pass_test/responses",10);
        int hardware_messages=0, getups=0, soccer=0, nonzero=0, auto_kick_exits=0, observed_messages=0;
        int auto_kick_starts=0, target_messages=0, hardware_targets=0, move_requests=0;
        double last_vx=0,last_vy=0,last_turn=0;
        std::string stage;
        std::string last_kick_uuid;
        auto record=[&](const Json& header,const std::string& text) {
            ++observed_messages;
            const int api=header.at("api_id");
            Json body=text.empty() ? Json() : Json::parse(text);
            if (api==2008) {
                check(body.at("version")==1,"Use the tested BMM get-up implementation");
                ++getups;
            }
            if (api==2000 && body.at("mode")==4) ++soccer;
            if (api==2038) {
                check(body.at("version")==1,"Use valid SDK VisualKickVersion::kV2 wire value 1");
                if (body.at("start")==true) ++auto_kick_starts;
                else ++auto_kick_exits;
            }
            if (api==2001) {
                ++move_requests;
                last_vx=body.at("vx"); last_vy=body.at("vy"); last_turn=body.at("vyaw");
                check(std::isfinite(last_vx) && std::abs(last_vx)<=.40001,"Forward speed limit");
                check(std::isfinite(last_vy) && std::abs(last_vy)<=.30001,"Lateral speed limit");
                check(std::isfinite(last_turn) && std::abs(last_turn)<=.60001,"Turn speed limit");
                if (std::abs(last_vx)+std::abs(last_vy)+std::abs(last_turn)>1e-5) ++nonzero;
            }
        };
        auto respond=[&](const std::string& uuid,const Json& header,const std::string& body) {
            const int api=header.at("api_id");
            if (api==2017) {
                booster_msgs::msg::RpcRespMsg response;
                response.uuid=uuid; response.header=Json{{"status",0}}.dump();
                response.body=Json{{"mode",soccer>0 ? 4 : 2}}.dump();
                responses->publish(response);
                return;
            }
            const bool kick=api==2038 && Json::parse(body).at("start")==true;
            const bool mode=api==2000 && Json::parse(body).at("mode")==4;
            if (kick || mode) {
                if (kick) last_kick_uuid=uuid;
                booster_msgs::msg::RpcRespMsg response;
                response.uuid=uuid; response.header=Json{{"status",0}}.dump();
                responses->publish(response);
            }
        };
        auto rpc=node->create_subscription<booster_msgs::msg::RpcReqMsg>("/person_pass_test/requests",100,
            [&](const booster_msgs::msg::RpcReqMsg& m) {
                ++hardware_messages;
                check(m.uuid.size()==36 && m.uuid[14]=='4',"Canonical request UUID");
                record(Json::parse(m.header),m.body);
                respond(m.uuid,Json::parse(m.header),m.body);
            });
        auto preview=node->create_subscription<std_msgs::msg::String>("/person_pass/preview",100,
            [&](const std_msgs::msg::String& m) {
                auto j=Json::parse(m.data); record(j.at("header"),j.at("body"));
                respond(j.at("uuid"),j.at("header"),j.at("body"));
            });
        auto target_record=[&](const brain::msg::Kick& m) {
            ++target_messages;
            check(std::abs(m.x-.5)<1e-5 && std::abs(m.y-.05)<1e-5,"Native skill receives ball coordinates");
            check(std::abs(m.goal_x-2.5)<1e-5 && std::abs(m.goal_y-.3)<1e-5
                  && std::abs(m.dir-std::atan2(.25,2.0))<1e-5,
                  "Native goal and direction select nearest person, never the farther person");
            check(std::abs(m.power-std::hypot(2.0,.25))<1e-5,
                  "Scale firmware power with the ball-to-receiver distance");
        };
        auto kick_target=node->create_subscription<brain::msg::Kick>("/person_pass_test/kick_ball",100,
            [&](const brain::msg::Kick& m) { ++hardware_targets; target_record(m); });
        auto kick_preview=node->create_subscription<brain::msg::Kick>("/person_pass/kick_preview",100,target_record);
        auto enable=node->create_client<std_srvs::srv::SetBool>("/person_pass/enable");
        auto status=node->create_subscription<std_msgs::msg::String>("/person_pass/status",rclcpp::QoS(1).transient_local(),
            [&](const std_msgs::msg::String& m) { stage=Json::parse(m.data).at("state"); });
        auto deadline=Clock::now()+std::chrono::seconds(8);
        while ((recovery->get_subscription_count()==0 || preview->get_publisher_count()==0
                || rpc->get_publisher_count()==0 || observed_messages==0) && Clock::now()<deadline) {
            rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        check(recovery->get_subscription_count()>0 && rpc->get_publisher_count()>0
              && observed_messages>0,"DDS discovery and request-channel delivery");
        auto feedback_time=Clock::now()-std::chrono::seconds(2);
        int last_state=-1,last_planner=-1;
        bool reverse_people=false;
        auto sample=[&](int state,int planner,double seconds,bool observations=true,
                        bool feedback=true,bool multiple=false,bool person=true) {
            const auto end=Clock::now()+std::chrono::duration<double>(seconds);
            while (Clock::now()<end) {
                if (feedback && (state!=last_state || planner!=last_planner
                    || Clock::now()-feedback_time>=std::chrono::seconds(1))) {
                    booster_interface::msg::RawBytesMsg r; r.msg={static_cast<uint8_t>(state),1,static_cast<uint8_t>(planner)};
                    recovery->publish(r);
                    feedback_time=Clock::now(); last_state=state; last_planner=planner;
                }
                booster_interface::msg::Odometer o; odom->publish(o);
                booster_interface::msg::LowState h; h.motor_state_serial.resize(2); head->publish(h);
                vision_interface::msg::Detections d; d.header.stamp=node->now();
                if (observations) for (int k=0;k<(person ? 2 : 1);++k) {
                    vision_interface::msg::DetectedObject object;
                    object.label=k ? "Person" : "Ball"; object.confidence=90;
                    object.position_projection={k ? 2.5F : .5F,k ? .3F : .05F,0.F};
                    d.detected_objects.push_back(object);
                }
                if (observations && multiple) {
                    vision_interface::msg::DetectedObject farther;
                    farther.label="Person"; farther.confidence=95;
                    farther.position_projection={4.F,2.F,0.F};
                    if (reverse_people) d.detected_objects.insert(d.detected_objects.begin()+1,farther);
                    else d.detected_objects.push_back(farther);
                    reverse_people=!reverse_people;
                }
                vision->publish(d); rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
            }
        };
        sample(2,1,1.2,true,true,false,false); check(getups>0,"Get-up request serialization");
        const int moving_before=nonzero;
        sample(2,23,.4,true,true,false,false); check(nonzero==moving_before,"No walking during native BMM get-up");
        sample(0,2,1.2,true,true,false,false); check(soccer>0,"Soccer mode serialization");
        const int ready_planner=preview_only ? 20 : 8;
        const int exits_before=auto_kick_exits;
        sample(0,ready_planner,1.0,true,true,false,false);
        check(stage=="WAIT_TARGETS" && auto_kick_starts==0,"Ball without a receiver must not start a pass");
        sample(0,ready_planner,1.8,true,true,true);
        check(nonzero==0,"Do not push the ball with nonzero Move requests");
        check(stage=="KICK" && auto_kick_starts==1 && target_messages>0,
              "Start native skill once and stream the nearest receiver target");
        check(auto_kick_exits>exits_before,"Exit previous visual kick even when already in a ready mode");
        const int handoff_moves=move_requests;
        sample(0,12,.4);
        check(stage=="KICK" && move_requests==handoff_moves && auto_kick_starts==1,
              "Accept native planner 12 without issuing Move or restarting the skill");
        const int exits_during=auto_kick_exits;
        sample(0,12,1.2,false);
        check(stage=="KICK" && auto_kick_exits==exits_during,
              "Target loss must not cancel the native swing");
        sample(0,12,2.6,false);
        check(stage=="WAIT_TARGETS" && last_vx==0 && last_vy==0 && last_turn==0,
              "Skill must end on its own timeout and return to waiting");
        check(auto_kick_exits==exits_during+1,"Timeout explicitly exits the native skill");
        for (bool value : {false,true}) {
            auto req=std::make_shared<std_srvs::srv::SetBool::Request>(); req->data=value;
            auto future=enable->async_send_request(req);
            check(rclcpp::spin_until_future_complete(node,future,std::chrono::seconds(2))
                  ==rclcpp::FutureReturnCode::SUCCESS,"Enable service response");
        }
        sample(0,ready_planner,1.1);
        check(auto_kick_starts>=2 && stage=="KICK","Explicit reset starts a new native attempt");
        const int exits_after_loss=auto_kick_exits;
        auto disable=std::make_shared<std_srvs::srv::SetBool::Request>(); disable->data=false;
        auto stopped=enable->async_send_request(disable);
        check(rclcpp::spin_until_future_complete(node,stopped,std::chrono::seconds(2))
              ==rclcpp::FutureReturnCode::SUCCESS,"Cancel service response");
        sample(0,ready_planner,.2);
        check(stage=="DISABLED" && auto_kick_exits>exits_after_loss && last_vx==0,
              "Cancel exits native skill immediately and stops");
        auto reenable=std::make_shared<std_srvs::srv::SetBool::Request>(); reenable->data=true;
        auto restarted=enable->async_send_request(reenable);
        check(rclcpp::spin_until_future_complete(node,restarted,std::chrono::seconds(2))
              ==rclcpp::FutureReturnCode::SUCCESS,"Re-enable service response");
        sample(0,ready_planner,1.1);
        check(stage=="KICK","Native skill active before stale-feedback test");
        sample(0,ready_planner,2.8,true,false);
        check(stage=="WAIT_ROBOT" && last_vx==0,"Expired robot feedback must stop");
        for (bool value : {false,true}) {
            auto req=std::make_shared<std_srvs::srv::SetBool::Request>(); req->data=value;
            auto future=enable->async_send_request(req);
            check(rclcpp::spin_until_future_complete(node,future,std::chrono::seconds(2))
                  ==rclcpp::FutureReturnCode::SUCCESS,"Reset before rejected-request check");
        }
        sample(0,ready_planner,1.1);
        check(stage=="KICK" && auto_kick_starts>=4,"New request before simulated firmware rejection");
        booster_msgs::msg::RpcRespMsg rejected;
        rejected.uuid=last_kick_uuid; rejected.header=Json{{"status",502}}.dump();
        responses->publish(rejected);
        sample(0,ready_planner,.3);
        check(stage=="DISABLED" && last_vx==0,"Firmware refusal must disable and exit the skill");
        check(preview_only ? hardware_messages==0 : hardware_messages>0,"Dry-run interception / remapped output");
        check(preview_only ? hardware_targets==0 : hardware_targets>0,"Dry-run never feeds targets to the hardware skill");
        stopChild(child); child=-1;
        rclcpp::shutdown();
        std::cout << "PASS ROS adapter with 1 Hz recovery feedback: " << (preview_only ? "dry-run intercepts every RPC" : "RPC output remapped to isolated test topic") << '\n';
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
