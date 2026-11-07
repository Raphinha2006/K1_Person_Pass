#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace person_pass {
struct Point { double x = 0, y = 0; };
inline double length(Point p) { return std::hypot(p.x, p.y); }
inline bool finite(Point p) { return std::isfinite(p.x) && std::isfinite(p.y); }
inline double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
inline bool recoveryPlanner(int planner) { return planner==10 || planner==23; }
struct Pose { double x = 0, y = 0, yaw = 0; };
inline Point toOdom(Point p, Pose o) {
    return {o.x + std::cos(o.yaw)*p.x - std::sin(o.yaw)*p.y,
            o.y + std::sin(o.yaw)*p.x + std::cos(o.yaw)*p.y};
}
inline Point toRobot(Point p, Pose o) {
    const double x = p.x-o.x, y = p.y-o.y;
    return {std::cos(o.yaw)*x + std::sin(o.yaw)*y,
            -std::sin(o.yaw)*x + std::cos(o.yaw)*y};
}
struct Config {
    double kick_duration = 7.0, robot_timeout = .8, startup_timeout = 30;
};
enum class Stage { Disabled, WaitRobot, Initialize, Recover, Settle, WaitTargets,
                   Blocked, Kick, Done, Fault };
inline const char* name(Stage s) {
    switch (s) {
    case Stage::Disabled: return "DISABLED"; case Stage::WaitRobot: return "WAIT_ROBOT";
    case Stage::Initialize: return "INITIALIZE"; case Stage::Recover: return "RECOVER";
    case Stage::Settle: return "SETTLE"; case Stage::WaitTargets: return "WAIT_TARGETS";
    case Stage::Blocked: return "BLOCKED";
    case Stage::Kick: return "KICK"; case Stage::Done: return "DONE";
    case Stage::Fault: return "FAULT";
    }
    return "UNKNOWN";
}
enum class Action { None, Prepare, Soccer, GetUp, StartKick };
struct Input {
    double now = 0;
    bool robot_fresh = false, recovery_available = false, soccer_ready = false;
    int recovery = 0, planner = 0;
    bool scene_valid = false, blocked = false;
    Point ball, person;
};
struct Output {
    Stage stage = Stage::WaitRobot;
    Action action = Action::None;
    bool send_velocity = true;
    double vx = 0, vy = 0, turn = 0;
};

// Planner indices below match CheckAndStandUp in the saved K1 baseline.
// They are firmware feedback indices, NOT the SDK RobotMode enum.
class Controller {
public:
    explicit Controller(Config c = {}) : c_(c) {
        for (double x : {c.kick_duration,c.robot_timeout,c.startup_timeout}) {
            if (!std::isfinite(x) || x <= 0) throw std::invalid_argument("Invalid controller parameter");
        }
    }
    void enable(bool value) {
        if (value && !enabled_) {
            done_=fault_=kicking_=false; tries_=0;
            ready_since_=boot_since_=last_action_=-1;
        }
        enabled_=value;
    }
    Output tick(const Input& i) {
        if (!enabled_) return emit({Stage::Disabled});
        if (fault_) return emit({Stage::Fault});
        if (!i.robot_fresh) {
            if (kicking_) { done_=true; kicking_=false; }
            ready_since_=-1;
            return emit({Stage::WaitRobot});
        }
        // K1 ready planners: 8/20 standing, 12/22 the soccer visual-kick planners.
        // Firmware reports 12 as soon as kSoccer is active, before any kick starts.
        const bool native_kick=(kicking_ || done_) && (i.planner==12 || i.planner==22);
        const bool standing=i.planner==8 || i.planner==20 || i.planner==12 || i.planner==22;
        if (i.recovery != 0 || (!standing && !native_kick)) {
            if (kicking_) { done_=true; kicking_=false; }
            ready_since_=-1;
            if (boot_since_ < 0) boot_since_=i.now;
            if (i.now-boot_since_ > c_.startup_timeout) {
                fault_=true; return emit({Stage::Fault});
            }
            Output out{(i.recovery || recoveryPlanner(i.planner)) ? Stage::Recover : Stage::Initialize};
            out.send_velocity=false;
            if (i.recovery == 2 && i.planner == 1 && i.recovery_available
                && (last_action_<0 || i.now-last_action_>=5) && tries_<2) {
                out.action=Action::GetUp; ++tries_; last_action_=i.now;
            } else if (i.recovery == 0 && !recoveryPlanner(i.planner)
                       && (last_action_<0 || i.now-last_action_>=1)) {
                out.action=i.planner==2 ? Action::Soccer : Action::Prepare;
                last_action_=i.now;
            }
            return emit(out);
        }
        if (!i.soccer_ready) {
            ready_since_=-1;
            if (boot_since_<0) boot_since_=i.now;
            if (i.now-boot_since_>c_.startup_timeout) {
                fault_=true; return emit({Stage::Fault});
            }
            Output out{Stage::Initialize};
            if (last_action_<0 || i.now-last_action_>=1) {
                out.action=Action::Soccer; last_action_=i.now;
            }
            return emit(out);
        }
        boot_since_=-1;
        if (ready_since_<0) ready_since_=i.now;
        if (i.now-ready_since_<.5) return emit({Stage::Settle});
        tries_=0;
        if (done_) return emit({Stage::Done});
        const bool valid=i.scene_valid && finite(i.ball) && length(i.ball)>.05;
        if (kicking_) {
            if (!valid || i.blocked || i.now-kick_since_>=c_.kick_duration) {
                kicking_=false; done_=true; return emit({Stage::Done});
            }
            // Native skill owns locomotion; never overwrite it with Move requests.
            return emit({Stage::Kick,Action::None,false});
        }
        if (!valid || i.blocked) {
            return emit({i.blocked ? Stage::Blocked : Stage::WaitTargets});
        }
        // Bypass demo decision/distance/angle gates; let the native skill approach and kick.
        kicking_=true; kick_since_=i.now;
        return emit({Stage::Kick,Action::StartKick});
    }
private:
    static Output emit(Output out) { return out; }
    Config c_;
    bool enabled_=false, done_=false, fault_=false, kicking_=false;
    int tries_=0;
    double ready_since_=-1, boot_since_=-1, last_action_=-1;
    double kick_since_=0;
};
} // namespace person_pass
