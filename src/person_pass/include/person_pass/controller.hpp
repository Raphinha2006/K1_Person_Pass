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
    double vx_limit = .4, vy_limit = .3, turn_limit = .6;
    double min_vx = .3, min_vy = .3, min_turn = .25;
    double stand_off = .45, alignment = .1, kick_speed = .35;
    double kick_duration = 2.0, robot_timeout = .8, startup_timeout = 30;
};
enum class Stage { Disabled, WaitRobot, Initialize, Recover, Settle, WaitTargets,
                   Blocked, Chase, Adjust, Align, Kick, Done, Fault };
inline const char* name(Stage s) {
    switch (s) {
    case Stage::Disabled: return "DISABLED"; case Stage::WaitRobot: return "WAIT_ROBOT";
    case Stage::Initialize: return "INITIALIZE"; case Stage::Recover: return "RECOVER";
    case Stage::Settle: return "SETTLE"; case Stage::WaitTargets: return "WAIT_TARGETS";
    case Stage::Blocked: return "BLOCKED"; case Stage::Chase: return "CHASE";
    case Stage::Adjust: return "ADJUST"; case Stage::Align: return "ALIGN";
    case Stage::Kick: return "KICK"; case Stage::Done: return "DONE";
    case Stage::Fault: return "FAULT";
    }
    return "UNKNOWN";
}
enum class Action { None, Prepare, Soccer, GetUp };
struct Input {
    double now = 0;
    bool robot_fresh = false, recovery_available = false;
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
        for (double x : {c.vx_limit,c.vy_limit,c.turn_limit,c.min_vx,c.min_vy,
                         c.min_turn,c.stand_off,c.alignment,c.kick_speed,
                         c.kick_duration,c.robot_timeout,c.startup_timeout}) {
            if (!std::isfinite(x) || x <= 0) throw std::invalid_argument("Invalid controller parameter");
        }
        if (c.min_vx > c.vx_limit || c.min_vy > c.vy_limit || c.min_turn > c.turn_limit
            || c.kick_speed > c.vx_limit || c.alignment > .3 || c.stand_off < .3 || c.stand_off > .6)
            throw std::invalid_argument("Inconsistent controller limits");
    }
    void enable(bool value) {
        if (value && !enabled_) {
            done_=fault_=kicking_=false; tries_=0;
            ready_since_=align_since_=boot_since_=last_action_=-1;
        }
        enabled_=value;
    }
    Output tick(const Input& i) {
        if (!enabled_) return emit({Stage::Disabled});
        if (fault_) return emit({Stage::Fault});
        if (!i.robot_fresh) {
            if (kicking_) { done_=true; kicking_=false; }
            ready_since_=align_since_=-1;
            return emit({Stage::WaitRobot});
        }
        if (i.recovery != 0 || (i.planner != 8 && i.planner != 20)) {
            if (kicking_) { done_=true; kicking_=false; }
            ready_since_=align_since_=-1;
            if (boot_since_ < 0) boot_since_=i.now;
            if (i.now-boot_since_ > c_.startup_timeout) {
                fault_=true; return emit({Stage::Fault});
            }
            Output out{(i.recovery || recoveryPlanner(i.planner)) ? Stage::Recover : Stage::Initialize};
            out.send_velocity=was_moving_;
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
        boot_since_=-1;
        if (ready_since_<0) ready_since_=i.now;
        if (i.now-ready_since_<.5) return emit({Stage::Settle});
        tries_=0;
        if (done_) return emit({Stage::Done});
        const bool valid=i.scene_valid && finite(i.ball) && finite(i.person)
            && length(i.ball)>.05 && length({i.person.x-i.ball.x,i.person.y-i.ball.y})>=1.0;
        const double range=length(i.ball);
        const double yaw=std::atan2(i.ball.y,i.ball.x);
        const double direction=std::atan2(i.person.y-i.ball.y,i.person.x-i.ball.x);
        const double error=wrap(direction-yaw);
        if (kicking_) {
            if (!valid || i.blocked || i.now-kick_since_>=c_.kick_duration
                || range-min_range_>.3 || i.ball.x<.12 || std::abs(error)>.18) {
                kicking_=false; done_=true; return emit({Stage::Done});
            }
            min_range_=std::min(min_range_,range);
            return kick(yaw);
        }
        if (!valid || i.blocked) {
            align_since_=-1;
            return emit({i.blocked ? Stage::Blocked : Stage::WaitTargets});
        }
        if (range>=.25 && range<=.65 && i.ball.x>.15
            && std::abs(yaw)<.1 && std::abs(error)<c_.alignment) {
            if (align_since_<0) align_since_=i.now;
            if (i.now-align_since_>=.35) {
                kicking_=true; kick_since_=i.now; min_range_=range;
                return kick(yaw);
            }
            return emit({Stage::Align});
        }
        align_since_=-1;
        if (range>1.0) {
            Point target{i.ball.x-c_.stand_off*std::cos(direction),
                         i.ball.y-c_.stand_off*std::sin(direction)};
            if (std::abs(error)>1.5707963267948966) {
                const double back=yaw+3.141592653589793;
                const double sign=wrap(back-direction)>0 ? 1 : -1;
                const double tangent=back+sign*std::acos(std::min(1.0,.65/range));
                target={i.ball.x+.65*std::cos(tangent),i.ball.y+.65*std::sin(tangent)};
            }
            const double heading=std::atan2(target.y,target.x);
            return emit({Stage::Chase,Action::None,true,
                         std::abs(heading)<.6 ? c_.vx_limit : 0,0,heading});
        }
        const double radial=std::abs(range-c_.stand_off)<.04 ? 0
            : std::clamp(range-c_.stand_off,-.15,.3);
        const double tangent=std::abs(error)<c_.alignment ? 0 : std::copysign(.15,error);
        const double ux=i.ball.x/range, uy=i.ball.y/range;
        return emit({Stage::Adjust,Action::None,true,
                     radial*ux+tangent*uy,radial*uy-tangent*ux,1.5*yaw});
    }
private:
    static double bounded(double value, double limit, double minimum, double deadband) {
        if (std::abs(value)<deadband) return 0;
        return std::copysign(std::clamp(std::abs(value),minimum,limit),value);
    }
    Output emit(Output out) {
        out.vx=bounded(out.vx,c_.vx_limit,c_.min_vx,.035);
        out.vy=bounded(out.vy,c_.vy_limit,c_.min_vy,.035);
        out.turn=bounded(out.turn,c_.turn_limit,c_.min_turn,.08);
        was_moving_=out.send_velocity && (out.vx!=0 || out.vy!=0 || out.turn!=0);
        return out;
    }
    Output kick(double yaw) {
        return emit({Stage::Kick,Action::None,true,c_.kick_speed*std::cos(yaw),
                     c_.kick_speed*std::sin(yaw),0});
    }
    Config c_;
    bool enabled_=false, done_=false, fault_=false, kicking_=false, was_moving_=false;
    int tries_=0;
    double ready_since_=-1, align_since_=-1, boot_since_=-1, last_action_=-1;
    double kick_since_=0, min_range_=0;
};
} // namespace person_pass
