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
    // Walk the ball into position first; the native skill only takes over once
    // the robot is behind the ball and lined up with the receiver.
    double vx_limit = .4, vy_limit = .3, turn_limit = .6;
    double min_vx = .3, min_vy = .3, min_turn = .25;
    double stand_off = .45, alignment = .1;
    double kick_duration = 7.0, kick_cooldown = 2.0, robot_timeout = .8, startup_timeout = 30;
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
enum class Action { None, Prepare, Soccer, GetUp, StartKick };
struct Input {
    double now = 0;
    bool robot_fresh = false, recovery_available = false, soccer_ready = false;
    int recovery = 0, planner = 0;
    bool scene_valid = false, blocked = false;
    bool ball_fresh = true, person_fresh = true;
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
                         c.min_turn,c.stand_off,c.alignment,c.kick_duration,c.kick_cooldown,
                         c.robot_timeout,c.startup_timeout}) {
            if (!std::isfinite(x) || x <= 0) throw std::invalid_argument("Invalid controller parameter");
        }
        if (c.min_vx > c.vx_limit || c.min_vy > c.vy_limit || c.min_turn > c.turn_limit
            || c.alignment > .3 || c.stand_off < .3 || c.stand_off > .6)
            throw std::invalid_argument("Inconsistent controller limits");
    }
    void enable(bool value) {
        if (value && !enabled_) {
            fault_=kicking_=false; tries_=0;
            ready_since_=align_since_=boot_since_=last_action_=cooldown_until_=-1;
        }
        enabled_=value;
    }
    bool coolingDown(double now) const { return now<cooldown_until_; }
    Output tick(const Input& i) {
        if (!enabled_) return emit({Stage::Disabled});
        if (fault_) return emit({Stage::Fault});
        if (!i.robot_fresh) {
            kicking_=false;
            ready_since_=align_since_=-1;
            return emit({Stage::WaitRobot});
        }
        // K1 ready planners: 8/20 standing, 12/22 the soccer visual-kick planners.
        // Firmware reports 12 as soon as kSoccer is active, before any kick starts.
        const bool native_kick=kicking_ && (i.planner==12 || i.planner==22);
        const bool standing=i.planner==8 || i.planner==20 || i.planner==12 || i.planner==22;
        if (i.recovery != 0 || (!standing && !native_kick)) {
            if (kicking_) { kicking_=false; cooldown_until_=-1; }
            ready_since_=align_since_=-1;
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
            ready_since_=align_since_=-1;
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
        // After each attempt keep looping: cool down, then look for the ball
        // again instead of ending the program after a single pass.
        if (i.now<cooldown_until_) return emit({Stage::WaitTargets});
        // A pass needs both the ball and a receiver; the old demo's decision,
        // distance and angle gates stay bypassed, but the receiver is required.
        const bool valid=i.scene_valid && finite(i.ball) && finite(i.person)
            && length(i.ball)>.05 && length({i.person.x-i.ball.x,i.person.y-i.ball.y})>=.5;
        if (kicking_) {
            // The native skill looks after the ball itself. The head camera loses
            // the ball while the robot steps over it, so a short target loss must
            // not cancel the attempt before the firmware actually swings the leg.
            if (i.now-kick_since_>=c_.kick_duration) {
                kicking_=false; align_since_=-1;
                cooldown_until_=i.now+c_.kick_cooldown;
                return emit({Stage::WaitTargets});
            }
            // Native skill owns locomotion; never overwrite it with Move requests.
            return emit({Stage::Kick,Action::None,false});
        }
        if (!valid || i.blocked) {
            align_since_=-1;
            return emit({i.blocked ? Stage::Blocked : Stage::WaitTargets});
        }
        const double range=length(i.ball);
        const double yaw=std::atan2(i.ball.y,i.ball.x);
        const double direction=std::atan2(i.person.y-i.ball.y,i.person.x-i.ball.x);
        const double error=wrap(direction-yaw);
        // Hand over only inside a kickable window. Starting the native skill from
        // far away lets it walk onto the ball, where the head camera loses it.
        // 0.3-3.5 m lets the skill walk up to the ball and swing; starting closer
        // than that leaves no room for the kick, starting farther loses the ball.
        if (range>=.3 && range<=3.5 && i.ball.x>.15
            && (std::abs(error)<.6 || range>=1.2)) {
            if (!i.ball_fresh || !i.person_fresh) {
                align_since_=-1;
                return emit({Stage::WaitTargets});
            }
            if (align_since_<0) align_since_=i.now;
            if (i.now-align_since_>=.2) {
                kicking_=true; kick_since_=i.now;
                return emit({Stage::Kick,Action::StartKick,false});
            }
            return emit({Stage::Align});
        }
        align_since_=-1;
        // Far from the ball: walk to a point behind it on the receiver's line.
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
        // Near the ball but not lined up yet: small radial/tangential correction.
        const double radial=std::abs(range-c_.stand_off)<.04 ? 0
            : std::clamp(range-c_.stand_off,-.15,.3);
        const double tangent=std::abs(error)<c_.alignment ? 0 : std::copysign(.15,error);
        const double ux=i.ball.x/range, uy=i.ball.y/range;
        return emit({Stage::Adjust,Action::None,true,
                     radial*ux+tangent*uy,radial*uy-tangent*ux,1.5*yaw});
    }
private:
    static double bounded(double value,double limit,double minimum,double deadband) {
        if (std::abs(value)<deadband) return 0;
        return std::copysign(std::clamp(std::abs(value),minimum,limit),value);
    }
    Output emit(Output out) {
        out.vx=bounded(out.vx,c_.vx_limit,c_.min_vx,.035);
        out.vy=bounded(out.vy,c_.vy_limit,c_.min_vy,.035);
        out.turn=bounded(out.turn,c_.turn_limit,c_.min_turn,.08);
        return out;
    }
    Config c_;
    bool enabled_=false, fault_=false, kicking_=false;
    int tries_=0;
    double ready_since_=-1, align_since_=-1, boot_since_=-1, last_action_=-1;
    double kick_since_=0, cooldown_until_=-1;
};
} // namespace person_pass
