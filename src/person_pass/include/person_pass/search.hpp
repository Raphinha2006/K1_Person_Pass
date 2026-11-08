#pragma once
#include "person_pass/controller.hpp"

namespace person_pass {
enum class SearchTarget { None, Ball, Person };
struct SearchOutput {
    SearchTarget target=SearchTarget::None;
    bool head=false, stop=false;
    double pitch=0, yaw=0, turn=0;
};

// Port of AAA CamFindBall's six poses and one-second dwell, followed by
// RobotFindBall's turn toward the last target bearing. No mode requests.
class TargetSearch {
public:
    SearchOutput tick(double now,bool allowed,bool ball_fresh,bool person_fresh,
                      Point last_ball,Point last_person,bool can_turn,double turn_limit,
                      bool /*ball_remembered*/=false,bool person_remembered=false,bool refresh=false) {
        // Finish one search before considering the other target. In particular,
        // losing the ball during a receiver scan must not restart its six seconds.
        const bool acquired=(target_==SearchTarget::Ball && ball_fresh)
            || (target_==SearchTarget::Person && person_fresh);
        auto target=target_;
        if (target==SearchTarget::None || acquired) {
            target=!person_fresh && !person_remembered ? SearchTarget::Person
                : (!ball_fresh ? SearchTarget::Ball
                    : (!person_fresh && refresh ? SearchTarget::Person : SearchTarget::None));
        }
        if (!allowed || target==SearchTarget::None) {
            const bool stop=turning_ || acquired;
            reset();
            SearchOutput out; out.stop=stop; return out;
        }
        const bool changed=target!=target_;
        const bool stop=acquired || (turning_ && (changed || !can_turn));
        if (changed) {
            target_=target; since_=now;
            const Point last=target==SearchTarget::Ball ? last_ball : last_person;
            direction_=finite(last) && last.y>0 ? 1 : -1;
        }
        const double elapsed=std::max(0.0,now-since_);
        const int index=static_cast<int>(std::fmod(elapsed,6.0));
        static constexpr double pitches[]={1,1,1,.2,.2,.2};
        static constexpr double yaws[]={1.1,0,-1.1,-1.1,0,1.1};
        turning_=can_turn && elapsed>=6;
        return {target_,true,stop,pitches[index],yaws[index],
                turning_ ? std::min(.6,turn_limit)*direction_ : 0};
    }
    void reset() { target_=SearchTarget::None; since_=0; turning_=false; }
private:
    SearchTarget target_=SearchTarget::None;
    double since_=0, direction_=-1;
    bool turning_=false;
};
} // namespace person_pass
