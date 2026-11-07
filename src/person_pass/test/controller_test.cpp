#include "person_pass/controller.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace person_pass;
static void check(bool condition,const char* message) {
    if (!condition) throw std::runtime_error(message);
}
static bool stopped(Output o) { return o.vx==0 && o.vy==0 && o.turn==0; }
static Input scene(double t=0) {
    Input i; i.now=t; i.robot_fresh=true; i.planner=20;
    i.scene_valid=true; i.ball={.5,0}; i.person={2.5,0}; return i;
}
static Output startKick(Controller& c,Input& i) {
    c.enable(true); i=scene(); c.tick(i);
    i.now=.6; check(c.tick(i).stage==Stage::Align,"Must settle before kicking");
    i.now=1.0; auto out=c.tick(i);
    check(out.stage==Stage::Kick && out.vx>0,"Must kick after stable alignment");
    return out;
}
int main() {
    try {
        const Pose pose{3,-2,1.5707963267948966};
        const Point p{2,-1}; const Point q=toRobot(toOdom(p,pose),pose);
        check(std::hypot(p.x-q.x,p.y-q.y)<1e-10,"Odometry coordinate round trip");
        const Point moved=toRobot(toOdom({2,0},{0,0,0}),{1,0,1.5707963267948966});
        check(std::abs(moved.x)<1e-10 && std::abs(moved.y+1)<1e-10,"Cached targets must follow robot motion");
        Controller boot; Input i=scene();
        check(stopped(boot.tick(i)),"Disabled controller must stop"); boot.enable(true);
        i.robot_fresh=false; check(boot.tick(i).stage==Stage::WaitRobot,"Require robot feedback");
        i=scene(); i.planner=0;
        auto o=boot.tick(i); check(o.action==Action::Prepare && !o.send_velocity,"Prepare before movement");
        i.now=.5; check(boot.tick(i).action==Action::None,"Do not flood mode requests");
        i.now=1.1; i.planner=2;
        check(boot.tick(i).action==Action::Soccer,"Request soccer after prepare feedback");
        i.now=1.2; i.planner=20;
        check(boot.tick(i).stage==Stage::Settle,"Wait for stable readiness");
        Controller walking; walking.enable(true); i=scene(); i.planner=8;
        check(walking.tick(i).stage==Stage::Settle,"Accept the original ready walking planner");
        i.now=.6; check(walking.tick(i).stage==Stage::Align,"Walking mode must allow the pass sequence");
        Controller fallen; fallen.enable(true); i=scene(); i.recovery=2; i.recovery_available=true; i.planner=1;
        o=fallen.tick(i); check(o.action==Action::GetUp && stopped(o),"Get up without walking");
        i.now=1; i.recovery=3;
        o=fallen.tick(i); check(o.action==Action::None && !o.send_velocity,"Do not interrupt getting up");
        i.now=31; check(fallen.tick(i).stage==Stage::Fault,"Startup timeout must latch a fault");
        i=scene(32); check(fallen.tick(i).stage==Stage::Fault,"Fault must not auto-rearm");
        fallen.enable(false); fallen.enable(true); check(fallen.tick(i).stage==Stage::Settle,"Explicit reset clears fault");
        Controller unavailable; unavailable.enable(true); i=scene(); i.recovery=2;
        check(unavailable.tick(i).action==Action::None,"Respect recovery availability");
        Controller attempts; attempts.enable(true); i=scene(); i.recovery=2; i.recovery_available=true; i.planner=1;
        check(attempts.tick(i).action==Action::GetUp,"First get-up request");
        i.now=6; check(attempts.tick(i).action==Action::GetUp,"Second bounded get-up request");
        i.now=12; check(attempts.tick(i).action==Action::None,"No unbounded get-up retries");
        Controller in_progress; in_progress.enable(true); i=scene();
        i.recovery=2; i.recovery_available=true; i.planner=1;
        check(in_progress.tick(i).action==Action::GetUp,"Begin native get-up");
        i.now=6; i.planner=10;
        o=in_progress.tick(i);
        check(o.stage==Stage::Recover && o.action==Action::None && stopped(o),
              "Do not retry while planner 10 is active even if recovery still says fallen");
        i.now=7; i.recovery=0;
        check(in_progress.tick(i).action==Action::None,"Do not change mode during planner 10");
        i.now=8; i.recovery=2; i.planner=1;
        check(in_progress.tick(i).action==Action::GetUp,"Retry only after native get-up returns to planner 1");
        Controller pass; Input active; o=startKick(pass,active);
        check(o.vx<=.4 && o.vy<=.3,"Kick must obey limits on its first command");
        active.now=3.1; check(pass.tick(active).stage==Stage::Done,"Single kick must time out");
        pass.enable(true); active.now=4;
        check(stopped(pass.tick(active)) && pass.tick(active).stage==Stage::Done,"Repeated enable=true must not rearm");
        pass.enable(false); check(pass.tick(active).stage==Stage::Disabled,"Disable stops");
        pass.enable(true); check(pass.tick(active).stage==Stage::Settle,"False->true starts a new attempt");
        for (int reason=0; reason<5; ++reason) {
            Controller abort; Input x; startKick(abort,x); x.now=1.1;
            if (reason==0) x.scene_valid=false;
            if (reason==1) x.blocked=true;
            if (reason==2) x.ball={.9,0};
            if (reason==3) x.person={2.5,2};
            if (reason==4) x.robot_fresh=false;
            check(stopped(abort.tick(x)),"Interrupted kick must stop immediately");
            x=scene(2); abort.tick(x); x.now=3;
            check(abort.tick(x).stage==Stage::Done,"Interrupted kick must not repeat");
        }
        for (double y : {-1.0,1.0}) {
            Controller side; side.enable(true); i=scene(); side.tick(i);
            i.now=.6; i.person.y=y; o=side.tick(i);
            check(o.stage==Stage::Adjust,"Off-axis person requires adjustment");
            check(o.vy*y<0,"Move to the opposite side of the ball from the receiver");
        }
        Controller chase; chase.enable(true); i=scene(); chase.tick(i);
        i.now=.6; i.ball={5,0}; i.person={7,0}; o=chase.tick(i);
        check(o.stage==Stage::Chase && o.vx>0,"Approach a distant ball");
        i.person={3,0}; o=chase.tick(i);
        check(o.stage==Stage::Chase && o.turn!=0,"Circle around instead of walking straight through the ball");
        i.scene_valid=false; check(stopped(chase.tick(i)),"Missing person or ball must stop chase");
        i.scene_valid=true; i.blocked=true;
        check(chase.tick(i).stage==Stage::Blocked,"Nearby obstacles must stop chase");
        i.blocked=false; i.ball.x=std::numeric_limits<double>::quiet_NaN();
        check(stopped(chase.tick(i)),"Invalid positions must not produce motion");
        bool rejected=false; try { Config c; c.kick_speed=2; Controller invalid(c); }
        catch (const std::invalid_argument&) { rejected=true; }
        check(rejected,"Reject inconsistent speed limits");
        std::cout << "PASS: startup, recovery, coordinate transforms, distant chase, left/right alignment, single pass, interruptions, stop and reset\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
