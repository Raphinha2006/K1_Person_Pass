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
    Input i; i.now=t; i.robot_fresh=true; i.planner=20; i.soccer_ready=true;
    i.scene_valid=true; i.ball={.5,0}; i.person={2.5,0}; return i;
}
static Output startKick(Controller& c,Input& i) {
    c.enable(true); i=scene(); c.tick(i);
    i.now=.6; auto out=c.tick(i);
    check(out.stage==Stage::Align && stopped(out),
          "Hold still for the alignment confirmation window");
    i.now=.81; out=c.tick(i);
    check(out.stage==Stage::Kick && out.action==Action::StartKick && stopped(out),
          "Hand over to the native kick without a walking push");
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
        i.now=.6; check(walking.tick(i).stage==Stage::Align,"Align before kicking in walking mode");
        i.now=.81; check(walking.tick(i).action==Action::StartKick,"Walking mode must allow the native kick");
        Controller mode; mode.enable(true); i=scene(); i.soccer_ready=false;
        check(mode.tick(i).action==Action::Soccer,"Even a ready walking planner requires soccer mode");
        i.now=.2; check(mode.tick(i).stage==Stage::Initialize,"Wait for soccer mode acknowledgement");
        i.soccer_ready=true; check(mode.tick(i).stage==Stage::Settle,"Settle after mode acknowledgement");
        i.now=.8; check(mode.tick(i).stage==Stage::Align,"Line up after acknowledged soccer mode");
        i.now=1.01; check(mode.tick(i).action==Action::StartKick,"Kick after acknowledged soccer mode");
        Controller soccer; soccer.enable(true); i=scene(); i.planner=12;
        check(soccer.tick(i).stage==Stage::Settle,"Accept firmware soccer planner 12");
        i.now=.6; check(soccer.tick(i).stage==Stage::Align,"Soccer planner still needs alignment");
        i.now=.81; check(soccer.tick(i).action==Action::StartKick,"Kick from soccer planner without mode churn");
        Controller chase; chase.enable(true); i=scene(); i.ball={5,0}; i.person={2,4};
        chase.tick(i); i.now=.6; o=chase.tick(i);
        check(o.stage==Stage::Chase && o.send_velocity && o.vx>0,
              "Walk around the ball when the receiver is badly placed");
        Controller detour; detour.enable(true); i=scene(); i.ball={5,0}; i.person={-2,0};
        detour.tick(i); i.now=.6; o=detour.tick(i);
        check(o.stage==Stage::Chase && o.send_velocity && o.vx>0 && o.turn>0,
              "Detour around the ball when the receiver is behind it");
        Controller adjust; adjust.enable(true); i=scene(); i.ball={.5,.3}; i.person={.5,-2};
        adjust.tick(i); i.now=.6; o=adjust.tick(i);
        check(o.stage==Stage::Adjust && o.send_velocity && o.turn>0,
              "Correct position and heading next to the ball");
        Controller behind; behind.enable(true); i=scene(); i.ball={5,1}; i.person={8,2.5};
        behind.tick(i); i.now=.6;
        check(behind.tick(i).stage==Stage::Chase,"Approach a distant ball before handing over");
        i.now=.81; check(behind.tick(i).action!=Action::StartKick,"Do not kick from outside the window");
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
        for (int planner : {10,23}) {
            Controller in_progress; in_progress.enable(true); i=scene();
            i.recovery=2; i.recovery_available=true; i.planner=1;
            check(in_progress.tick(i).action==Action::GetUp,"Begin native get-up");
            i.now=6; i.planner=planner;
            o=in_progress.tick(i);
            check(o.stage==Stage::Recover && o.action==Action::None && stopped(o),
                  "Do not retry during either recovery planner even if recovery still says fallen");
            i.now=7; i.recovery=0;
            check(in_progress.tick(i).action==Action::None,"Do not change mode during native recovery");
            i.now=8; i.recovery=2; i.planner=1;
            check(in_progress.tick(i).action==Action::GetUp,"Retry only after native get-up returns to planner 1");
        }
        Controller pass; Input active; o=startKick(pass,active);
        check(stopped(o),"Starting the native kick must not push the ball with Move");
        for (int planner : {20,12,22}) {
            active.now=.7; active.planner=planner; o=pass.tick(active);
            check(o.stage==Stage::Kick && o.action==Action::None && !o.send_velocity,
                  "Do not restart, send Move or switch modes while the native kick owns control");
        }
        active.now=8.0;
        check(pass.tick(active).stage==Stage::WaitTargets,"Kick must end and enter cooldown");
        check(pass.tick(active).stage==Stage::WaitTargets,"Cooldown must not restart immediately");
        active.now=10.1;
        check(pass.tick(active).stage==Stage::Align,"After cooldown the loop retries the pass");
        active.now=10.31;
        check(pass.tick(active).action==Action::StartKick,"Loop starts a second native kick");
        pass.enable(false); check(pass.tick(active).stage==Stage::Disabled,"Disable stops");
        active.planner=20; active.now=10.4;
        pass.enable(true); check(pass.tick(active).stage==Stage::Settle,"False->true restarts the loop");
        for (int reason=0; reason<2; ++reason) {
            Controller abort; Input x; startKick(abort,x); x.now=1.1;
            if (reason==0) x.recovery=3;
            if (reason==1) x.robot_fresh=false;
            check(stopped(abort.tick(x)),"Interrupted kick must stop immediately");
            x=scene(2); abort.tick(x); x.now=3;
            check(abort.tick(x).stage==Stage::Align,"Interrupted kick must become retryable");
        }
        Controller lost; Input l; startKick(lost,l); l.now=1.2; l.scene_valid=false;
        auto keep=lost.tick(l);
        check(keep.stage==Stage::Kick && !keep.send_velocity,
              "Temporary target loss must keep the native swing");
        l.now=8.0; check(lost.tick(l).stage==Stage::WaitTargets,"Kick must still end on its timeout");
        Controller waiting; waiting.enable(true); i=scene(); waiting.tick(i);
        i.now=.6; i.scene_valid=false;
        check(waiting.tick(i).stage==Stage::WaitTargets,"Missing targets must prevent a start");
        i.scene_valid=true; i.blocked=true;
        check(waiting.tick(i).stage==Stage::Blocked,"Nearby obstacles must prevent a start");
        i.blocked=false; i.person={.5,-.2};
        check(waiting.tick(i).stage==Stage::WaitTargets,"Receiver too close to the ball must not start a pass");
        i.person={2.5,0}; i.ball={.5,0};
        check(waiting.tick(i).stage==Stage::Align,"Valid ball and receiver line up for a pass");
        bool rejected=false; try { Config c; c.kick_duration=0; Controller invalid(c); }
        catch (const std::invalid_argument&) { rejected=true; }
        check(rejected,"Reject invalid skill duration");
        rejected=false; try { Config c; c.alignment=.5; Controller invalid(c); }
        catch (const std::invalid_argument&) { rejected=true; }
        check(rejected,"Reject an alignment threshold wider than the ball");
        std::cout << "PASS: startup, recovery, transforms, chase/align, native kick handoff, single attempt, interruptions, stop and reset\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
