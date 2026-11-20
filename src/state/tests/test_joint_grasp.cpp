#include "rover_state/centered_grasp.hpp"
#include <cassert>
#include <iostream>
using namespace rover_state;
int main(){
 assert(grasp_approach_speed(.8,.3,.10)==.10);
 assert(grasp_approach_speed(.38,.3,.10)==.10);
 assert(grasp_approach_speed(.2,.3,.10)==.10);
 assert(grasp_approach_speed(.30,.25,.10)==.10);
 assert(grasp_approach_speed(.30,.25,.08)==.08);
 CenteredGraspParameters p; Pose robot{{0,0,0},0};
 auto free=[](Pose){return true;};
 auto check=[&](Point target){auto a=joint_grasp_adjustment(robot,0,target,p,.30,free);assert(a);Pose end=robot;end.yaw=a->yaw;end.p.x+=a->translation*cos(end.yaw);end.p.y+=a->translation*sin(end.yaw);assert(p.contains(end,target));assert(std::abs(a->yaw)<=p.maximum_turn+1e-8);assert(std::abs(a->translation)<=.30+1e-8);return *a;};
 check({.6,0,0});check({.75,.12,0});check({.6,-.274,0});
 auto reverse=check({.38,0,0});
 assert(reverse.translation<0); // target only 8 cm ahead of front: back into sector
 check({.32,0,0}); // target only 2 cm ahead of front edge
 assert(!joint_grasp_adjustment(robot,0,{.38,0,0},p,.10,free));
 assert(!joint_grasp_adjustment(robot,0,{.38,0,0},p,.30,[](Pose q){return q.p.x>-.05;}));
 assert(!joint_grasp_adjustment(robot,0,{.6,.7,0},p,.30,free));
 assert(!joint_grasp_adjustment(robot,0,{.6,0,0},p,.30,[](Pose){return false;}));
 assert(!joint_grasp_adjustment(robot,0,{1.,0,0},p,.01,free));
 check({.6,.12,0});
 p.preferred_radius=.25;
 auto first_far=grasp_distance_correction(robot,{.5869,.007,0},p,.015);
 auto second_far=grasp_distance_correction(robot,{.6575,.0082,0},p,.015);
 assert(first_far && std::abs(*first_far-.0369)<1e-6);
 assert(second_far && std::abs(*second_far-.1075)<1e-6);
 auto centred=check({.56,.125,0});
 Pose turned=robot;turned.yaw=centred.yaw;
 assert(std::abs(alignment(turned,{.56,.125,0},p.front).lateral)<.05);
 assert(!joint_grasp_adjustment(robot,0,{.75,0,0},p,.30,[](Pose q){return q.p.x<.05;}));
 std::cout<<"joint adjustment: geometry, turn/translation limits and blocked sweeps passed\n";
}
