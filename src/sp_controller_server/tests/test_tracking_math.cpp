#include <gtest/gtest.h>
#include "tracking_math.hpp"
using namespace nav_tracking;

TEST(Frames, RotatingBaseNeedsInverseRotation) {
  const auto body=worldToBody(Vec2(1.0,0.0),std::acos(-1.0)/2.0);
  EXPECT_NEAR(body.x(),0.0,1e-12); EXPECT_NEAR(body.y(),-1.0,1e-12);
}
TEST(ReferencePath, DeduplicationProjectionAndBraking) {
  ReferencePath path;
  path.set({Vec2(0,0),Vec2(0,0),Vec2(1,0),Vec2(1,1)},1.2,0.8,0.8);
  ASSERT_EQ(path.points.size(),3u);
  EXPECT_NEAR(path.nearest(Vec2(0.4,0.2)),0.4,1e-10);
  EXPECT_NEAR(path.length(),2.0,1e-10);
  EXPECT_DOUBLE_EQ(path.speed(path.length()),0.0);
  EXPECT_NEAR((path.at(0.4)-Vec2(0.4,0)).norm(),0.0,1e-10);
  for (std::size_t i=1;i<path.speeds.size();++i)
    EXPECT_LE(path.speeds[i-1]*path.speeds[i-1],path.speeds[i]*path.speeds[i]+1.6*(path.arc[i]-path.arc[i-1])+1e-9);
}
TEST(ReferencePath, EmptyAndSinglePointPlansAreFinite) {
  ReferencePath path; path.set({},1,1,1);
  EXPECT_TRUE(path.points.empty());
  path.set({Vec2(2,3)},1,1,1);
  Eigen::MatrixXd p,v; path.preview(Vec2(2,3),Vec2::Zero(),5,0.1,p,v);
  EXPECT_TRUE(p.allFinite()); EXPECT_TRUE(v.isZero());
  EXPECT_NEAR((p.row(0).transpose()-Vec2(2,3)).norm(),0.0,1e-10);
}
TEST(Mpc, ConstraintsAndStationaryEquilibrium) {
  VelocityMpc m; m.speed_limit=0.7; m.initialize();
  Eigen::MatrixXd p=Eigen::MatrixXd::Zero(m.horizon,2),v=p;
  EXPECT_LT(m.solve(Vec2::Zero(),Vec2::Zero(),Vec2::Zero(),p,v).norm(),1e-10);
  p.setConstant(100.0);
  const auto command=m.solve(Vec2::Zero(),Vec2::Zero(),Vec2::Zero(),p,v);
  EXPECT_LE(command.norm(),0.7+1e-10);
  EXPECT_GT(command.x(),0.0); EXPECT_GT(command.y(),0.0);
}
TEST(Mpc, ClosedLoopConvergesToGoal) {
  VelocityMpc m; m.initialize();
  Vec2 pos=Vec2::Zero(),vel=Vec2::Zero(),last=Vec2::Zero(),goal(1.0,0.4);
  Eigen::MatrixXd p(m.horizon,2),v=Eigen::MatrixXd::Zero(m.horizon,2);
  for (int i=0;i<m.horizon;++i) p.row(i)=goal.transpose();
  for (int i=0;i<200;++i) {
    last=m.solve(pos,vel,last,p,v);
    EXPECT_TRUE(last.allFinite()); EXPECT_LE(last.norm(),m.speed_limit+1e-9);
    const double a=std::exp(-m.dt/m.tau);
    vel=a*vel+(1-a)*last; pos+=m.dt*vel;
  }
  EXPECT_LT((pos-goal).norm(),0.015); EXPECT_LT(vel.norm(),0.02);
}
TEST(Mpc, RejectInvalidConfiguration) {
  VelocityMpc m; m.horizon=0; EXPECT_THROW(m.initialize(),std::invalid_argument);
}
TEST(Mpc, DelayedCommandsAreFixedAndLoopConverges) {
  VelocityMpc m; m.horizon=28; m.dt=0.07; m.tau=0.315; m.initialize();
  Vec2 pos=Vec2::Zero(),vel=Vec2::Zero(),last=Vec2::Zero(),goal(1.0,0.4);
  Eigen::MatrixXd p(m.horizon,2),v=Eigen::MatrixXd::Zero(m.horizon,2);
  Eigen::MatrixXd queue=Eigen::MatrixXd::Zero(3,2);
  for (int i=0;i<m.horizon;++i) p.row(i)=goal.transpose();
  for (int i=0;i<240;++i) {
    last=m.solve(pos,vel,last,p,v,queue);
    EXPECT_TRUE(last.allFinite()); EXPECT_LE(last.norm(),m.speed_limit+1e-9);
    const double a=std::exp(-m.dt/m.tau);
    vel=a*vel+(1-a)*queue.row(0).transpose(); pos+=m.dt*vel;
    queue.topRows(2)=queue.bottomRows(2).eval(); queue.row(2)=last.transpose();
  }
  EXPECT_LT((pos-goal).norm(),0.015); EXPECT_LT(vel.norm(),0.02);
}
