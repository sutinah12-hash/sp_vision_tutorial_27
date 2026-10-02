#include <gtest/gtest.h>
#include "sampling_predictive.hpp"
using namespace nav_tracking;
TEST(SamplingPredictive, EquilibriumAndDeterministicSeed) {
  SamplingPredictive a,b; a.initialize(); b.initialize();
  Eigen::MatrixXd p=Eigen::MatrixXd::Zero(a.horizon,2),v=p;
  EXPECT_LT(a.solve(Vec2::Zero(),Vec2::Zero(),Vec2::Zero(),p,v).norm(),1e-9);
  a.reset(); p.col(0).setOnes();
  const auto ua=a.solve(Vec2::Zero(),Vec2::Zero(),Vec2::Zero(),p,v);
  const auto ub=b.solve(Vec2::Zero(),Vec2::Zero(),Vec2::Zero(),p,v);
  EXPECT_LT((ua-ub).norm(),1e-12); EXPECT_GT(ua.x(),0);
}
TEST(SamplingPredictive, DelayedClosedLoopRespectsLimitsAndConverges) {
  SamplingPredictive m; m.samples=65; m.initialize(); m.update_dt=m.dt;
  Vec2 p=Vec2::Zero(),v=Vec2::Zero(),u=Vec2::Zero(),goal(.8,.3);
  Eigen::MatrixXd rp(m.horizon,2),rv=Eigen::MatrixXd::Zero(m.horizon,2),queue=Eigen::MatrixXd::Zero(3,2);
  for (int i=0;i<m.horizon;++i) rp.row(i)=goal.transpose();
  for (int i=0;i<300;++i) {
    const Vec2 next=m.solve(p,v,u,rp,rv,queue);
    EXPECT_TRUE(next.allFinite()); EXPECT_LE(next.norm(),m.speed_limit+1e-9);
    EXPECT_LE((next-u).norm(),m.command_acceleration*m.dt+1e-9); u=next;
    const double alpha=std::exp(-m.dt/m.tau);
    v+=bounded((1-alpha)*(queue.row(0).transpose()-v),2*m.dt); p+=m.dt*v;
    queue.topRows(2)=queue.bottomRows(2).eval(); queue.row(2)=u.transpose();
  }
  EXPECT_LT((p-goal).norm(),.05); EXPECT_LT(v.norm(),.05);
}
