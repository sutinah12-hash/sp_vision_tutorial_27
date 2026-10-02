#pragma once
#include "tracking_math.hpp"
#include <random>

namespace nav_tracking {
// Experimental MPPI-inspired soft-weighted random shooting with temporally
// interpolated noise. This is NOT a reproduction of Nav2 MPPI or SVG-MPPI.
// Same delay/reference as the QP baseline; rollouts additionally limit actuator
// acceleration. Global obstacle avoidance remains the planner's responsibility.
class SamplingPredictive {
 public:
  int horizon=28, samples=257, iterations=3, support_points=6;
  double dt=.07, tau=.315, speed_limit=1.25, command_acceleration=1.7;
  double temperature=.04, noise_sigma=.35, update_dt=.02;
  Eigen::MatrixXd prediction;

  void initialize() {
    if (horizon<2 || samples<3 || samples%2==0 || iterations<1 ||
        support_points<2 || support_points>horizon || dt<=0 || tau<=0 ||
        speed_limit<=0 || temperature<=0 || noise_sigma<=0)
      throw std::invalid_argument("Invalid sampling predictive configuration");
    alpha_=std::exp(-dt/tau); u_=Eigen::MatrixXd::Zero(horizon,2);
    noise_.assign(samples,Eigen::MatrixXd::Zero(horizon,2));
    std::mt19937 generator(2704); std::normal_distribution<double> normal(0,1);
    for (int sample=1;sample<samples;sample+=2) {
      Eigen::MatrixXd knots(support_points,2);
      for (int i=0;i<support_points;++i) for (int d=0;d<2;++d) knots(i,d)=normal(generator);
      for (int i=0;i<horizon;++i) {
        const double x=double(i)*(support_points-1)/(horizon-1);
        const int left=std::min(int(x),support_points-2); const double t=x-left;
        noise_[sample].row(i)=(1-t)*knots.row(left)+t*knots.row(left+1);
      }
      noise_[sample+1]=-noise_[sample];
    }
    reset();
  }
  void reset() { u_.setZero(); fresh_=true; }

  Vec2 solve(const Vec2 & position,const Vec2 & velocity,const Vec2 & previous,
             const Eigen::MatrixXd & ref_p,const Eigen::MatrixXd & ref_v,
             const Eigen::MatrixXd & pending=Eigen::MatrixXd()) {
    if (!position.allFinite() || !velocity.allFinite() || !previous.allFinite() ||
        ref_p.rows()!=horizon || ref_v.rows()!=horizon || ref_p.cols()!=2 || ref_v.cols()!=2 ||
        !ref_p.allFinite() || !ref_v.allFinite() || pending.rows()>=horizon ||
        (pending.rows()>0 && pending.cols()!=2) || !pending.allFinite()) return Vec2::Zero();
    Eigen::MatrixXd guide=ref_v;
    for (int i=0;i+1<horizon;++i)
      guide.row(i)+=tau*(ref_v.row(i+1)-ref_v.row(i))/dt;
    if (fresh_) { u_=guide; fresh_=false; }
    else {
      const double shift=std::clamp(update_dt/dt,0.0,1.0);
      const Eigen::MatrixXd old=u_;
      for (int i=0;i+1<horizon;++i) u_.row(i)=(1-shift)*old.row(i)+shift*old.row(i+1);
      u_=0.95*u_+0.05*guide;
    }
    project(u_,previous,pending);
    std::vector<Eigen::MatrixXd> candidates(samples);
    Eigen::VectorXd costs(samples);
    for (int iteration=0;iteration<iterations;++iteration) {
      const double sigma=noise_sigma/std::sqrt(1.0+iteration);
      for (int k=0;k<samples;++k) {
        candidates[k]=u_+sigma*noise_[k]; project(candidates[k],previous,pending);
        costs[k]=rollout(position,velocity,previous,candidates[k],ref_p,ref_v,nullptr);
      }
      const double best=costs.minCoeff();
      Eigen::MatrixXd average=Eigen::MatrixXd::Zero(horizon,2); double total=0;
      for (int k=0;k<samples;++k) {
        const double weight=std::exp(-(costs[k]-best)/temperature);
        average+=weight*candidates[k]; total+=weight;
      }
      u_=average/total; project(u_,previous,pending);
    }
    rollout(position,velocity,previous,u_,ref_p,ref_v,&prediction);
    return Vec2(u_.row(pending.rows()).transpose());
  }

 private:
  double alpha_=0;
  bool fresh_=true;
  Eigen::MatrixXd u_;
  std::vector<Eigen::MatrixXd> noise_;
  void project(Eigen::MatrixXd & commands,const Vec2 & previous,const Eigen::MatrixXd & pending) {
    Vec2 last=previous;
    for (int i=0;i<horizon;++i) {
      if (i<pending.rows()) { commands.row(i)=pending.row(i); continue; }
      Vec2 next=bounded(Vec2(commands.row(i).transpose()),speed_limit);
      next=last+bounded(next-last,command_acceleration*dt);
      commands.row(i)=next.transpose(); last=next;
    }
  }
  double rollout(Vec2 position,Vec2 velocity,Vec2 last,const Eigen::MatrixXd & commands,
      const Eigen::MatrixXd & ref_p,const Eigen::MatrixXd & ref_v,Eigen::MatrixXd * predicted) {
    if (predicted) predicted->resize(horizon,2);
    double cost=0;
    for (int i=0;i<horizon;++i) {
      const Vec2 command=commands.row(i).transpose();
      velocity+=bounded((1-alpha_)*(command-velocity),2.0*dt);
      position+=dt*velocity;
      cost+=(i+1==horizon ? 72.0 : 24.0)*(position-ref_p.row(i).transpose()).squaredNorm()
        +1.5*(velocity-ref_v.row(i).transpose()).squaredNorm()
        +.7*(command-last).squaredNorm()+.02*command.squaredNorm();
      last=command;
      if (predicted) predicted->row(i)=position.transpose();
    }
    return cost/horizon;
  }
};
}
