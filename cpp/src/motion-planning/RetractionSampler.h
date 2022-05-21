#ifndef RETRACTION_SAMPLER_H
#define RETRACTION_SAMPLER_H

#include <util/macros.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSampler.h>

#include <stdexcept>

#include <cmath>  // for std::cbrt()

namespace motion_planning {

/** Sample the retraction value to be pseudo-uniform in workspace
 *
 * The more retracted the robot becomes, the more constrained and constricted
 * the robot tip becomes.  The rest of the configuration variables have less
 * and less affect on the final robot pose simply because it is so short.
 * Therefore, this sampler is to randomly sample the retraction in the same way
 * we would sample the radius of a sphere such that combined with a random unit
 * vector would constitute a uniform random sample inside of a sphere in R^3.
 * This is chosen because the robot workspace is limited by the sphere about
 * the base of the robot with radius equal to the extended length of the robot.
 *
 * A maximum retraction value corresponds to a radius of zero, and a retraction
 * of zero corresponds to a radius of the maximum robot length.  So it is
 * inversely related to the sphere radius.
 */
class RetractionSampler : public ompl::base::StateSampler {
public:
  using State = ompl::base::State;

  RetractionSampler(const ompl::base::StateSpace *space)
    : ompl::base::StateSampler(space)
  {
    if (space_->getType() != ompl::base::STATE_SPACE_REAL_VECTOR) {
      throw std::invalid_argument("state space must be a RealVectorStateSpace");
    }
    if (space_->getDimension() != 1) {
      throw std::invalid_argument("state space has more than one dimension");
    }
    const auto vspace = space_->as<ompl::base::RealVectorStateSpace>();
    const auto &bounds = vspace->getBounds();
    if (bounds.low.size() != 1 || bounds.low[0] != 0.0) {
      throw std::invalid_argument("state space bottom is not zero - not supported");
    }
  }

  /// sample radius uniformly for a sphere in R^3
  /// r = CDF^{-1} (u) = R * cube_root(u)
  virtual void sampleUniform(State *state) {
    const auto vspace = space_->as<ompl::base::RealVectorStateSpace>();
    const auto &bounds = vspace->getBounds();
    double R = bounds.high[0];
    double u = rng_.uniform01();
    double r = R * std::cbrt(u); // r represents extension
    double retraction = R - r;   // convert to retraction
    const auto vstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
    vstate->values[0] = retraction;
  }

  virtual void sampleUniformNear(State *state, const State *near, double distance) {
    UNUSED_VAR(state);
    UNUSED_VAR(near);
    UNUSED_VAR(distance);
    throw std::runtime_error("unimplemented");
  }

  virtual void sampleGaussian(State *state, const State *mean, double stdDev) {
    UNUSED_VAR(state);
    UNUSED_VAR(mean);
    UNUSED_VAR(stdDev);
    throw std::runtime_error("unimplemented");
  }
}; // end of class RetractionSampler

} // end of namespace motion_planning

#endif // RETRACTION_SAMPLER_H
