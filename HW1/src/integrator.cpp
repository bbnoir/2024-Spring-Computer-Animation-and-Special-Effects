#include "integrator.h"

#include "configs.h"

void ExplicitEuler::integrate(const std::vector<Particles *> &particles, std::function<void(void)>) const {
  // TODO: Integrate velocity and acceleration
  //   1. Integrate velocity.
  //   2. Integrate acceleration.
  //   3. You should not compute position using acceleration. Since some part only update velocity. (e.g. impulse)
  // Note:
  //   1. You don't need the simulation function in explicit euler.
  //   2. You should do this first because it is very simple. Then you can chech your collision is correct or not.
  //   3. This can be done in 5 lines. (Hint: You can add / multiply all particles at once since it is a large matrix.)
  for (auto &particle : particles) {
    Eigen::Matrix4Xf deltaVelocity = particle->acceleration() * deltaTime;
    Eigen::Matrix4Xf deltaPosition = particle->velocity() * deltaTime;
    particle->velocity() += deltaVelocity;
    particle->position() += deltaPosition;
  }
}

void ImplicitEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.
  std::vector<Eigen::Matrix4Xf> oriPositions, oriVelocitys;
  for (auto &particle : particles) {
    oriPositions.push_back(particle->position());
    oriVelocitys.push_back(particle->velocity());
    Eigen::Matrix4Xf deltaVelocity = particle->acceleration() * deltaTime;
    Eigen::Matrix4Xf deltaPosition = particle->velocity() * deltaTime;
    particle->velocity() += deltaVelocity;
    particle->position() += deltaPosition;
  }
  simulateOneStep();
  for (auto &particle : particles) {
    particle->position() = oriPositions[&particle - &particles[0]] + deltaTime * particle->velocity();
    particle->velocity() = oriVelocitys[&particle - &particles[0]] + deltaTime * particle->acceleration();
  }
}

void MidpointEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.
}

void RungeKuttaFourth::integrate(const std::vector<Particles *> &particles,
                                 std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Compute k1, k2, k3, k4
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.
}
