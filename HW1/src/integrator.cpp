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
  for (int i = 0; i < particles.size(); i++) {
    particles[i]->velocity() += particles[i]->acceleration() * deltaTime;
    particles[i]->position() += particles[i]->velocity() * deltaTime;
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
  for (int i = 0; i < particles.size(); i++) {
    oriPositions.push_back(particles[i]->position());
    oriVelocitys.push_back(particles[i]->velocity());
    particles[i]->position() += particles[i]->velocity() * deltaTime;
    particles[i]->velocity() += particles[i]->acceleration() * deltaTime;
  }
  simulateOneStep();
  for (int i = 0; i < particles.size(); i++) {
    particles[i]->position() = oriPositions[i] + deltaTime * particles[i]->velocity();
    particles[i]->velocity() = oriVelocitys[i] + deltaTime * particles[i]->acceleration();
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
  std::vector<Eigen::Matrix4Xf> oriPositions, oriVelocitys;
  for (int i = 0; i < particles.size(); i++) {
    oriPositions.push_back(particles[i]->position());
    oriVelocitys.push_back(particles[i]->velocity());
    particles[i]->position() += particles[i]->velocity() * deltaTime * 0.5f;
    particles[i]->velocity() += particles[i]->acceleration() * deltaTime * 0.5f;
  }
  simulateOneStep();
  for (int i = 0; i < particles.size(); i++) {
    particles[i]->position() = oriPositions[i] + deltaTime * particles[i]->velocity();
    particles[i]->velocity() = oriVelocitys[i] + deltaTime * particles[i]->acceleration();
  }
}

void RungeKuttaFourth::integrate(const std::vector<Particles *> &particles,
                                 std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Compute k1, k2, k3, k4
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.
  std::vector<Eigen::Matrix4Xf> oriPositions, oriVelocitys, k1, k2, k3, k4, l1, l2, l3, l4;
  for (int i = 0; i < particles.size(); i++) {
    oriPositions.push_back(particles[i]->position());
    oriVelocitys.push_back(particles[i]->velocity());
    k1.push_back(particles[i]->velocity() * deltaTime);
    l1.push_back(particles[i]->acceleration() * deltaTime);
    particles[i]->position() = oriPositions[i] + k1[i] * 0.5f;
    particles[i]->velocity() = oriVelocitys[i] + l1[i] * 0.5f;
  }
  simulateOneStep();
  for (int i = 0; i < particles.size(); i++) {
    k2.push_back(particles[i]->velocity() * deltaTime);
    l2.push_back(particles[i]->acceleration() * deltaTime);
    particles[i]->position() = oriPositions[i] + k2[i] * 0.5f;
    particles[i]->velocity() = oriVelocitys[i] + l2[i] * 0.5f;
  }
  simulateOneStep();
  for (int i = 0; i < particles.size(); i++) {
    k3.push_back(particles[i]->velocity() * deltaTime);
    l3.push_back(particles[i]->acceleration() * deltaTime);
    particles[i]->position() = oriPositions[i] + k3[i];
    particles[i]->velocity() = oriVelocitys[i] + l3[i];
  }
  simulateOneStep();
  for (int i = 0; i < particles.size(); i++) {
    k4.push_back(particles[i]->velocity() * deltaTime);
    l4.push_back(particles[i]->acceleration() * deltaTime);
    particles[i]->position() = oriPositions[i] + (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0f;
    particles[i]->velocity() = oriVelocitys[i] + (l1[i] + 2 * l2[i] + 2 * l3[i] + l4[i]) / 6.0f;
  }
}
