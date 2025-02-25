#include "particles.h"

Particles::Particles(int size, float mass_) noexcept :
    _position(4, size), _velocity(4, size), _acceleration(4, size), _mass(size, mass_) , _constraint(4, size) {
  _position.setZero();
  _velocity.setZero();
  _acceleration.setZero();
  _constraint.setOnes();
}

void Particles::setZero() {
  _position.setZero();
  _velocity.setZero();
  _acceleration.setZero();
  _constraint.setOnes();
}

void Particles::resize(int newSize) {
  _position.conservativeResize(Eigen::NoChange, newSize);
  _velocity.conservativeResize(Eigen::NoChange, newSize);
  _acceleration.conservativeResize(Eigen::NoChange, newSize);
  _mass.resize(newSize, 0.0f);
  _constraint.conservativeResize(Eigen::NoChange, newSize);
}
