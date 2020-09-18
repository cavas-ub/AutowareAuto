// Copyright 2018 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <common/types.hpp>
#include "motion_model/constant_acceleration.hpp"

using autoware::common::types::float32_t;

namespace autoware
{
namespace motion
{
namespace motion_model
{

template<std::int32_t NUMBER_OF_STATES, typename StatesT>
GenericConstantAcceleration<NUMBER_OF_STATES, StatesT> &
GenericConstantAcceleration<NUMBER_OF_STATES, StatesT>::operator=(
  const GenericConstantAcceleration<NUMBER_OF_STATES, StatesT> & rhs)
{
  if (this != &rhs) {
    m_state = rhs.m_state;
  }
  return *this;
}


template<std::int32_t NUMBER_OF_STATES, typename StatesT>
void GenericConstantAcceleration<NUMBER_OF_STATES, StatesT>::predict(
  const std::chrono::nanoseconds & dt)
{
  predict(m_state, dt);
}

template<std::int32_t NUMBER_OF_STATES, typename StatesT>
void GenericConstantAcceleration<NUMBER_OF_STATES, StatesT>::compute_jacobian_and_predict(
  Eigen::Matrix<float32_t, NUMBER_OF_STATES, NUMBER_OF_STATES> & F,
  const std::chrono::nanoseconds & dt)
{
  compute_jacobian(F, dt);
  predict(dt);
}

template<std::int32_t NUMBER_OF_STATES, typename StatesT>
float GenericConstantAcceleration<NUMBER_OF_STATES, StatesT>::operator[](
  const index_t idx) const {return m_state(idx);}

template<std::int32_t NUMBER_OF_STATES, typename StatesT>
void GenericConstantAcceleration<NUMBER_OF_STATES, StatesT>::reset(
  const Eigen::Matrix<float32_t, NUMBER_OF_STATES, 1U> & x)
{
  m_state = x;
}
template<std::int32_t NUMBER_OF_STATES, typename StatesT>
const Eigen::Matrix<float32_t, NUMBER_OF_STATES, 1U> &
GenericConstantAcceleration<NUMBER_OF_STATES, StatesT>::get_state() const
{
  return m_state;
}

template<>
void GenericConstantAcceleration<6U, StatesPoseVelocityAcceleration2D>::compute_jacobian(
  Eigen::Matrix<float32_t, 6U, 6U> & F,
  const std::chrono::nanoseconds & dt)
{
  // identity matrix
  F.setIdentity();

  using float_seconds_t = std::chrono::duration<float>;
  const float32_t dt_s = float_seconds_t{dt}.count();
  // only nonzero elements are ones along diagonal + constant terms for velocity
  F(States::POSE_X, States::VELOCITY_X) = dt_s;
  F(States::POSE_Y, States::VELOCITY_Y) = dt_s;
  F(States::VELOCITY_X, States::ACCELERATION_X) = dt_s;
  F(States::VELOCITY_Y, States::ACCELERATION_Y) = dt_s;
  const float32_t dt2_s2 = dt_s * dt_s * 0.5F;
  F(States::POSE_X, States::ACCELERATION_X) = dt2_s2;
  F(States::POSE_Y, States::ACCELERATION_Y) = dt2_s2;
}

template<>
void GenericConstantAcceleration<9U, StatesPoseVelocityAcceleration3D>::compute_jacobian(
  Eigen::Matrix<float32_t, 9U, 9U> & F,
  const std::chrono::nanoseconds & dt)
{
  // identity matrix
  F.setIdentity();

  using float_seconds_t = std::chrono::duration<float>;
  const float32_t dt_s = float_seconds_t{dt}.count();
  // only nonzero elements are ones along diagonal + constant terms for velocity
  F(States::POSE_X, States::VELOCITY_X) = dt_s;
  F(States::POSE_Y, States::VELOCITY_Y) = dt_s;
  F(States::POSE_Z, States::VELOCITY_Z) = dt_s;
  F(States::VELOCITY_X, States::ACCELERATION_X) = dt_s;
  F(States::VELOCITY_Y, States::ACCELERATION_Y) = dt_s;
  F(States::VELOCITY_Z, States::ACCELERATION_Z) = dt_s;
  const float32_t dt2_s2 = dt_s * dt_s * 0.5F;
  F(States::POSE_X, States::ACCELERATION_X) = dt2_s2;
  F(States::POSE_Y, States::ACCELERATION_Y) = dt2_s2;
  F(States::POSE_Z, States::ACCELERATION_Z) = dt2_s2;
}

template<>
void GenericConstantAcceleration<6U, StatesPoseVelocityAcceleration2D>::predict(
  Eigen::Matrix<float32_t, 6U, 1U> & x,
  const std::chrono::nanoseconds & dt) const
{
  using float_seconds_t = std::chrono::duration<float>;
  const float32_t dt_s = float_seconds_t{dt}.count();
  const float32_t ax_dt = dt_s * m_state(States::ACCELERATION_X);
  const float32_t ay_dt = dt_s * m_state(States::ACCELERATION_Y);
  x(States::POSE_X) =
    m_state(States::POSE_X) + (dt_s * m_state(States::VELOCITY_X)) + (0.5F * dt_s * ax_dt);
  x(States::POSE_Y) =
    m_state(States::POSE_Y) + (dt_s * m_state(States::VELOCITY_Y)) + (0.5F * dt_s * ay_dt);
  x(States::VELOCITY_X) = m_state(States::VELOCITY_X) + ax_dt;
  x(States::VELOCITY_Y) = m_state(States::VELOCITY_Y) + ay_dt;
  x(States::ACCELERATION_X) = m_state(States::ACCELERATION_X);
  x(States::ACCELERATION_Y) = m_state(States::ACCELERATION_Y);
}

template<>
void GenericConstantAcceleration<9U, StatesPoseVelocityAcceleration3D>::predict(
  Eigen::Matrix<float32_t, 9U, 1U> & x,
  const std::chrono::nanoseconds & dt) const
{
  using float_seconds_t = std::chrono::duration<float>;
  const float32_t dt_s = float_seconds_t{dt}.count();
  const float32_t ax_dt = dt_s * m_state(States::ACCELERATION_X);
  const float32_t ay_dt = dt_s * m_state(States::ACCELERATION_Y);
  const float32_t az_dt = dt_s * m_state(States::ACCELERATION_Z);
  x(States::POSE_X) =
    m_state(States::POSE_X) + (dt_s * m_state(States::VELOCITY_X)) + (0.5F * dt_s * ax_dt);
  x(States::POSE_Y) =
    m_state(States::POSE_Y) + (dt_s * m_state(States::VELOCITY_Y)) + (0.5F * dt_s * ay_dt);
  x(States::POSE_Z) =
    m_state(States::POSE_Z) + (dt_s * m_state(States::VELOCITY_Z)) + (0.5F * dt_s * az_dt);
  x(States::VELOCITY_X) = m_state(States::VELOCITY_X) + ax_dt;
  x(States::VELOCITY_Y) = m_state(States::VELOCITY_Y) + ay_dt;
  x(States::VELOCITY_Z) = m_state(States::VELOCITY_Z) + ay_dt;
  x(States::ACCELERATION_X) = m_state(States::ACCELERATION_X);
  x(States::ACCELERATION_Y) = m_state(States::ACCELERATION_Y);
  x(States::ACCELERATION_Z) = m_state(States::ACCELERATION_Z);
}

template class GenericConstantAcceleration<6, StatesPoseVelocityAcceleration2D>;
template class GenericConstantAcceleration<9, StatesPoseVelocityAcceleration3D>;

}  // namespace motion_model
}  // namespace motion
}  // namespace autoware
