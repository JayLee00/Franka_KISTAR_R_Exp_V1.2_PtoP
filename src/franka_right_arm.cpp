#include "franka_right_arm.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "shm.h"

namespace {

constexpr int kRightArmIndex = 0;
constexpr int kRightHandIndex = 0;
constexpr int kDefaultControlPeriodUs = 1000;
constexpr int kDefaultPrintIntervalUs = 100000;
constexpr double kTargetEps = 1e-4;
constexpr double kDefaultSpeedFactor = 0.1;
constexpr double kMinSpeedFactor = 1e-3;
constexpr double kMaxSpeedFactor = 1.0;

std::mutex g_shm_mutex;
SHMmsgs* g_shm_msgs = nullptr;
int g_shm_id = -1;

void ensureShmAttached() {
  std::lock_guard<std::mutex> lock(g_shm_mutex);
  if (g_shm_msgs == nullptr) {
    init_shm(shm_msg_key, g_shm_id, &g_shm_msgs);
  }
}

void updateRightArmStateToShm(const franka::RobotState& state) {
  std::lock_guard<std::mutex> lock(g_shm_mutex);
  if (g_shm_msgs == nullptr) {
    return;
  }
  for (int i = 0; i < Arm_DOF; ++i) {
    g_shm_msgs->Arm_j_pos[kRightArmIndex][i] = state.q[i];
    g_shm_msgs->Arm_j_vel[kRightArmIndex][i] = state.dq[i];
    g_shm_msgs->Arm_j_tq[kRightArmIndex][i] = state.tau_J[i];
  }
  std::memcpy(g_shm_msgs->Arm_C_Pos[kRightArmIndex], state.O_T_EE.data(), 16 * sizeof(double));
}

std::array<double, 7> readRightArmTargetFromShm(const std::array<double, 7>& fallback) {
  std::array<double, 7> target = fallback;
  std::lock_guard<std::mutex> lock(g_shm_mutex);
  if (g_shm_msgs == nullptr) {
    return target;
  }
  for (int i = 0; i < Arm_DOF; ++i) {
    target[i] = g_shm_msgs->Arm_j_tar[kRightArmIndex][i];
  }
  return target;
}

double readRightArmSpeedFromShm(double fallback) {
  double speed = fallback;
  std::lock_guard<std::mutex> lock(g_shm_mutex);
  if (g_shm_msgs == nullptr) {
    return speed;
  }
  speed = g_shm_msgs->Arm_Speed_Factor[kRightArmIndex];
  return speed;
}

double sanitizeSpeedFactor(double speed, double fallback) {
  if (!std::isfinite(speed)) {
    return fallback;
  }
  if (speed < kMinSpeedFactor) {
    return kMinSpeedFactor;
  }
  if (speed > kMaxSpeedFactor) {
    return kMaxSpeedFactor;
  }
  return speed;
}

bool isTargetChanged(const std::array<double, 7>& a, const std::array<double, 7>& b) {
  for (size_t i = 0; i < a.size(); ++i) {
    if (std::abs(a[i] - b[i]) > kTargetEps) {
      return true;
    }
  }
  return false;
}

}  // namespace

void* franka_control_thread_R(void* ptr) {
  FrankaRightArmConfig default_cfg{"172.16.0.1", kDefaultSpeedFactor, kDefaultControlPeriodUs, kDefaultPrintIntervalUs};
  const FrankaRightArmConfig* cfg = static_cast<const FrankaRightArmConfig*>(ptr);
  const FrankaRightArmConfig& cfg_ref = (cfg == nullptr) ? default_cfg : *cfg;
  const int control_period_us = (cfg_ref.control_period_us > 0) ? cfg_ref.control_period_us : kDefaultControlPeriodUs;

  try {
    ensureShmAttached();
    std::cerr << "Connecting Franka right arm: " << cfg_ref.robot_ip << std::endl;

    franka::Robot robot(cfg_ref.robot_ip);
    setDefaultBehavior(robot);
    robot.automaticErrorRecovery();

    //std::lock_guard<std::mutex> lock(g_shm_mutex);
    //g_shm_msgs->j_tar[0] = 0;

     // Optional: tighter collision limits for safety (same as communication_test)
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    franka::RobotState robot_state = robot.readOnce();
    updateRightArmStateToShm(robot_state);

    std::array<double, 7> last_target = robot_state.q;
    {
      std::lock_guard<std::mutex> lock(g_shm_mutex);
      if (g_shm_msgs != nullptr) {
        for (int i = 0; i < Arm_DOF; ++i) {
          g_shm_msgs->Arm_j_tar[kRightArmIndex][i] = robot_state.q[i];
        }
        g_shm_msgs->Arm_Speed_Factor[kRightArmIndex] = kDefaultSpeedFactor;
      }
    }

    while (true) {
      robot_state = robot.readOnce();
      updateRightArmStateToShm(robot_state);

      const std::array<double, 7> target = readRightArmTargetFromShm(last_target);
      if (isTargetChanged(target, last_target)) {
        const double speed_factor =
            sanitizeSpeedFactor(readRightArmSpeedFromShm(cfg_ref.speed_factor), cfg_ref.speed_factor);
        MotionGenerator motion_generator(speed_factor, target);
        robot.control(motion_generator);
        last_target = target;
        robot_state = robot.readOnce();
        updateRightArmStateToShm(robot_state);
      }

      std::this_thread::sleep_for(std::chrono::microseconds(control_period_us));
    }
  } catch (const franka::Exception& e) {
    std::cerr << "Franka right arm: " << e.what() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Franka right arm error: " << e.what() << std::endl;
  }

  return nullptr;
}

void* data_print_thread(void* ptr) {
  int interval_us = kDefaultPrintIntervalUs;
  if (ptr != nullptr) {
    interval_us = *static_cast<int*>(ptr);
  }
  if (interval_us <= 0) {
    interval_us = kDefaultPrintIntervalUs;
  }

  ensureShmAttached();

  while (true) {
    if (g_shm_msgs != nullptr) {
      std::lock_guard<std::mutex> lock(g_shm_mutex);
      std::cout << "\rR_arm q[0..2]=[" << g_shm_msgs->Arm_j_pos[kRightArmIndex][0] << ", "
                << g_shm_msgs->Arm_j_pos[kRightArmIndex][1] << ", "
                << g_shm_msgs->Arm_j_pos[kRightArmIndex][2] << "] "
                << "tau[0..2]=[" << g_shm_msgs->Arm_j_tq[kRightArmIndex][0] << ", "
                << g_shm_msgs->Arm_j_tq[kRightArmIndex][1] << ", "
                << g_shm_msgs->Arm_j_tq[kRightArmIndex][2] << "] "
                << "Hand_j0=" << g_shm_msgs->j_pos[kRightHandIndex][0] << "   ";
      std::cout.flush();
    }
    std::this_thread::sleep_for(std::chrono::microseconds(interval_us));
  }

  return nullptr;
}
