#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>

#include "stm32rcos/core.hpp"

#include "stm32rcos/peripheral/can.hpp"

#include "encoder_base.hpp"

namespace stm32rcos {
namespace module {

enum class CyberGearRunMode : uint8_t {
  OPERATION_CONTROL = 0x00,
  POSITION = 0x01,
  SPEED = 0x02,
  CURRENT = 0x03,
};

enum class CyberGearParameter : uint16_t {
  RUN_MODE = 0x7005,
  IQ_REF = 0x7006,
  SPD_REF = 0x700A,
  LIMIT_TORQUE = 0x700B,
  CUR_KP = 0x7010,
  CUR_KI = 0x7011,
  CUR_FILT_GAIN = 0x7014,
  LOC_REF = 0x7016,
  LIMIT_SPD = 0x7017,
  LIMIT_CUR = 0x7018,
  MECH_POS = 0x7019,
  IQF = 0x701A,
  MECH_VEL = 0x701B,
  VBUS = 0x701C,
  ROTATION = 0x701D,
  LOC_KP = 0x701E,
  SPD_KP = 0x701F,
  SPD_KI = 0x7020,
};

struct CyberGearMotorFeedback {
  uint8_t motor_can_id;
  uint8_t fault_status;
  uint8_t mode_status;
  float rad;
  float radps;
  float torque;
  float temperature;
};

template <class CAN_> class CyberGear {
public:
  CyberGear(CAN_ &can, uint8_t motor_can_id, uint8_t host_can_id)
      : can_{can}, motor_can_id_{motor_can_id}, host_can_id_{host_can_id} {
    can_.attach_rx_queue({motor_can_id_ << 8, 0x800FF00, true}, rx_queue_);
  }

  ~CyberGear() { can_.detach_rx_queue(rx_queue_); }

  std::optional<CyberGearMotorFeedback>
  set_operation_control(float torque, float rad, float radps, float kp,
                        float kd) {
    int16_t torque_int =
        std::clamp((torque + 12.0f) / 24.0f * 65535.0f, 0.0f, 65535.0f);
    int16_t rad_int = std::clamp<float>(
        (rad + 4.0f * M_PI) / (8.0f * M_PI) * 65535.0f, 0.0f, 65535.0f);
    int16_t radps_int =
        std::clamp((rad + 30.0f) / 60.0f * 65535, 0.0f, 65535.0f);
    int16_t kp_int = std::clamp(kp / 500.0f * 65535.0f, 0.0f, 65535.0f);
    int16_t kd_int = std::clamp(kd / 5.0f * 65535.0f, 0.0f, 65535.0f);
    std::optional<CyberGearMessage> res = transmit_message(
        {CommunicationType::TYPE_1,
         motor_can_id_,
         {(rad_int >> 8) & 0xFF, rad_int & 0xFF, (radps_int >> 8) & 0xFF,
          radps_int & 0xFF, (kp_int >> 8) & 0xFF, kp_int & 0xFF,
          (kd_int >> 8) & 0xFF, kd_int & 0xFF},

         torque_int});
    if (!res) {
      return std::nullopt;
    }
    return to_cyber_gear_motor_feedback(*res);
  }

  std::optional<CyberGearMotorFeedback> enable() {
    std::optional<CyberGearMessage> res = transmit_message(
        {CommunicationType::TYPE_3, motor_can_id_, {}, host_can_id_});
    if (!res) {
      return std::nullopt;
    }
    return to_cyber_gear_motor_feedback(*res);
  }

  std::optional<CyberGearMotorFeedback> stop(bool clear_fault) {
    std::optional<CyberGearMessage> res =
        transmit_message({CommunicationType::TYPE_4,
                          motor_can_id_,
                          {static_cast<uint8_t>(clear_fault)},
                          host_can_id_});
    if (!res) {
      return std::nullopt;
    }
    return to_cyber_gear_motor_feedback(*res);
  }

  std::optional<CyberGearMotorFeedback> set_mechanical_zero_position() {
    std::optional<CyberGearMessage> res = transmit_message(
        {CommunicationType::TYPE_6, motor_can_id_, {1}, host_can_id_});
    if (!res) {
      return std::nullopt;
    }
    return to_cyber_gear_motor_feedback(*res);
  }

  template <class T>
  std::optional<T> read_parameter(CyberGearParameter parameter) {
    uint16_t index = utility::to_underlying(parameter);
    std::optional<CyberGearMessage> res = transmit_message(
        {CommunicationType::TYPE_17,
         motor_can_id_,
         {static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8)},
         host_can_id_});
    if (!res) {
      return std::nullopt;
    }
    if (res->type != CommunicationType::TYPE_17) {
      return std::nullopt;
    }
    T data;
    std::memcpy(&data, &res->data1[4], sizeof(T));
    return data;
  }

  template <class T>
  std::optional<CyberGearMotorFeedback>
  write_parameter(CyberGearParameter parameter, T data) {
    uint16_t index = utility::to_underlying(parameter);
    CyberGearMessage msg = {
        CommunicationType::TYPE_18,
        motor_can_id_,
        {static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8)},
        host_can_id_};
    std::memcpy(&msg.data1[4], &data, sizeof(T));
    std::optional<CyberGearMessage> res = transmit_message(msg);
    if (!res) {
      return std::nullopt;
    }
    return to_cyber_gear_motor_feedback(*res);
  }

private:
  enum class CommunicationType : uint8_t {
    TYPE_0 = 0,
    TYPE_1 = 1,
    TYPE_2 = 2,
    TYPE_3 = 3,
    TYPE_4 = 4,
    TYPE_6 = 6,
    TYPE_7 = 7,
    TYPE_17 = 17,
    TYPE_18 = 18,
    TYPE_21 = 21,
    TYPE_22 = 22,
  };

  struct CyberGearMessage {
    CommunicationType type;
    uint8_t target_address;
    std::array<uint8_t, 8> data1;
    uint16_t data2;
  };

  CAN_ &can_;
  core::Queue<peripheral::CANMessage> rx_queue_{8};
  uint8_t motor_can_id_;
  uint8_t host_can_id_;

  static inline peripheral::CANMessage
  to_can_message(const CyberGearMessage &cyber_gear_msg) {
    return {static_cast<uint32_t>(
                utility::to_underlying(cyber_gear_msg.type) << 24 |
                cyber_gear_msg.data2 << 8 | cyber_gear_msg.target_address),
            true, 8, cyber_gear_msg.data1};
  }

  static inline std::optional<CyberGearMessage>
  to_cyber_gear_message(const peripheral::CANMessage &can_msg) {
    CyberGearMessage cyber_gear_msg;
    uint8_t type = (can_msg.id >> 24) & 0x1F;
    cyber_gear_msg.target_address = can_msg.id & 0xFF;
    cyber_gear_msg.data1 = can_msg.data;
    cyber_gear_msg.data2 = (can_msg.id >> 8) & 0xFFFF;

    switch (type) {
    case 0:
      cyber_gear_msg.type = CommunicationType::TYPE_0;
      break;
    case 1:
      cyber_gear_msg.type = CommunicationType::TYPE_1;
      break;
    case 2:
      cyber_gear_msg.type = CommunicationType::TYPE_2;
      break;
    case 3:
      cyber_gear_msg.type = CommunicationType::TYPE_3;
      break;
    case 4:
      cyber_gear_msg.type = CommunicationType::TYPE_4;
      break;
    case 6:
      cyber_gear_msg.type = CommunicationType::TYPE_6;
      break;
    case 7:
      cyber_gear_msg.type = CommunicationType::TYPE_7;
      break;
    case 17:
      cyber_gear_msg.type = CommunicationType::TYPE_17;
      break;
    case 18:
      cyber_gear_msg.type = CommunicationType::TYPE_18;
      break;
    case 21:
      cyber_gear_msg.type = CommunicationType::TYPE_21;
      break;
    case 22:
      cyber_gear_msg.type = CommunicationType::TYPE_22;
      break;
    default:
      return std::nullopt;
    }
    return cyber_gear_msg;
  }

  static inline std::optional<CyberGearMotorFeedback>
  to_cyber_gear_motor_feedback(const CyberGearMessage &msg) {
    if (msg.type != CommunicationType::TYPE_2) {
      return std::nullopt;
    }
    return CyberGearMotorFeedback{
        msg.data2 & 0xFF,
        (msg.data2 >> 8) & 0x3F,
        (msg.data2 >> 14) & 0x3,
        ((msg.data1[0] << 8) | msg.data1[1]) / 65535.0f * 8.0f * M_PI -
            4.0f * M_PI,
        ((msg.data1[2] << 8) | msg.data1[3]) / 65535.0f * 60.0f - 30.0f,
        ((msg.data1[4] << 8) | msg.data1[5]) / 65535.0f * 24.0f - 12.0f,
        ((msg.data1[6] << 8) | msg.data1[7]) / 10.0f};
  }

  std::optional<CyberGearMessage>
  transmit_message(const CyberGearMessage &msg) {
    peripheral::CANMessage can_msg = to_can_message(msg);
    rx_queue_.clear();
    if (!can_.transmit(can_msg, 5)) {
      return std::nullopt;
    }
    if (!rx_queue_.pop(can_msg, 5)) {
      return std::nullopt;
    }
    return to_cyber_gear_message(can_msg);
  }
}; // namespace stm32rcos

} // namespace module
} // namespace stm32rcos
