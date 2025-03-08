#pragma once

#include "main.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <iterator>

#include "stm32rcos/core/queue.hpp"

#include "can.hpp"

namespace stm32rcos {
namespace peripheral {

class BxCAN {
public:
  static BxCAN &get_instance(CAN_HandleTypeDef *hcan);

  bool start() {
    if (HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING) !=
        HAL_OK) {
      return false;
    }
    return HAL_CAN_Start(hcan_) == HAL_OK;
  }

  bool stop() {
    if (HAL_CAN_DeactivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING) !=
        HAL_OK) {
      return false;
    }
    return HAL_CAN_Stop(hcan_) == HAL_OK;
  }

  bool transmit(const CANMessage &msg) {
    CAN_TxHeaderTypeDef tx_header = create_tx_header(msg);
    uint32_t tx_mailbox;
    return HAL_CAN_AddTxMessage(hcan_, &tx_header, msg.data.data(),
                                &tx_mailbox) == HAL_OK;
  }

  bool attach_rx_queue(const CANFilter &filter,
                       core::Queue<CANMessage> &queue) {
    size_t rx_queue_index = find_rx_queue_index(nullptr);
    if (rx_queue_index >= FILTER_BANK_SIZE) {
      return false;
    }
    CAN_FilterTypeDef filter_config = create_filter_config(
        filter, rx_queue_index_to_filter_index(hcan_, rx_queue_index));
    if (HAL_CAN_ConfigFilter(hcan_, &filter_config) != HAL_OK) {
      return false;
    }
    rx_queues_[rx_queue_index] = &queue;
    return true;
  }

  bool detach_rx_queue(const core::Queue<CANMessage> &queue) {
    size_t rx_queue_index = find_rx_queue_index(&queue);
    if (rx_queue_index >= FILTER_BANK_SIZE) {
      return false;
    }
    CAN_FilterTypeDef filter_config{};
    filter_config.FilterBank =
        rx_queue_index_to_filter_index(hcan_, rx_queue_index);
    filter_config.FilterActivation = DISABLE;
    if (HAL_CAN_ConfigFilter(hcan_, &filter_config) != HAL_OK) {
      return false;
    }
    rx_queues_[rx_queue_index] = nullptr;
    return true;
  }

private:
  static constexpr uint32_t FILTER_BANK_SIZE = 14;

  CAN_HandleTypeDef *hcan_;
  std::array<core::Queue<CANMessage> *, FILTER_BANK_SIZE> rx_queues_{};

  BxCAN(CAN_HandleTypeDef *hcan) : hcan_{hcan} {}

  size_t find_rx_queue_index(const core::Queue<CANMessage> *queue) {
    return std::distance(
        rx_queues_.begin(),
        std::find(rx_queues_.begin(), rx_queues_.end(), queue));
  }

  static inline uint32_t
  rx_queue_index_to_filter_index(const CAN_HandleTypeDef *hcan,
                                 size_t rx_queue_index) {
#ifdef CAN2
    if (hcan->Instance == CAN2) {
      return rx_queue_index + FILTER_BANK_SIZE;
    }
#endif
    return rx_queue_index;
  }

  static inline size_t
  filter_index_to_rx_queue_index(const CAN_HandleTypeDef *hcan,
                                 uint32_t filter_index) {
#ifdef CAN2
    if (hcan->Instance == CAN2) {
      return filter_index - FILTER_BANK_SIZE;
    }
#endif
    return filter_index;
  }

  static inline CAN_FilterTypeDef create_filter_config(const CANFilter &filter,
                                                       uint32_t filter_index) {
    CAN_FilterTypeDef filter_config{};
    if (filter.ide) {
      filter_config.FilterIdHigh = filter.id >> 13;
      filter_config.FilterIdLow = ((filter.id << 3) & 0xFFFF) | 0x4;
      filter_config.FilterMaskIdHigh = filter.mask >> 13;
      filter_config.FilterMaskIdLow = ((filter.mask << 3) & 0xFFFF) | 0x4;
    } else {
      filter_config.FilterIdHigh = filter.id << 5;
      filter_config.FilterIdLow = 0x0;
      filter_config.FilterMaskIdHigh = filter.mask << 5;
      filter_config.FilterMaskIdLow = 0x0;
    }
    filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter_config.FilterBank = filter_index;
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_config.FilterActivation = ENABLE;
    filter_config.SlaveStartFilterBank = FILTER_BANK_SIZE;
    return filter_config;
  }

  static inline CAN_TxHeaderTypeDef create_tx_header(const CANMessage &msg) {
    CAN_TxHeaderTypeDef tx_header{};
    if (msg.ide) {
      tx_header.ExtId = msg.id;
      tx_header.IDE = CAN_ID_EXT;
    } else {
      tx_header.StdId = msg.id;
      tx_header.IDE = CAN_ID_STD;
    }
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = msg.dlc;
    tx_header.TransmitGlobalTime = DISABLE;
    return tx_header;
  }

  friend void ::HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
};

} // namespace peripheral
} // namespace stm32rcos
