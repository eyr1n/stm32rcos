#pragma once

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <vector>

#include <FreeRTOS.h>
#include <task.h>

#include "stm32_hal.h"

#include "stm32rcos/core/queue.hpp"

#include "can.hpp"

namespace stm32rcos {
namespace peripheral {

class FDCAN {
public:
  static FDCAN &get_instance(FDCAN_HandleTypeDef *hfdcan);

  bool start() {
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan_, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
      return false;
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0) != HAL_OK) {
      return false;
    }
    return HAL_FDCAN_Start(hfdcan_) == HAL_OK;
  }

  bool stop() {
    if (HAL_FDCAN_Stop(hfdcan_) != HAL_OK) {
      return false;
    }
    return HAL_FDCAN_DeactivateNotification(
               hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == HAL_OK;
  }

  bool transmit(const CANMessage &msg, uint32_t timeout) {
    FDCAN_TxHeaderTypeDef tx_header = create_tx_header(msg);
    TimeOut_t timeout_state;
    vTaskSetTimeOutState(&timeout_state);
    while (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &tx_header,
                                         msg.data.data()) != HAL_OK) {
      if (xTaskCheckForTimeOut(&timeout_state, &timeout) == pdTRUE) {
        return false;
      }
      osDelay(1);
    }
    return true;
  }

  bool attach_rx_queue(const CANFilter &filter,
                       core::Queue<CANMessage> &queue) {
    if (filter.ide) {
      size_t rx_queue_index = find_ext_rx_queue_index(nullptr);
      if (rx_queue_index >= ext_rx_queues_.size()) {
        return false;
      }
      FDCAN_FilterTypeDef filter_config =
          create_filter_config(filter, rx_queue_index);
      if (HAL_FDCAN_ConfigFilter(hfdcan_, &filter_config) != HAL_OK) {
        return false;
      }
      ext_rx_queues_[rx_queue_index] = &queue;
    } else {
      size_t rx_queue_index = find_std_rx_queue_index(nullptr);
      if (rx_queue_index >= std_rx_queues_.size()) {
        return false;
      }
      FDCAN_FilterTypeDef filter_config =
          create_filter_config(filter, rx_queue_index);
      if (HAL_FDCAN_ConfigFilter(hfdcan_, &filter_config) != HAL_OK) {
        return false;
      }
      std_rx_queues_[rx_queue_index] = &queue;
    }
    return true;
  }

  bool detach_rx_queue(const core::Queue<CANMessage> &queue) {
    size_t rx_queue_index = find_std_rx_queue_index(&queue);
    if (rx_queue_index < std_rx_queues_.size()) {
      FDCAN_FilterTypeDef filter_config{};
      filter_config.FilterIndex = rx_queue_index;
      filter_config.FilterConfig = FDCAN_FILTER_DISABLE;
      if (HAL_FDCAN_ConfigFilter(hfdcan_, &filter_config) != HAL_OK) {
        return false;
      }
      std_rx_queues_[rx_queue_index] = nullptr;
    } else {
      rx_queue_index = find_ext_rx_queue_index(&queue);
      if (rx_queue_index >= ext_rx_queues_.size()) {
        return false;
      }
      FDCAN_FilterTypeDef filter_config{};
      filter_config.FilterIndex = rx_queue_index;
      filter_config.FilterConfig = FDCAN_FILTER_DISABLE;
      if (HAL_FDCAN_ConfigFilter(hfdcan_, &filter_config) != HAL_OK) {
        return false;
      }
      ext_rx_queues_[rx_queue_index] = nullptr;
    }
    return true;
  }

private:
  FDCAN_HandleTypeDef *hfdcan_;
  std::vector<core::Queue<CANMessage> *> std_rx_queues_{};
  std::vector<core::Queue<CANMessage> *> ext_rx_queues_{};

  FDCAN(FDCAN_HandleTypeDef *hfdcan)
      : hfdcan_{hfdcan}, std_rx_queues_(hfdcan_->Init.StdFiltersNbr, nullptr),
        ext_rx_queues_(hfdcan_->Init.ExtFiltersNbr, nullptr) {}

  FDCAN(const FDCAN &) = delete;
  FDCAN &operator=(const FDCAN &) = delete;
  FDCAN(FDCAN &&) = delete;
  FDCAN &operator=(FDCAN &&) = delete;

  size_t find_std_rx_queue_index(const core::Queue<CANMessage> *queue) {
    return std::distance(
        std_rx_queues_.begin(),
        std::find(std_rx_queues_.begin(), std_rx_queues_.end(), queue));
  }

  size_t find_ext_rx_queue_index(const core::Queue<CANMessage> *queue) {
    return std::distance(
        ext_rx_queues_.begin(),
        std::find(ext_rx_queues_.begin(), ext_rx_queues_.end(), queue));
  }

  static inline FDCAN_FilterTypeDef
  create_filter_config(const CANFilter &filter, uint32_t filter_index) {
    FDCAN_FilterTypeDef filter_config{};
    if (filter.ide) {
      filter_config.IdType = FDCAN_EXTENDED_ID;
    } else {
      filter_config.IdType = FDCAN_STANDARD_ID;
    }
    filter_config.FilterIndex = filter_index;
    filter_config.FilterType = FDCAN_FILTER_MASK;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter_config.FilterID1 = filter.id;
    filter_config.FilterID2 = filter.mask;
    return filter_config;
  }

  static inline FDCAN_TxHeaderTypeDef create_tx_header(const CANMessage &msg) {
    FDCAN_TxHeaderTypeDef tx_header{};
    tx_header.Identifier = msg.id;
    if (msg.ide) {
      tx_header.IdType = FDCAN_EXTENDED_ID;
    } else {
      tx_header.IdType = FDCAN_STANDARD_ID;
    }
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    switch (msg.dlc) {
    case 0:
      tx_header.DataLength = FDCAN_DLC_BYTES_0;
      break;
    case 1:
      tx_header.DataLength = FDCAN_DLC_BYTES_1;
      break;
    case 2:
      tx_header.DataLength = FDCAN_DLC_BYTES_2;
      break;
    case 3:
      tx_header.DataLength = FDCAN_DLC_BYTES_3;
      break;
    case 4:
      tx_header.DataLength = FDCAN_DLC_BYTES_4;
      break;
    case 5:
      tx_header.DataLength = FDCAN_DLC_BYTES_5;
      break;
    case 6:
      tx_header.DataLength = FDCAN_DLC_BYTES_6;
      break;
    case 7:
      tx_header.DataLength = FDCAN_DLC_BYTES_7;
      break;
    case 8:
      tx_header.DataLength = FDCAN_DLC_BYTES_8;
      break;
    }
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    return tx_header;
  }

  static inline void update_rx_message(CANMessage &msg,
                                       const FDCAN_RxHeaderTypeDef &rx_header) {
    msg.id = rx_header.Identifier;
    if (rx_header.IdType == FDCAN_STANDARD_ID) {
      msg.ide = false;
    } else if (rx_header.IdType == FDCAN_EXTENDED_ID) {
      msg.ide = true;
    }
    switch (rx_header.DataLength) {
    case FDCAN_DLC_BYTES_0:
      msg.dlc = 0;
      break;
    case FDCAN_DLC_BYTES_1:
      msg.dlc = 1;
      break;
    case FDCAN_DLC_BYTES_2:
      msg.dlc = 2;
      break;
    case FDCAN_DLC_BYTES_3:
      msg.dlc = 3;
      break;
    case FDCAN_DLC_BYTES_4:
      msg.dlc = 4;
      break;
    case FDCAN_DLC_BYTES_5:
      msg.dlc = 5;
      break;
    case FDCAN_DLC_BYTES_6:
      msg.dlc = 6;
      break;
    case FDCAN_DLC_BYTES_7:
      msg.dlc = 7;
      break;
    case FDCAN_DLC_BYTES_8:
      msg.dlc = 8;
      break;
    }
  }

  friend void ::HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t RxFifo0ITs);
};

} // namespace peripheral
} // namespace stm32rcos
