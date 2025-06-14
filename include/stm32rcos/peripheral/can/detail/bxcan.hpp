#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <iterator>

#include <stm32cubemx_helper/context.hpp>
#include <stm32cubemx_helper/device.hpp>

#include "stm32rcos/core.hpp"

#include "../can_base.hpp"
#include "../can_filter.hpp"
#include "../can_message.hpp"

namespace stm32rcos {
namespace peripheral {

template <auto *Handle, class HandleType> class Can;

template <auto *Handle>
class Can<Handle, CAN_HandleTypeDef *> : public CanBase {
public:
  Can() {
    stm32cubemx_helper::set_context<Handle, Can>(this);
    HAL_CAN_RegisterCallback(
        Handle, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
        [](CAN_HandleTypeDef *hcan) {
          static CAN_RxHeaderTypeDef rx_header;
          static CanMessage msg;

          auto bxcan = stm32cubemx_helper::get_context<Handle, Can>();

          while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,
                                      msg.data.data()) == HAL_OK) {
            if (rx_header.FilterMatchIndex >= Can::FILTER_BANK_SIZE) {
              continue;
            }
            core::Queue<CanMessage> *rx_queue =
                bxcan->rx_queues_[rx_header.FilterMatchIndex];
            if (rx_queue) {
              update_rx_message(msg, rx_header);
              rx_queue->push(msg, 0);
            }
          }
        });
  }

  ~Can() override {
    HAL_CAN_UnRegisterCallback(Handle, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID);
    stm32cubemx_helper::set_context<Handle, Can>(nullptr);
  }

  bool start() override {
    if (HAL_CAN_ActivateNotification(Handle, CAN_IT_RX_FIFO0_MSG_PENDING) !=
        HAL_OK) {
      return false;
    }
    return HAL_CAN_Start(Handle) == HAL_OK;
  }

  bool stop() override {
    if (HAL_CAN_Stop(Handle) != HAL_OK) {
      return false;
    }
    return HAL_CAN_DeactivateNotification(
               Handle, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK;
  }

  bool transmit(const CanMessage &msg, uint32_t timeout) override {
    CAN_TxHeaderTypeDef tx_header = create_tx_header(msg);
    uint32_t tx_mailbox;
    core::TimeoutHelper timeout_helper;
    while (HAL_CAN_AddTxMessage(Handle, &tx_header, msg.data.data(),
                                &tx_mailbox) != HAL_OK) {
      if (timeout_helper.is_timeout(timeout)) {
        return false;
      }
      osDelay(1);
    }
    return true;
  }

  bool attach_rx_queue(const CanFilter &filter,
                       core::Queue<CanMessage> &queue) override {
    size_t rx_queue_index = find_rx_queue_index(nullptr);
    if (rx_queue_index >= FILTER_BANK_SIZE) {
      return false;
    }
    CAN_FilterTypeDef filter_config = create_filter_config(
        filter, rx_queue_index_to_filter_index(Handle, rx_queue_index));
    if (HAL_CAN_ConfigFilter(Handle, &filter_config) != HAL_OK) {
      return false;
    }
    rx_queues_[rx_queue_index] = &queue;
    return true;
  }

  bool detach_rx_queue(const core::Queue<CanMessage> &queue) override {
    size_t rx_queue_index = find_rx_queue_index(&queue);
    if (rx_queue_index >= FILTER_BANK_SIZE) {
      return false;
    }
    CAN_FilterTypeDef filter_config{};
    filter_config.FilterBank =
        rx_queue_index_to_filter_index(Handle, rx_queue_index);
    filter_config.FilterActivation = DISABLE;
    if (HAL_CAN_ConfigFilter(Handle, &filter_config) != HAL_OK) {
      return false;
    }
    rx_queues_[rx_queue_index] = nullptr;
    return true;
  }

private:
  static constexpr uint32_t FILTER_BANK_SIZE = 14;

  std::array<core::Queue<CanMessage> *, FILTER_BANK_SIZE> rx_queues_{};

  Can(const Can &) = delete;
  Can &operator=(const Can &) = delete;

  size_t find_rx_queue_index(const core::Queue<CanMessage> *queue) {
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

  static inline CAN_FilterTypeDef create_filter_config(const CanFilter &filter,
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

  static inline CAN_TxHeaderTypeDef create_tx_header(const CanMessage &msg) {
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

  static inline void update_rx_message(CanMessage &msg,
                                       const CAN_RxHeaderTypeDef &rx_header) {
    switch (rx_header.IDE) {
    case CAN_ID_STD:
      msg.id = rx_header.StdId;
      msg.ide = false;
      break;
    case CAN_ID_EXT:
      msg.id = rx_header.ExtId;
      msg.ide = true;
      break;
    }
    msg.dlc = rx_header.DLC;
  }
};

} // namespace peripheral
} // namespace stm32rcos
