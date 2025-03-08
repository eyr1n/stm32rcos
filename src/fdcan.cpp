#include "main.h"

#ifdef HAL_FDCAN_MODULE_ENABLED

#include "stm32rcos/peripheral/fdcan.hpp"

using stm32rcos::core::Queue;
using stm32rcos::peripheral::CANMessage;
using stm32rcos::peripheral::FDCAN;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  static FDCAN_RxHeaderTypeDef rx_header;
  static CANMessage msg;

  FDCAN &can = FDCAN::get_instance(hfdcan);

  while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header,
                                msg.data.data()) == HAL_OK) {
    if (rx_header.IsFilterMatchingFrame == 1) {
      continue;
    }
    size_t rx_queue_index = rx_header.FilterIndex;
    if (rx_queue_index >= FDCAN::FILTER_BANK_SIZE) {
      continue;
    }
    Queue<CANMessage> *rx_queue = can.rx_queues_[rx_queue_index];
    if (rx_queue) {
      FDCAN::update_rx_message(msg, rx_header);
      rx_queue->push(msg, 0);
    }
  }
}

#endif
