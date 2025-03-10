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
    if (rx_header.IdType == FDCAN_STANDARD_ID) {
      if (rx_header.FilterIndex >= can.std_rx_queues_.size()) {
        continue;
      }
      Queue<CANMessage> *rx_queue = can.std_rx_queues_[rx_header.FilterIndex];
      if (rx_queue) {
        FDCAN::update_rx_message(msg, rx_header);
        rx_queue->push(msg, 0);
      }
    } else if (rx_header.IdType == FDCAN_EXTENDED_ID) {
      if (rx_header.FilterIndex >= can.ext_rx_queues_.size()) {
        continue;
      }
      Queue<CANMessage> *rx_queue = can.ext_rx_queues_[rx_header.FilterIndex];
      if (rx_queue) {
        FDCAN::update_rx_message(msg, rx_header);
        rx_queue->push(msg, 0);
      }
    }
  }
}

#endif
