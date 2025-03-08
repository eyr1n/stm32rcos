#include "main.h"

#ifdef HAL_CAN_MODULE_ENABLED

#include "stm32rcos/peripheral/bxcan.hpp"

using stm32rcos::core::Queue;
using stm32rcos::peripheral::BxCAN;
using stm32rcos::peripheral::CANMessage;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  static CAN_RxHeaderTypeDef rx_header;
  static CANMessage msg;

  BxCAN &can = BxCAN::get_instance(hcan);

  while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,
                              msg.data.data()) == HAL_OK) {
    size_t rx_queue_index =
        BxCAN::filter_index_to_rx_queue_index(hcan, rx_header.FilterMatchIndex);
    if (rx_queue_index >= BxCAN::FILTER_BANK_SIZE) {
      continue;
    }
    Queue<CANMessage> *rx_queue = can.rx_queues_[rx_queue_index];
    if (rx_queue) {
      BxCAN::update_rx_message(msg, rx_header);
      rx_queue->push(msg, 0);
    }
  }
}

#endif
