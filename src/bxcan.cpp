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

    size_t rx_queue_index =
        BxCAN::filter_index_to_rx_queue_index(hcan, rx_header.FilterMatchIndex);
    if (rx_queue_index >= BxCAN::FILTER_BANK_SIZE) {
      continue;
    }
    Queue<CANMessage> *rx_queue = can.rx_queues_[rx_queue_index];
    if (rx_queue) {
      rx_queue->push(msg, 0);
    }
  }
}

#endif
