#include "RoboMasterController.hpp"
#include <cmath>


RoboMasterController::RoboMasterController(CAN_HandleTypeDef* hcan_ptr, bool disable_first_four, bool disable_second_four) {
    this->disable_first_four = disable_first_four;
    this->disable_second_four = disable_second_four;
    this->hcan_ptr = hcan_ptr;
}

void RoboMasterController::start() {
    // Start the RoboMaster
    HAL_CAN_Start(hcan_ptr);
    HAL_CAN_ActivateNotification(hcan_ptr, CAN_IT_RX_FIFO0_MSG_PENDING);

    // Set filter for receiving
    CAN_FilterTypeDef filter;
    filter.FilterIdHigh         = 0;                        // フィルターID(上位16ビット)
    filter.FilterIdLow          = 0;                        // フィルターID(下位16ビット)
    filter.FilterMaskIdHigh     = 0;                        // フィルターマスク(上位16ビット)
    filter.FilterMaskIdLow      = 0;                        // フィルターマスク(下位16ビット)
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;    // フィルタースケール
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;         // フィルターに割り当てるFIFO
    filter.FilterBank           = 0;                        // フィルターバンクNo
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;    // フィルターモード
    filter.SlaveStartFilterBank = 14;                       // スレーブCANの開始フィルターバンクNo
    filter.FilterActivation     = ENABLE;                   // フィルター無効／有効
    HAL_CAN_ConfigFilter(hcan_ptr, &filter);

    // // Send initial message
    // uint8_t initial_message[] = {0x02,0x50,0x00,0x00,0x00,0x00,0x00,0x00};  
    // if(!disable_first_four) send_force(RoboMasterValues::ID::FirstFour, initial_message, sizeof(initial_message));
    // if(!disable_second_four) send_force(RoboMasterValues::ID::SecondFour, initial_message, sizeof(initial_message));
}

void RoboMasterController::send(uint32_t id, uint8_t* data, uint32_t size) {
    // Send a CAN message
    if(HAL_CAN_GetTxMailboxesFreeLevel(hcan_ptr) == 0) {
        return;
    }
    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = id;
    tx_header.IDE = 8U;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = size;
    tx_header.TransmitGlobalTime = DISABLE;
    uint32_t tx_mailbox;
    if(HAL_CAN_AddTxMessage(hcan_ptr, &tx_header, data, &tx_mailbox) != HAL_OK) {
        return;
    }
}

void RoboMasterController::send_force(uint32_t id, uint8_t* data, uint32_t size) {
    // Send a CAN message
    while(HAL_CAN_GetTxMailboxesFreeLevel(hcan_ptr) == 0) {
        HAL_Delay(1);
    }
    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = size;
    tx_header.TransmitGlobalTime = DISABLE;
    uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(hcan_ptr, &tx_header, data, &tx_mailbox);
}

// ループ処理
void RoboMasterController::timer_callback() {
    for (int i = 0; i < 8; i++) {
        // 位置制御 (P制御)
        if(target[i].mode >= RoboMasterValues::ControlMode::Position){
            float error = target[i].pos_radian - receive_data[i].pos_radian_estimated;
            float vel_rps = pid_gain[i].pos_kp * error;
            target[i].vel_rps = (int16_t)vel_rps;
        }

        // 速度制御 (P制御)
        if(target[i].mode >= RoboMasterValues::ControlMode::Velocity){
            float error = target[i].vel_rps - receive_data[i].vel_rps;
            speed_error_integral[i] += error;
            float current = pid_gain[i].speed_kp * error + pid_gain[i].speed_ki * speed_error_integral[i];
            target[i].current = (int16_t)current;
        }
    }

    // 目標電流の送信
    if (!disable_first_four) {
        setCurrentFirstFour(target[0].current, target[1].current, target[2].current, target[3].current);
    }
    if (!disable_second_four) {
        setCurrentSecondFour(target[4].current, target[5].current, target[6].current, target[7].current);
    }
}

void RoboMasterController::setCurrentFirstFour(int16_t current0, int16_t current1, int16_t current2, int16_t current3)
{
    // ビッグエンディアンに変換して送信
    uint8_t currents[8];
    currents[0] = current0 >> 8; currents[1] = current0 & 0xFF;
    currents[2] = current1 >> 8; currents[3] = current1 & 0xFF;
    currents[4] = current2 >> 8; currents[5] = current2 & 0xFF;
    currents[6] = current3 >> 8; currents[7] = current3 & 0xFF;
    send(RoboMasterValues::ID::FirstFour, (uint8_t*)currents, sizeof(currents));
}

void RoboMasterController::setCurrentSecondFour(int16_t current4, int16_t current5, int16_t current6, int16_t current7)
{
    // ビッグエンディアンに変換して送信
    uint8_t currents[8];
    currents[0] = current4 >> 8; currents[1] = current4 & 0xFF;
    currents[2] = current5 >> 8; currents[3] = current5 & 0xFF;
    currents[4] = current6 >> 8; currents[5] = current6 & 0xFF;
    currents[6] = current7 >> 8; currents[7] = current7 & 0xFF;
    send(RoboMasterValues::ID::SecondFour, (uint8_t*)currents, sizeof(currents));
}

// 受信時の処理 割り込みで呼び出す
void RoboMasterController::onReceive(uint8_t id, uint8_t* data, uint8_t size) {
    if (size < 7) {
        return;
    }

    // モーター番号
    uint8_t motor_id = id - RoboMasterValues::ID::ReceiveZeroPoint;

    // ビッグエンディアンからリトルエンディアンに変換
    int16_t angle_raw = (data[0] << 8) | data[1];
    int16_t speed_raw = (data[2] << 8) | data[3];
    uint16_t current_raw = (data[4] << 8) | data[5];

    // 角度
    float pos_radian_raw = (float)angle_raw / RoboMasterValues::RESOLUTION * 2 * M_PI;
    if (pos_radian_raw - receive_data[motor_id].pos_radian_absolute > M_PI) {
        pos_counts[motor_id]--;
    } else if (pos_radian_raw - receive_data[motor_id].pos_radian_absolute < -M_PI) {
        pos_counts[motor_id]++;
    }
    receive_data[motor_id].pos_radian_absolute = pos_radian_raw;

    // 初回受信時を零点とする。
    if(first_receive[motor_id] == false){
        first_receive[motor_id] = true;
        initial_pos[motor_id] = pos_radian_raw;
    }

    receive_data[motor_id].pos_radian_estimated = pos_radian_raw + pos_counts[motor_id] * 2 * M_PI - initial_pos[motor_id];

    // 速度
    receive_data[motor_id].vel_rps = speed_raw / 60.0f;

    // 電流
    receive_data[motor_id].current = current_raw;

    // 温度
    receive_data[motor_id].temperature = data[6];
}

void RoboMasterController::setPIDGain(uint8_t id, float speed_kp, float pos_kp) {
    pid_gain[id].speed_kp = speed_kp;
    pid_gain[id].pos_kp = pos_kp;
}

void RoboMasterController::setControlMode(uint8_t id, RoboMasterValues::ControlMode mode) {
    target[id].mode = mode;
    speed_error_integral[id] = 0;
}

void RoboMasterController::setTargetSpeed(uint8_t id, float velocity) {
    target[id].vel_rps = velocity;
}

void RoboMasterController::setTargetPosition(uint8_t id, float position) {
    target[id].pos_radian = position;
}

void RoboMasterController::setTargetCurrent(uint8_t id, int16_t current) {
    target[id].current = current;
}