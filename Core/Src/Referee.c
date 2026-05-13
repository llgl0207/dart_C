#include "Referee.h"
#include "referee_protocol.h"

#include <string.h>

static int Referee_init(referee_t *me, const referee_settings_t *cfg);
static int Referee_update(referee_t *me, uint32_t now_tick);
static int Referee_rx_bytes(referee_t *me, const uint8_t *data, size_t len, uint32_t now_tick);
static int Referee_start_uart_rx(referee_t *me);
static int Referee_on_uart_rx_cplt(referee_t *me, uint32_t now_tick);
static int Referee_get_dart_info(referee_t *me, ext_dart_info_t *out);
static int Referee_get_dart_client_cmd(referee_t *me, ext_dart_client_cmd_t *out);
static int Referee_get_dart_param(referee_t *me, referee_dart_param_item_t *out);
static referee_status_t Referee_get_status(referee_t *me);
static void Referee_deinit(referee_t *me);

static void referee_parser_reset(referee_t *me);
static int referee_try_parse_frame(referee_t *me, uint32_t now_tick);
static void referee_handle_cmd(referee_t *me, uint16_t cmd_id, const uint8_t *payload, uint16_t payload_len, uint32_t now_tick);

static uint16_t referee_get_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static referee_ops_t referee_default_ops = {
    .init = Referee_init,
    .update = Referee_update,
    .rx_bytes = Referee_rx_bytes,
    .start_uart_rx = Referee_start_uart_rx,
    .on_uart_rx_cplt = Referee_on_uart_rx_cplt,
    .get_dart_info = Referee_get_dart_info,
    .get_dart_client_cmd = Referee_get_dart_client_cmd,
    .get_dart_param = Referee_get_dart_param,
    .get_status = Referee_get_status,
    .deinit = Referee_deinit
};

const referee_settings_t g_referee_settings_default = {
    .timeout_ms = 100,// 100ms 超时时间，超过后置离线
    .max_frame_len = 128,
    .huart = &huart6
};

const referee_runtime_t g_referee_runtime_init = {
    .last_rx_tick = 0,
    .frame_count = 0,
    .status = Referee_Status_Offline,
    .online = 0
};

static int Referee_init(referee_t *me, const referee_settings_t *cfg)
{
    if ((me == NULL) || (cfg == NULL)) {
        return Referee_ErrParam;
    }

    if ((cfg->huart == NULL) || (cfg->max_frame_len == 0U) || (cfg->max_frame_len > (uint16_t)sizeof(me->frame_buf))) {
        return Referee_ErrParam;
    }

    me->cfg = *cfg;
    me->runtime = g_referee_runtime_init;
    memset(&me->dart, 0, sizeof(me->dart));
    memset(&me->robot, 0, sizeof(me->robot));
    me->rx_byte = 0U;
    referee_parser_reset(me);

    return Referee_Ok;
}

static int Referee_update(referee_t *me, uint32_t now_tick)
{
    if (me == NULL) {
        return Referee_ErrParam;
    }

    if ((me->runtime.online != 0U) && ((now_tick - me->runtime.last_rx_tick) > me->cfg.timeout_ms)) {
        me->runtime.online = 0U;
        me->runtime.status = Referee_Status_Offline;
    }

    return Referee_Ok;
}

static int Referee_rx_bytes(referee_t *me, const uint8_t *data, size_t len, uint32_t now_tick)
{
    size_t i;

    if ((me == NULL) || ((data == NULL) && (len != 0U))) {
        return Referee_ErrParam;
    }

    for (i = 0U; i < len; i++) {
        if (me->frame_pos >= me->cfg.max_frame_len) {
            me->runtime.parse_err_count++;
            referee_parser_reset(me);
        }

        me->frame_buf[me->frame_pos++] = data[i];
        (void)referee_try_parse_frame(me, now_tick);
    }

    return Referee_Ok;
}

static int Referee_start_uart_rx(referee_t *me)
{
    if ((me == NULL) || (me->cfg.huart == NULL)) {
        return Referee_ErrParam;
    }

    if (HAL_UART_Receive_IT(me->cfg.huart, &me->rx_byte, 1U) != HAL_OK) {
        return Referee_ErrBusy;
    }

    return Referee_Ok;
}

static int Referee_on_uart_rx_cplt(referee_t *me, uint32_t now_tick)
{
    int ret;

    if (me == NULL) {
        return Referee_ErrParam;
    }

    ret = Referee_rx_bytes(me, &me->rx_byte, 1U, now_tick);
    if (ret != Referee_Ok) {
        return ret;
    }

    if (HAL_UART_Receive_IT(me->cfg.huart, &me->rx_byte, 1U) != HAL_OK) {
        return Referee_ErrBusy;
    }

    return Referee_Ok;
}

static int Referee_get_dart_info(referee_t *me, ext_dart_info_t *out)
{
    if ((me == NULL) || (out == NULL)) {
        return Referee_ErrParam;
    }

    if (me->dart.has_dart_info == 0U) {
        return Referee_ErrState;
    }

    *out = me->dart.dart_info;

    return Referee_Ok;
}

static int Referee_get_dart_client_cmd(referee_t *me, ext_dart_client_cmd_t *out)
{
    if ((me == NULL) || (out == NULL)) {
        return Referee_ErrParam;
    }

    if (me->dart.has_dart_client_cmd == 0U) {
        return Referee_ErrState;
    }

    *out = me->dart.dart_client_cmd;

    return Referee_Ok;
}

static int Referee_get_dart_param(referee_t *me, referee_dart_param_item_t *out)
{
    if ((me == NULL) || (out == NULL)) {
        return Referee_ErrParam;
    }

    if (me->dart.has_dart_param == 0U) {
        return Referee_ErrState;
    }

    *out = me->dart.dart_param_item;
    me->dart.has_dart_param = 0U; // 消费后清除标志

    return Referee_Ok;
}

static referee_status_t Referee_get_status(referee_t *me)
{
    if (me == NULL) {
        return Referee_Status_Error;
    }

    return me->runtime.status;
}

static void Referee_deinit(referee_t *me)
{
    if (me == NULL) {
        return;
    }

    memset(&me->runtime, 0, sizeof(me->runtime));
    memset(&me->dart, 0, sizeof(me->dart));
    memset(&me->robot, 0, sizeof(me->robot));
    me->runtime.status = Referee_Status_Offline;
    referee_parser_reset(me);
    me->ops = NULL;
}

static void referee_parser_reset(referee_t *me)
{
    me->frame_pos = 0U;
    me->expected_len = 0U;
}

static int referee_try_parse_frame(referee_t *me, uint32_t now_tick)
{
    uint16_t data_len;
    uint16_t cmd_id;

    if (me->frame_pos == 0U) {
        return Referee_ErrState;
    }

    if (me->frame_buf[0] != REFEREE_SOF) {
        me->runtime.parse_err_count++;
        referee_parser_reset(me);
        return Referee_ErrFrame;
    }

    if (me->frame_pos < LEN_HEADER) {
        return Referee_ErrState;
    }

    if (me->expected_len == 0U) {
        data_len = referee_get_u16_le(&me->frame_buf[DATA_LENGTH]);
        me->expected_len = (uint16_t)(LEN_HEADER + LEN_CMDID + data_len + LEN_TAIL);
        if ((me->expected_len > me->cfg.max_frame_len) || (me->expected_len > (uint16_t)sizeof(me->frame_buf))) {
            me->runtime.parse_err_count++;
            referee_parser_reset(me);
            return Referee_ErrFrame;
        }
    }

    if (me->frame_pos < me->expected_len) {
        return Referee_ErrState;
    }

    cmd_id = referee_get_u16_le(&me->frame_buf[CMD_ID_Offset]);
    data_len = referee_get_u16_le(&me->frame_buf[DATA_LENGTH]);
    referee_handle_cmd(me, cmd_id, &me->frame_buf[DATA_Offset], data_len, now_tick);

    me->runtime.frame_count++;
    me->runtime.parse_ok_count++;
    me->runtime.last_rx_tick = now_tick;
    me->runtime.online = 1U;
    me->runtime.status = Referee_Status_Online;

    referee_parser_reset(me);
    return Referee_Ok;
}

static void referee_handle_cmd(referee_t *me, uint16_t cmd_id, const uint8_t *payload, uint16_t payload_len, uint32_t now_tick)
{
    if ((me == NULL) || (payload == NULL)) {
        return;
    }

    if ((cmd_id == ID_Dart_Info) && (payload_len >= LEN_dart_info_t)) {
        memcpy(&me->dart.dart_info, payload, sizeof(ext_dart_info_t));
        me->dart.has_dart_info = 1U;
        me->dart.dart_info_update_tick = now_tick;
        return;
    }

    if ((cmd_id == ID_Ddart_Client_Cmd) && (payload_len >= LEN_dart_client_cmd)) {
        memcpy(&me->dart.dart_client_cmd, payload, sizeof(ext_dart_client_cmd_t));
        me->dart.has_dart_client_cmd = 1U;
        me->dart.dart_cmd_update_tick = now_tick;
        return;
    }

    if ((cmd_id == ID_game_robot_state) && (payload_len >= LEN_game_robot_state)) {
        memcpy(&me->robot.robot_status, payload, sizeof(ext_game_robot_status_t));
        me->robot.has_robot_status = 1U;
        me->robot.robot_status_update_tick = now_tick;
        return;
    }

    if ((cmd_id == ID_game_robot_survivors) && (payload_len >= LEN_game_robot_HP)) {
        memcpy(&me->robot.robot_hp, payload, sizeof(ext_game_robot_HP_t));
        me->robot.has_robot_hp = 1U;
        me->robot.robot_hp_update_tick = now_tick;
        return;
    }

    /* 0x0301: 机器人交互数据（选手端下发飞镖参数）
       数据段: [子内容ID(2B)] [发送者ID(2B)] [接收者ID(2B)] [自定义数据]
       子内容ID=0x0001: 自定义飞镖参数
       自定义数据(9 bytes):
         [0]    : dart_id (0-3)
         [1-2]  : v1Speed (int16, Big Endian)
         [3-4]  : v2Speed (int16, Big Endian)
         [5-8]  : yaw绝对角度 (int32, Big Endian, 245000=中心)
    */
    if ((cmd_id == ID_student_interactive) && (payload_len >= 15U)) {
        uint16_t sub_id = referee_get_u16_le(&payload[0]);
        if (sub_id == 0x0001U) {
            const uint8_t *d = &payload[6]; // 跳过子内容ID(2)+发送者ID(2)+接收者ID(2)
            me->dart.dart_param_item.dart_id = d[0] & 0x03U;
            me->dart.dart_param_item.v1Speed = (int16_t)(((uint16_t)d[1] << 8) | (uint16_t)d[2]);
            me->dart.dart_param_item.v2Speed = (int16_t)(((uint16_t)d[3] << 8) | (uint16_t)d[4]);
            me->dart.dart_param_item.yaw    = (int32_t)(((uint32_t)d[5] << 24) | ((uint32_t)d[6] << 16) |
                                                        ((uint32_t)d[7] << 8)  | (uint32_t)d[8]);
            me->dart.has_dart_param = 1U;
            me->dart.dart_param_update_tick = now_tick;
        }
        return;
    }
}

int Referee_Init(referee_t *me, const referee_settings_t *cfg)
{
    if (me == NULL) {
        return Referee_ErrParam;
    }

    me->ops = &referee_default_ops;

    if (cfg == NULL) {
        return me->ops->init(me, &g_referee_settings_default);
    }

    return me->ops->init(me, cfg);
}

int Referee_Update(referee_t *me, uint32_t now_tick)
{
    if ((me == NULL) || (me->ops == NULL) || (me->ops->update == NULL)) {
        return Referee_ErrState;
    }

    return me->ops->update(me, now_tick);
}

int Referee_RxBytes(referee_t *me, const uint8_t *data, size_t len, uint32_t now_tick)
{
    if ((me == NULL) || (me->ops == NULL) || (me->ops->rx_bytes == NULL)) {
        return Referee_ErrState;
    }

    return me->ops->rx_bytes(me, data, len, now_tick);
}

int Referee_StartUartReceive(referee_t *me)
{
    if ((me == NULL) || (me->ops == NULL) || (me->ops->start_uart_rx == NULL)) {
        return Referee_ErrState;
    }

    return me->ops->start_uart_rx(me);
}

int Referee_OnUartRxCplt(referee_t *me, uint32_t now_tick)
{
    if ((me == NULL) || (me->ops == NULL) || (me->ops->on_uart_rx_cplt == NULL)) {
        return Referee_ErrState;
    }

    return me->ops->on_uart_rx_cplt(me, now_tick);
}

int Referee_GetDartInfo(referee_t *me, ext_dart_info_t *out)
{
    if ((me == NULL) || (me->ops == NULL) || (me->ops->get_dart_info == NULL)) {
        return Referee_ErrState;
    }

    return me->ops->get_dart_info(me, out);
}

int Referee_GetDartClientCmd(referee_t *me, ext_dart_client_cmd_t *out)
{
    if ((me == NULL) || (me->ops == NULL) || (me->ops->get_dart_client_cmd == NULL)) {
        return Referee_ErrState;
    }

    return me->ops->get_dart_client_cmd(me, out);
}

int Referee_GetDartParam(referee_t *me, referee_dart_param_item_t *out)
{
    if ((me == NULL) || (me->ops == NULL) || (me->ops->get_dart_param == NULL)) {
        return Referee_ErrState;
    }

    return me->ops->get_dart_param(me, out);
}

referee_status_t Referee_GetStatus(referee_t *me)
{
    if ((me == NULL) || (me->ops == NULL) || (me->ops->get_status == NULL)) {
        return Referee_Status_Error;
    }

    return me->ops->get_status(me);
}

void Referee_Deinit(referee_t *me)
{
    if ((me == NULL) || (me->ops == NULL) || (me->ops->deinit == NULL)) {
        return;
    }

    me->ops->deinit(me);
}