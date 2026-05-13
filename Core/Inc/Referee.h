#ifndef __REFEREE_H
#define __REFEREE_H

#include <stddef.h>
#include <stdint.h>

#include "usart.h"
#include "referee_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct referee referee_t;

typedef enum {
    Referee_Status_Offline = 0,
    Referee_Status_Online = 1,
    Referee_Status_Error = 2
} referee_status_t;//裁判机状态

typedef enum {
    Referee_Ok = 0,
    Referee_ErrParam = -1,
    Referee_ErrState = -2,
    Referee_ErrFrame = -3,
    Referee_ErrBusy = -4
} referee_result_t;//裁判机结果

typedef struct {
    uint32_t timeout_ms;       // 超时时间，超过后置离线
    uint16_t max_frame_len;    // 单帧最大长度，防止异常长度占满解析器
    UART_HandleTypeDef *huart; // 当前模块绑定的 UART
} referee_settings_t;//裁判机设置   

// 从 0x0301 解析出的单发飞镖发射参数
typedef struct {
    uint8_t  dart_id;   // 0-3
    int16_t  v1Speed;   // 摩擦轮 1/3 速度
    int16_t  v2Speed;   // 摩擦轮 2/4 速度
    int32_t  yaw;       // 云台绝对角度(245000=中心)
} referee_dart_param_item_t;

typedef struct {
    ext_dart_info_t dart_info;//飞镖信息
    ext_dart_client_cmd_t dart_client_cmd;//飞镖客户端命令
    referee_dart_param_item_t dart_param_item;//从0x0301收到的飞镖参数
    uint8_t has_dart_info;
    uint8_t has_dart_client_cmd;
    uint8_t has_dart_param;      // 收到新0x0301参数后置1
    uint32_t dart_info_update_tick;
    uint32_t dart_cmd_update_tick;
    uint32_t dart_param_update_tick;
} referee_dart_data_t;//飞镖从裁判系统获取的数据

typedef struct {
    ext_game_robot_status_t robot_status;//机器人状态数据(0x0201)
    ext_game_robot_HP_t robot_hp;//比赛机器人血量数据(0x0003)
    uint8_t has_robot_status;
    uint8_t has_robot_hp;
    uint32_t robot_status_update_tick;
    uint32_t robot_hp_update_tick;
} referee_robot_data_t;//机器人状态与血量数据

typedef struct {
    uint32_t last_rx_tick;
    uint32_t frame_count;
    uint32_t parse_ok_count;
    uint32_t parse_err_count;
    referee_status_t status;
    uint8_t online;
} referee_runtime_t;//运行时数据

typedef struct {//可用函数操作表
    int (*init)(referee_t *me, const referee_settings_t *cfg);
    int (*update)(referee_t *me, uint32_t now_tick);
    int (*rx_bytes)(referee_t *me, const uint8_t *data, size_t len, uint32_t now_tick);//接收字节
    int (*start_uart_rx)(referee_t *me);
    int (*on_uart_rx_cplt)(referee_t *me, uint32_t now_tick);
    int (*get_dart_info)(referee_t *me, ext_dart_info_t *out);
    int (*get_dart_client_cmd)(referee_t *me, ext_dart_client_cmd_t *out);
    int (*get_dart_param)(referee_t *me, referee_dart_param_item_t *out);
    referee_status_t (*get_status)(referee_t *me);
    void (*deinit)(referee_t *me);
} referee_ops_t;//裁判机操作函数

struct referee {
    referee_ops_t *ops;
    referee_settings_t cfg;
    referee_runtime_t runtime;
    referee_dart_data_t dart;
    referee_robot_data_t robot;

    uint8_t rx_byte;
    uint8_t frame_buf[256];
    uint16_t frame_pos;
    uint16_t expected_len;
};

extern const referee_settings_t g_referee_settings_default;
extern const referee_runtime_t g_referee_runtime_init;

int Referee_Init(referee_t *me, const referee_settings_t *cfg);
int Referee_Update(referee_t *me, uint32_t now_tick);
int Referee_RxBytes(referee_t *me, const uint8_t *data, size_t len, uint32_t now_tick);
int Referee_StartUartReceive(referee_t *me);
int Referee_OnUartRxCplt(referee_t *me, uint32_t now_tick);
int Referee_GetDartInfo(referee_t *me, ext_dart_info_t *out);
int Referee_GetDartClientCmd(referee_t *me, ext_dart_client_cmd_t *out);
int Referee_GetDartParam(referee_t *me, referee_dart_param_item_t *out);
referee_status_t Referee_GetStatus(referee_t *me);
void Referee_Deinit(referee_t *me);

#ifdef __cplusplus
}
#endif

#endif