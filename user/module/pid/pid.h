#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    /* PID 参数 */
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  /* 最大输出 */
    fp32 max_iout; /* 积分项最大输出 */

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  /* 微分项缓冲：最新、次新、最旧 */
    fp32 error[3]; /* 误差缓冲：最新、次新、最旧 */

} pid_type_def;

/**
  * @brief          初始化 PID 参数
  * @param[out]     pid: PID 结构体指针
  * @param[in]      mode: PID_POSITION 位置式 PID 或 PID_DELTA 微分式
  * @param[in]      PID: [0]=Kp [1]=Ki [2]=Kd
  * @param[in]      max_out: 最大输出
  * @param[in]      max_iout: 积分项最大输出
  * @retval         无
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          PID 计算
  * @param[out]     pid: PID 结构体指针
  * @param[in]      ref: 反馈值
  * @param[in]      set: 设定值
  * @retval         PID 输出值
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          重置 PID 参数
  * @param[out]     pid: PID 结构体指针
  * @retval         无
  */
extern void PID_clear(pid_type_def *pid);

#endif
