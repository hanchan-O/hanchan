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
    /* PID ??? */
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  /* ???? */
    fp32 max_iout; /* ?????? */

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  /* ????0 ???1 ???2 ??? */
    fp32 error[3]; /* ????0 ???1 ???2 ??? */

} pid_type_def;

/**
  * @brief          ??? PID ???
  * @param[out]     pid: PID ????
  * @param[in]      mode: PID_POSITION ??????PID_DELTA ???
  * @param[in]      PID: [0]=Kp [1]=Ki [2]=Kd
  * @param[in]      max_out: ????
  * @param[in]      max_iout: ????
  * @retval         ?
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          PID ??
  * @param[out]     pid: PID ????
  * @param[in]      ref: ???
  * @param[in]      set: ???
  * @retval         PID ??
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          ?? PID ???????
  * @param[out]     pid: PID ????
  * @retval         ?
  */
extern void PID_clear(pid_type_def *pid);

#endif
