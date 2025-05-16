#ifndef _LIBELYBOT_FDCAN_H
#define _LIBELYBOT_FDCAN_H


#include "main.h"




/*
*   MOTOR_MODEL ����ѡ�����ͺţ����������������ء�
*   MOTOR_MODEL = 5046
*   MOTOR_MODEL = 4538
*   MOTOR_MODEL = 50471  5047����
*   MOTOR_MODEL = 50472  5047˫��
*/
#define  MOTOR_MODEL  5046

#define  MOTOR_5046_TQE_CORRECT_FLOAT    1.2037
#define  MOTOR_5046_TQE_CORRECT_INT32    1.2038
#define  MOTOR_5046_TQE_CORRECT_INT16    1.2001

#define  MOTOR_4538_TQE_CORRECT_FLOAT    1.1925
#define  MOTOR_4538_TQE_CORRECT_INT32    1.2090
#define  MOTOR_4538_TQE_CORRECT_INT16    1.2029

#define  MOTOR_5047_1_TQE_CORRECT_FLOAT  1
#define  MOTOR_5047_1_TQE_CORRECT_INT32  1
#define  MOTOR_5047_1_TQE_CORRECT_INT16  1

#define  MOTOR_5047_2_TQE_CORRECT_FLOAT  1
#define  MOTOR_5047_2_TQE_CORRECT_INT32  1
#define  MOTOR_5047_2_TQE_CORRECT_INT16  1


/*
*   POS_FLAG ���ڲ���λ��ģʽ��������ģʽ������ģʽֻ���������Ͳ�ͬ��Ч������ -0.5ת~0.5ת ������ת��
*   POS_FLAG = 1  float
*   POS_FLAG = 2  int32
*   POS_FLAG = 3  int8
*/
#define  POS_FLAG  0


/*
*   READ_MOTOR_FLAG ���ڸı��ȡ���״̬����������
*   READ_MOTOR_FLAG = 1  float
*   READ_MOTOR_FLAG = 2  int32
*   READ_MOTOR_FLAG = 3  int8
*/
#define  READ_MOTOR_FLAG   1


/*
*   POS_REZERO ���ڲ��Ե����������㹦�ܡ�
*   ����Դ˹��ܽ� POS_REZERO ��ע�ͼ��ɡ�
*   Ч���ǵ���ϵ� 3 ��󽫵�ǰλ����Ϊ���
*   ע�⣺���õ��ֹͣ����������λ��������Ч
*/
// #define  POS_REZERO


/*
*   MOTOR_STOP ���ڲ��Ե��ֹͣ���ܡ�
*   ����Դ˹��ܽ� MOTOR_STOP ��ע�ͼ��ɡ�
*   Ч���ǵ���ϵ� 3 ���ֹͣ��
*   ע�⣺����ϵ�����ƺ���ʹ�ã����� MOTOR_STOP �겻��ı��κε�����ƺ�����
*/
// #define  MOTOR_STOP


/*
*   MOTOR_BRAKE ���ڲ��Ե��ɲ�����ܹ��ܡ�
*   ����Դ˹��ܽ� MOTOR_BRAKE ��ע�ͼ��ɡ�
*   Ч���ǵ���ϵ� 3 ���ɲ����
*   ע�⣺����ϵ�����ƺ���ʹ�ã����� MOTOR_BRAKE �겻��ı��κε�����ƺ�����
*/
// #define  MOTOR_BRAKE


/* �����������͵������� */
#define  NAN_FLOAT  NAN
#define  NAN_INT32  0x80000000
#define  NAN_INT16  0x8000
#define  NAN_INT8   0x80


// �˴����ݵ���ĸ�����ID�ţ��Զ�����������
typedef enum
{
    MOTOR1,
    MOTOR2,
    MOTOR3,
    MOTOR4,
    MOTOR5,
    MOTOR6,
    MOTOR7,
    MOTOR8,
    MOTOR9,
    MOTOR10,
    MOTOR11,
    MOTOR12,
    MOTOR13,
    MOTOR14,
    MOTOR15,
    MOTOR16
} motor_e;


#if READ_MOTOR_FLAG == 1
#define MOTOR_SIZE 18
typedef float motor_state_type;
#elif READ_MOTOR_FLAG == 2
#define MOTOR_SIZE 18
typedef int32_t motor_state_type;
#elif READ_MOTOR_FLAG == 3
#define MOTOR_SIZE 10
typedef int16_t motor_state_type;
#endif


typedef struct
{
    motor_state_type mode;
    motor_state_type position;
    motor_state_type velocity;
    motor_state_type torque;
    uint16_t id;
} motor_state_s;

typedef struct
{
    union
    {
        motor_state_s motor;
        uint8_t data[MOTOR_SIZE];
    };
} motor_state_t;


extern motor_state_t motor_state;
extern uint8_t motor_read_flag;

uint16_t get_fdcan_data_size(uint32_t dlc);

/* dq ��ѹģʽ */
void set_dq_volt_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float volt);
void set_dq_volt_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t volt);
void set_dq_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t volt);

/* dq ����ģʽ */
void set_dq_current_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float current);
void set_dq_current_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t current);
void set_dq_current_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t current);

/* ���ؿ��� */
void set_torque_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float torque);
void set_torque_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t torque);
void set_torque_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t torque);

/* λ�á��ٶȺ����ؿ��� */
void set_pos_vel_tqe_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos, float val, float torque);
void set_pos_vel_tqe_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos, int32_t val, int32_t torque);
void set_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos, int16_t val, int16_t torque);

/* λ�� */
void set_pos_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos);
void set_pos_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos);
void set_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos);

/* �ٶ� */
void set_val_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float val);
void set_val_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t val);
void set_val_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t val);

/* λ�á��ٶȡ����ء�PD���� */
void set_pos_val_tqe_pd_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos, float val, float tqe, float kp, float kd);
void set_pos_val_tqe_pd_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos, int32_t val, int32_t tqe, float rkp, float rkd);
void set_pos_val_tqe_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos, int16_t val, int16_t tqe, float rkp, float rkd);

/* �ٶȡ��ٶ����� */
void set_val_valmax_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t val, int16_t vel_max);

/* λ�á��ٶȡ����ٶ����ƣ����ο��ƣ� */
void set_pos_valmax_acc_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos, float vel_max, float acc);
void set_pos_valmax_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos, int32_t vel_max, int32_t acc);
void set_pos_valmax_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos, int16_t vel_max, int16_t acc);

/* �ٶȡ����ٶȿ��� */
void set_val_acc_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float val, float acc);
void set_val_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t val, int32_t acc);
void set_val_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t val, int16_t acc);

/* ������� */
void set_pos_rezero(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);

/* �������� */
void set_conf_write(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);

/* ���ֹͣ */
void set_motor_stop(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);

/* ���ɲ�� */
void set_motor_brake(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);

/* ��ȡ���״̬ */
void read_motor_state_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);
void read_motor_state_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);
void read_motor_state_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor);


#endif
