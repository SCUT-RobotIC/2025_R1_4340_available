#include "libelybot_fdcan.h"
#include "dm_motor_drv.h"
#include "string.h"


FDCAN_RxHeaderTypeDef fdcan_rx_header1;
uint8_t fdcan1_rdata[64] = {0};

motor_state_t motor_state;
uint8_t motor_read_flag = 0;



//static void print_data(uint8_t *data, uint16_t len)
//{
//    for (int i = 0; i < len; i++)
//    {
//        printf("%d\r\n", &data[i]);
//    }
//    printf("\r\n\r\n");
//}


uint16_t get_fdcan_data_size(uint32_t dlc)
{
    uint16_t size = 0;
    if(dlc <= FDCAN_DLC_BYTES_8)
    {
        size = dlc >> 16;
    }
    else if(dlc == FDCAN_DLC_BYTES_12)
    {
        size = 12;
    }
    else if(dlc == FDCAN_DLC_BYTES_16)
    {
        size = 16;
    }
    else if(dlc == FDCAN_DLC_BYTES_20)
    {
        size = 20;
    }
    else if(dlc == FDCAN_DLC_BYTES_24)
    {
        size = 24;
    }
    else if(dlc == FDCAN_DLC_BYTES_32)
    {
        size = 32;
    }
    else if(dlc == FDCAN_DLC_BYTES_48)
    {
        size = 48;
    }
    else if(dlc == FDCAN_DLC_BYTES_64)
    {
        size = 64;
    }

    return size;
}

/**
 * @brief ��ѹ���� float
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param volt ��ѹ������0.3 -> 0.3v
 */
void set_dq_volt_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float volt)
{
    //                          dq��ѹģʽ   2��16λ      d                        q                       ռλ��fdcan��
    static uint8_t cmd[] = {0x01, 0x00, 0x08, 0x0E, 0x1a, 0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    *(float *)&cmd[9] = volt;

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ��ѹ���� int32
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param volt ��ѹ����λ��0.001V
 */
void set_dq_volt_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t volt)
{
    //                   		dq��ѹģʽ   2��32λ      d                        q                       ռλ��fdcan��
    static uint8_t cmd[] = {0x01, 0x00, 0x08, 0x0A, 0x1a, 0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    *(int32_t *)&cmd[9] = volt;

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ��ѹ���� int16
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param volt ��ѹ����λ��0.1V
 */
void set_dq_volt_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t volt)
{
    //                   		dq��ѹģʽ   2��16λ      d           q           ռλ��fdcan��
    static uint8_t cmd[] = {0x01, 0x00, 0x08, 0x06, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    *(int16_t *)&cmd[7] = volt;

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief �������� float
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param volt ����������0.3 -> 0.3A
 */
void set_dq_current_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float current)
{
    //                   		dq����ģʽ   2��32λ      q����       			  d����       			  ռλ��fdcan��
    static uint8_t cmd[] = {0x01, 0x00, 0x09, 0x0E, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    *(float *)&cmd[5] = current;

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief �������� int32
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param volt ��������λ��0.001A
 */
void set_dq_current_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t current)
{
    //                   		dq����ģʽ   2��32λ      q����       			  d����       			   ռλ��fdcan��
    static uint8_t cmd[] = {0x01, 0x00, 0x09, 0x0A, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    *(int32_t *)&cmd[5] = current;

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief �������� int16
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param volt ��������λ��0.1A
 */
void set_dq_current_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t current)
{
    //                   		dq����ģʽ   2��16λ      q����       d����       ռλ��fdcan��
    static uint8_t cmd[] = {0x01, 0x00, 0x09, 0x06, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    *(int16_t *)&cmd[5] = current;

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ���ؿ��� float
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param torque ���أ���λ��xxx NM
 */
void set_torque_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float torque)
{
#if MOTOR_MODEL == 5046
    torque *= (double)MOTOR_5046_TQE_CORRECT_FLOAT;
#elif MOTOR_MODEL == 4538
    torque *= (double)MOTOR_4538_TQE_CORRECT_FLOAT;
#elif MOTOR_MODEL == 50471
    torque *= (double)MOTOR_5047_1_TQE_CORRECT_FLOAT;
#elif MOTOR_MODEL == 50472
    torque *= (double)MOTOR_5047_2_TQE_CORRECT_FLOAT;
#endif
    //                     		  λ��ģʽ    float  6��        λ��
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0c, 0x06, 0x20, 0x00, 0x00,
                            // 			�ٶ�                 	����
                            0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xcc,
                            //			kp                	    kd
                            0xcc, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            //			�������                ռλ��fdcan��
                            0x00, 0x00, 0x00, 0x00, 0xc0, 0x7f, 0x50, 0x50
                           };

    *(int32_t *)&cmd[14] = *(int32_t *)&torque;
    // memcpy(&cmd[14], &torque, sizeof(float));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief ���ؿ��� int32
 * @param fdcanHandle &hfdcanx
 * @param motor id ���ID
 * @param torque ���أ���λ��xxx NM
 */
void set_torque_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t torque)
{
#if MOTOR_MODEL == 5046
    torque *= MOTOR_5046_TQE_CORRECT_INT32;
#elif MOTOR_MODEL == 4538
    torque *= MOTOR_4538_TQE_CORRECT_INT32;
#elif MOTOR_MODEL == 50471
    torque *= MOTOR_5047_1_TQE_CORRECT_INT32;
#elif MOTOR_MODEL == 50472
    torque *= MOTOR_5047_2_TQE_CORRECT_INT32;
#endif
    //                     λ��ģʽ    		 int32  6��    	    λ��
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x08, 0x06, 0x20, 0x00, 0x00,
                            //          �ٶ�                    ����
                            0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            //          kp  					kd
                            0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            //          �������   		 	    ռλ��fdcan��
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x50, 0x50
                           };

    *(int32_t *)&cmd[14] = *(int32_t *)&torque;
    // memcpy(&cmd[14], &torque, sizeof(int32_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief ���ؿ��� int16
 * @param fdcanHandle &hfdcanx
 * @param motor id ���ID
 * @param torque ���أ���λ��xxx NM
 */
void set_torque_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t torque)
{
#if MOTOR_MODEL == 5046
    torque *= MOTOR_5046_TQE_CORRECT_INT16;
#elif MOTOR_MODEL == 4538
    torque *= MOTOR_4538_TQE_CORRECT_INT16;
#elif MOTOR_MODEL == 50471
    torque *= MOTOR_5047_1_TQE_CORRECT_INT16;
#elif MOTOR_MODEL == 50472
    torque *= MOTOR_5047_2_TQE_CORRECT_INT16;
#endif
    //                     		λ��ģʽ     int16   6��        λ��
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x04, 0x06, 0x20, 0x00, 0x80,
                            //�ٶ�      ����        kp          kd          �������    ռλ��fdcan��
                            0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x50, 0x50
                           };

    *(int16_t *)&cmd[10] = *(int16_t *)&torque;
    // memcpy(&cmd[10], &torque, sizeof(int16_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief ���λ��-�ٶ�-ǰ������(�������)���ƣ�float��
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param pos λ�ã���λ 1 Ȧ���� pos = 0.5 ��ʾת�� 0.5 Ȧ��λ�á�
 * @param val �ٶȣ���λ 1 ת/�룬�� val = 0.5 ��ʾ 0.5 ת/��
 * @param torque ������أ���λ��1NM���� torque = 1.1 ��ʾ�������Ϊ 1.1NM
 */
void set_pos_vel_tqe_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos, float val, float torque)
{
#if MOTOR_MODEL == 5046
    torque *= (double)MOTOR_5046_TQE_CORRECT_FLOAT;
#elif MOTOR_MODEL == 4538
    torque *= (double)MOTOR_4538_TQE_CORRECT_FLOAT;
#elif MOTOR_MODEL == 50471
    torque *= (double)MOTOR_5047_1_TQE_CORRECT_FLOAT;
#elif MOTOR_MODEL == 50472
    torque *= (double)MOTOR_5047_2_TQE_CORRECT_FLOAT;
#endif
    //                           λ��ģʽ     int32       λ��                    �ٶ�                    			  ����                    ֹͣλ��                ռλ��fdcan��
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0e, 0x20, 0x00, 0x00, 0xc0, 0x7f, 0xcd, 0xcc, 0xcc, 0x3d, 0x0e, 0x25, 0x00, 0x00, 0x80, 0x3f, 0x9a, 0x99, 0x00, 0x00, 0x50};

    // *(int32_t *)&cmd[9] = *(int32_t *)&val;
    // *(int32_t *)&cmd[15] = *(int32_t *)&torque;
    // *(int32_t *)&cmd[19] = *(int32_t *)&pos;

    memcpy(&cmd[9], &val, sizeof(float));
    memcpy(&cmd[15], &torque, sizeof(float));
    memcpy(&cmd[19], &pos, sizeof(float));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ���λ��-�ٶ�-ǰ������(�������)���ƣ�int32��
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param pos λ�ã���λ 0.00001 Ȧ���� pos = 50000 ��ʾת�� 0.5 Ȧ��λ�á�
 * @param val �ٶȣ���λ 0.00001 ת/�룬�� val = 50000 ��ʾ 0.5 ת/��
 * @param torque ������أ���λ��0.00001 NM���� torque = 110000 ��ʾ�������Ϊ 1.1NM
 */
void set_pos_vel_tqe_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos, int32_t val, int32_t torque)
{
#if MOTOR_MODEL == 5046
    torque *= MOTOR_5046_TQE_CORRECT_INT32;
#elif MOTOR_MODEL == 4538
    torque *= MOTOR_4538_TQE_CORRECT_INT32;
#elif MOTOR_MODEL == 50471
    torque *= MOTOR_5047_1_TQE_CORRECT_INT32;
#elif MOTOR_MODEL == 50472
    torque *= MOTOR_5047_2_TQE_CORRECT_INT32;
#endif
    //                           λ��ģʽ     int32       λ��                    �ٶ�                    			  ����                    ֹͣλ��                ռλ��fdcan��
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0a, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50};

    // *(int32_t *)&cmd[9] =  val;
    // *(int32_t *)&cmd[13] = torque;
    // *(int32_t *)&cmd[17] = pos;

    memcpy(&cmd[9], &val, sizeof(int32_t));
    memcpy(&cmd[15], &torque, sizeof(int32_t));
    memcpy(&cmd[19], &pos, sizeof(int32_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ���λ��-�ٶ�-ǰ������(�������)���ƣ�int16��
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param pos λ�ã���λ 0.0001 Ȧ���� pos = 5000 ��ʾת�� 0.5 Ȧ��λ�á�
 * @param val �ٶȣ���λ 0.00025 ת/�룬�� val = 400 ��ʾ 0.1 ת/��
 * @param torque ������أ���λ��0.01 NM���� torque = 110 ��ʾ�������Ϊ 1.1NM
 */
void set_pos_vel_tqe_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos, int16_t val, int16_t torque)
{
#if MOTOR_MODEL == 5046
    torque *= MOTOR_5046_TQE_CORRECT_INT16;
#elif MOTOR_MODEL == 4538
    torque *= MOTOR_4538_TQE_CORRECT_INT16;
#elif MOTOR_MODEL == 50471
    torque *= MOTOR_5047_1_TQE_CORRECT_INT16;
#elif MOTOR_MODEL == 50472
    torque *= MOTOR_5047_2_TQE_CORRECT_INT16;
#endif
    //                           λ��ģʽ   2��int16      λ��        �ٶ�		 2��int16	  ����		  ռλ��fdcan��
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x06, 0x25, 0x00, 0x00, 0x50, 0x50};

    // *(int16_t *)&cmd[7] =  val;
    // *(int16_t *)&cmd[11] = torque;
    // *(int16_t *)&cmd[13] = pos;

    memcpy(&cmd[7], &val, sizeof(int16_t));
    memcpy(&cmd[11], &torque, sizeof(int16_t));
    memcpy(&cmd[13], &pos, sizeof(int16_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ���λ�ÿ��� float
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param pos λ�ã���λ 1 Ȧ���� pos = 0.5 ��ʾת�� 0.5 Ȧ��λ�á�
 */
void set_pos_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos)
{
    //                           λ��ģʽ   1��float      λ��                    ռλ��fdcan��
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0D, 0x20, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[5], &pos, sizeof(float));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ���λ�ÿ��� int32
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param pos λ�ã���λ 0.00001 Ȧ���� pos = 50000 ��ʾת�� 0.5 Ȧ��λ�á�
 */
void set_pos_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos)
{
    //                           λ��ģʽ   1��int32      λ��                    ռλ��fdcan��
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x09, 0x20, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[5], &pos, sizeof(int32_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ���λ�ÿ��� int16
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param pos λ�ã���λ 0.0001 Ȧ���� pos = 5000 ��ʾת�� 0.5 Ȧ��λ�á�
 */
void set_pos_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos)
{
    //                          λ��ģʽ     1��int16     λ��
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x05, 0x20, 0x00, 0x00};

    memcpy(&cmd[5], &pos, sizeof(int16_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ����ٶȿ��� float
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param val �ٶȣ���λ 1 ת/�룬�� val = 0.1 -> 0.1 ת/��
 */
void set_val_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float val)
{
    //							  λ��ģʽ     2��float 	λ��					�ٶ�					ռλ��fdcan��
    static uint8_t cmd[16] = {0x01, 0x00, 0x0A, 0x0E, 0x20, 0x00, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[9], &val, sizeof(float));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ����ٶȿ��� int32
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param val �ٶȣ���λ 0.00001 ת/�룬�� val = 50000 ��ʾ 0.5 ת/��
 */
void set_val_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t val)
{
    //							  λ��ģʽ     2��int32 	λ��					�ٶ�					ռλ��fdcan��
    static uint8_t cmd[16] = {0x01, 0x00, 0x0A, 0x0A, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[9], &val, sizeof(int32_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ����ٶȿ��� int16
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param val �ٶȣ���λ 0.00025 ת/�룬�� val = 400 ��ʾ 0.1 ת/��
 */
void set_val_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t val)
{
    //							λ��ģʽ     2��int16 	  λ��		  �ٶ�		  ռλ��fdcan��
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[7], &val, sizeof(int16_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ���λ�á��ٶȡ�������ء�Kp��Kd���� float ����
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param pos λ�ã���λ 1 Ȧ���� pos = 0.5 ��ʾת�� 0.5 Ȧ��λ�á�
 * @param val �ٶȣ���λ 1 ת/�룬�� val = 0.5 ��ʾ 0.5 ת/��
 * @param tqe ������أ���λ��1NM���� torque = 1.1 ��ʾ�������Ϊ 1.1NM
 * @param rkp ��� Kp �����������ʵ Kp = ����ڲ�Kp * rkp
 * @param rkd ��� Kd �����������ʵ Kd = ����ڲ�Kd * rkd
 */
void set_pos_val_tqe_pd_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos, float val, float tqe, float rkp, float rkd)
{
#if MOTOR_MODEL == 5046
    tqe *= (double)MOTOR_5046_TQE_CORRECT_FLOAT;
#elif MOTOR_MODEL == 4538
    tqe *= (double)MOTOR_4538_TQE_CORRECT_FLOAT;
#elif MOTOR_MODEL == 50471
    tqe *= (double)MOTOR_5047_1_TQE_CORRECT_FLOAT;
#elif MOTOR_MODEL == 50472
    tqe *= (double)MOTOR_5047_2_TQE_CORRECT_FLOAT;
#endif
    //                           λ��ģʽ    2��float     λ�ã�NAN��
    static uint8_t cmd[] = {0x01, 0x00, 0x0a, 0x0e, 0x20, 0x00, 0x00, 0xc0, 0x7f,
                            //�ٶ�                  float  4��
                            0xcd, 0xcc, 0xcc, 0x3d, 0x0c, 0x04, 0x23,
                            //rKp                   rKd                     �������                ֹͣλ��
                            0xcd, 0xcc, 0x4c, 0x3d, 0xcd, 0xcc, 0x4c, 0x3d, 0x00, 0x00, 0x80, 0x3f, 0x9a, 0x99, 0x99, 0x3e
                           };

    memcpy(&cmd[9], &val, sizeof(float));
    memcpy(&cmd[16], &rkp, sizeof(float));
    memcpy(&cmd[20], &rkd, sizeof(float));
    memcpy(&cmd[24], &tqe, sizeof(float));
    memcpy(&cmd[28], &pos, sizeof(float));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ���λ�á��ٶȡ�������ء�Kp��Kd���� int32
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param pos λ�ã���λ 0.00001 Ȧ���� pos = 50000 ��ʾת�� 0.5 Ȧ��λ��
 * @param val �ٶȣ���λ 0.00001 ת/�룬�� val = 50000 ��ʾ 0.5 ת/��
 * @param tqe ������أ���λ��0.00001 NM���� torque = 110000 ��ʾ�������Ϊ 1.1NM
 * @param rkp ��� Kp �����������ʵ Kp = ����ڲ�Kp * rkp
 * @param rkd ��� Kd �����������ʵ Kd = ����ڲ�Kd * rkd
 */
void set_pos_val_tqe_pd_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos, int32_t val, int32_t tqe, float rkp, float rkd)
{
#if MOTOR_MODEL == 5046
    tqe *= MOTOR_5046_TQE_CORRECT_INT32;
#elif MOTOR_MODEL == 4538
    tqe *= MOTOR_4538_TQE_CORRECT_INT32;
#elif MOTOR_MODEL == 50471
    tqe *= MOTOR_5047_1_TQE_CORRECT_INT32;
#elif MOTOR_MODEL == 50472
    tqe *= MOTOR_5047_2_TQE_CORRECT_INT32;
#endif
    //                            λ��ģʽ
    static uint8_t cmd[48] = {0x01, 0x00, 0x0a, 0x0A, 0x20,
                              //λ�ã�NAN��           �ٶ�
                              0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x0E, 0x23,
                              //rKp                   rKd
                              0x00, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0xc0, 0x7f, 0x0A, 0x25,
                              //�������              ֹͣλ��
                              0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80,
                              //ռλ��fdcan��
                              0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50
                             };

    memcpy(&cmd[9], &val, sizeof(int32_t));
    memcpy(&cmd[15], &rkp, sizeof(float));
    memcpy(&cmd[19], &rkd, sizeof(float));
    memcpy(&cmd[25], &tqe, sizeof(int32_t));
    memcpy(&cmd[29], &pos, sizeof(int32_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, 48);
}


/**
 * @brief ���λ�á��ٶȡ�������ء�Kp��Kd���� int16
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param pos λ�ã���λ 0.0001 Ȧ���� pos = 5000 ��ʾת�� 0.5 Ȧ��λ�á�
 * @param val �ٶȣ���λ 0.00025 ת/�룬�� val = 400 ��ʾ 0.1 ת/��
 * @param tqe ������أ���λ��0.01 NM���� torque = 110 ��ʾ�������Ϊ 1.1NM
 * @param rkp ��� Kp �����������ʵ Kp = ����ڲ�Kp * rkp
 * @param rkd ��� Kd �����������ʵ Kd = ����ڲ�Kd * rkd
 */
void set_pos_val_tqe_pd_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos, int16_t val, int16_t tqe, float rkp, float rkd)
{
#if MOTOR_MODEL == 5046
    tqe *= MOTOR_5046_TQE_CORRECT_INT16;
#elif MOTOR_MODEL == 4538
    tqe *= MOTOR_4538_TQE_CORRECT_INT16;
#elif MOTOR_MODEL == 50471
    tqe *= MOTOR_5047_1_TQE_CORRECT_INT16;
#elif MOTOR_MODEL == 50472
    tqe *= MOTOR_5047_2_TQE_CORRECT_INT16;
#endif
    //                           λ��ģʽ
    static uint8_t cmd[32] = {0x01, 0x00, 0x0a, 0x06, 0x20,
                              //λ�ã�NAN���ٶ�
                              0x00, 0x80, 0x00, 0x80, 0x0e, 0x23,
                              //rKp       			  rKd
                              0x00, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0xc0, 0x7f, 0x06, 0x25,
                              //�������  ֹͣλ��
                              0x00, 0x80, 0x00, 0x80
                             };

    memcpy(&cmd[7], &val, sizeof(int16_t));
    memcpy(&cmd[11], &rkp, sizeof(float));
    memcpy(&cmd[15], &rkd, sizeof(float));
    memcpy(&cmd[21], &tqe, sizeof(int16_t));
    memcpy(&cmd[23], &pos, sizeof(int16_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ����ٶȡ��ٶ��޷����ƣ���� val > val_max������ val_max�� int16
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param val �ٶȣ���λ 0.00025 ת/�룬�� val = 400 ��ʾ 0.1 ת/��
 * @param val_max �ٶ��޷�����λ 0.00025 ת/�룬�� val = 400 ��ʾ 0.1 ת/��
 */
void set_val_valmax_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t val, int16_t val_max)
{
    //							λ��ģʽ				  λ��        �ٶ�		  			  �ٶ�����
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x05, 0x28, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[11], &val_max, sizeof(int16_t));
    memcpy(&cmd[7], &val, sizeof(int16_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief λ�á��ٶȡ����ٶ����ƣ����ο��ƣ� float
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param pos λ�ã���λ 1 Ȧ���� pos = 0.5 ��ʾת�� 0.5 Ȧ��λ�á�
 * @param vel_max �ٶ����ƣ���λ 1 ת/�룬�� val = 0.5 ��ʾ 0.5 ת/��
 * @param acc ���ٶȣ���λ��1 ת/��^2
 */
void set_pos_valmax_acc_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float pos, float vel_max, float acc)
{
    // 							λ��ģʽ				  λ��		  						  �ٶ�����
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0D, 0x20, 0x00, 0x00, 0x00, 0x80, 0x0E, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50};

    memcpy(&cmd[5], &pos, sizeof(float));
    memcpy(&cmd[11], &vel_max, sizeof(float));
    memcpy(&cmd[15], &acc, sizeof(float));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief λ�á��ٶȡ����ٶ����ƣ����ο��ƣ� int32
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param pos λ�ã���λ 0.0001 Ȧ���� pos = 5000 ��ʾת�� 0.5 Ȧ��λ��
 * @param val_max �ٶ����ƣ���λ 0.00001 ת/�룬�� val = 50000 ��ʾ 0.5 ת/��
 * @param acc ���ٶȣ���λ 0.00001 ת/��^2���� acc = 50000 ��ʾ 0.5 ת/��^2
 */
void set_pos_valmax_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t pos, int32_t vel_max, int32_t acc)
{
    // 							λ��ģʽ				  λ��		  						  �ٶ�����
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x09, 0x20, 0x00, 0x00, 0x00, 0x80, 0x0A, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50};

    memcpy(&cmd[5], &pos, sizeof(int32_t));
    memcpy(&cmd[11], &vel_max, sizeof(int32_t));
    memcpy(&cmd[15], &acc, sizeof(int32_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief λ�á��ٶȡ����ٶ����ƣ����ο��ƣ� int16
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param pos λ�ã���λ 0.00001 Ȧ���� pos = 50000 ��ʾת�� 0.5 Ȧ��λ��
 * @param vel_max �ٶȣ���λ 0.00025 ת/�룬�� val = 400 ��ʾ 0.1 ת/��
 * @param acc ���ٶȣ���λ 0.00025 ת/��^2���� acc = 400 ��ʾ 0.1 ת/��^2
 */
void set_pos_valmax_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t pos, int16_t vel_max, int16_t acc)
{
    // 							λ��ģʽ				  λ��		  �ٶ�����
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x05, 0x20, 0x00, 0x80, 0x06, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    memcpy(&cmd[5], &pos, sizeof(int16_t));
    memcpy(&cmd[9], &vel_max, sizeof(int16_t));
    memcpy(&cmd[11], &acc, sizeof(int16_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ����ٶȡ����ٶȿ��� float
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param val �ٶȣ���λ�� 1 ת/�룬�� val = 0.5 ��ʾ 0.5 ת/��
 * @param acc ���ٶȣ���λ��1 ת/��^2
 */
void set_val_acc_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, float val, float acc)
{
    //							λ��ģʽ				  λ��					  �ٶ�								  ���ٶ�
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0E, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x0D, 0x29, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[9], &val, sizeof(float));
    memcpy(&cmd[15], &acc, sizeof(float));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ����ٶȡ����ٶȿ��� int32
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param val �ٶȣ���λ 0.00001 ת/�룬�� val = 50000 ��ʾ 0.5 ת/��
 * @param acc ���ٶȣ���λ 0.00001 ת/��^2���� val = 50000 ��ʾ 0.5 ת/��^2
 */
void set_val_acc_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int32_t val, int32_t acc)
{
    //							λ��ģʽ				  λ��					  �ٶ�								  ���ٶ�
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x0A, 0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x09, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50, 0x50, 0x50};

    memcpy(&cmd[9], &val, sizeof(int32_t));
    memcpy(&cmd[15], &acc, sizeof(int32_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ����ٶȡ����ٶȿ��� int16
 * @param fdcanHandle &hfdcanx
 * @param motor ���ID
 * @param val �ٶȣ���λ 0.00025 ת/�룬�� val = 400 ��ʾ 0.1 ת/��
 * @param acc ���ٶȣ���λ 0.00025 ת/��^2���� val = 400 ��ʾ 0.1 ת/��^2
 */
void set_val_acc_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor, int16_t val, int16_t acc)
{
    //							λ��ģʽ				  λ��		  �ٶ�					  ���ٶ�
    static uint8_t cmd[] = {0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x05, 0x29, 0x00, 0x00, 0x50, 0x50, 0x50};

    memcpy(&cmd[7], &val, sizeof(int16_t));
    memcpy(&cmd[11], &acc, sizeof(int16_t));

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ��������λ
 * @param fdcanHandle &hfdcanx
 * @param motor id ���ID
 */
void set_pos_rezero(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    static uint8_t cmd[] = {0x40, 0x01, 0x15, 0x64, 0x20, 0x63, 0x66, 0x67, 0x2d, 0x73, 0x65, 0x74, 0x2d, 0x6f, 0x75, 0x74, 0x70, 0x75, 0x74, 0x20, 0x30, 0x2e, 0x30, 0x0a};

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));

    HAL_Delay(1000);

    set_conf_write(fdcanHandle, motor);
}


/**
 * @brief ����������
 * @param fdcanHandle &hfdcanx
 * @param motor id ���ID
 */
void set_conf_write(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    static uint8_t cmd[] = {0x40, 0x01, 0x0B, 0x63, 0x6F, 0x6E, 0x66, 0x20, 0x77, 0x72, 0x69, 0x74, 0x65, 0x0A, 0x50, 0x50};

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ���ֹͣ��ע�⣺���õ��ֹͣ����������λ��������Ч
 * @param fdcanHandle &hfdcanx
 * @param motor id ���ID
 */
void set_motor_stop(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x00};

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ���ɲ��
 * @param fdcanHandle &hfdcanx
 * @param motor id ���ID
 */
void set_motor_brake(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x0f};

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, cmd, sizeof(cmd));
}


/**
 * @brief ��ȡ���״̬ float��״̬��λ�á��ٶȡ�ת��
 * @param fdcanHandle &hfdcanx
 * @param motor id ���ID
 */
void read_motor_state_float(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    const uint8_t cmd[] = {0x1C, 0x04, 0x00};

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief ��ȡ���״̬ int32��״̬��λ�á��ٶȡ�ת��
 * @param fdcanHandle &hfdcanx
 * @param motor id ���ID
 */
void read_motor_state_int32(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    const uint8_t cmd[] = {0x18, 0x04, 0x00};

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief ��ȡ���״̬ int16��״̬��λ�á��ٶȡ�ת��
 * @param fdcanHandle &hfdcanx
 * @param motor id ���ID
 */
void read_motor_state_int16(FDCAN_HandleTypeDef *fdcanHandle, motor_e motor)
{
    const uint8_t cmd[] = {0x14, 0x04, 0x00};

    fdcanx_send_data(fdcanHandle, 0x8000 | motor, (uint8_t *)cmd, sizeof(cmd));
}






