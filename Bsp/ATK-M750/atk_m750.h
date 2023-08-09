#ifndef __ATK_M750_H
#define __ATK_M750_H


#define DTU_RX_CMD_BUF_SIZE (512)

typedef enum
{
    DTU_WORKMODE_NET = 0,  /*����͸��ģʽ*/
    DTU_WORKMODE_HTTP,     /*http͸��ģʽ*/
    DTU_WORKMODE_MQTT,     /*mqtt͸��ģʽ*/
    DTU_WORKMODE_ALIYUN,   /*������͸��ģʽ*/
    DTU_WORKMODE_ONENET,   /*OneNET͸��ģʽ*/
    DTU_WORKMODE_BAIDUYUN, /*�ٶ���͸��ģʽ*/
} _dtu_work_mode_eu;

typedef struct
{
    uint32_t timeout; /*ָ��ȴ���ʱʱ�䣬��λ��100ms*/
    char *read_cmd;   /*��ѯ����ָ��      ��ο�DTU AT�û��ֲ�˵��������д*/
    char *write_cmd;  /*���ò���ָ��      ��ο�DTU AT�û��ֲ�˵��������д*/
} _dtu_atcmd_st;

typedef struct {
    uint16_t year;    
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t timezone;  
}ST_Time;


void dtu_get_urc_info(uint8_t ch);

void send_data_to_dtu(uint8_t *data, uint32_t size);

int dtu_config_init(_dtu_work_mode_eu work_mode);

int dtu_enter_configmode(void);
int dtu_enter_transfermode(void);

#endif
