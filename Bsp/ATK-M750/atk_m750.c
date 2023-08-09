/*******************************************
	*�ļ��� ��  VCU4.3
	*��   �ߣ�  �޳�
	*�޸�ʱ�䣺 2022.10.5
	*��   ����  ������ԭ�ӻ��������޸Ķ���
*******************************************/

#include "usart.h"
#include "atk_m750.h"
#include "RingBuffer.h"
#include "string.h"
#include "stdlib.h"

extern RingBuffer *p_uart3_rxbuf;

ST_Time Timedat;

static uint8_t dtu_rxcmdbuf[DTU_RX_CMD_BUF_SIZE]; /*����DTU������ݻ���*/

/**
 * @brief       �������ݵ�DTU
 * 
 * @param       data:   ��Ҫ�������ݵĻ����ַ
 * @param       size:   �������ݴ�С
 * 
 * @return      ��
 * 
*/
void send_data_to_dtu(uint8_t *data, uint32_t size)
{
	HAL_UART_Transmit(&huart3,data, size,0xff);
}

/**
 * @brief       �������DTU����������У��
 * 
 * @param       cmd     :   ��Ҫ���͵�ATָ��
 * @param       ask     :   ��ҪУ���Ӧ������
 * @param       timeout :   ATָ��У�鳬ʱʱ�䣬��λ��100ms
 * 
 * @return      1  :   У��ask���ݳɹ�
 *              0  :   DTU����OK
 *             -1  :   DTU����ERROR
 *             -2  :   ����ATָ��У�鳬ʱ
 */
static int send_cmd_to_dtu(char *cmd, char *ask, uint32_t timeout)
{
    uint32_t rx_len = 0;
	
    /*��ʼ����������*/
    memset(dtu_rxcmdbuf, 0, DTU_RX_CMD_BUF_SIZE);
    RingBuffer_Reset(p_uart3_rxbuf);
    /*����ATָ�DTU*/
    send_data_to_dtu((uint8_t *)cmd, strlen(cmd));
    /*�ȴ�DTUӦ��ATָ����*/
    while (1)
    {
//				printf("RingBuffer_Out = %s\r\n",dtu_rxcmdbuf);
        if (strstr((char *)dtu_rxcmdbuf, ask) != NULL)
        {
            return 1;
        }
        else if (strstr((char *)dtu_rxcmdbuf, "OK") != NULL)
        {
            return 0;
        }
        else if (strstr((char *)dtu_rxcmdbuf, "ERROR") != NULL)
        {
            return -1;
        }
        if (RingBuffer_Len(p_uart3_rxbuf) > 0)
        {
            RingBuffer_Out(p_uart3_rxbuf, &dtu_rxcmdbuf[rx_len++], 1); //�Ӵ��ڻ����ж�һ���ֽ�
            if (rx_len >= DTU_RX_CMD_BUF_SIZE) /*����Ӧ�����ݳ���������ERROR*/
            {
                return -1;
            }
        }
        else
        {
            timeout--;

            if (timeout == 0)
            {
                return -2;
            }
						HAL_Delay(100);
        }
    }
}

/**
 * @brief       DTU��������״̬
 * 
 * @param       ��
 * 
 * @return      0  :    �ɹ���������״̬
 *             -1  :    ��������״̬ʧ��
 */
 int dtu_enter_configmode(void)
{
    int res;
    /* 1.����+++׼����������״̬ */
    res = send_cmd_to_dtu("+++", "atk", 5);
    if (res == -1) /*����ERRRO��ʾDTU�Ѿ���������״̬*/
    {
        return 0;
    }
    /* 2.����atkȷ�Ͻ�������״̬ */
    res = send_cmd_to_dtu("atk", "OK", 5);
    if (res == -2)
    {
        return -1;
    }
    return 0;
}

/**
 * @brief       DTU����͸��״̬
 * 
 * @param       ��
 * 
 * @return      0  :    �ɹ�����͸��״̬
 *             -1  :    ����͸��״̬ʧ��
 */
int dtu_enter_transfermode(void)
{
    if (send_cmd_to_dtu("ATO\r\n", "OK", 5) >= 0)
    {
        return 0;
    }
    return -1;
}

/**
 * @brief       DTU�Զ��ϱ�URC��Ϣ������:����+ATK ERROR��Ϣ
 * 
 * @param       data    :   ���յ�DTU��URC���ݻ���
 * @param       len     :   URC���ݳ���
 * 
 * @return      ��
 */
static void dtu_urc_atk_error(const char *data, uint32_t len)
{
    printf("\r\nURC :   dtu_urc_atk_error\r\n");
}

/**
 * @brief       DTU�Զ��ϱ�URC��Ϣ������:����Please check SIM Card��Ϣ
 * 
 * @param       data    :   ���յ�DTU��URC���ݻ���
 * @param       len     :   URC���ݳ���
 * 
 * @return      ��
 */
static void dtu_urc_error_sim(const char *data, uint32_t len)
{
    printf("\r\nURC :   dtu_urc_error_sim\r\n");
}

/**
 * @brief       DTU�Զ��ϱ�URC��Ϣ������:����Please check GPRS��Ϣ
 * 
 * @param       data    :   ���յ�DTU��URC���ݻ���
 * @param       len     :   URC���ݳ���
 * 
 * @return      ��
 */
static void dtu_urc_error_gprs(const char *data, uint32_t len)
{
    printf("\r\nURC :   dtu_urc_error_gprs\r\n");
}

/**
 * @brief       DTU�Զ��ϱ�URC��Ϣ������:����Please check CSQ��Ϣ
 * 
 * @param       data    :   ���յ�DTU��URC���ݻ���
 * @param       len     :   URC���ݳ���
 * 
 * @return      ��
 */
static void dtu_urc_error_csq(const char *data, uint32_t len)
{
    printf("\r\nURC :   dtu_urc_error_csq\r\n");
}

/**
 * @brief       DTU�Զ��ϱ�URC��Ϣ������:����Please check MQTT Parameter��Ϣ
 * 
 * @param       data    :   ���յ�DTU��URC���ݻ���
 * @param       len     :   URC���ݳ���
 * 
 * @return      ��
 */
static void dtu_urc_error_mqtt(const char *data, uint32_t len)
{
    printf("\r\nURC :   dtu_urc_error_mqtt\r\n");
}

typedef struct
{
    const char *urc_info;                         /*DTU�Զ��ϱ���URC��Ϣ*/
    void (*func)(const char *data, uint32_t len); /*�ص�������*/
} _dtu_urc_st;

#define DTU_ATK_M750_URC_SIZE 5
static _dtu_urc_st DTU_ATK_M750_URC[DTU_ATK_M750_URC_SIZE] =
    {
       
        {"+ATK ERROR:",                         dtu_urc_atk_error},         /*DTU�������⣬��Ҫ��ϵ����֧�ֽ���ȷ��*/
        {"Please check SIM Card !!!\r\n",       dtu_urc_error_sim},         /*DTUδ��⵽�ֻ���,�����ֻ����Ƿ���ȷ����*/
        {"Please check GPRS !!!\r\n",           dtu_urc_error_gprs},        /*����SIM���Ƿ�Ƿ��*/
        {"Please check CSQ !!!\r\n",            dtu_urc_error_csq},         /*���������Ƿ���ȷ���룬��ȷ������λ�õ���ȷ��*/
        {"Please check MQTT Parameter !!!\r\n", dtu_urc_error_mqtt},        /*MQTT��������*/
};

/**
 * @brief       ����DTU�����ϱ���URC��Ϣ���ݣ�ע�⣺����ÿ����һ���ֽ����ݣ�����Ҫͨ��������ڴ������
 * 
 * @param       ch    :   ���ڽ��յ�һ���ֽ�����
 * 
 * @return      ��
 */
void dtu_get_urc_info(uint8_t ch)
{
    static uint8_t ch_last = 0;
    static uint32_t rx_len = 0;
    int i;

    /*����DTU����*/
    dtu_rxcmdbuf[rx_len++] = ch;
    if (rx_len >= DTU_RX_CMD_BUF_SIZE)
    { /*��������*/
        ch_last = 0;
        rx_len = 0;
        memset(dtu_rxcmdbuf, 0, DTU_RX_CMD_BUF_SIZE);
    }

    /*����DTU��URC����*/
    if ((ch_last == '\r') && (ch == '\n'))
    {
        for (i = 0; i < DTU_ATK_M750_URC_SIZE; i++)
        {
            if (strstr((char *)dtu_rxcmdbuf, DTU_ATK_M750_URC[i].urc_info) == (char *)dtu_rxcmdbuf)
            {
                DTU_ATK_M750_URC[i].func((char *)dtu_rxcmdbuf, strlen((char *)dtu_rxcmdbuf));
            }
        }

        ch_last = 0;
        rx_len = 0;
        memset(dtu_rxcmdbuf, 0, DTU_RX_CMD_BUF_SIZE);
    }

    ch_last = ch;
}

static const _dtu_atcmd_st dtu_net_param_info[] = {

    /*1.ѡ����ģʽΪ������͸��ģʽ*/
    {5, "AT+WORK\r\n",      "AT+WORK=\"NET\"\r\n"},

    /*2.��������͸��ģʽ�Ĺ�������*/
    {5, "AT+LINK1EN\r\n",   "AT+LINK1EN=\"ON\"\r\n"},
    {5, "AT+LINK1\r\n",     "AT+LINK1=\"TCP\",\"cloud.alientek.com\",\"59666\"\r\n"},
    {5, "AT+LINK1MD\r\n",   "AT+LINK1MD=\"LONG\"\r\n"},
    {5, "AT+LINK1TM\r\n",   "AT+LINK1TM=\"5\"\r\n"},

    {5, "AT+LINK2EN\r\n",   "AT+LINK2EN=\"OFF\"\r\n"},
    {5, "AT+LINK3EN\r\n",   "AT+LINK3EN=\"OFF\"\r\n"},
    {5, "AT+LINK4EN\r\n",   "AT+LINK4EN=\"OFF\"\r\n"},

    /*3.����ԭ���ƹ��ܣ�Ĭ�ϴ�*/
    {5, "AT+SVREN\r\n",     "AT+SVREN=\"ON\"\r\n"},
    {5, "AT+SVRNUM\r\n",    "AT+SVRNUM=\"12345678901234567890\"\r\n"},
    {5, "AT+SVRKEY\r\n",    "AT+SVRKEY=\"12345678\"\r\n"},

    /*4.�������������ܣ�Ĭ�ϴ�         ע�⣺ǿ�ҽ��鿪�����������ܣ�����*/
    {5, "AT+HRTEN\r\n",     "AT+HRTEN=\"ON\"\r\n"},
    {5, "AT+HRTDT\r\n",     "AT+HRTDT=\"414C49454E54454B2D4852544454\"\r\n"},
    {5, "AT+HRTTM\r\n",     "AT+HRTTM=\"120\"\r\n"},

    /*5.����ע������ܣ�Ĭ�Ϲر� */
    {5, "AT+REGEN\r\n",     "AT+REGEN=\"OFF\"\r\n"},
    {5, "AT+REGDT\r\n",     "AT+REGDT=\"414C49454E54454B2D5245474454\"\r\n"},
    {5, "AT+REGMD\r\n",     "AT+REGMD=\"LINK\"\r\n"},
    {5, "AT+REGTP\r\n",     "AT+REGTP=\"IMEI\"\r\n"},

    /*6.����������������*/
};

static const _dtu_atcmd_st dtu_http_param_info[] = {

    /*1.ѡ����ģʽΪ��HTTP͸��ģʽ*/
    {5, "AT+WORK\r\n",      "AT+WORK=\"HTTP\"\r\n"},

    /*2.����HTTP͸��ģʽ�Ĺ�������*/
    {5, "AT+HTTPMD\r\n",    "AT+HTTPMD=\"GET\"\r\n"},
    {5, "AT+HTTPURL\r\n",   "AT+HTTPURL=\"https://cloud.alientek.com/testfordtu?data=\"\r\n"},
    {5, "AT+HTTPTM\r\n",    "AT+HTTPTM=\"10\"\r\n"},
    {5, "AT+HTTPHD\r\n",    "AT+HTTPHD=\"Connection:close\"\r\n"},

    /*3.����������������*/
};

static const _dtu_atcmd_st dtu_mqtt_param_info[] = {

    /*1.ѡ����ģʽΪ��MQTT͸��ģʽ*/
    {5, "AT+WORK\r\n",      "AT+WORK=\"MQTT\"\r\n"},

    /*2.����MQTT͸��ģʽ�Ĺ�������*/
    {5, "AT+MQTTCD\r\n",    "AT+MQTTCD=\"alientek\"\r\n"},
    {5, "AT+MQTTUN\r\n",    "AT+MQTTUN=\"admin\"\r\n"},
    {5, "AT+MQTTPW\r\n",    "AT+MQTTPW=\"password\"\r\n"},
    {5, "AT+MQTTIP\r\n",    "AT+MQTTIP=\"cloud.alientek.com\",\"1883\"\r\n"},
    {5, "AT+MQTTSUB\r\n",   "AT+MQTTSUB=\"atk/sub\"\r\n"},
    {5, "AT+MQTTPUB\r\n",   "AT+MQTTPUB=\"atk/pub\"\r\n"},
    {5, "AT+MQTTCON\r\n",   "AT+MQTTCON=\"0\",\"0\",\"1\",\"300\"\r\n"},

    /*3.����������������*/
};

static const _dtu_atcmd_st dtu_aliyun_param_info[] = {

    /*1.ѡ����ģʽΪ��������͸��ģʽ*/
    {5, "AT+WORK\r\n",      "AT+WORK=\"ALIYUN\"\r\n"},

    /*2.����MQTT͸��ģʽ�Ĺ�������*/
    {5, "AT+ALIPK\r\n",     "AT+ALIPK=\"ProductKey\"\r\n"},
    {5, "AT+ALIDS\r\n",     "AT+ALIDS=\"DeviceSecret\"\r\n"},
    {5, "AT+ALIDN\r\n",     "AT+ALIDN=\"DeviceName\"\r\n"},
    {5, "AT+ALIRI\r\n",     "AT+ALIRI=\"cn-shanghai\"\r\n"},
    {5, "AT+ALISUB\r\n",    "AT+ALISUB=\"get\"\r\n"},
    {5, "AT+ALIPUB\r\n",    "AT+ALIPUB=\"updata\"\r\n"},
    {5, "AT+ALICON\r\n",    "AT+ALICON=\"0\",\"0\",\"1\",\"300\"\r\n"},

    /*3.����������������*/
};

static const _dtu_atcmd_st dtu_onenet_param_info[] = {

    /*1.ѡ����ģʽΪ��OneNET͸��ģʽ*/
    {5, "AT+WORK\r\n",      "AT+WORK=\"ONENET\"\r\n"},

    /*2.����OneNET͸��ģʽ�Ĺ�������*/
    {5, "AT+ONEDI\r\n",     "AT+ONEDI=\"1096612439\"\r\n"},
    {5, "AT+ONEPI\r\n",     "AT+ONEPI=\"609364\"\r\n"},
    {5, "AT+ONEAI\r\n",     "AT+ONEAI=\"MPY22JP05026022P\"\r\n"},
    {5, "AT+ONEIP\r\n",     "AT+ONEIP=\"mqtt.heclouds.com\",\"6002\"\r\n"},
    {5, "AT+ONECON\r\n",    "AT+ONECON=\"3\",\"0\",\"0\",\"1\",\"300\"\r\n"},

    /*3.����������������*/
};

static const _dtu_atcmd_st dtu_baiduyun_param_info[] = {

    /*1.ѡ����ģʽΪ���ٶ���͸��ģʽ*/
    {5, "AT+WORK\r\n",      "AT+WORK=\"BAIDUYUN\"\r\n"},

    /*2.���ðٶ���͸��ģʽ�Ĺ�������*/
    {5, "AT+BAIEP\r\n",     "AT+BAIEP=\"alientek\"\r\n"},
    {5, "AT+BAINM\r\n",     "AT+BAINM=\"name\"\r\n"},
    {5, "AT+BAIKEY\r\n",    "AT+BAIKEY=\"key\"\r\n"},
    {5, "AT+BAIRI\r\n",     "AT+BAIRI=\"gz\"\r\n"},
    {5, "AT+BAISUB\r\n",    "AT+BAISUB=\"sub\"\r\n"},
    {5, "AT+BAIPUB\r\n",    "AT+BAIPUB=\"pub\"\r\n"},
    {5, "AT+BAICON\r\n",    "AT+BAICON=\"0\",\"0\",\"0\",\"1\",\"300\"\r\n"},

    /*3.����������������*/
};

/**
 * @brief       ����DTU��������
 * 
 * @param       work_param      :   ����ģʽ���ATָ�����
 * @param       num             :   ��Ҫ���õ�ATָ���������
 * 
 * @return      0  :    ���в������óɹ�
 *              n  :    �ڼ�����������ʧ�ܣ�1-n
 */
static int dtu_config_work_param(_dtu_atcmd_st *work_param, uint8_t num)
{
    int i;
    int res = 0;

    for (i = 0; i < num; i++)
    {
        res = send_cmd_to_dtu((work_param + i)->read_cmd,
                              (work_param + i)->write_cmd + strlen((work_param + i)->read_cmd) - 1,
                              work_param[i].timeout);

        if (res == 1) /*���DTU�ڲ���������Ҫ���õĲ���һ�£�����Ҫ�ظ�ȥ����*/
        {
            continue;
        }
        else /*DTU�ڲ����������ò�����һ�£���Ҫ����DTU�ڲ�����*/
        {
            res = send_cmd_to_dtu((work_param + i)->write_cmd,
                                  "OK",
                                  (work_param + i)->timeout);

            if (res < 0)
            {
                return i+1;
            }
        }
    }

    return 0;
}

/**
 * @brief       ��ʼ��DTU�Ĺ���״̬
 * 
 * @param       work_mode   :   DTU����ģʽ
 *  @arg        DTU_WORKMODE_NET,       0,  ����͸��ģʽ
 *  @arg        DTU_WORKMODE_HTTP,      1,  http͸��ģʽ
 *  @arg        DTU_WORKMODE_MQTT,      2,  mqtt͸��ģʽ
 *  @arg        DTU_WORKMODE_ALIYUN,    3,  ������͸��ģʽ
 *  @arg        DTU_WORKMODE_ONENET,    4,  OneNET͸��ģʽ
 *  @arg        DTU_WORKMODE_BAIDUYUN,  5,  �ٶ���͸��ģʽ
 * 
 * @return      0   :   ��ʼ���ɹ�
 *             -1   :   ������״̬ʧ��
 *             -2   :   DTU������������ʧ��
 *             -3   ��  DTU����͸��״̬ʧ��
 */
int dtu_config_init(_dtu_work_mode_eu work_mode)
{
    int res;

    /*1.DTU��������״̬*/
    res = dtu_enter_configmode();
    if ( res != 0 )
    {
        return -1;
    }
    /*2.����DTU�Ĺ�������*/
    switch (work_mode)
    {
        case DTU_WORKMODE_NET:
        {
            res = dtu_config_work_param((_dtu_atcmd_st *)&dtu_net_param_info, sizeof(dtu_net_param_info) / sizeof(_dtu_atcmd_st));
            break;
        }
        case DTU_WORKMODE_HTTP:
        {
            res = dtu_config_work_param((_dtu_atcmd_st *)&dtu_http_param_info, sizeof(dtu_http_param_info) / sizeof(_dtu_atcmd_st));
            break;
        }
        case DTU_WORKMODE_MQTT:
        {
            res = dtu_config_work_param((_dtu_atcmd_st *)&dtu_mqtt_param_info, sizeof(dtu_mqtt_param_info) / sizeof(_dtu_atcmd_st));
            break;
        }
        case DTU_WORKMODE_ALIYUN:
        {
            res = dtu_config_work_param((_dtu_atcmd_st *)&dtu_aliyun_param_info, sizeof(dtu_aliyun_param_info) / sizeof(_dtu_atcmd_st));
            break;
        }
        case DTU_WORKMODE_ONENET:
        {
            res = dtu_config_work_param((_dtu_atcmd_st *)&dtu_onenet_param_info, sizeof(dtu_onenet_param_info) / sizeof(_dtu_atcmd_st));
            break;
        }
        case DTU_WORKMODE_BAIDUYUN:
        {
            res = dtu_config_work_param((_dtu_atcmd_st *)&dtu_baiduyun_param_info, sizeof(dtu_baiduyun_param_info) / sizeof(_dtu_atcmd_st));
            break;
        }
        default:
        {
            break;
        }
    }

    if( res != 0 )
    {
        return -2;
    }



    /*3.DTU����͸��״̬*/
    res = dtu_enter_transfermode();
    if( res != 0 )
    {
        return -3;
    }
    return 0;
}





/**
 * @brief       ���Ͷ���ʾ��
 * @param       phone : �ֻ�����
 * @param       sms_msg: ��������,ֻ����Ӣ�ģ���֧�����Ķ���
 * @retval       1 : ���ŷ���OK��0�����ŷ��ʹ���
 */

int dtu_send_sms(char *phone, char *sms_msg)
{
    #define DTU_SMS_SEND_BUF_MAX    (1024)
    static char dtu_sms_buf[1024];

    int res;
    int ret = 0;

    /*1.DTU��������״̬*/
    res = dtu_enter_configmode();
    if ( res != 0 )
    {
        return -1;
    }

    snprintf(dtu_sms_buf, DTU_SMS_SEND_BUF_MAX, "AT+SMSEND=\"%s\",\"%s\"\r\n", phone, sms_msg);

    /* 2.DTU���Ͷ��� */
    res = send_cmd_to_dtu(dtu_sms_buf, "SMSEND OK", 100);
    if( res == 1 )
    {
        ret = 1;
    }

    /*3.DTU����͸��״̬*/
    res = dtu_enter_transfermode();
    if( res != 0 )
    {
        return -3;
    }

    return ret;
}


