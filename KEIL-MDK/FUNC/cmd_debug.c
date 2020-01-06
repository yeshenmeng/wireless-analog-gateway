#include "cmd_debug.h"
#include "string_operate.h"
#include "lora_transmission.h"
#include "string.h"
#include "uart_svc.h"
#include "calendar.h"
#include "sys_param.h"


typedef int (*data_parse_t)(uint8_t* data, uint8_t size);
typedef int (*attr_set_t)(uint8_t* data, uint8_t size);

const char cmd_type_end_mark = ':'; 
const char cmd_dev_addr_end_mark = ' ';
const char cmd_attr_end_mark = ' '; 
const char cmd_attr_value_end_mark = ' '; 
const char* cmd_type[] = {
	"w200",
	"w201",
};

#define W200_ATTR_NUMS		21

const char* cmd_w200_attr_tb[] = {
	"long_addr",
	"short_addr",
	"mode",
	"interval",
	"time_stamp",
	"time_offset",
	"x_thres",
	"y_thres",
	"z_thres",
	"gateway_addr",
	"print_ctrl",
	"dev_ctrl",
	"period",
	"sensor_freq",
	"accel_slope",
	"data_points",
	"lora_freq",
	"lora_power",
	"lora_bw",
	"lora_sf",
	"dev_param",
};

#define W200_PRINT_CTRL_NUMS	6
const char* cmd_w200_print_ctrl_tb[] = {
	"raw_open",
	"raw_close",
	"parse_open",
	"parse_close",
	"reply_open",
	"reply_close",
};

#define W200_DEV_CTRL_NUMS	2
const char* cmd_w200_dev_ctrl_tb[] = {
	"reply_open",
	"reply_close",
};

const char* cmd_w200_reply_tb[] = {
	"w200:param setting success",
	"w200:error code 1,param long_addr setting error:",
	"w200:error code 2,param short_addr setting error:",
	"w200:error code 3,param mode setting error:",
	"w200:error code 4,param interval setting error:",
	"w200:error code 5,param time_stamp_addr setting error:",
	"w200:error code 6,param timee_offset setting error:",
	"w200:error code 7,param x_thres setting error:",
	"w200:error code 8,param y_thres setting error:",
	"w200:error code 9,param z_thres setting error:",
	"w200:error code 10,param dev_addr setting error:",
	"w200:error code 11,invalid command:",
	"w200:error code 12,invalid attribute:",
	"w200:error code 13,no instructions exist",
	"w200:error code 14,param gateway_addr setting error:",
	"w200:error code 15,device is not exist",
	"w200:error code 16,param print_ctrl setting error:",
	"w200:error code 17,param dev_ctrl setting error:",
	"w200:error code 18,param period setting error:",
	"w200:error code 19,param sensor_freq setting error:",
	"w200:error code 20,param accel_slope setting error:",
	"w200:error code 21,param data_points setting error:",
	"w200:error code 22,param lora_freq setting error:",
	"w200:error code 23,param lora_power setting error:",
	"w200:error code 24,param lora_bw setting error:",
	"w200:error code 25,param lora_sf setting error:",
};

const char* cmd_w201_attr_tb[] = {
	"long_addr",
	"short_addr",
	"mode",
	"interval",
	"time_stamp",
	"time_offset",
	"x_thres",
	"y_thres",
	"z_thres",
	"gateway_addr",
	"print_ctrl",
	"dev_ctrl",
};

static int cmd_w200_data_parse(uint8_t* data, uint8_t size);
static int cmd_w201_data_parse(uint8_t* data, uint8_t size);
data_parse_t data_parse[sizeof(cmd_type) / sizeof(cmd_type[0])] = {
	cmd_w200_data_parse,
	cmd_w201_data_parse,
};

static int cmd_long_addr_attr_set(uint8_t* data, uint8_t size);
static int cmd_short_addr_attr_set(uint8_t* data, uint8_t size);
static int cmd_mode_attr_set(uint8_t* data, uint8_t size);
static int cmd_interval_attr_set(uint8_t* data, uint8_t size);
static int cmd_time_stamp_attr_set(uint8_t* data, uint8_t size);
static int cmd_time_offset_attr_set(uint8_t* data, uint8_t size);
static int cmd_x_thres_attr_set(uint8_t* data, uint8_t size);
static int cmd_y_thres_attr_set(uint8_t* data, uint8_t size);
static int cmd_z_thres_attr_set(uint8_t* data, uint8_t size);
static int cmd_gateway_addr_attr_set(uint8_t* data, uint8_t size);
static int cmd_print_ctrl_attr_set(uint8_t* data, uint8_t size);
static int cmd_dev_ctrl_attr_set(uint8_t* data, uint8_t size);
static int cmd_period_attr_set(uint8_t* data, uint8_t size);
static int cmd_sensor_freq_attr_set(uint8_t* data, uint8_t size);
static int cmd_accel_slope_attr_set(uint8_t* data, uint8_t size);
static int cmd_data_points_attr_set(uint8_t* data, uint8_t size);
static int cmd_lora_freq_attr_set(uint8_t* data, uint8_t size);
static int cmd_lora_power_attr_set(uint8_t* data, uint8_t size);
static int cmd_lora_bw_attr_set(uint8_t* data, uint8_t size);
static int cmd_lora_sf_attr_set(uint8_t* data, uint8_t size);
static int cmd_lora_param_attr_get(uint8_t* data, uint8_t size);
attr_set_t w200_attr_set[W200_ATTR_NUMS] = {
	cmd_long_addr_attr_set,
	cmd_short_addr_attr_set,
	cmd_mode_attr_set,
	cmd_interval_attr_set,
	cmd_time_stamp_attr_set,
	cmd_time_offset_attr_set,
	cmd_x_thres_attr_set,
	cmd_y_thres_attr_set,
	cmd_z_thres_attr_set,
	cmd_gateway_addr_attr_set,
	cmd_print_ctrl_attr_set,
	cmd_dev_ctrl_attr_set,
	cmd_period_attr_set,
	cmd_sensor_freq_attr_set,
	cmd_accel_slope_attr_set,
	cmd_data_points_attr_set,
	cmd_lora_freq_attr_set,
	cmd_lora_power_attr_set,
	cmd_lora_bw_attr_set,
	cmd_lora_sf_attr_set,
	cmd_lora_param_attr_get,
};

typedef struct {
	uint8_t dev_short_addr[2];
	uint8_t long_addr[8];
	uint8_t gateway_addr[8];
	uint8_t short_addr[2];
	uint8_t mode;
	uint32_t interval;
	uint32_t time_stamp;
	uint16_t time_offset;
	uint32_t period;
	uint8_t sensor_freq;
	uint16_t accel_slope;
	uint16_t data_points;
}comm_attr_t;

typedef struct {
	comm_attr_t comm_attr;
	float x_thres;
	float y_thres;
	float z_thres;
}w200_attr_t;

static w200_attr_t w200_attr;
static w200_attr_t w200_attr_tmp;
static uint8_t cmd_data_rx_size = 0;
static uint8_t cmd_data_buf[255];
static uint8_t w200_reply_mark = 0;
static char w200_reply_msg[100];
static uint8_t w200_data_process_mark = 0;
static uint32_t w200_data_flag = 0;
static char lora_param[255];

static uint16_t m_lora_freq;
static int8_t m_lora_power;
static double m_lora_bw;
static uint8_t m_lora_sf;
	
static double lora_bw_tb[] = {7.81,10.24,15.63,20.83,31.25,41.67,62.5,125,250,500};
static int cmd_lora_param_attr_get(uint8_t* data, uint8_t size)
{
	sys_param_t* param = Sys_ParamGetHandle();
	
	memset(lora_param, 0X00, sizeof(lora_param));
	
	uint8_t i = 0;
	sprintf(lora_param, "%s", "lora频率:");
	for(i = 0; i < sizeof(lora_param); i++)
	{
		if(lora_param[i] == '\0')
		{
			break;
		}
	}
	int_to_string(param->lora_freq, &lora_param[i], 10);
	for(i = 0; i < sizeof(lora_param); i++)
	{
		if(lora_param[i] == '\0')
		{
			break;
		}
	}
	lora_param[i] = ' ';
	
	sprintf(&lora_param[i+1], "%s", "lora功率:");
	for(i = 0; i < sizeof(lora_param); i++)
	{
		if(lora_param[i] == '\0')
		{
			break;
		}
	}
	int_to_string(param->lora_power, &lora_param[i], 10);
	for(i = 0; i < sizeof(lora_param); i++)
	{
		if(lora_param[i] == '\0')
		{
			break;
		}
	}
	lora_param[i] = ' ';
	
	sprintf(&lora_param[i+1], "%s", "lora带宽:");
	for(i = 0; i < sizeof(lora_param); i++)
	{
		if(lora_param[i] == '\0')
		{
			break;
		}
	}
	double bw = lora_bw_tb[param->lora_bw];
	if(bw < 62.5)
	{
		sprintf(&lora_param[i], "%.2f", bw);
	}
	else if(bw == 62.5)
	{
		sprintf(&lora_param[i], "%.1f", bw);
	}
	else
	{
		sprintf(&lora_param[i], "%.0f", bw);
	}
	for(i = 0; i < sizeof(lora_param); i++)
	{
		if(lora_param[i] == '\0')
		{
			break;
		}
	}
	lora_param[i] = ' ';
	
	sprintf(&lora_param[i+1], "%s", "lora扩频因子:");
	for(i = 0; i < sizeof(lora_param); i++)
	{
		if(lora_param[i] == '\0')
		{
			break;
		}
	}
	int_to_string(param->lora_sf, &lora_param[i], 10);
	for(i = 0; i < sizeof(lora_param); i++)
	{
		if(lora_param[i] == '\0')
		{
			break;
		}
	}
	lora_param[i] = ' ';
	
	sprintf(&lora_param[i+1], "%s", "网关地址:");
	for(i = 0; i < sizeof(lora_param); i++)
	{
		if(lora_param[i] == '\0')
		{
			break;
		}
	}
	bytes_to_hex_string(param->dev_gateway_addr, &lora_param[i], sizeof(param->dev_gateway_addr), 0);
	for(i = 0; i < sizeof(lora_param); i++)
	{
		if(lora_param[i] == '\0')
		{
			break;
		}
	}
	lora_param[i] = ' ';
	
	sprintf(&lora_param[i+1], "%s", "软件版本:");
	for(i = 0; i < sizeof(lora_param); i++)
	{
		if(lora_param[i] == '\0')
		{
			break;
		}
	}
	sprintf(&lora_param[i++], "%d", SYS_SW_MAIN_VERSION);
	lora_param[i++] = '.';
	sprintf(&lora_param[i++], "%d", SYS_SW_SUB_VERSION);
	lora_param[i++] = '.';
	sprintf(&lora_param[i++], "%d", SYS_SW_MODIFY_VERSION);
	for(i = 0; i < sizeof(lora_param); i++)
	{
		if(lora_param[i] == '\0')
		{
			break;
		}
	}
	lora_param[i] = '\n';
	
	printf("%s",lora_param);
	w200_reply_mark = 0XFF;//不打印信息
	return -1;
}

static int cmd_lora_sf_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	int value;
	if(string_to_integer((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 26;
		return -1;
	}
	
	uint8_t sf = value & 0xFF;
	if(sf < 7 || sf > 12)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 26;
		return -1;
	}
	
	m_lora_sf = value & 0xFF;
	return 0;
}

static int cmd_lora_bw_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	double value;
	if(string_to_double((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 25;
		return -1;
	}
	
	int i = 0;
	double bw = value;
	for(i = 0; i < sizeof(lora_bw_tb)/sizeof(lora_bw_tb[0]); i++)
	{
		if(bw == lora_bw_tb[i])
		{
			break;
		}
	}
	
	if(i >= sizeof(lora_bw_tb)/sizeof(lora_bw_tb[0]))
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 25;
		return -1;
	}
	
	m_lora_bw = value;
	return 0;
}

static int cmd_lora_power_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	int value;
	if(string_to_integer((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 24;
		return -1;
	}
	
	int8_t power = value & 0xFF;
	if(power < -9 || power > 22)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 24;
		return -1;
	}
	
	m_lora_power = value & 0xFF;
	return 0;
}

static int cmd_lora_freq_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	int value;
	if(string_to_integer((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 23;
		return -1;
	}
	
	uint16_t freq = value & 0xFFFF;
	if(freq < 410 || freq > 800)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 23;
		return -1;
	}
	
	m_lora_freq = value & 0xFFFF;
	return 0;
}

static int cmd_data_points_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	int value;
	if(string_to_integer((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 22;
		return -1;
	}
	w200_attr_tmp.comm_attr.data_points = value & 0xFFFF;
	return 0;
}

static int cmd_accel_slope_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	int value;
	if(string_to_integer((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 21;
		return -1;
	}
	w200_attr_tmp.comm_attr.accel_slope = value & 0xFFFF;
	return 0;
}

static int cmd_sensor_freq_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	int value;
	if(string_to_integer((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 20;
		return -1;
	}
	w200_attr_tmp.comm_attr.sensor_freq = value & 0xFF;
	return 0;
}

static int cmd_period_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	int value;
	if(string_to_integer((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 19;
		return -1;
	}
	w200_attr_tmp.comm_attr.period = value;
	return 0;
}

static int cmd_long_addr_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	
	if(size != 16)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 2;
		return -1;
	}
	
	for(int i = 0; i < size; i++)
	{
		if(is_hex_or_digit(str[i]) == 0) //判断字符是否是16进制字符
		{
			strcpy(w200_reply_msg, str);
			w200_reply_mark = 2;
			return -1;
		}
	}
	
	hex_string_to_bytes((const char*)str, w200_attr_tmp.comm_attr.long_addr, strlen(str));
	return 0;
}

static int cmd_short_addr_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	
	if(size != 4)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 3;
		return -1;
	}
	
	for(int i = 0; i < size; i++)
	{
		if(is_hex_or_digit(str[i]) == 0) //判断字符是否是16进制字符
		{
			strcpy(w200_reply_msg, str);
			w200_reply_mark = 3;
			return -1;
		}
	}
	
	hex_string_to_bytes((const char*)str, w200_attr_tmp.comm_attr.short_addr, strlen(str));
	return 0;
}

static int cmd_mode_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	int value;
	if(string_to_integer((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 4;
		return -1;
	}
	w200_attr_tmp.comm_attr.mode = value & 0xFF;
	return 0;
}

static int cmd_interval_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	int value;
	if(string_to_integer((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 5;
		return -1;
	}
	w200_attr_tmp.comm_attr.interval = value;
	return 0;
}

static int cmd_time_stamp_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	int value;
	if(string_to_integer((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 6;
		return -1;
	}
	w200_attr_tmp.comm_attr.time_stamp = value;
	return 0;
}

static int cmd_time_offset_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	int value;
	if(string_to_integer((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 7;
		return -1;
	}
	w200_attr_tmp.comm_attr.time_offset = value & 0xFFFF;
	return 0;
}

static int cmd_x_thres_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	double value;
	if(string_to_double((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 8;
		return -1;
	}
	w200_attr_tmp.x_thres = value;
	return 0;
}

static int cmd_y_thres_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	double value;
	if(string_to_double((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 9;
		return -1;
	}
	w200_attr_tmp.y_thres = value;
	return 0;
}

static int cmd_z_thres_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	double value;
	if(string_to_double((const char*)str, &value) < 0)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 10;
		return -1;
	}
	w200_attr_tmp.z_thres = value;
	return 0;
}

static int cmd_gateway_addr_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	
	if(size != 16)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 15;
		return -1;
	}
	
	for(int i = 0; i < size; i++)
	{
		if(is_hex_or_digit(str[i]) == 0) //判断字符是否是16进制字符
		{
			strcpy(w200_reply_msg, str);
			w200_reply_mark = 15;
			return -1;
		}
	}
	
	hex_string_to_bytes((const char*)str, w200_attr_tmp.comm_attr.gateway_addr, strlen(str));
	return 0;
}

static int cmd_print_ctrl_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';

	int i;
	for(i = 0; i < W200_PRINT_CTRL_NUMS; i++)
	{
		if(strcmp(str, cmd_w200_print_ctrl_tb[i]) == 0)
		{
			extern ctrl_class_t ctrl_class;
			if(i == 0)
			{
				ctrl_class.print_ctrl |= 0X01;
			}
			else if(i == 1)
			{
				ctrl_class.print_ctrl &= ~0X01;
			}
			else if(i == 2)
			{
				ctrl_class.print_ctrl |= 0X02;
			}
			else if(i == 3)
			{
				ctrl_class.print_ctrl &= ~0X02;
			}
			else if(i == 4)
			{
				ctrl_class.print_ctrl |= 0X04;
			}
			else if(i == 5)
			{
				ctrl_class.print_ctrl &= ~0X04;
			}
			break;
		}
	}
	
	if(i >= W200_PRINT_CTRL_NUMS)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 17;
		return -1;
	}
	
	return 0;
}

static int cmd_dev_ctrl_attr_set(uint8_t* data, uint8_t size)
{
	char str[size+1];
	bytes_to_char(data, str, size);
	str[size] = '\0';
	
	int i;
	for(i = 0; i < W200_DEV_CTRL_NUMS; i++)
	{
		if(strcmp(str, cmd_w200_dev_ctrl_tb[i]) == 0)
		{
			extern ctrl_class_t ctrl_class;
			if(i == 0)
			{
				ctrl_class.dev_ctrl |= 0X01;
			}
			else if(i == 1)
			{
				ctrl_class.dev_ctrl &= ~0X01;
			}
			break;
		}
	}
	
	if(i >= W200_DEV_CTRL_NUMS)
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 18;
		return -1;
	}
	
	return 0;
}

static int cmd_w200_attr_value_parse(uint8_t* data, uint8_t size, uint8_t index)
{
	uint8_t i;
	for(i = 0; i < size; i++)
	{
		if(*(char*)&data[i] == cmd_attr_value_end_mark) //判断属性值结束位置
		{
			break;
		}
	}
	
	if(i > size)
	{
		w200_reply_mark = index + 2;
		return -1;
	}

	w200_data_flag |= (1 << index);
	if(w200_attr_set[index](data, i) < 0) //保存属性值
	{
		return -1;
	}
	
	if(i == size) //数据解析完成
	{
		return 0;
	}
	
	return i;
}

static int cmd_w200_attr_parse(uint8_t* data, uint8_t size)
{
	uint8_t i;
	for(i = 0; i < size; i++)
	{
		if(*(char*)&data[i] == cmd_attr_end_mark) //判断属性结束位置
		{
			break;
		}
	}
	
	if(i >= size)
	{
		char str[size+1];
		bytes_to_char(data, str, size);
		str[i] = '\0';
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 13;
		return -1;
	}
	
	char str[i+1];
	bytes_to_char(data, str, i); //拷贝出属性类型
	str[i] = '\0';
	
	for(uint8_t j = 0; j < W200_ATTR_NUMS; j++)
	{
		if(strcmp(str, cmd_w200_attr_tb[j]) == 0) //判断属性类型是否有效
		{
			int len = cmd_w200_attr_value_parse(&data[i+1], size-i-1, j); //解析属性值
			if(len > -1)
			{
				if(len == 0)
				{
					return 0;
				}
				
				if(cmd_w200_attr_parse(&data[i+len+2], size-i-len-2) == 0)
				{
					return 0;
				}
				else
				{
					return -1;
				}
			}
			else
			{
				return len;
			}
		}
	}
	
	strcpy(w200_reply_msg, str);
	w200_reply_mark = 13;
	return -1;
}

static int cmd_w200_data_parse(uint8_t* data, uint8_t size)
{
	uint8_t i;
	for(i = 0; i < size; i++)
	{
		if(*(char*)&data[i] == cmd_dev_addr_end_mark) //判断测点地址结束位置
		{
			break;
		}
	}
	
	if(i >= size || i != 4)
	{
		char str[i+1];
		bytes_to_char(data, str, i);
		str[i] = '\0';
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 11;
		return -1;
	}
	
	char (*p_str)[4] = (char (*)[4])data; //定义一个指向一个数组的指针，该数组有4个元素
	for(int j = 0; j < 4; j++)
	{
		if(is_hex_or_digit((*p_str)[j]) == 0) //判断字符是否是16进制字符
		{
			char str[i+1];
			bytes_to_char(data, str, i);
			str[i] = '\0';
			strcpy(w200_reply_msg, str);
			w200_reply_mark = 11;
			return -1;
		}
	}
	hex_string_to_bytes((const char*)p_str, w200_attr_tmp.comm_attr.dev_short_addr, i); //保存测点地址
	
	if(cmd_w200_attr_parse(&data[i+1], size-i-1) < 0) //解析属性
	{
		return -1;
	}
	
	return 0;
}

static int cmd_w201_data_parse(uint8_t* data, uint8_t size)
{
	return 0;
}

void cmd_data_rx(uint8_t* data, uint8_t size)
{
	if(cmd_data_rx_size != 0)
	{
		return;
	}
	
	memcpy(cmd_data_buf, data, size);
	cmd_data_rx_size = size;
}

void cmd_data_parse(void)
{
	w200_data_flag = 0;
	if(cmd_data_rx_size == 0)
	{
		return;
	}
	
	uint8_t i;
	for(i = 0; i < cmd_data_rx_size; i++)
	{
		if(*(char*)&cmd_data_buf[i] == cmd_type_end_mark) //判断命令类型结束位置
		{
			break;
		}
	}
	
	if(i >= cmd_data_rx_size)
	{
		w200_reply_mark = 14;
		cmd_data_rx_size = 0;
		return;
	}
	
	char str[i+1];
	bytes_to_char(cmd_data_buf, str, i); //拷贝出命令
	str[i] = '\0';
	
	uint8_t j;
	for(j = 0; j < sizeof(cmd_type) / sizeof(cmd_type[0]); j++)
	{
		if(strcmp(cmd_type[j], str) == 0) //检查命令是否有效
		{
			if(data_parse[j](&cmd_data_buf[i+1], cmd_data_rx_size-i-1) < 0) //数据解析
			{
				cmd_data_rx_size = 0;
				return;
			}
			break;
		}
	}
	
	if(j >= sizeof(cmd_type) / sizeof(cmd_type[0]))
	{
		strcpy(w200_reply_msg, str);
		w200_reply_mark = 12;
		cmd_data_rx_size = 0;
		return;
	}
	
	memcpy(&w200_attr, &w200_attr_tmp, sizeof(w200_attr_tmp));
	cmd_data_rx_size = 0;
	w200_reply_mark = 1;
	w200_data_process_mark = 1;
}

void cmd_data_process(void)
{
	if(w200_data_process_mark == 1)
	{
		w200_data_process_mark = 0;
		extern lora_reply_data_t lora_reply_data;
		lora_reply_data.status = w200_data_flag;
		
		extern peer_data_t peer_data;
		uint8_t is_exit_dev = 0;
		if(peer_data.current_conn_nums != 0 &&
		   (lora_reply_data.status & (~(GATEWAY_ADDR|PRINT_CTRL|DEV_CTRL|LORA_FREQ|LORA_POWER|LORA_BW|LORA_SF))))
		{
			for(int i = 0; i < peer_data.current_conn_nums; i++)
			{
				if(*(uint16_t*)w200_attr.comm_attr.dev_short_addr == 0XFFFF || 
				   *(uint16_t*)&peer_data.peer_attr[i].long_addr[6] == *(uint16_t*)w200_attr.comm_attr.dev_short_addr)
				{
					is_exit_dev = 1;
					peer_data.peer_attr[i].set_flag = 1;
					
					if(*(uint16_t*)w200_attr.comm_attr.dev_short_addr != 0XFFFF)
					{
						break;
					}
				}
			}
		}
		else if(lora_reply_data.status & (GATEWAY_ADDR|PRINT_CTRL|DEV_CTRL|LORA_FREQ|LORA_POWER|LORA_BW|LORA_SF))
		{
			is_exit_dev = 1;
		}
		
		if(is_exit_dev == 0)
		{
			w200_reply_mark = 16;
			return;
		}
		
		if(lora_reply_data.status & PERIOD)
		{
			memcpy(&lora_reply_data.period, &w200_attr.comm_attr.period, sizeof(w200_attr.comm_attr.period));
		}
		
		if(lora_reply_data.status & SENSOR_FREQ)
		{
			memcpy(&lora_reply_data.sensor_freq, &w200_attr.comm_attr.sensor_freq, sizeof(w200_attr.comm_attr.sensor_freq));
		}
		
		if(lora_reply_data.status & ACCEL_SLOPE)
		{
			memcpy(&lora_reply_data.accel_slope, &w200_attr.comm_attr.accel_slope, sizeof(w200_attr.comm_attr.accel_slope));
		}
		
		if(lora_reply_data.status & DATA_POINTS)
		{
			memcpy(&lora_reply_data.data_points, &w200_attr.comm_attr.data_points, sizeof(w200_attr.comm_attr.data_points));
		}
		
		if(lora_reply_data.status & LONG_ADDR)
		{
			memcpy(lora_reply_data.long_addr, w200_attr.comm_attr.long_addr, sizeof(w200_attr.comm_attr.long_addr));
		}
		
		if(lora_reply_data.status & SHORT_ADDR)
		{
			memcpy(lora_reply_data.short_addr, w200_attr.comm_attr.short_addr, sizeof(w200_attr.comm_attr.short_addr));
		}
		
		if(lora_reply_data.status & MODE)
		{
			memcpy(&lora_reply_data.mode, &w200_attr.comm_attr.mode, sizeof(w200_attr.comm_attr.mode));
		}
		
		if(lora_reply_data.status & INTERVAL)
		{
			memcpy(&lora_reply_data.interval, &w200_attr.comm_attr.interval, sizeof(w200_attr.comm_attr.interval));
		}
		
		if(lora_reply_data.status & TIME_STAMP)
		{
			Calendar_t* calendar_mod = Calendar_GetHandle();
			calendar_mod->SetTimeStamp(w200_attr.comm_attr.time_stamp);
//			memcpy(&lora_reply_data.time_stamp, &w200_attr.comm_attr.time_stamp, sizeof(w200_attr.comm_attr.time_stamp));
		}
		
		if(lora_reply_data.status & TIME_OFFSET)
		{
			memcpy(&lora_reply_data.time_offset, &w200_attr.comm_attr.time_offset, sizeof(w200_attr.comm_attr.time_offset));
		}
		
		if(lora_reply_data.status & X_THRES)
		{
			memcpy(&lora_reply_data.x_thres, &w200_attr.x_thres, sizeof(w200_attr.x_thres));
		}
		
		if(lora_reply_data.status & Y_THRES)
		{
			memcpy(&lora_reply_data.y_thres, &w200_attr.y_thres, sizeof(w200_attr.y_thres));
		}
		
		if(lora_reply_data.status & Z_THRES)
		{
			memcpy(&lora_reply_data.z_thres, &w200_attr.z_thres, sizeof(w200_attr.z_thres));
		}
		
		if(lora_reply_data.status & GATEWAY_ADDR)
		{
			memcpy(&lora_reply_data.gateway_addr, &w200_attr.comm_attr.gateway_addr, sizeof(w200_attr.comm_attr.gateway_addr));
		}
		
		if(lora_reply_data.status & PRINT_CTRL)
		{
			lora_reply_data.status &= ~PRINT_CTRL;
		}
		
		if(lora_reply_data.status & DEV_CTRL)
		{
			lora_reply_data.status &= ~DEV_CTRL;
		}
		
		if(lora_reply_data.status & LORA_FREQ ||
		   lora_reply_data.status & LORA_POWER ||
		   lora_reply_data.status & LORA_BW ||
		   lora_reply_data.status & LORA_SF)
		{
			lora_reply_data.status &= ~LORA_FREQ;
			lora_reply_data.status &= ~LORA_POWER;
			lora_reply_data.status &= ~LORA_BW;
			lora_reply_data.status &= ~LORA_SF;
			
			//设置lora
			sys_param_t* param = Sys_ParamGetHandle();
			param->lora_freq = m_lora_freq;
			param->lora_power = m_lora_power;
			for(int i=0; i<10; i++)
			{
				if(m_lora_bw == lora_bw_tb[i])
				{
					param->lora_bw = i;
					break;
				}
			}
			param->lora_sf = m_lora_sf;
			
			extern void LORA_Config(void);
			extern void LORA_ConfigDefault(void);
			extern void LORA_TaskStart(void);
			LORA_ConfigDefault();
			nrf_delay_ms(10);
			LORA_Config();
			LORA_TaskStart();
		}
	}
}

void cmd_data_reply(void)
{
	switch(w200_reply_mark)
	{
		case 1:
			printf("%s\n", cmd_w200_reply_tb[0]);
			break;
		case 2:
			printf("%s", cmd_w200_reply_tb[1]);
			printf("%s\n", w200_reply_msg);
			break;
		case 3:
			printf("%s", cmd_w200_reply_tb[2]);
			printf("%s\n", w200_reply_msg);
			break;
		case 4:
			printf("%s", cmd_w200_reply_tb[3]);
			printf("%s\n", w200_reply_msg);
			break;
		case 5:
			printf("%s", cmd_w200_reply_tb[4]);
			printf("%s\n", w200_reply_msg);
			break;
		case 6:
			printf("%s", cmd_w200_reply_tb[5]);
			printf("%s\n", w200_reply_msg);
			break;
		case 7:
			printf("%s", cmd_w200_reply_tb[6]);
			printf("%s\n", w200_reply_msg);
			break;
		case 8:
			printf("%s", cmd_w200_reply_tb[7]);
			printf("%s\n", w200_reply_msg);
			break;
		case 9:
			printf("%s", cmd_w200_reply_tb[8]);
			printf("%s\n", w200_reply_msg);
			break;
		case 10:
			printf("%s", cmd_w200_reply_tb[9]);
			printf("%s\n", w200_reply_msg);
			break;
		case 11:
			printf("%s", cmd_w200_reply_tb[10]);
			printf("%s\n", w200_reply_msg);
			break;
		case 12:
			printf("%s", cmd_w200_reply_tb[11]);
			printf("%s\n", w200_reply_msg);
			break;
		case 13:
			printf("%s", cmd_w200_reply_tb[12]);
			printf("%s\n", w200_reply_msg);
			break;
		case 14:
			printf("%s\n", cmd_w200_reply_tb[13]);
			break;
		case 15:
			printf("%s", cmd_w200_reply_tb[14]);
			printf("%s\n", w200_reply_msg);
			break;
		case 16:
			printf("%s\n", cmd_w200_reply_tb[15]);
			break;
		case 17:
			printf("%s", cmd_w200_reply_tb[16]);
			printf("%s\n", w200_reply_msg);
			break;
		case 18:
			printf("%s", cmd_w200_reply_tb[17]);
			printf("%s\n", w200_reply_msg);
			break;
		case 19:
			printf("%s", cmd_w200_reply_tb[18]);
			printf("%s\n", w200_reply_msg);
			break;
		case 20:
			printf("%s", cmd_w200_reply_tb[19]);
			printf("%s\n", w200_reply_msg);
			break;
		case 21:
			printf("%s", cmd_w200_reply_tb[20]);
			printf("%s\n", w200_reply_msg);
			break;
		case 22:
			printf("%s", cmd_w200_reply_tb[21]);
			printf("%s\n", w200_reply_msg);
			break;
		case 23:
			printf("%s", cmd_w200_reply_tb[22]);
			printf("%s\n", w200_reply_msg);
			break;
		case 24:
			printf("%s", cmd_w200_reply_tb[23]);
			printf("%s\n", w200_reply_msg);
			break;
		case 25:
			printf("%s", cmd_w200_reply_tb[24]);
			printf("%s\n", w200_reply_msg);
			break;
		case 26:
			printf("%s", cmd_w200_reply_tb[25]);
			printf("%s\n", w200_reply_msg);
			break;
		case 0:
			break;
	}
	
	w200_reply_mark = 0;
}

void cmd_init(void)
{
	sys_param_t* param = Sys_ParamGetHandle();
	m_lora_freq = param->lora_freq;
	m_lora_power = param->lora_power;
	m_lora_bw = lora_bw_tb[param->lora_bw];
	m_lora_sf = param->lora_sf;
	cmd_lora_param_attr_get(NULL, NULL);
}









