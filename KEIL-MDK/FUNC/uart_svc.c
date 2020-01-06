#include "uart_svc.h"
#include "nrf_uart.h"
#include "app_uart.h"
#include "app_timer.h"
#include "cmd_debug.h"
#include "inclinometer.h"
#include "string_operate.h"
#include "lora_transmission.h"
#include "string.h"
#include "host_net_swap.h"
#include "sx1262.h"


#define UART_TX_BUF_SIZE 256       //串口发送缓存大小（字节数）
#define UART_RX_BUF_SIZE 256       //串口接收缓存大小（字节数）

static uint16_t UartRxBufSize = 0;
static uint8_t UartRxBuf[255];
static uint8_t cmd_flag = 0;
static uint16_t LoraRxBufSize = 0;
static uint8_t LoraRxBuf[255];
static uint8_t LoraRxFlag = 0;

//0X01:打印原始数据
//0X02:打印解析数据
//0X04:打印回复数据
ctrl_class_t ctrl_class = {
	.print_ctrl = 0X02,
	.dev_ctrl = 0X01,
};
peer_data_t peer_data = {0};

void uart_send(uint8_t* data, uint16_t size)
{
	for(int i=0; i<size; i++)
	{
		while(app_uart_put(data[i]) != NRF_SUCCESS);
	}
}

void uart_receive(uint8_t* buf, uint16_t size)
{
	for(int i=0; i<size; i++)
	{
		while(app_uart_get(&buf[i]) != NRF_SUCCESS);
	}
}

APP_TIMER_DEF(uart_id);
void SWT_UartCallback(void* param)
{
	cmd_flag = 1;
	return;
}
void uart_timer_init(void)
{
	app_timer_create(&uart_id,
					APP_TIMER_MODE_SINGLE_SHOT,
					SWT_UartCallback);
}

void bytes_to_chars(uint8_t* des, char* src, uint16_t size)
{
	for(int i=0; i<size; i++)
	{
		src[i] = des[i];
	}
}

void uart_error_handle(app_uart_evt_t * p_event)
{
	//通讯错误事件
	if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
	{
		APP_ERROR_HANDLER(p_event->data.error_communication);
	}
	//FIFO错误事件
	else if (p_event->evt_type == APP_UART_FIFO_ERROR)
	{
		APP_ERROR_HANDLER(p_event->data.error_code);
	}
	//串口接收事件
	else if (p_event->evt_type == APP_UART_DATA_READY)
	{
		app_uart_get(&UartRxBuf[UartRxBufSize]);
		UartRxBufSize++;
		app_timer_start(uart_id, APP_TIMER_TICKS(20), NULL);
	}
	//串口发送完成事件
	else if (p_event->evt_type == APP_UART_TX_EMPTY)
	{
		;
	}
}

void Lora_RxHandler(uint8_t* p_data, uint8_t size)
{
	LoraRxFlag = 1;
	LoraRxBufSize = size;
	memcpy(LoraRxBuf, p_data, size);
}

void iot_param_cfg(void)
{
	if(cmd_flag == 1)
	{
		cmd_flag = 0;
		
		cmd_data_rx(UartRxBuf, UartRxBufSize);
		cmd_data_parse();
		cmd_data_process();
		cmd_data_reply();
		
		memset(UartRxBuf, 0X00, UartRxBufSize);
		UartRxBufSize = 0;
	}
}

void iot_conn_process(void)
{
	extern lora_reply_data_t lora_reply_data;
	memcpy(lora_reply_data.long_addr, &LoraRxBuf[5], 8);

	if(ctrl_class.print_ctrl & 0X02)
	{
		char str[17];
		bytes_to_hex_string(lora_reply_data.long_addr, str, 8, 0);
		char div_1 = ':';
		char div_2 = ' ';
		char div_3 = ' ';
//		if(lora_reply_data.long_addr[0] == 0XC8)
//			printf("倾角测点连接%c",div_1);
//		else if(lora_reply_data.long_addr[0] == 0XC9)
//			printf("崩塌计测点连接%c",div_1);
		printf("测点连接%c",div_1);
		printf("长地址%c%s%c",div_2,str,div_3);
	}

	if(ctrl_class.print_ctrl & 0X02)
	{
		extern sx1262_drive_t* lora_obj_get(void);
		char div_1 = ':';
		printf("无线信号强度%c%d",div_1,lora_obj_get()->radio_state.rssi);
		printf("\n");
	}
		
	if(ctrl_class.dev_ctrl & 0X01 && (lora_reply_data.long_addr[0]==0XC8||lora_reply_data.long_addr[0]==0XC9))
	{
		extern void Lora_ConnReply(void);
		Lora_ConnReply();
		
		int i = 0;
		for(i = 0; i < peer_data.current_conn_nums; i++)
		{
			if(*(uint32_t*)lora_reply_data.long_addr  == *(uint32_t*)peer_data.peer_attr[i].long_addr &&
			   *(uint32_t*)&lora_reply_data.long_addr[4]  == *(uint32_t*)&peer_data.peer_attr[i].long_addr[4])
			{
				peer_data.peer_attr[i].init_flag = 1;
				return;
			}
		}
		
		if(i >=  peer_data.current_conn_nums)
		{
			memcpy(peer_data.peer_attr[peer_data.current_conn_nums].long_addr, lora_reply_data.long_addr, 8);
			peer_data.peer_attr[peer_data.current_conn_nums].init_flag = 1;
			peer_data.current_conn_nums++;
		}
	}
}

uint8_t long_addr_match(uint8_t* param, uint8_t* value, uint8_t len)
{
	for(int i=0; i<len; i++)
	{
		if(param[i] != value[i])
		{
			return 1;
		}
	}
	return 0;
}

uint8_t device_long_addr[8];
void iot_data_push_process(void)
{
	if(ctrl_class.print_ctrl & 0X02)
	{
		char div_1 = ':';
		char div_2 = ' ';
		char div_3 = ' ';
		float value = 0;
		uint8_t index = 0;
		if(LoraRxBuf[4] == 40)
		{
			index += 3;
		}
	
		extern lora_reply_data_t lora_reply_data;
		printf("测点发送数据%c",div_1);
		
		
		char str[17];
		bytes_to_hex_string(&LoraRxBuf[5], str, 8, 0);
		printf("长地址%c%s%c",div_2,str,div_3);
		
		printf("电量%c%d%c",div_2,LoraRxBuf[19+index],div_3);
		
		swap_reverse(&LoraRxBuf[20+index], 4);
		memcpy(&value, &LoraRxBuf[20+index], 4);
		printf("温度%c%.1f%c",div_2,value,div_3);

		uint8_t long_addr[8];
		memcpy(long_addr, &LoraRxBuf[5], 8);
		if(long_addr[0] == 0XC8)
		{
			swap_reverse(&LoraRxBuf[24+index], 4);
			memcpy(&value, &LoraRxBuf[24+index], 4);
			printf("X轴角度%c%.3f%c",div_2,value,div_3);
			
			swap_reverse(&LoraRxBuf[28+index], 4);
			memcpy(&value, &LoraRxBuf[28+index], 4);
			printf("Y轴角度%c%.3f%c",div_2,value,div_3);
			
			swap_reverse(&LoraRxBuf[32+index], 4);
			memcpy(&value, &LoraRxBuf[32+index], 4);
			printf("Z轴角度%c%.3f%c",div_2,value,div_3);
		}
		else if(long_addr[0] == 0XC9)
		{
			swap_reverse(&LoraRxBuf[24+index], 4);
			memcpy(&value, &LoraRxBuf[24+index], 4);
			printf("X轴加速度%c%.3f%c",div_2,value,div_3);
			
			swap_reverse(&LoraRxBuf[28+index], 4);
			memcpy(&value, &LoraRxBuf[28+index], 4);
			printf("Y轴加速度%c%.3f%c",div_2,value,div_3);
			
			swap_reverse(&LoraRxBuf[32+index], 4);
			memcpy(&value, &LoraRxBuf[32+index], 4);
			printf("Z轴加速度%c%.3f%c",div_2,value,div_3);
			
			if(LoraRxBuf[4] == 46)
			{
				swap_reverse(&LoraRxBuf[36+index], 4);
				memcpy(&value, &LoraRxBuf[36+index], 4);
				printf("X轴角度%c%.3f%c",div_2,value,div_3);
				
				swap_reverse(&LoraRxBuf[40+index], 4);
				memcpy(&value, &LoraRxBuf[40+index], 4);
				printf("Y轴角度%c%.3f%c",div_2,value,div_3);
				
				swap_reverse(&LoraRxBuf[44+index], 4);
				memcpy(&value, &LoraRxBuf[44+index], 4);
				printf("Z轴角度%c%.3f%c",div_2,value,div_3);
			}
		}
	}
	
	if(ctrl_class.print_ctrl & 0X02)
	{
		extern sx1262_drive_t* lora_obj_get(void);
		char div_1 = ':';
		printf("无线信号强度%c%d",div_1,lora_obj_get()->radio_state.rssi);
		printf("\n");
	}
	
	if(ctrl_class.dev_ctrl & 0X01)
	{
		int i;
		uint8_t long_addr[8];
		memcpy(long_addr, &LoraRxBuf[5], 8);
		for(i = 0; i < peer_data.current_conn_nums; i++)
		{
			if(long_addr_match(peer_data.peer_attr[i].long_addr, long_addr, 8) == 0)
			{
				break;
			}
		}
		
		if(i >= peer_data.current_conn_nums)
		{
			return;
		}
		
		uint8_t reply_flag = 0;
		extern lora_reply_data_t lora_reply_data;
		
		if(long_addr[0] == 0XC8)
		{
			reply_flag = 1;
		}
		else if(long_addr[0] == 0XC9)
		{
			reply_flag = 1;
		}
		
		if(!reply_flag)
			return;
		
		memcpy(device_long_addr, long_addr, 8);
		if(long_addr[0] == 0XC8)
		{
			extern void Lora_DataReply(void);
			Lora_DataReply();
		}
		else if(long_addr[0] == 0XC9)
		{
			extern void Lora_C_DataReply(void);
			Lora_C_DataReply();
		}
	}
}

void iot_data_lost_rate_process(void)
{
	if(ctrl_class.print_ctrl & 0X02)
	{
		extern lora_reply_data_t lora_reply_data;
		memcpy(lora_reply_data.long_addr, &LoraRxBuf[5], 8);
		extern uint8_t payload_length;
		payload_length = LoraRxBuf[13];
		
		char div_1 = ':';
		printf("LORA丢包率测试%c",div_1);
		extern sx1262_drive_t* lora_obj_get(void);
		printf("无线信号强度%c%d",div_1,lora_obj_get()->radio_state.rssi);
		printf("\n");
	}
	
	if(ctrl_class.dev_ctrl & 0X01)
	{
		extern void Lora_TestReply(void);
		Lora_TestReply();
	}
}

void uart_test(void)
{
	if(LoraRxFlag == 1)
	{
		LoraRxFlag = 0;

		if(ctrl_class.print_ctrl & 0X01)
		{
			char div_1 = ':';
			extern lora_reply_data_t lora_reply_data;
//			if(lora_reply_data.long_addr[0] == 0XC8)
//				printf("倾角测点发送原始数据%c",div_1);
//			else if(lora_reply_data.long_addr[0] == 0XC9)
//				printf("崩塌计测点发送原始数据%c",div_1);
			printf("测点发送原始数据%c",div_1);
			for(int i=0; i<LoraRxBufSize; i++)
			{
				printf("0x%02x ", LoraRxBuf[i]);
				if(i%30 == 0 && i != 0)
				{
					nrf_delay_ms(10);
				}
			}
			nrf_delay_ms(5);
		}
		
		if(*(uint32_t*)LoraRxBuf == 0X01000000)
		{
			printf("\n");
			
			if(!((ctrl_class.print_ctrl & 0X02) || (ctrl_class.print_ctrl & 0X04)))
			{
				printf("\n");
			}
			iot_conn_process();
			printf("\n");
		}
		else if(*(uint32_t*)LoraRxBuf == 0X03000000)
		{
			printf("\n");
			
			if(!((ctrl_class.print_ctrl & 0X02) || (ctrl_class.print_ctrl & 0X04)))
			{
				printf("\n");
			}
			iot_data_push_process();
			printf("\n");
		}
		else if(*(uint32_t*)LoraRxBuf == 0X05000000)
		{
			printf("\n");
			
			if(!((ctrl_class.print_ctrl & 0X02) || (ctrl_class.print_ctrl & 0X04)))
			{
				printf("\n");
			}
			iot_data_lost_rate_process();
			printf("\n");
		}
		else
		{
			extern sx1262_drive_t* lora_obj_get(void);
			char div_1 = ':';
			printf("无线信号强度%c%d",div_1,lora_obj_get()->radio_state.rssi);
			printf("\n");
			printf("\n");
		}
	}
	
	iot_param_cfg();
}

void uart_run(void)
{
	uart_test();
}

//串口配置
void uart_init(void)
{
	uint32_t err_code;

	//定义串口通讯参数配置结构体并初始化
	const app_uart_comm_params_t comm_params =
	{
		UART_RX_PIN,//定义uart接收引脚
		UART_TX_PIN,//定义uart发送引脚
		UART_RTS_PIN,//定义uart RTS引脚，流控关闭后虽然定义了RTS和CTS引脚，但是驱动程序会忽略，不会配置这两个引脚，两个引脚仍可作为IO使用
		UART_CTS_PIN,//定义uart CTS引脚
		APP_UART_FLOW_CONTROL_DISABLED,//关闭uart硬件流控
		false,//禁止奇偶检验
		NRF_UART_BAUDRATE_115200//uart波特率设置为115200bps
	};

	//初始化串口，注册串口事件回调函数
	//函数会启动一次接收，如果接收引脚不接会报错
	APP_UART_FIFO_INIT(&comm_params,
					 UART_RX_BUF_SIZE,
					 UART_TX_BUF_SIZE,
					 uart_error_handle,
					 APP_IRQ_PRIORITY_LOWEST,
					 err_code);

	APP_ERROR_CHECK(err_code);
	uart_timer_init();
}

#if 0
char* header_str = "header";
char* length_str = "length";
char* short_addr_str = "short_addr";
char* attr_nums_str = "attr_nums";
char* gas_id_str = "gas_id";
char* temp_id_str = "temp_id";
char* x_accel_id_str = "x_accel_id";
char* y_accel_id_str = "y_accel_id";
char* z_accel_id_str = "z_accel_id";
char* x_angle_id_str = "x_angle_id";
char* y_angle_id_str = "y_angle_id";
char* z_angle_id_str = "z_angle_id";
char* gas_value_str = "gas_value";
char* temp_value_str = "temp_value";
char* x_accel_value_str = "x_accel_value";
char* y_accel_value_str = "y_accel_value";
char* z_accel_value_str = "z_accel_value";
char* x_angle_value_str = "x_angle_value";
char* y_angle_value_str = "y_angle_value";
char* z_angle_value_str = "z_angle_value";

if(*(uint32_t*)LoraRxBuf == 0X03000000)
{
	printf("%s,",header_str);
	printf("%s,",length_str);
	printf("%s,",short_addr_str);
	printf("%s,",attr_nums_str);
	printf("%s,",gas_id_str);
	printf("%s,",temp_id_str);
	printf("%s,",x_accel_id_str);
	printf("%s,",y_accel_id_str);
	printf("%s,",z_accel_id_str);
	nrf_delay_ms(10);
	if(LoraRxBuf[4] == 40)
	{
		printf("%s,",x_angle_id_str);
		printf("%s,",y_angle_id_str);
		printf("%s,",z_angle_id_str);
	}
	printf("%s,",gas_value_str);
	printf("%s,",temp_value_str);
	printf("%s,",x_accel_value_str);
	printf("%s,",y_accel_value_str);
	printf("%s,",z_accel_value_str);
	if(LoraRxBuf[4] == 40)
	{
		printf("%s,",x_angle_value_str);
		printf("%s,",y_angle_value_str);
		printf("%s,",z_angle_value_str);
	}
	nrf_delay_ms(10);
	printf("\n");
	swap_reverse(&LoraRxBuf[0], 4);
	printf("%-*d,",strlen(header_str),*(uint32_t*)&LoraRxBuf[0]); //头
	printf("%-*d,",strlen(length_str),LoraRxBuf[4]); //长度
	printf("0x%02x,0x%02x ,", LoraRxBuf[5],LoraRxBuf[6]); //短地址
	printf("%-*d,",strlen(attr_nums_str),LoraRxBuf[7]); //属性个数
	printf("%-*d,",strlen(gas_id_str),LoraRxBuf[8]); //电量属性号
	printf("%-*d,",strlen(temp_id_str),LoraRxBuf[9]); //温度属性号
	printf("%-*d,",strlen(x_accel_id_str),LoraRxBuf[10]); //x加速度属性号
	printf("%-*d,",strlen(y_accel_id_str),LoraRxBuf[11]); //y加速度属性号
	printf("%-*d,",strlen(z_accel_id_str),LoraRxBuf[12]); //z加速度属性号
	nrf_delay_ms(10);
	uint8_t index = 0;
	if(LoraRxBuf[4] == 40)
	{
		printf("%-*d,",strlen(x_angle_id_str),LoraRxBuf[13]); //x角度属性号
		printf("%-*d,",strlen(y_angle_id_str),LoraRxBuf[14]); //y角度属性号
		printf("%-*d,",strlen(z_angle_id_str),LoraRxBuf[15]); //z角度属性号
		index += 3;
	}
	printf("%-*d,",strlen(gas_value_str),LoraRxBuf[13+index]); //电量值
	swap_reverse(&LoraRxBuf[14+index], 4);
	float value = 0;
	memcpy(&value, &LoraRxBuf[14+index], 4);
	printf("%-*.3f,",strlen(temp_value_str),value); //温度值
	swap_reverse(&LoraRxBuf[18+index], 4);
	memcpy(&value, &LoraRxBuf[18+index], 4);
	printf("%-*.3f,",strlen(x_accel_value_str),value); //x加速度值
	swap_reverse(&LoraRxBuf[22+index], 4);
	memcpy(&value, &LoraRxBuf[22+index], 4);
	printf("%-*.3f,",strlen(y_accel_value_str),value); //y加速度值
	swap_reverse(&LoraRxBuf[26+index], 4);
	memcpy(&value, &LoraRxBuf[26+index], 4);
	printf("%-*.3f,",strlen(z_accel_value_str),value); //z加速度值
	nrf_delay_ms(10);
	if(LoraRxBuf[4] == 40)
	{
		swap_reverse(&LoraRxBuf[30+index], 4);
		memcpy(&value, &LoraRxBuf[30+index], 4);
		printf("%-*.3f,",strlen(x_angle_value_str),value); //x角度值
		swap_reverse(&LoraRxBuf[34+index], 4);
		memcpy(&value, &LoraRxBuf[34+index], 4);
		printf("%-*.3f,",strlen(y_angle_value_str),value); //y角度值
		swap_reverse(&LoraRxBuf[38+index], 4);
		memcpy(&value, &LoraRxBuf[38+index], 4);
		printf("%-*.3f,",strlen(z_angle_value_str),value); //z角度值
	}
	printf("\n");
}
#endif











