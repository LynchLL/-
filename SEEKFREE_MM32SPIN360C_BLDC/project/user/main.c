

#include "HAL_conf.h"
#include "HAL_device.h"


#include "gpio.h"
#include "adc.h"
#include "timer.h"
#include "pwm_input.h"
#include "opamp.h"
#include "comp.h"
#include "uart.h"

#include "hall.h"
#include "pid.h"
#include "motor.h"
#include "move_filter.h"
#include "virsco.h"

//特别说明：该开源项目仅用于各参赛同学在设计自己的小车时进行参考，硬件和软件都不能够直接拷贝使用在自己的作品中，请大家参考后重新完成硬件和软件的设计制作。
//特别说明：该开源项目仅用于各参赛同学在设计自己的小车时进行参考，硬件和软件都不能够直接拷贝使用在自己的作品中，请大家参考后重新完成硬件和软件的设计制作。
//特别说明：该开源项目仅用于各参赛同学在设计自己的小车时进行参考，硬件和软件都不能够直接拷贝使用在自己的作品中，请大家参考后重新完成硬件和软件的设计制作。
int main(void)
{
    //初始化LED引脚
    led_init();
    
    //初始化串口并设置波特率
    uart_init(115200);

    //初始化运放，将采样电阻的电压进行放大，然后传递给adc进行采集
    opamp_init();
    
    //初始化adc通道，adc用于采集电源电压、母线电流、相电流的
    adc_init();

    //初始化比较器，用于实现堵转保护，比较器触发时自动关闭PWM输出，无需代码手动关闭
	comp_init();
    
    //滑动平均滤波初始化
	move_filter_init(&speed_filter);
	
    //霍尔初始化
	hall_init();
    
    //pi闭环初始化
	closed_loop_pi_init();
    
    //速度曲线初始化
	motor_speed_curve_init();
    
    //初始化输出速度与方向信息的引脚
    motor_information_out_init();
    
    //输入信号捕获初始化
	pwm_input_init();
    
    //初始化定时器1，用于输出互补PWM
	tim1_complementary_pwm(PWM_PRIOD_LOAD, DEADTIME_LOAD);
    
    //初始化定时器3，用于PI闭环计算
	tim3_init(TIM3_PSC_LOAD, TIM3_PRIOD);
	while(1)
	{
        //发送数据到虚拟示波器 虚拟示波器下载链接 https://pan.baidu.com/s/198CMXTZsbI3HAEqNXDngBw
		data_conversion(speed_filter.data_average/10, duty_cycle*1000, closed_loop.out, adc_information.current_bus_filter, virtual_scope_data);
        write_buffer(virtual_scope_data, sizeof(virtual_scope_data));

	}
}

