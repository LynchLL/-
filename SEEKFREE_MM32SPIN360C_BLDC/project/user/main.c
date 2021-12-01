

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

//�ر�˵�����ÿ�Դ��Ŀ�����ڸ�����ͬѧ������Լ���С��ʱ���вο���Ӳ������������ܹ�ֱ�ӿ���ʹ�����Լ�����Ʒ�У����Ҳο����������Ӳ������������������
//�ر�˵�����ÿ�Դ��Ŀ�����ڸ�����ͬѧ������Լ���С��ʱ���вο���Ӳ������������ܹ�ֱ�ӿ���ʹ�����Լ�����Ʒ�У����Ҳο����������Ӳ������������������
//�ر�˵�����ÿ�Դ��Ŀ�����ڸ�����ͬѧ������Լ���С��ʱ���вο���Ӳ������������ܹ�ֱ�ӿ���ʹ�����Լ�����Ʒ�У����Ҳο����������Ӳ������������������
int main(void)
{
    //��ʼ��LED����
    led_init();
    
    //��ʼ�����ڲ����ò�����
    uart_init(115200);

    //��ʼ���˷ţ�����������ĵ�ѹ���зŴ�Ȼ�󴫵ݸ�adc���вɼ�
    opamp_init();
    
    //��ʼ��adcͨ����adc���ڲɼ���Դ��ѹ��ĸ�ߵ������������
    adc_init();

    //��ʼ���Ƚ���������ʵ�ֶ�ת�������Ƚ�������ʱ�Զ��ر�PWM�������������ֶ��ر�
	comp_init();
    
    //����ƽ���˲���ʼ��
	move_filter_init(&speed_filter);
	
    //������ʼ��
	hall_init();
    
    //pi�ջ���ʼ��
	closed_loop_pi_init();
    
    //�ٶ����߳�ʼ��
	motor_speed_curve_init();
    
    //��ʼ������ٶ��뷽����Ϣ������
    motor_information_out_init();
    
    //�����źŲ����ʼ��
	pwm_input_init();
    
    //��ʼ����ʱ��1�������������PWM
	tim1_complementary_pwm(PWM_PRIOD_LOAD, DEADTIME_LOAD);
    
    //��ʼ����ʱ��3������PI�ջ�����
	tim3_init(TIM3_PSC_LOAD, TIM3_PRIOD);
	while(1)
	{
        //�������ݵ�����ʾ���� ����ʾ������������ https://pan.baidu.com/s/198CMXTZsbI3HAEqNXDngBw
		data_conversion(speed_filter.data_average/10, duty_cycle*1000, closed_loop.out, adc_information.current_bus_filter, virtual_scope_data);
        write_buffer(virtual_scope_data, sizeof(virtual_scope_data));

	}
}

