/*******************************************************************************
Copyright (2016~2017),TopBand CO.,LTD 
FileName: PID.c
Author: Hunter Zhu
Date:   2017.08.15
Description: ͨѶЭ�����
Version: v0.0

Function List: 
1. InitSpdPI     		PID��ʼ��
2. CalcSpdPI			PID����
History: 2017/8/15 v0.0  build this moudle
*******************************************************************************/

/*-------------------- Includes -----------------------*/
#include "timer.h"
#include "motor.h"
#include "pid.h"


closed_loop_struct closed_loop;



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ֵ����
//  @param      void 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
int32 myabs(int32 x)
{
    return (x>=0?x:-x);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PI�ջ�����
//  @param      read_speed  ��ǰ�ٶ� 
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void closed_loop_pi_calc(int32 read_speed)
{
    if(read_speed)
    {
        closed_loop.real_speed = read_speed;
    }
    closed_loop.real_speed = read_speed;
    
    closed_loop.error = closed_loop.target_speed - closed_loop.real_speed;

    closed_loop.pout = closed_loop.error * (closed_loop.kp + (float)myabs(closed_loop.error/1000)/1800);
    
    //����ϵ�����������ж�̬����
    closed_loop.iout += closed_loop.error * (closed_loop.ki + (float)myabs(closed_loop.error/1000)/38000);
    
    //�����޷�
    if(closed_loop.iout > closed_loop.out_max)
    {
        closed_loop.iout = closed_loop.out_max;
    }
    else if(closed_loop.iout < -closed_loop.out_max)
    {
        closed_loop.iout = -closed_loop.out_max;
    }
    
    //���Ŀ���ٶ�Ϊ0���ߵ�����ر����������
    if((0 == closed_loop.target_speed )|| (MOTOR_DISABLE == motor_control.en_status))
    {
        closed_loop.iout = 0;
    }
    
    closed_loop.out = closed_loop.iout + closed_loop.pout;
    
    //����޷�
    if(closed_loop.out_max < closed_loop.out)
    {
        closed_loop.out = closed_loop.out_max;
    }
    
    if(-closed_loop.out_max > closed_loop.out)
    {
        closed_loop.out = -closed_loop.out_max;
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      PI�ջ������ʼ��
//  @param      void   
//  @return     void					
//  @since      
//-------------------------------------------------------------------------------------------------------------------
void closed_loop_pi_init(void)
{
    closed_loop.out_max = PWM_PRIOD_LOAD;
    closed_loop.kp = 0.001;
    closed_loop.ki = 0.00001;
    closed_loop.out = 0;
    closed_loop.real_speed = 0;
}
