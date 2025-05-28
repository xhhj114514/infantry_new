//app
#include "chassis.h"
#include "robot_def.h"

//module
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "general_def.h"

//bsp
#include "bsp_dwt.h"
#include "arm_math.h"

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
/* 设为静态避免参数传递的开销 */
/************************************** CommUsed **************************************/
static Publisher_t *chassis_pub;                                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                                   // 用于订阅底盘的控制命令
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;                         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data;                 // 底盘回传的反馈数据

/*********************************** CalculateSpeed ***********************************/
static float sin_theta, cos_theta;                                  // 麦轮解算用
static float chassis_vx, chassis_vy;                                // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;                            // 四轮速度
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // 四轮电机实例

/************************************ SuperCapUsed ************************************/
static SuperCapInstance *cap;                                       // 超级电容
static uint16_t power_data;                                         // 发给功率控制板，使功率控制板能稳定在那个功率 

/********************************************************************************************
***************************************      Init     ***************************************
*********************************************************************************************/
void ChassisInit()
{
/***************************************** MotorInit *****************************************/
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&chassis_motor_config);

/****************************************SuperCapCommInit****************************************/
    SuperCap_Init_Config_s capconfig = {
            .can_config = {
                .can_handle = &hcan1,
                .rx_id = 0x311,
                .tx_id = 0x310,
            },
            .recv_data_len = sizeof(int16_t),
            .send_data_len = sizeof(uint16_t),
        };
     cap=SuperCapInit(&capconfig);
/*****************************************PubSubCommInit*****************************************/
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}


/************************************************************************************************
***************************************      Function      **************************************
*************************************************************************************************/

/**************************************** MoveChassis *******************************************/
static void ChassisStateSet()
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }
}

/**
 * @brief 计算每个底盘电机的输出,正运动学解算
 *        
 */
static void MecanumCalculate()
{   
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta; 
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    vt_lf = chassis_vx - chassis_vy - chassis_cmd_recv.wz ;
    vt_lb = chassis_vx + chassis_vy - chassis_cmd_recv.wz ;
    vt_rb = chassis_vx - chassis_vy + chassis_cmd_recv.wz ;
    vt_rf = chassis_vx + chassis_vy + chassis_cmd_recv.wz ;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void ChassisOutput()
{
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/*****************************************SendToPowerLimitBoard*****************************************/
static void SendPowerData()
{
    if( chassis_cmd_recv.robot_level<=6)
    {
        if(chassis_feedback_data.vol<=20)
        {
            if(chassis_cmd_recv.buffer_energy<=60&&chassis_cmd_recv.buffer_energy>40)
            power_data=80;
            else if(chassis_cmd_recv.buffer_energy<=40&&chassis_cmd_recv.buffer_energy>20)
            power_data=65;
            else if(chassis_cmd_recv.buffer_energy<=20)
            power_data=chassis_cmd_recv.power_limit;
        }
        else
        {
            power_data=chassis_cmd_recv.power_limit+5;
        }
    }
    else if( chassis_cmd_recv.robot_level>=6&&chassis_cmd_recv.robot_level<=8)
    {
        if(chassis_feedback_data.vol<=20)
        {
            if(chassis_cmd_recv.buffer_energy<=60&&chassis_cmd_recv.buffer_energy>40)
            power_data=90;
            else if(chassis_cmd_recv.buffer_energy<=40&&chassis_cmd_recv.buffer_energy>20)
            power_data=80;
            else if(chassis_cmd_recv.buffer_energy<=20)
            power_data=chassis_cmd_recv.power_limit;
        }
        else
        {
            power_data=chassis_cmd_recv.power_limit+5;
        }
    }
    else
    {
        if(chassis_feedback_data.vol<=20)
        {
            if(chassis_cmd_recv.buffer_energy<=60&&chassis_cmd_recv.buffer_energy>40)
            power_data=105;
            else if(chassis_cmd_recv.buffer_energy<=40&&chassis_cmd_recv.buffer_energy>20)
            power_data=95;
            else if(chassis_cmd_recv.buffer_energy<=20)
            power_data=chassis_cmd_recv.power_limit;
        }
        else
        {
            power_data=chassis_cmd_recv.power_limit+5;
        }
    }
    chassis_feedback_data.vol=cap->cap_msg.vol;
}


/*********************************************************************************************************
 *********************************************      TASK     *********************************************
**********************************************************************************************************/
void ChassisTask()
{
/********************************************   GetRecvData  *********************************************/
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
/****************************************     ControlChassis     *****************************************/
    ChassisStateSet();
    MecanumCalculate();
    ChassisOutput();

/*******************************************     SendData     ********************************************/
    // 推送反馈消息
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
    // SendPowerData();
    // SuperCapSend(cap, (uint8_t*)&power_data);
}
