// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "buzzer.h"
#include "referee_UI.h"
#include "referee_task.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

//comm->通信，这里指的是和整个模块实例进行通信
/* cmd应用包含的模块实例指针和交互信息存储*/
/************************************** HuartUsed **************************************/
static RC_ctrl_t *rc_data;                              // 遥控器数据,  初始化时返回（串口3）
static Minipc_Recv_s *minipc_recv_data;                 // 视觉接收数据,初始化时返回（串口1）
static Minipc_Send_s minipc_send_data;                  // 视觉发送数据            （串口1）
static referee_info_t* referee_data;                    // 用于获取裁判系统的数据   （串口6）
static Referee_Interactive_info_t ui_data;              // UI数据                  (串口6)

/**************************************ChassisUsed**************************************/
static Publisher_t *chassis_cmd_pub;                    // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub;                  // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send;             // 发送给底盘应用的信息
static Chassis_Upload_Data_s chassis_fetch_data;        // 从底盘应用接收的反馈信息信息
static float chassis_speed_buff_1,chassis_rotate_buff_1;//1是无超电时的等级加成
static float chassis_speed_buff_2,chassis_rotate_buff_2;//2是有超电时的等级加成
static float chassis_speed_buff_3;//无陀螺
/************************************** GimbalUsed **************************************/
static Publisher_t *gimbal_cmd_pub;                     // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;                   // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;               // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data;          // 从云台获取的反馈信息
static uint8_t gimbal_location_init=0;                  // 云台电机设置零位使用

/**************************************  ShootUsed  **************************************/
static Publisher_t *shoot_cmd_pub;                      // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;                    // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;                 // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data;            // 从发射获取的反馈信息

/****************************************  Other  ****************************************/
static DataLebel_t DataLebel;                           // 用于记录时间或标志位
static  BuzzzerInstance *aim_success_buzzer;            // 判断是否能击打目标
static float cnt1;

/********************************************************************************************
***************************************      Init     ***************************************
*********************************************************************************************/
void RobotCMDInit()
{
/**************************************  HuartInit  **************************************/
    rc_data = RemoteControlInit(&huart3);               // 遥控器通信串口
    minipc_recv_data = minipcInit(&huart1);             // 视觉通信串口
    referee_data= UITaskInit(&huart6,&ui_data);         // UI通信串口

/**************************************GimbalCommInit**************************************/
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_cmd_send.pitch = 0;

/************************************** ShootCommInit **************************************/
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

/**************************************ChassisCommInit**************************************/
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

/**************************************   BufferInit  **************************************/
    Buzzer_config_s aim_success_buzzer_config= {
        .alarm_level=ALARM_LEVEL_ABOVE_MEDIUM,
        .octave=OCTAVE_2,
    };
    aim_success_buzzer= BuzzerRegister(&aim_success_buzzer_config);
}


/*********************************************************************************************
***************************************      Function      ***********************************
**********************************************************************************************/

/**************************************      BasicSet      *************************************/
/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle,yaw_align_angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度


#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > yaw_align_angle && angle <= 180.0f + yaw_align_angle)
        chassis_cmd_send.offset_angle = angle - yaw_align_angle;
    else if (angle > 180.0f + yaw_align_angle)
        chassis_cmd_send.offset_angle = angle - yaw_align_angle - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - yaw_align_angle;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE_1)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE_1;
    else if (angle <= yaw_align_angle && angle >= YAW_ALIGN_ANGLE_1 - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE_1;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE_1 + 360.0f;
#endif
}

/**
 * @brief Pitch轴限位，设定云台为正常工作模式
 *
 */
static void GimbalPitchLimit()
{
    gimbal_cmd_send.gimbal_mode=GIMBAL_GYRO_MODE;
    // //云台软件限位
    if(gimbal_cmd_send.pitch<PITCH_MIN_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    else if (gimbal_cmd_send.pitch>PITCH_MAX_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    else
    gimbal_cmd_send.pitch=gimbal_cmd_send.pitch;
}

/**
 * @brief 判断视觉有没有发信息
 *
 */
static void VisionJudge()
{
    //有深度代表有视觉信息
    if(minipc_recv_data->Vision.deep!=0)
    {
        DataLebel.aim_flag=1;
        //检测到装甲板，开启蜂鸣器
        AlarmSetStatus(aim_success_buzzer, ALARM_ON);
        //与装甲板中心的距离越近，蜂鸣器越响
        if(abs(minipc_recv_data->Vision.yaw)>1&&aim_success_buzzer->loudness<0.5)
        {
            aim_success_buzzer->loudness=0.5*(1/abs(minipc_recv_data->Vision.yaw));
        }
        else if(abs(minipc_recv_data->Vision.yaw)<1)
        {
            //离装甲板距离较近时，开火
            aim_success_buzzer->loudness=0.5;
            if(DataLebel.reverse_flag==1)
            {
                DataLebel.fire_flag=0;
            }
            else
            {
                DataLebel.fire_flag=1;
            }
        }
    }
     //检测不到装甲板，关蜂鸣器，关火
    else if(minipc_recv_data->Vision.deep==0 && DataLebel.aim_flag==1)       
    {
        DataLebel.fire_flag=0;
        DataLebel.aim_flag=0;
        AlarmSetStatus(aim_success_buzzer, ALARM_OFF);    
    }
}

/**
 * @brief 判断有无超电
 */
static void PowerCapJudge()
{
    if(chassis_fetch_data.vol>12&&chassis_fetch_data.vol<23)
    {
        chassis_fetch_data.power_flag=1;
    }
    else
    {
        chassis_fetch_data.power_flag=0;
    }
}

/**
 * @brief 底盘旋转速度设定
 *
 */
static void ChassisRotateSet()
{
    // 根据控制模式设定旋转速度
    switch (chassis_cmd_send.chassis_mode)
    {
        //底盘跟随
        case CHASSIS_FOLLOW_GIMBAL_YAW: 
            chassis_cmd_send.wz =-20.0*abs(chassis_cmd_send.offset_angle)*chassis_cmd_send.offset_angle;
        break;
        //小陀螺
        case CHASSIS_ROTATE: 
            chassis_cmd_send.wz =(10000+100*sin(DWT_GetTimeline_s()))*chassis_cmd_send.chassis_rotate_buff;
        break;
        case CHASSIS_MOVE:
            chassis_cmd_send.wz =-10.0*abs(chassis_cmd_send.offset_angle)*chassis_cmd_send.offset_angle;
        break;

        //未知
        default:
        break;
    }
}

/**
 * @brief 基础设定，包括偏角计算、云台限位，超电判断，自瞄判断，以及发射基本模式设定
 *
 */
 static uint8_t aaa;
static void BasicSet()
{
    CalcOffsetAngle();
    GimbalPitchLimit();
    PowerCapJudge();
    VisionJudge();
    ChassisRotateSet();

    //发射基本模式设定
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.shoot_rate=8;
    chassis_cmd_send.power_limit=referee_data->GameRobotState.chassis_power_limit;

    if(rc_data[TEMP].rc.dial>200)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }
}

/************************************** GimbalSet   **************************************/
/**
 * @brief 云台遥控器控制
 *
 */
static void GimbalRC()
{
    gimbal_cmd_send.yaw -= 0.0005f * (float)rc_data[TEMP].rc.rocker_right_x;//0
    gimbal_cmd_send.pitch -= 0.00003f * (float)rc_data[TEMP].rc.rocker_right_y;
}

/**
 * @brief 云台视觉控制
 *
 */
static void GimbalAC()
{
    gimbal_cmd_send.yaw-=0.007f*minipc_recv_data->Vision.yaw;   //往右获得的yaw是减
}

/************************************** ChassisSet   **************************************/
/**
 * @brief 底盘遥控器控制
 *
 */
static void ChassisRC()
{
    chassis_cmd_send.vx =- 30.0f * (float)rc_data[TEMP].rc.rocker_left_y; // _水平方向
    chassis_cmd_send.vy =30.0f * (float)rc_data[TEMP].rc.rocker_left_x; // 竖直方向
    chassis_cmd_send.chassis_rotate_buff=1.0;
    chassis_cmd_send.chassis_speed_buff=1;
    if (switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.chassis_mode=CHASSIS_FOLLOW_GIMBAL_YAW;
    }
    if(switch_is_mid(rc_data[TEMP].rc.switch_left))
        chassis_cmd_send.chassis_mode=CHASSIS_ROTATE;
}

/************************************** ShootSet   **************************************/
/**
 * @brief 发射机构遥控器控制
 *
 */
static void ShootRC()
{
    if(rc_data->rc.dial>200)
    {
        shoot_cmd_send.loader_mode=LOAD_BURSTFIRE;
    }
    else if (rc_data->rc.dial<-200)
    {
        shoot_cmd_send.loader_mode=LOAD_REVERSE;
        DataLebel.reverse_flag=1;
    }
    else
    {
        shoot_cmd_send.loader_mode=LOAD_STOP;
        DataLebel.reverse_flag=0;
    }
}

/************************************** AutoAimSet   **************************************/
/**
 * @brief 自动瞄准设置
 * 
 * 本函数根据当前的数据标志来控制云台的动作和射击模式
 * 它首先检查是否处于瞄准状态，如果是，则调用云台控制函数
 * 进一步检查是否需要开火，如果是，则设置射击模式为连发
 *
 */
static void AutoAimSet()
{
    GimbalAC();
    if(rc_data[TEMP].mouse.press_l==1)
    {
        if(DataLebel.reverse_flag==1)
        {
            shoot_cmd_send.loader_mode = LOAD_REVERSE;
        }
        else
        {
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
        }
    }
    else
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }
}

/**************************************RemoteControlSet**************************************/
/**
 * @brief  RemoteControlSet函数用于根据遥控器输入控制机器人，包括底盘、云台和射击系统的控制。
 * 该函数首先调用ChassisRC控制底盘运动，然后根据遥控器左侧开关的状态决定是否启用自动瞄准模式。
 * 在自动瞄准模式下，会调用AutoAimSet进行自动瞄准设置，并根据是否已瞄准目标来决定是否继续遥控云台和射击系统。
 * 如果不启用自动瞄准模式，则直接遥控云台和射击系统。
 */
static void RemoteControlSet()
{
    ChassisRC();

    if(switch_is_up(rc_data[TEMP].rc.switch_left)) 
    {
        gimbal_cmd_send.autoaim_mode=ANGRY;
        AutoAimSet();
        if(DataLebel.aim_flag!=1)
        {
            ShootRC();
            GimbalRC();
        }
    }
    else
    {
        GimbalRC();
        ShootRC();
    }
}

/**************************************ComputerControlSet**************************************/
/**
 * @brief 纯手瞄
 */
static void NoneAutoMouseControl()
{
    gimbal_cmd_send.yaw -= (float)rc_data[TEMP].mouse.x / 660 *3 ; 
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660/57*1.5 ;
    if(rc_data[TEMP].mouse.press_l==1)
    {
        if(DataLebel.reverse_flag==1)
        {
            shoot_cmd_send.loader_mode = LOAD_REVERSE;
        }
        else
        {
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
        }
    }
    else
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }            
}

/**
 * @brief 鼠标控制函数
 * 如果鼠标右键被按下，自动瞄准模式将开启，否则将关闭
 * 在自动瞄准模式下，将调用AutoAimSet函数进行自动瞄准设置
 * 如果自动瞄准设置未成功（aim_flag不为1），则调用NoneAutoMouseControl函数进行非自动模式下的鼠标控制
 * 在非自动瞄准模式下，,如果找不到目标也将用手瞄
 */
static void MouseControl()
{
    if(rc_data[TEMP].mouse.press_r==1)
    {
        if(DataLebel.aim_flag==1)
        {
            gimbal_cmd_send.autoaim_mode=ANGRY;
        }
        else
        {
            gimbal_cmd_send.autoaim_mode=NOTHING;
        }
    }
    else if(rc_data[TEMP].mouse.press_r==0)
    {
        if(DataLebel.aim_flag==1)
        {
            gimbal_cmd_send.autoaim_mode=FOUND;
        }
        else
        {
            gimbal_cmd_send.autoaim_mode=AUTO_OFF;
        }
    
    }
    if(gimbal_cmd_send.autoaim_mode==ANGRY)
    {
        AutoAimSet();
        gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660/57*1.5 ;
    }
    else
    {
        NoneAutoMouseControl();
    }
}

/**
 * @brief 键盘控制函数
 */
static void KeyControl()
{

    //根据R键控制底盘模式
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) 
    {
    case 0:
        if(rc_data[TEMP].key_count[KEY_PRESS][Key_C]%2==0)
        chassis_cmd_send.chassis_mode =CHASSIS_FOLLOW_GIMBAL_YAW;
        else
        chassis_cmd_send.chassis_mode =CHASSIS_MOVE;
        break;
    default:
        chassis_cmd_send.chassis_mode =CHASSIS_ROTATE;
    }

    // 根据W/S键设置纵向速度，根据A/D键设置横向速度
    chassis_cmd_send.vx = -(rc_data[TEMP].key[KEY_PRESS].w * 10000 - rc_data[TEMP].key[KEY_PRESS].s * 10000)*chassis_cmd_send.chassis_speed_buff; 
    chassis_cmd_send.vy = -(rc_data[TEMP].key[KEY_PRESS].d * 10000 - rc_data[TEMP].key[KEY_PRESS].a * 10000)*chassis_cmd_send.chassis_speed_buff;

    // 根据机器人等级设置速度和旋转缓冲系数(无超电使用)
    switch (referee_data->GameRobotState.robot_level)
    {
    case 1:
        chassis_rotate_buff_1 = 1;
        chassis_speed_buff_1  = 1;
        break;
    case 2:         
        chassis_rotate_buff_1 = 1.09;
        chassis_speed_buff_1  = 1.09;
        break;
    case 3:
        chassis_rotate_buff_1 = 1.16;
        chassis_speed_buff_1  = 1.16;
        break;
    case 4:
        chassis_rotate_buff_1 = 1.21;
        chassis_speed_buff_1  = 1.21;
        break;
    case 5:
        chassis_rotate_buff_1 = 1.23;
        chassis_speed_buff_1  = 1.3;
        break;
    case 6:
        chassis_rotate_buff_1 = 1.3;
        chassis_speed_buff_1  = 1.4;
        break;
    case 7:
        chassis_rotate_buff_1 = 1.38;
        chassis_speed_buff_1  = 1.52;
        break;
    case 8:
        chassis_rotate_buff_1 = 1.45;
        chassis_speed_buff_1  = 1.6;
        break;
    case 9:
        chassis_rotate_buff_1 = 1.55;
        chassis_speed_buff_1  = 1.72;
        break;
    case 10:
        chassis_rotate_buff_1 = 1.7;
        chassis_speed_buff_1  = 1.81;
        break;
    default:
        chassis_rotate_buff_1 = 1;
        chassis_speed_buff_1  = 1;
        break;
    }

    //3级以下，对标功率优先3级（血量优先6级）
    if(referee_data->GameRobotState.robot_level<=3)
    {
        chassis_speed_buff_2= 1.6;
        // 8 1.45
        if(chassis_fetch_data.vol<=24&&chassis_fetch_data.vol>=18)
        {
            chassis_rotate_buff_2= 1.42*((chassis_fetch_data.vol-17)*0.01+1);
        }
        //7 1.45
        else if(chassis_fetch_data.vol<18&&chassis_fetch_data.vol>=14)
        {
            chassis_rotate_buff_2= 1.36*((chassis_fetch_data.vol-13)*0.012+1);
        }
        //6 1.38
        else if(chassis_fetch_data.vol<14&&chassis_fetch_data.vol>=10)
        {
            chassis_rotate_buff_2= 1.31*((chassis_fetch_data.vol-9)*0.0114+1);
        }
        //5 1.3
        else
        {
            chassis_rotate_buff_2=chassis_speed_buff_1;
        }
    }
    //4-6，对标功率优先6级（血量优先8级），
    else if (referee_data->GameRobotState.robot_level>3&&referee_data->GameRobotState.robot_level<=6)
    {
        chassis_speed_buff_2= 1.72;
        //9 1.55
        if(chassis_fetch_data.vol<=24&&chassis_fetch_data.vol>=18)
        {
            chassis_rotate_buff_2= 1.44*((chassis_fetch_data.vol-17)*0.011+1);
        }
        //8 1.45
        else if(chassis_fetch_data.vol<18&&chassis_fetch_data.vol>=14)
        {
            chassis_rotate_buff_2= 1.4*((chassis_fetch_data.vol-13)*0.01+1);
        }
        //7 1.38
        else if(chassis_fetch_data.vol<14&&chassis_fetch_data.vol>=10)
        {
            chassis_rotate_buff_2= 1.32*((chassis_fetch_data.vol-9)*0.015+1);
        }
        //6 1.3
        else
        {
            chassis_rotate_buff_2=chassis_speed_buff_1;
        }
    }
    //7-10，对标功率优先10级（血量优先8级），
    else if (referee_data->GameRobotState.robot_level>6&&referee_data->GameRobotState.robot_level<=10)
    {
        chassis_speed_buff_2= 1.81;
        //10
        if(chassis_fetch_data.vol<=24&&chassis_fetch_data.vol>=18)
        {
            chassis_rotate_buff_2= 1.55*((chassis_fetch_data.vol-17)*0.016+1);
        }
        //9 1.55
        else if(chassis_fetch_data.vol<18&&chassis_fetch_data.vol>=14)
        {
            chassis_rotate_buff_2= 1.45*((chassis_fetch_data.vol-13)*0.013+1);
        }
        //8 1.45
        else if(chassis_fetch_data.vol<14&&chassis_fetch_data.vol>=10)
        {
            chassis_rotate_buff_2= 1.41*((chassis_fetch_data.vol-9)*0.01+1);
        }
        //7 1.38
        else
        {
            chassis_rotate_buff_2=chassis_speed_buff_1;
        }    
    }

    //选择最快的那个速度
    if(chassis_cmd_send.chassis_mode==CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        chassis_speed_buff_3=chassis_speed_buff_2+0.6;
    }
    else
    {
        chassis_speed_buff_3=chassis_speed_buff_2;
    }

    chassis_cmd_send.chassis_speed_buff = (chassis_speed_buff_1 >= chassis_speed_buff_3) ? chassis_speed_buff_1 : chassis_speed_buff_3;

    // 选择最快的旋转速度
    chassis_cmd_send.chassis_rotate_buff = (chassis_rotate_buff_1 >= chassis_rotate_buff_2) ? chassis_rotate_buff_1 : chassis_rotate_buff_2;

    //根据Q键设置拨盘模式
    if(rc_data[TEMP].key[KEY_PRESS].q)
    {
        DataLebel.reverse_flag=1;
    }
    else
    {
        DataLebel.reverse_flag=0;
    }
}

/**
 * @brief 用电脑操作
 *
 */
static void MouseKeySet()
{
    MouseControl();
    KeyControl();
}
/**************************************   STOP   **************************************/
/**
 * @brief 停止
 */
static void AnythingStop()
{
    gimbal_cmd_send.gimbal_mode=GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.loader_mode = LOAD_STOP;
    //重置与小电脑通信失败的标志位
    DataLebel.cmd_error_flag=0;
}

/**************************************  SetMode   **************************************/

/**
 * @brief 根据遥控器开关的不同位置，执行不同的函数
 * 它通过检查遥控器数据右开关的上、中、下位置来决定接下来的操作
 *
 */
static void ControlDataDeal()
{
    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) 
    {
        BasicSet();
        RemoteControlSet();
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) 
    {
        BasicSet();
        MouseKeySet();   
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right)) 
    {
        AnythingStop();
    }
}

/**************************************   SendData   **************************************/
static void SendToUIData()
{
    ui_data.autoaim_mode=gimbal_cmd_send.autoaim_mode;
    ui_data.chassis_mode=chassis_cmd_send.chassis_mode;
    ui_data.loader_mode=shoot_cmd_send.loader_mode;
    ui_data.chassis_power_data.cap_vol=chassis_fetch_data.vol;
}

static void JudgeEnermy()
{
    if(referee_data->GameRobotState.robot_id>7)
    {
        minipc_send_data.Vision.detect_color=COLOR_RED;
    }
    else
    {
        minipc_send_data.Vision.detect_color=COLOR_BLUE;
    }
}

static void SendPowerLimit()
{
    chassis_cmd_send.buffer_energy=referee_data->PowerHeatData.buffer_energy;
    chassis_cmd_send.power_limit=referee_data->GameRobotState.chassis_power_limit;
    chassis_cmd_send.robot_level=referee_data->GameRobotState.robot_level;
}

/*********************************************************************************************
***************************************      TASK      ***************************************
**********************************************************************************************/
void RobotCMDTask()
{
/**************************************  GetFetchData  **************************************/
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

/*************************************     Control     **************************************/
    ControlDataDeal();
 
/**************************************    SendData    **************************************/
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    SendMinipcData(&minipc_send_data);
    SendToUIData();
    SendPowerLimit();
}
