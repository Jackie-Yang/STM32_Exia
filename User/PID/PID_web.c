#include "CONTROL.h"
#include "IMU1.h"       
#include "moto.h"
#include "RFdate.h"
#include <math.h>
extern T_RC_Data                         Rc_D;                //遥控通道数据;

extern u8 txbuf[4];         //发送缓冲
extern u8 rxbuf[4];         //接收缓冲
extern u16 test1[3]; //接收到NRf24L01数据
extern S_INT16_XYZ ACC_F,GYRO_F;

PID PID_ROL,PID_PIT,PID_YAW;

extern S_INT16_XYZ        MPU6050_ACC_LAST,MPU6050_GYRO_LAST;       


int Motor_Ele=0;                                           //俯仰期望
int Motor_Ail=0;                                           //横滚期望

//u8 ARMED = 0;

//float rol_i=0,pit_i=0,yaw_p=0;
float thr=0;

S_FLOAT_XYZ EXP_ANGLE ,DIF_ANGLE;
PID1 PID_Motor;
/*********************************/
float Pitch_i,Roll_i,Yaw_i;                                   //积分项
float Pitch_old,Roll_old,Yaw_old;                 //角度保存
float Pitch_d,Roll_d,Yaw_d;          //微分项
float RC_Pitch,RC_Roll,RC_Yaw;                       //姿态角
float Pitch_shell_out,Roll_shell_out,Yaw_shell_out;//外环总输出
        //外环PID参数
float Pitch_shell_kp=280;//30 140
float Pitch_shell_kd=0;//
float Pitch_shell_ki=0;//
/*********************************/
float Roll_shell_kp=250;//30		d
float Roll_shell_kd=0;//10                 
float Roll_shell_ki=0;//0.08
/*********************************/
float Yaw_shell_kp=1.5;//10;//30
float Yaw_shell_kd=0;//10                 
float Yaw_shell_ki=0;//0.08;//0.08
float Gyro_radian_old_x,Gyro_radian_old_y,Gyro_radian_old_z;//陀螺仪保存
float pitch_core_kp_out,pitch_core_kd_out,Roll_core_kp_out,Roll_core_kd_out,Yaw_core_kp_out,Yaw_core_kd_out;//内环单项输出
float Pitch_core_out,Roll_core_out,Yaw_core_out;//内环总输出       
       
//内环PID参数
//float Pitch_core_kp=0.040;
//float Pitch_core_kd=0.008;////0.007;//0.07;
float Pitch_core_kp=0.040;
float Pitch_core_kd=0.002;////0.007;//0.07;

float Roll_core_kp=0.040;//;
float Roll_core_kd=0.002;////0.007;//06;//0.07;

float Yaw_core_kp=0.046;//;
float Yaw_core_kd=0.012;////0.007;//06;//0.07;


int16_t moto1=0,moto2=0,moto3=0,moto4=0;

float tempjd=0;
void CONTROL(float rol, float pit, float yaw)
{
       
         RC_Pitch=(Rc_D.PITCH-1500)/20;
       
        ////////////////////////外环角度环(PID)///////////////////////////////
  Pitch_i+=(Q_ANGLE.Pitch-RC_Pitch);
//------Pitch  PID-------------------//
  Pitch_shell_out = Pitch_shell_kp*(Q_ANGLE.Pitch-RC_Pitch) + Pitch_shell_ki*Pitch_i + Pitch_shell_kd*Pitch_d;
//角度保存
  Pitch_old=Q_ANGLE.Pitch;
/*********************************************************/       
       
        RC_Roll=(Rc_D.ROLL-1500)/20;
        Roll_i+=(Q_ANGLE.Rool-RC_Roll);
//-------------Roll积分限幅----------------//
  if(Roll_i>300) Roll_i=300;
  else if(Roll_i<-300) Roll_i=-300;
//-------------Roll微分--------------------//
  Roll_d=Q_ANGLE.Rool-Roll_old;
//-------------Roll  PID-------------------//
  Roll_shell_out  = Roll_shell_kp*(Q_ANGLE.Rool-RC_Roll) + Roll_shell_ki*Roll_i + Roll_shell_kd*Roll_d;
//------------Roll角度保存------------------//
  Roll_old=Q_ANGLE.Rool;
       
       
        RC_Yaw=(Rc_D.YAW-1500)*10;
//-------------Yaw微分--------------------//
  Yaw_d=MPU6050_GYRO_LAST.Z-Yaw_old;
//-------------Yaw PID-------------------//
  Yaw_shell_out  = Yaw_shell_kp*(MPU6050_GYRO_LAST.Z-RC_Yaw) + Yaw_shell_ki*Yaw_i + Yaw_shell_kd*Yaw_d;
//------------Yaw角度保存------------------//
  Yaw_old=MPU6050_GYRO_LAST.Z;
       
       
        ////////////////////////内环角速度环(PD)///////////////////////////////       
  pitch_core_kp_out = Pitch_core_kp * (Pitch_shell_out + MPU6050_GYRO_LAST.Y * 3.5);
  pitch_core_kd_out = Pitch_core_kd * (MPU6050_GYRO_LAST.Y   - Gyro_radian_old_y);

  Roll_core_kp_out  = Roll_core_kp  * (Roll_shell_out  + MPU6050_GYRO_LAST.X *3.5);
  Roll_core_kd_out  = Roll_core_kd  * (MPU6050_GYRO_LAST.X   - Gyro_radian_old_x);

  Yaw_core_kp_out  = Yaw_core_kp  * (Yaw_shell_out  + MPU6050_GYRO_LAST.Z * 1);
  Yaw_core_kd_out  = Yaw_core_kd  * (MPU6050_GYRO_LAST.Z   - Gyro_radian_old_z);
       
       
        Pitch_core_out = pitch_core_kp_out + pitch_core_kd_out;
  Roll_core_out  = Roll_core_kp_out  + Roll_core_kd_out;
  Yaw_core_out   = Yaw_core_kp_out   + Yaw_core_kd_out;

  Gyro_radian_old_y = MPU6050_GYRO_LAST.X;
  Gyro_radian_old_x = MPU6050_GYRO_LAST.Y;
  Gyro_radian_old_z = MPU6050_GYRO_LAST.Z;   //储存历史值
       
//--------------------将输出值融合到四个电机--------------------------------//

       
        if(Rc_D.THROTTLE>1020)
        {
  thr=Rc_D.THROTTLE- 1000;

//                if(Rc_D.THROTTLE<=2000)
//                {
//  moto1=(int16_t)(thr  - Pitch_core_out);//- yaw);
//        moto2=(int16_t)(thr  - Pitch_core_out);//+ yaw);       
//        moto3=(int16_t)(thr  + Pitch_core_out);// - yaw);
//        moto4=(int16_t)(thr  + Pitch_core_out);//+ yaw);       
   
//  moto1=(int16_t)(thr  - Roll_core_out);//- yaw);
//        moto2=(int16_t)(thr  + Roll_core_out);//+ yaw);       
//        moto3=(int16_t)(thr  + Roll_core_out);// - yaw);
//        moto4=(int16_t)(thr  - Roll_core_out);//+ yaw);

//  moto1=(int16_t)(thr  - Yaw_core_out);//- yaw);
//        moto2=(int16_t)(thr  + Yaw_core_out);//+ yaw);       
//        moto3=(int16_t)(thr  - Yaw_core_out);// - yaw);
//        moto4=(int16_t)(thr  + Yaw_core_out);//+ yaw);                       
                       
//moto1=(int16_t)(thr - Roll_core_out - Pitch_core_out);
//moto2=(int16_t)(thr + Roll_core_out - Pitch_core_out);       
//moto3=(int16_t)(thr + Roll_core_out + Pitch_core_out);
//moto4=(int16_t)(thr - Roll_core_out + Pitch_core_out);       
//                       
  moto1=(int16_t)(thr - Roll_core_out - Pitch_core_out- Yaw_core_out);
        moto2=(int16_t)(thr + Roll_core_out - Pitch_core_out+ Yaw_core_out);       
        moto3=(int16_t)(thr + Roll_core_out + Pitch_core_out- Yaw_core_out);
        moto4=(int16_t)(thr - Roll_core_out + Pitch_core_out+ Yaw_core_out);                       
                       
//                }
  }
        else
        {
                moto1 = 0;
                moto2 = 0;
                moto3 = 0;
                moto4 = 0;
        }
        MOTO_PWMRFLASH(moto1,moto2,moto3,moto4);//        Moto_PwmRflash(moto1,moto2,moto3,moto4);
}









        void GET_EXPRAD(void)                        //?????úí????è,2??ó????ê±?úí????è?a0,0
        {
                EXP_ANGLE.X = (float)(-(Rc_Get.ROLL-1500)/30.0f);      
                EXP_ANGLE.Y = (float)(-(Rc_Get.PITCH-1500)/30.0f);
                EXP_ANGLE.Z = (float)(Rc_Get.YAW);
        //        printf("%f %f\n",MPU6050_ACC_LAST.Y*cos(Q_ANGLE.X/57.3)-MPU6050_ACC_LAST.Z*sin(Q_ANGLE.X/57.3),MPU6050_ACC_LAST.X*cos(-Q_ANGLE.Y/57.3)-MPU6050_ACC_LAST.Z*sin(-Q_ANGLE.Y/57.3));
        //        DIF_ANGLE.X = (ACC_AVG.Y*cos(Q_ANGLE.X/57.3)-ACC_AVG.Z*sin(Q_ANGLE.X/57.3))/500;
        //        DIF_ANGLE.Y = (ACC_AVG.X*cos(-Q_ANGLE.Y/57.3)-ACC_AVG.Z*sin(-Q_ANGLE.Y/57.3)/500);
          DIF_ANGLE.X = EXP_ANGLE.X - Q_ANGLE.X;
          DIF_ANGLE.Y = EXP_ANGLE.Y - Q_ANGLE.Y;
        //        DIF_ANGLE.Z = EXP_ANGLE.Z - GYRO_I[0].Z;
        //        DIF_ANGLE.X = EXP_ANGLE.X - GYRO_I[0].X;
        //        DIF_ANGLE.Y = EXP_ANGLE.Y - GYRO_I[0].Y;
        //        DIF_ANGLE.Z = EXP_ANGLE.Z - GYRO_I[0].Z;
        }







        void CONTROL(void)
        {
                static float thr=0,rool=0,pitch=0,yaw=0;
                static float rool_i=0,pitch_i=0;
                static float rool_dif=0,pitch_dif=0;
                static float rool_speed_dif=0,pitch_speed_dif=0;
                float rool_out,pitch_out;
                uint16_t THROTTLE;

                GET_EXPRAD();
               
                rool         = PID_ROL.P * DIF_ANGLE.X;      
                rool_i += PID_ROL.I * DIF_ANGLE.X * 0.002;
                rool_i = between(rool_i,30,-30);
                rool += rool_i;
                rool += PID_ROL.D * (DIF_ANGLE.X-rool_dif) * 500;
                rool_dif = DIF_ANGLE.X;
          ///////////      
                pitch         = +PID_ROL.P * DIF_ANGLE.Y;      
                pitch_i += PID_ROL.I * DIF_ANGLE.Y * 0.002;
                pitch_i = between(pitch,30,-30);
                pitch += pitch_i;               
                pitch += PID_ROL.D * (DIF_ANGLE.Y-pitch_dif) * 500;
                pitch_dif = DIF_ANGLE.Y;
                ///////////
        //      
                rool -= GYRO_AVG.X* Gyro_G;
                rool_out=PID_PIT.P*rool;
                rool_out += PID_PIT.D*(rool- rool_speed_dif)*500;
                rool_speed_dif = rool;
               
                pitch -= GYRO_AVG.Y* Gyro_G;
                pitch_out=PID_PIT.P*pitch;
                pitch_out += PID_PIT.D*(pitch- pitch_speed_dif)*500;
                pitch_speed_dif=pitch;

        //        rool=PID_PIT.I*(rool-GYRO_AVG.X* Gyro_G);
        //        pitch=PID_PIT.I*(pitch-GYRO_AVG.Y* Gyro_G);
               
        //        PID_YAW.dout = 20 * (MPU6050_GYRO_LAST.Z* Gyro_G-(Rc_Get.YAW-1500)/10);
                PID_YAW.dout = 10 * (GYRO_AVG.Z* Gyro_G-(Rc_Get.YAW-1500)/10);

                PID_ROL.OUT = rool_out;
                PID_PIT.OUT = pitch_out;
                PID_YAW.OUT = PID_YAW.dout;
               
        /////////////
        //        GYRO_I[0].Z += EXP_ANGLE.Z/3000;
        //        yaw = -10 * GYRO_I[0].Z;
        //      
        //        yaw -= 3 * GYRO_F.Z;      
                THROTTLE=Rc_Get.THROTTLE;
                if(THROTTLE>1050)
                {
        //                if(THROTTLE>1950)
        //                {
        //                        THROTTLE=1950;
        //                }
                        THROTTLE = THROTTLE/cos(Q_ANGLE.X/57.3)/cos(Q_ANGLE.Y/57.3);      
                      
                        moto1 = THROTTLE - 1000 + (int16_t)PID_ROL.OUT - (int16_t)PID_PIT.OUT - (int16_t)PID_YAW.OUT;
                        moto2 = THROTTLE - 1000 + (int16_t)PID_ROL.OUT + (int16_t)PID_PIT.OUT + (int16_t)PID_YAW.OUT;
                        moto3 = THROTTLE - 1000 - (int16_t)PID_ROL.OUT + (int16_t)PID_PIT.OUT - (int16_t)PID_YAW.OUT;
                        moto4 = THROTTLE - 1000 - (int16_t)PID_ROL.OUT - (int16_t)PID_PIT.OUT + (int16_t)PID_YAW.OUT;
                }
                else
                {
                        moto1 = 0;
                        moto2 = 0;
                        moto3 = 0;
                        moto4 = 0;
                }
                if(Q_ANGLE.X>45||Q_ANGLE.Y>45||Q_ANGLE.X<-45||Q_ANGLE.Y<-45)
                {
                        ARMED=0;
                        LED3_OFF;
                }
        //        printf("moto=%d %d %d %d\n",moto1,moto2,moto3,moto4);
                if(ARMED)        MOTO_PWMRFLASH(moto1,moto2,moto3,moto4);
                else                        MOTO_PWMRFLASH(0,0,0,0);
        }
