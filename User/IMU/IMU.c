#include "IMU.h"
#include "math.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "TIMER.h"
#include "Setup.h"
#include "PID.h"		  
#include "filter.h"


#define Kp 4.0f//2.0f     //proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005//0.03f//0.005f   //integral gain governs rate of convergence of gyroscope biases
#define halfT 0.005f  //half the sample period,halfT 0.5f��Ҫ���ݾ�����̬����������������T����̬�������ڣ�T*���ٶ�=΢�ֽǶ�
//float Yaw;
//#define q30  1073741824.0f
//float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float q0, q1, q2, q3;
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;






void init_quaternion(void)
{ 
 // unsigned long timestamp;
 // signed short int accel[3], mag[3];
  float init_Yaw, init_Pitch, init_Roll;

  READ_MPU6050_Accel();
    

	  	    
	  init_ax=(float)(MPU6050_Accel_X / 16384.0f);	   //��λת�����������ٶȵĵ�λ��m/s2
	  init_ay=(float)(MPU6050_Accel_Y / 16384.0f);
      init_az=(float)(MPU6050_Accel_Z / 16384.0f);

	  Read_HMC5883L();
 
	  init_mx =(float)HMC5883L_X;						
	  init_my =(float)HMC5883L_Y;
	  init_mz =(float)-HMC5883L_Z;

		//������y��Ϊǰ������    
		init_Roll = -atan2(init_ax, init_az);    //����ĵ�λ�ǻ��ȣ�����Ҫ�۲���Ӧ����57.3ת��Ϊ�Ƕ�
		init_Pitch=  asin(init_ay);              //init_Pitch = asin(ay / 1);      
		init_Yaw  =  atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
		                   init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//������atan2(my, mx)�����е�init_Roll��init_Pitch�ǻ���
		if(init_Yaw < 0){init_Yaw = init_Yaw + 2*3.141593;}
		if(init_Yaw > 360){init_Yaw = init_Yaw - 2*3.141593;}				            
		//����ʼ��ŷ����ת���ɳ�ʼ����Ԫ����ע��sin(a)��λ�õĲ�ͬ������ȷ����xyz��ת����Pitch����Roll����Yaw������ZXY˳����ת,Qzyx=Qz*Qy*Qx�����е�init_YawRollPtich�ǽǶ�        
		q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
		q1 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   ��x����ת��pitch
		q2 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   ��y����ת��roll
		q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   ��z����ת��Yaw



//������x��Ϊǰ������
//  init_Roll  = atan2(init_ay, init_az);
//  init_Pitch = -asin(init_ax);              //init_Pitch = asin(ax / 1);      
//  init_Yaw   = -atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
//                      init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));                       //atan2(mx, my);
//  q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
//  q1 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   ��x����ת��roll
//  q2 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   ��y����ת��pitch
//  q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   ��z����ת��Yaw
 	    
    //	printf("��ʼ����Ԫ����Yaw=%f, Pitch=%f, Roll=%f \n\r", init_Yaw*57.295780, init_Pitch*57.295780, init_Roll*57.295780);
  	
}


void AHRSupdate(void) 
{
	//u32 time = system_time;

   float norm;//, halfT;
   float hx, hy, hz, bz, by;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;

   /*����֮��ĳ���ʹ�ã����ټ���ʱ��*/
        //auxiliary variables to reduce number of repeated operations��
   float q0q0 = q0*q0;
   float q0q1 = q0*q1;
   float q0q2 = q0*q2;
   float q0q3 = q0*q3;
   float q1q1 = q1*q1;
   float q1q2 = q1*q2;
   float q1q3 = q1*q3;
   float q2q2 = q2*q2;   
   float q2q3 = q2*q3;
   float q3q3 = q3*q3;
   
   

   float gx, gy, gz, ax, ay,  az,  mx,  my,  mz;
   float Pitch_temp,Roll_temp;	//����ֵ
   float Accel_x,Accel_y,Accel_z;

    READ_MPU6050_Accel();
	READ_MPU6050_Gyro();
	Read_HMC5883L();

	ax = (float)MPU6050_Accel_X;
	Accel_x = ax / 16384.0f;
	ay = (float)MPU6050_Accel_Y;
	Accel_y = ay / 16384.0f;
	az = (float)MPU6050_Accel_Z;
	Accel_z = az / 16384.0f;


	Pitch.Gyro_cur = (float)MPU6050_Gyro_X / 131.0;
	gx = Pitch.Gyro_cur / 57.3;

	Roll.Gyro_cur = (float)MPU6050_Gyro_Y / 131.0;
	gy = Roll.Gyro_cur / 57.3;

	Yaw.Gyro_cur = (float)MPU6050_Gyro_Z / 131.0;
	gz = Yaw.Gyro_cur / 57.3;


	mx = (float)HMC5883L_X;
	my = (float)HMC5883L_Y;
	mz = -(float)HMC5883L_Z;


          
/*��һ������ֵ�����ٶȼƺʹ����Ƶĵ�λ��ʲô������ν����Ϊ�����ڴ˱����˹�һ������*/        
   //normalise the measurements
   norm = invSqrt(ax*ax + ay*ay + az*az);       
   ax = ax * norm;
   ay = ay * norm;
   az = az * norm;
   norm = invSqrt(mx*mx + my*my + mz*mz);          
   mx = mx * norm;
   my = my * norm;
   mz = mz * norm;         
        
/*�ӻ�������ϵ�ĵ������̲⵽��ʸ��ת�ɵ�������ϵ�µĴų�ʸ��hxyz������ֵ������������Ǵӷ���������ϵ����������ϵ��ת����ʽ*/
   //compute reference direction of flux
   hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);

/*�����������ϵ�µĴų�ʸ��bxyz���ο�ֵ����
��Ϊ����ش�ˮƽ�нǣ�������֪��0�ȣ���ȥ��ƫ�ǵ����أ��̶��򱱣����Ҷ���byָ������������by=ĳֵ��bx=0
������ο��ش�ʸ���ڴ�ֱ����Ҳ�з���bz��������ÿ���ط����ǲ�һ���ġ�
�����޷���֪��Ҳ���޷������ںϣ��и��ʺ�����ֱ���������ںϵļ��ٶȼƣ�������ֱ�ӴӲ���ֵhz�ϸ��ƹ�����bz=hz��
�ų�ˮƽ�������ο�ֵ�Ͳ���ֵ�Ĵ�СӦ����һ�µ�(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))��
��Ϊbx=0�����Ծͼ򻯳�(by*by)  = ((hx*hx) + (hy*hy))�������by�������޸�by��bxָ����Զ����ĸ���ָ������*/
//   bx = sqrtf((hx*hx) + (hy*hy));
   by = sqrtf((hx*hx) + (hy*hy));
   bz = hz;        
    
   // estimated direction of gravity and flux (v and w)����������Ǵ���������ϵ������������ϵ��ת����ʽ(ת�þ���)
   vx = 2*(q1q3 - q0q2);
   vy = 2*(q0q1 + q2q3);
   vz = q0q0 - q1q1 - q2q2 + q3q3;

/*���ǰѵ�������ϵ�ϵĴų�ʸ��bxyz��ת����������wxyz��
��Ϊbx=0�����������漰��bx�Ĳ��ֶ���ʡ���ˡ�ͬ��by=0�����������漰��by�Ĳ���Ҳ���Ա�ʡ�ԣ�������Լ������Ǹ���ָ���йء�
������������vxyz�����㣬��Ϊ����g��az=1��ax=ay=0�����������漰��gxgy�Ĳ���Ҳ��ʡ����
����Կ���������ʽ��wxyz�Ĺ�ʽ����by����ay��0������bz����az��1�����ͱ����vxyz�Ĺ�ʽ�ˣ�����q0q0+q1q1+q2q2+q3q3=1����*/
//   wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//   wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//   wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
   wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
           
//���ڰѼ��ٶȵĲ���ʸ���Ͳο�ʸ����������Ѵų��Ĳ���ʸ���Ͳο�ʸ��Ҳ����������������������ݡ�
   // error is sum of cross product between reference direction of fields and direction measured by sensors
   ex = (ay*vz - az*vy) + (my*wz - mz*wy);
   ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
   ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
   
  // halfT=0.005f;            //GET_NOWTIME();		//�õ�ÿ����̬���µ����ڵ�һ�� (s)
   
   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //�ܹؼ���һ�仰��ԭ�㷨û��
   {
      // integral error scaled integral gain
      exInt = exInt + ex*Ki * halfT;			   //���Բ������ڵ�һ��
      eyInt = eyInt + ey*Ki * halfT;
      ezInt = ezInt + ez*Ki * halfT;
      // adjusted gyroscope measurements
      gx = gx + Kp*ex + exInt;
      gy = gy + Kp*ey + eyInt;
      gz = gz + Kp*ez + ezInt;
   }         

   // integrate quaternion rate and normalise����Ԫ�������㷨
   q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
   q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
   q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
   q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        
   // normalise quaternion
   norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
   q0 = q0 * norm;       //w
   q1 = q1 * norm;       //x
   q2 = q2 * norm;       //y
   q3 = q3 * norm;       //z
        
///*����Ԫ�������Pitch  Roll  Yaw
//����57.3��Ϊ�˽�����ת��Ϊ�Ƕ�*/
	Yaw.angle_cur   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.3;  //ƫ���ǣ���z��ת��	
    if(Yaw.angle_cur < 0 ){Yaw.angle_cur = Yaw.angle_cur + 360;}
	if(Yaw.angle_cur > 360 ){Yaw.angle_cur = Yaw.angle_cur - 360;}

	Pitch_temp = asin(2*q2*q3 + 2*q0*q1); //�����ǣ���x��ת��	 
    Roll_temp  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1); //�����ǣ���y��ת��

	High.Accel_last = High.Accel_cur;
	High.Accel_cur = 9.8f * (Accel_z / sqrt(tan(Roll_temp) * tan(Roll_temp) + tan(Pitch_temp) * tan(Pitch_temp) + 1) - Accel_x * sin(Roll_temp) + Accel_y * sin(Pitch_temp) - 1);
	//invsqrt��1/sqrt()

	Pitch.angle_cur = Pitch_temp * 57.3;
	Roll.angle_cur = Roll_temp * 57.3;

/*���������Ԫ�������Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
����57.3��Ϊ�˽�����ת��Ϊ�Ƕ�*/
	
//	Pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.3; //�����ǣ���y��ת��	 
//    Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.3; //�����ǣ���x��ת��
//	Yaw   = atan2(2*q1*q2 + 2*q0*q3,-2*q2*q2 - 2*q3*q3 + 1) * 57.3;  //ƫ���ǣ���z��ת��

//	printf("q0=%f, q1=%f, q2=%f, q3=%f, Yaw=%f, Pitch=%f, Roll=%f, halfT=%f \n\r", q0, q1, q2, q3, Yaw, Pitch, Roll, halfT);
  //  printf("Yaw=%f, Pitch=%f, Roll=%f \n\r", Yaw, Pitch, Roll);
//	DMA_Buff_In_16((u16)(system_time-time),30);

    stQuadrotor_State.f_Yaw = Yaw.angle_cur;
    stQuadrotor_State.f_Pitch = Pitch.angle_cur;
    stQuadrotor_State.f_Roll = Roll.angle_cur;
    stQuadrotor_State.f_High_Accel = High.Accel_cur;
    stQuadrotor_State_DMA_BUFF = stQuadrotor_State;
    //DMA_Buff[TEMP3_INDEX + 2] = (u16)((int16_t)High.Accel_cur * 100);

}


//void High_cal(void)
//{
//	High.speed_last = High.speed_cur;
//	High.speed_cur = High.speed_last + (High.Accel_last + High.Accel_cur) * 9.8f / 200.0f;  
//	//���ٶȻ��֣����μ��ٶ�ƽ���������������ٶ�9.8�����Բ���ʱ��0.01s��
//	High.high = High.high + (High.speed_last + High.speed_cur) / 200.0f;
//
//	DMA_Buff_In_16((int16_t)(High.speed_cur * 1000),HIGH_SPEED_INDEX);
//	DMA_Buff_In_16((int16_t)(High.high * 1000),HIGH_INDEX);
//}




//void IMUupdata(float gx, float gy, float gz, float ax, float ay, float az)  
//{  
//    float recipNorm;                //ƽ�����ĵ���  
//    float halfvx, halfvy, halfvz;           //�ڵ�ǰ��������ϵ�У������������������ϵķ���  
//    float halfex, halfey, halfez;           //��ǰ���ٶȼƲ�õ��������ٶ����������ϵķ����뵱ǰ��̬���������ϵ��������������,������ò���ķ�ʽ  
//    float qa, qb, qc;  
//      
//        gx = gx * PI / 180;                             //ת��Ϊ������  
//        gy = gy * PI / 180;  
//        gz = gz * PI / 180;  
//      
//    //������ٶȼƴ�����������״̬�����ܻ���������������������̬���㣬��Ϊ�������ĸ���������  
//    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))   
//    {  
//        //��λ�����ٶȼ�,���������ڱ���˼��ٶȼƵ�����֮����Ҫ�޸�Kp����,��Ϊ�����һ����  
//        recipNorm = invSqrt(ax * ax + ay * ay + az * az);  
//        ax *= recipNorm;  
//        ay *= recipNorm;  
//        az *= recipNorm;  
//  
//        //����ǰ��̬���������������ϵķ����������  
//        //���Ƿ���������ת����ĵ�����,ע���ǵ�������ϵ(nϵ)����������ϵ(bϵ)��,��ҪŪ����.���������bϵ��nϵ,ת�ü���  
//        //���Բ������������Ķ��ǹ���bϵ��ֵ��Ϊ�˷��㣬����һ�㽫bϵת����nϵ���е���������⡣�������ﲢ����Ҫ����������Ϊ�����Ƕ������ǽ��в���  
//        halfvx = g_q1 * g_q3 - g_q0 * g_q2;  
//        halfvy = g_q0 * g_q1 + g_q2 * g_q3;  
//        halfvz = g_q0 * g_q0 - 0.5f + g_q3 * g_q3;  
//      
//        //�����ɵ�ǰ��̬���������������ϵķ�������ٶȼƲ�õ��������������ϵķ����Ĳ�,���������ά�ռ�Ĳ��(������)�������  
//        //���㹫ʽ�ɾ��������Ƶ����� ��ʽ�μ�http://en.wikipedia.org/wiki/Cross_product �е�Mnemonic����  
//        halfex = (ay * halfvz - az * halfvy);  
//        halfey = (az * halfvx - ax * halfvz);  
//        halfez = (ax * halfvy - ay * halfvx);  
//  
//        //���������,���ڵ�ǰ��̬����������������뵱ǰ���ٶȼƲ�õ����������Ĳ�ֵ���л����������  
//        if(g_twoKi > 0.0f)   
//        {  
//            g_integralFBx += g_twoKi * halfex * CNTLCYCLE;      //Ki����  
//            g_integralFBy += g_twoKi * halfey * CNTLCYCLE;  
//            g_integralFBz += g_twoKi * halfez * CNTLCYCLE;  
//            gx += g_integralFBx;        //�������������������ϣ����������ǵ�ֵ  
//            gy += g_integralFBy;  
//            gz += g_integralFBz;  
//        }  
//        else        //�����л������㣬ֻ���б�������  
//        {  
//            g_integralFBx = 0.0f;  
//            g_integralFBy = 0.0f;  
//            g_integralFBz = 0.0f;  
//        }  
//          
//        //ֱ��Ӧ�ñ������ڣ����������ǵ�ֵ  
//        gx += g_twoKp * halfex;  
//        gy += g_twoKp * halfey;  
//        gz += g_twoKp * halfez;  
//    }  
//      
//    //����Ϊ��Ԫ��΢�ַ���.�������Ǻ���Ԫ���������������̬���µĺ�������  
//    //���㷽���ɾ��������Ƶ�����  
//    //  .   1        
//    //  q = - * q x Omega    ʽ���������Ԫ���ĵ���,�ұߵ�x����Ԫ���˷�,Omega�������ǵ�ֵ(�����ٶ�)  
//    //      2  
//    //   .  
//    //  [q0]    [0  -wx -wy -wz]    [q0]  
//    //   .                
//    //  [q1]    [wx   0 wz      -wy]    [q1]  
//    //   .   =                   *   
//    //  [q2]    [wy  -wz     0  wx ]    [q2]  
//    //   .            
//    //  [q3]    [wz      wy -wx 0  ]    [q3]  
//    gx *= (0.5f * CNTLCYCLE);  
//    gy *= (0.5f * CNTLCYCLE);  
//    gz *= (0.5f * CNTLCYCLE);  
//    qa = g_q0;  
//    qb = g_q1;  
//    qc = g_q2;  
//    g_q0 += (-qb * gx - qc * gy - g_q3 * gz);  
//    g_q1 += ( qa * gx + qc * gz - g_q3 * gy);  
//    g_q2 += ( qa * gy - qb * gz + g_q3 * gx);  
//    g_q3 += ( qa * gz + qb * gy -   qc * gx);  
//      
//    //��λ����Ԫ��,�������ڵ�λ����Ԫ���ڿռ���תʱ�ǲ��������,������ת�Ƕ�.�����������Դ�������������任  
//    recipNorm = invSqrt(g_q0 * g_q0 + g_q1 * g_q1 + g_q2 * g_q2 + g_q3 * g_q3);  
//    g_q0 *= recipNorm;  
//    g_q1 *= recipNorm;  
//    g_q2 *= recipNorm;  
//    g_q3 *= recipNorm;  
//      
//    //��Ԫ����ŷ����ת����ת��˳��ΪZ-Y-X,�μ�.pdfһ��,P24  
//    //ע���ʱ��ת��˳����1-2-3����X-Y-Z���������ڻ�ͼ���㣬������������һ��ת����������Z��X������˳��û��  
//    g_Yaw = atan2(2 * g_q1 * g_q2 + 2 * g_q0 * g_q3, g_q1 * g_q1 + g_q0 * g_q0 - g_q3 * g_q3 - g_q2 * g_q2) * 180 / PI; //Yaw  
//    g_Roll = asin(-2 * g_q1 * g_q3 + 2 * g_q0* g_q2) * 180 / PI;                                //Roll  
//    g_Pitch = atan2(2 * g_q2 * g_q3 + 2 * g_q0 * g_q1, -2 * g_q1 * g_q1 - 2 * g_q2* g_q2 + 1) * 180 / PI;           //Pitch  
//}  


/*******************************************************************************
���ټ��� 1/Sqrt(x)��Դ������3��һ�δ��룬�����0x5f3759df���������Ĵ����4�� 	
*******************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
