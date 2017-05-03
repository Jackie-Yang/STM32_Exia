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
#define halfT 0.005f  //half the sample period,halfT 0.5f需要根据具体姿态更新周期来调整，T是姿态更新周期，T*角速度=微分角度
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
    

	  	    
	  init_ax=(float)(MPU6050_Accel_X / 16384.0f);	   //单位转化成重力加速度的单位：m/s2
	  init_ay=(float)(MPU6050_Accel_Y / 16384.0f);
      init_az=(float)(MPU6050_Accel_Z / 16384.0f);

	  Read_HMC5883L();
 
	  init_mx =(float)HMC5883L_X;						
	  init_my =(float)HMC5883L_Y;
	  init_mz =(float)-HMC5883L_Z;

		//陀螺仪y轴为前进方向    
		init_Roll = -atan2(init_ax, init_az);    //算出的单位是弧度，如需要观察则应乘以57.3转化为角度
		init_Pitch=  asin(init_ay);              //init_Pitch = asin(ay / 1);      
		init_Yaw  =  atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
		                   init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//类似于atan2(my, mx)，其中的init_Roll和init_Pitch是弧度
		if(init_Yaw < 0){init_Yaw = init_Yaw + 2*3.141593;}
		if(init_Yaw > 360){init_Yaw = init_Yaw - 2*3.141593;}				            
		//将初始化欧拉角转换成初始化四元数，注意sin(a)的位置的不同，可以确定绕xyz轴转动是Pitch还是Roll还是Yaw，按照ZXY顺序旋转,Qzyx=Qz*Qy*Qx，其中的init_YawRollPtich是角度        
		q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
		q1 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   绕x轴旋转是pitch
		q2 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   绕y轴旋转是roll
		q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   绕z轴旋转是Yaw



//陀螺仪x轴为前进方向
//  init_Roll  = atan2(init_ay, init_az);
//  init_Pitch = -asin(init_ax);              //init_Pitch = asin(ax / 1);      
//  init_Yaw   = -atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
//                      init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));                       //atan2(mx, my);
//  q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
//  q1 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   绕x轴旋转是roll
//  q2 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   绕y轴旋转是pitch
//  q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   绕z轴旋转是Yaw
 	    
    //	printf("初始化四元数：Yaw=%f, Pitch=%f, Roll=%f \n\r", init_Yaw*57.295780, init_Pitch*57.295780, init_Roll*57.295780);
  	
}


void AHRSupdate(void) 
{
	//u32 time = system_time;

   float norm;//, halfT;
   float hx, hy, hz, bz, by;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;

   /*方便之后的程序使用，减少计算时间*/
        //auxiliary variables to reduce number of repeated operations，
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
   float Pitch_temp,Roll_temp;	//弧度值
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


          
/*归一化测量值，加速度计和磁力计的单位是什么都无所谓，因为它们在此被作了归一化处理*/        
   //normalise the measurements
   norm = invSqrt(ax*ax + ay*ay + az*az);       
   ax = ax * norm;
   ay = ay * norm;
   az = az * norm;
   norm = invSqrt(mx*mx + my*my + mz*mz);          
   mx = mx * norm;
   my = my * norm;
   mz = mz * norm;         
        
/*从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值），下面这个是从飞行器坐标系到世界坐标系的转换公式*/
   //compute reference direction of flux
   hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);

/*计算地理坐标系下的磁场矢量bxyz（参考值）。
因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），我定义by指向正北，所以by=某值，bx=0
但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
因为bx=0，所以就简化成(by*by)  = ((hx*hx) + (hy*hy))。可算出by。这里修改by和bx指向可以定义哪个轴指向正北*/
//   bx = sqrtf((hx*hx) + (hy*hy));
   by = sqrtf((hx*hx) + (hy*hy));
   bz = hz;        
    
   // estimated direction of gravity and flux (v and w)，下面这个是从世界坐标系到飞行器坐标系的转换公式(转置矩阵)
   vx = 2*(q1q3 - q0q2);
   vy = 2*(q0q1 + q2q3);
   vz = q0q0 - q1q1 - q2q2 + q3q3;

/*我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
因为bx=0，所以所有涉及到bx的部分都被省略了。同理by=0，所以所有涉及到by的部分也可以被省略，这根据自己定义那个轴指北有关。
类似上面重力vxyz的推算，因为重力g的az=1，ax=ay=0，所以上面涉及到gxgy的部分也被省略了
你可以看看两个公式：wxyz的公式，把by换成ay（0），把bz换成az（1），就变成了vxyz的公式了（其中q0q0+q1q1+q2q2+q3q3=1）。*/
//   wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//   wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//   wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
   wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
           
//现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
   // error is sum of cross product between reference direction of fields and direction measured by sensors
   ex = (ay*vz - az*vy) + (my*wz - mz*wy);
   ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
   ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
   
  // halfT=0.005f;            //GET_NOWTIME();		//得到每次姿态更新的周期的一半 (s)
   
   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //很关键的一句话，原算法没有
   {
      // integral error scaled integral gain
      exInt = exInt + ex*Ki * halfT;			   //乘以采样周期的一半
      eyInt = eyInt + ey*Ki * halfT;
      ezInt = ezInt + ez*Ki * halfT;
      // adjusted gyroscope measurements
      gx = gx + Kp*ex + exInt;
      gy = gy + Kp*ey + eyInt;
      gz = gz + Kp*ez + ezInt;
   }         

   // integrate quaternion rate and normalise，四元数更新算法
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
        
///*由四元数计算出Pitch  Roll  Yaw
//乘以57.3是为了将弧度转化为角度*/
	Yaw.angle_cur   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.3;  //偏航角，绕z轴转动	
    if(Yaw.angle_cur < 0 ){Yaw.angle_cur = Yaw.angle_cur + 360;}
	if(Yaw.angle_cur > 360 ){Yaw.angle_cur = Yaw.angle_cur - 360;}

	Pitch_temp = asin(2*q2*q3 + 2*q0*q1); //俯仰角，绕x轴转动	 
    Roll_temp  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1); //滚动角，绕y轴转动

	High.Accel_last = High.Accel_cur;
	High.Accel_cur = 9.8f * (Accel_z / sqrt(tan(Roll_temp) * tan(Roll_temp) + tan(Pitch_temp) * tan(Pitch_temp) + 1) - Accel_x * sin(Roll_temp) + Accel_y * sin(Pitch_temp) - 1);
	//invsqrt是1/sqrt()

	Pitch.angle_cur = Pitch_temp * 57.3;
	Roll.angle_cur = Roll_temp * 57.3;

/*最初的由四元数计算出Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
乘以57.3是为了将弧度转化为角度*/
	
//	Pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.3; //俯仰角，绕y轴转动	 
//    Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.3; //滚动角，绕x轴转动
//	Yaw   = atan2(2*q1*q2 + 2*q0*q3,-2*q2*q2 - 2*q3*q3 + 1) * 57.3;  //偏航角，绕z轴转动

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
//	//加速度积分（两次加速度平均，乘以重力加速度9.8，乘以采样时间0.01s）
//	High.high = High.high + (High.speed_last + High.speed_cur) / 200.0f;
//
//	DMA_Buff_In_16((int16_t)(High.speed_cur * 1000),HIGH_SPEED_INDEX);
//	DMA_Buff_In_16((int16_t)(High.high * 1000),HIGH_INDEX);
//}




//void IMUupdata(float gx, float gy, float gz, float ax, float ay, float az)  
//{  
//    float recipNorm;                //平方根的倒数  
//    float halfvx, halfvy, halfvz;           //在当前载体坐标系中，重力分量在三个轴上的分量  
//    float halfex, halfey, halfez;           //当前加速度计测得的重力加速度在三个轴上的分量与当前姿态在三个轴上的重力分量的误差,这里采用差积的方式  
//    float qa, qb, qc;  
//      
//        gx = gx * PI / 180;                             //转换为弧度制  
//        gy = gy * PI / 180;  
//        gz = gz * PI / 180;  
//      
//    //如果加速度计处于自由落体状态，可能会出现这种情况，不进行姿态解算，因为会产生分母无穷大的情况  
//    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))   
//    {  
//        //单位化加速度计,意义在于在变更了加速度计的量程之后不需要修改Kp参数,因为这里归一化了  
//        recipNorm = invSqrt(ax * ax + ay * ay + az * az);  
//        ax *= recipNorm;  
//        ay *= recipNorm;  
//        az *= recipNorm;  
//  
//        //将当前姿态的重力在三个轴上的分量分离出来  
//        //就是方向余弦旋转矩阵的第三列,注意是地理坐标系(n系)到载体坐标系(b系)的,不要弄反了.如果书上是b系到n系,转置即可  
//        //惯性测量器件测量的都是关于b系的值，为了方便，我们一般将b系转换到n系进行导航参数求解。但是这里并不需要这样做，因为这里是对陀螺仪进行补偿  
//        halfvx = g_q1 * g_q3 - g_q0 * g_q2;  
//        halfvy = g_q0 * g_q1 + g_q2 * g_q3;  
//        halfvz = g_q0 * g_q0 - 0.5f + g_q3 * g_q3;  
//      
//        //计算由当前姿态的重力在三个轴上的分量与加速度计测得的重力在三个轴上的分量的差,这里采用三维空间的差积(向量积)方法求差  
//        //计算公式由矩阵运算推导而来 公式参见http://en.wikipedia.org/wiki/Cross_product 中的Mnemonic部分  
//        halfex = (ay * halfvz - az * halfvy);  
//        halfey = (az * halfvx - ax * halfvz);  
//        halfez = (ax * halfvy - ay * halfvx);  
//  
//        //积分求误差,关于当前姿态分离出的重力分量与当前加速度计测得的重力分量的差值进行积分消除误差  
//        if(g_twoKi > 0.0f)   
//        {  
//            g_integralFBx += g_twoKi * halfex * CNTLCYCLE;      //Ki积分  
//            g_integralFBy += g_twoKi * halfey * CNTLCYCLE;  
//            g_integralFBz += g_twoKi * halfez * CNTLCYCLE;  
//            gx += g_integralFBx;        //将积分误差反馈到陀螺仪上，修正陀螺仪的值  
//            gy += g_integralFBy;  
//            gz += g_integralFBz;  
//        }  
//        else        //不进行积分运算，只进行比例调节  
//        {  
//            g_integralFBx = 0.0f;  
//            g_integralFBy = 0.0f;  
//            g_integralFBz = 0.0f;  
//        }  
//          
//        //直接应用比例调节，修正陀螺仪的值  
//        gx += g_twoKp * halfex;  
//        gy += g_twoKp * halfey;  
//        gz += g_twoKp * halfez;  
//    }  
//      
//    //以下为四元数微分方程.将陀螺仪和四元数结合起来，是姿态更新的核心算子  
//    //计算方法由矩阵运算推导而来  
//    //  .   1        
//    //  q = - * q x Omega    式中左边是四元数的倒数,右边的x是四元数乘法,Omega是陀螺仪的值(即角速度)  
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
//    //单位化四元数,意义在于单位化四元数在空间旋转时是不会拉伸的,仅有旋转角度.这类似与线性代数里面的正交变换  
//    recipNorm = invSqrt(g_q0 * g_q0 + g_q1 * g_q1 + g_q2 * g_q2 + g_q3 * g_q3);  
//    g_q0 *= recipNorm;  
//    g_q1 *= recipNorm;  
//    g_q2 *= recipNorm;  
//    g_q3 *= recipNorm;  
//      
//    //四元数到欧拉角转换，转换顺序为Z-Y-X,参见.pdf一文,P24  
//    //注意此时的转换顺序是1-2-3，即X-Y-Z。但是由于画图方便，作者这里做了一个转换，即调换Z和X，所以顺序没变  
//    g_Yaw = atan2(2 * g_q1 * g_q2 + 2 * g_q0 * g_q3, g_q1 * g_q1 + g_q0 * g_q0 - g_q3 * g_q3 - g_q2 * g_q2) * 180 / PI; //Yaw  
//    g_Roll = asin(-2 * g_q1 * g_q3 + 2 * g_q0* g_q2) * 180 / PI;                                //Roll  
//    g_Pitch = atan2(2 * g_q2 * g_q3 + 2 * g_q0 * g_q1, -2 * g_q1 * g_q1 - 2 * g_q2* g_q2 + 1) * 180 / PI;           //Pitch  
//}  


/*******************************************************************************
快速计算 1/Sqrt(x)，源自雷神3的一段代码，神奇的0x5f3759df！比正常的代码快4倍 	
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
