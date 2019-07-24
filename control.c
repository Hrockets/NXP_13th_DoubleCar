#include "control.h"
#include "NRF2401.H"
#include "Wireless.h"
#include "headfile.h"

//变量定义
uint8_t chose_dajiaoline = 0;//打角行
uint8_t huan_line[8];//环岛打角行
uint8_t huancount=0;//环岛计数
uint8_t ruhuan_line_add;//入环增加的打角行
uint8_t dajiao_line=0;//设定的基础打角行
uint8_t podao_flag=0;//坡道标志
int dianboGeLi_count=0;//颠簸隔离
uint8_t dianbo_dajiaofield=0;//颠簸一段时间限制打角场数设定
uint8_t huan_ru_p[8];//入环p设定
uint8_t huan_chu_p[8];//出环p
uint8_t huan_in_p[8];//环内p
double chose_p=0, chose_d=0;//舵机pd
float Long_Straight_P = 0.3, Long_Straight_D = 0.2;//长直道pd
float Short_Straight_P = 0.6, Short_Straight_D = 0.5;//短直道pd
float SS_P = 0.6, SS_D = 0.7;//小Spd
float Curve_P = 0.9, Curve_D = 1.9;//弯道pd
float Ramp_P = 0.6, Ramp_D = 0.4;//坡道pd	
uint8_t Ahuiche_mend_p=0, Ahuiche_recover_p=0,Ahuiche_mend_d=0, Ahuiche_recover_d=0;//起始会车区补线,恢复的p d
uint8_t Bhuiche_mend_p=0, Bhuiche_recover_p=0,Bhuiche_mend_d=0, Bhuiche_recover_d=0;//中间会车区补线,恢复的p d
uint8_t my_Element = 0, Last_my_Element = 0;//赛道类型
uint8_t die_dangche_Flag=0;//放弃会车 直接冲出去
uint8_t Realbegin_Count=0;//放弃会车 这个场数之后冲出去
int angleChange = 0;//舵机值变化的角度
int image_speedset = 0;//根据不同元素给出的设定速度
int speed_longstraight = 450;//长直道速度
uint16_t speed_shortstraight = 420;
uint16_t speed_SS = 420, speed_ramp = 300, speed_normal = 360;//小S、坡道、弯道 速度
uint16_t motor_str_I = 12,motor_str_P  = 15;//电机直道pi
uint16_t motor_cur_I = 10,motor_cur_P = 10;//电机弯道pi
uint8_t RW_flag = 0;//这次入弯减速是否有效
uint8_t Cut_Chang=0;//减速剩余场数
uint8_t Cut_finishFlag = 0;//减速完成标志位
uint8_t xiaopo_speed=0;//缓坡速度
uint16_t huan_size[8];//环岛加速
uint8_t CarFlag=2;//会车时前后车 0-前车 1-后车
int MiddleStop_Init=0;//中间会车几场停下来
int huichecount=0;//中间会车停车场数记录
int after_huiche_count=0;//会车之后记场
int EndStop_Init=0;//最后会车几场停下来
int tingchecount=0;//最后会车停车场数记录
uint8_t have_stop=0;//车已经停下来了
int stop_count=0;//最后一次会车完成之后准备停车记场
uint8_t RealStop_Count=0;//最后会车完成之后几场停下来 设定
uint8_t longtime_huiche_Flag=0;//双车模式 一辆车失败保成绩开关
uint8_t car1_lowspeed=0;//会车后车减速的大小
uint8_t motor_stop=0;//停车标志位
uint8_t dianbofield=0;//颠簸之后占空比输出限幅场数设定
int dianbospeed=0;//颠簸设定的占空比
int init_dianji_go=0;//电机的起始占空比
uint8_t guancha=0;//会车启动时直冲 该变量用于检测前车是否已停下
uint8_t my_Curve_Flag = 0;//确定赛道类型时使用的弯道标记
//会车相关变量
uint8_t huicheNum=0;
uint8_t begin_stop=0;
int Mid_huiche_Count=0;
int Menu_Mid_huiche_Count=0;
uint8_t tongshi_huiche=0;
uint8_t test=0;
int houche_huiche_count=0;
int huiche_Menu_uwb=0;
extern int last_distance[5];
int houche_huiche_Menu_count=0;
int huicheyanshi_count=0;
uint8_t huicheyanshi=0;
uint8_t last_huicheyanshi=0;
uint8_t Stop_Line_Huiche=0,huichejuli=0,huiche=0,xueruo=0,send_fail_count=0,send_fail_flag=0;
uint8_t houchexueruo=0;
int  car0_stop_count=0;
uint8_t hou_send=0;
int End_huiche_count=0;
int Menu_End_huiche_Count=0;
int after_lasthuiche_count=0;

//引用变量
//编码器
extern int Add_Pulse;//编码器转速和
//颠簸
extern uint8_t qi;//颠簸起
extern uint8_t dianbo ;//颠簸标志
extern int dianboGeLi;//颠簸隔离
extern uint8_t sure_huan_after_dianbo;//确认颠簸之后的环岛
extern uint8_t dianbo_zhichong;//颠簸直冲标志位
extern uint8_t dian_pan_huan;
//坡道
extern int podaojichang;//坡道记场
extern int PanPoJishu;//坡道数量
extern uint8_t dian_pan_huan;//颠簸稳定状态
//图像
extern uint8_t RowNum,L_black[70],R_black[70],LCenter[70];//左右线
extern uint8_t L_init ,R_init ,R_init_record,L_init_record;//左右起始行
extern uint8_t Last_Line;//截断行
extern uint8_t mend_by_right;//疑似环岛借助正确线拉线
extern double Judge_Slope;//小草法斜率
extern uint8_t Judge_Memory;//小草法有效长度
extern uint8_t fixvalue[70];//正常赛道宽度
extern int CNT;//帧数记录
extern uint8_t Cross_Xie_Out_i, Cross_Zhong_Out_i, Cross_Zhi_Out_i;//十字标记
extern uint8_t xiaopo_jiansu;//缓坡
extern uint8_t small_s;//小S
extern uint8_t SS_needCut;//小S需要减速
extern uint8_t small_sCanGo;//小S可以开始标志
//控制
extern int image_error, image_derror, image_lasterror;//偏差
extern double MotorFuzzyLP, MotorFuzzyLD, MotorFuzzyRP, MotorFuzzyRD;//计算得出的电机pd
//会车区
extern uint8_t mend_by_xuxian;//会车区补线
extern uint8_t mend_recover;//会车处于恢复阶段
extern uint8_t xuxianjilu;//第几次判断到会车区 中间会车是1
extern uint8_t podaoF;//坡道开关
extern uint8_t dianboF;//颠簸开关
extern uint8_t huanline_second;
extern  uint8_t youDaSi;
uint8_t huichequF = 0;//会车区检测开关
uint8_t huichejuli_cut=0;
int zhidao_huiche_count=0; 
uint8_t last_CarFlag=2;
extern uint8_t Judge_Start;//起始丢线
extern uint8_t SS_dajiao_line;
extern uint8_t Car;
//一段距离减速
//变量
//一段距离减速
//菜单
uint8_t AI_Element[8]={1,2,1,2,0,0,0,0};//1代表坡道 2代表颠簸 3会车区 4单纯减速 0关闭

uint8_t AI_Element_set[8] = {0,0,0,0,0,0,0,0};//人工智能是在那种元素之后记录的 0-环岛 1-坡道 2-会车区
uint8_t AI_num_set[8] = {0,0,0,0,0,0,0,0};//人工智能的元素数量设定
uint8_t cutCM_In[8] = {2,1,7,0,0}, cutCM_Out[8] = {4,4,4,0,0};//减速的路程

uint8_t podaoF_set = 0, dianboF_set = 0, huichequF_set = 0;//大开关 覆盖所有AI参数

uint8_t cutIsland_set[8] = {10,10,10,10,10,10,10,10}, cutRamp_set[8] = {10,10,10,10,10,10,10,10}, cutDashed_set[8] = {10,10,10,10,10,10,10,10};
uint8_t AI_CutCnt = 0;//第几个环岛

uint8_t AI_Use[8] = {0,0,0,0,0,0,0,0};//代表是否已经使用过这次人工智能减速

//uint16_t AI_SpeedSet[5] = {100,100,100,10,10};//减速的速度
uint64_t Island_distance[9] = {0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000};//每个环岛所在的距离
uint64_t Ramp_distance[9] = {0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000};//每个坡道所在的距离
uint64_t DashedField_distance = 100000;//虚线的距离
//状态
uint8_t AI_State = 0;//0-正常 1-减速限制中
uint8_t last_huancount = 0;//记录上一次环岛数量 判断是不是有环岛数量的变化
uint8_t last_PanPoJishu = 0;//记录上一次坡道数量
uint8_t last_dianbo = 0;
int64_t encoderCNT = 0;//编码器计数和
uint16_t encoderCM = 0;//实际换算出来的米数（厘米）
uint16_t AI_Speed = 100;//减速的速度
uint8_t AI_RecoveryCnt = 0;//记场恢复 10场恢复到正常
uint8_t AI_Distance_By = 0;//0-环岛 1-坡道

uint8_t Cut_AfterSS = 0, Cut_AfterIsland = 0;//小S、环岛 之后减速
uint8_t CurveFirst_ByIsland = 0;//环岛直入弯标记

uint8_t wandao_huiche_flag=0;

//函数
extern int Mabs(int a,int b);//取绝对值最大的函数
int my_My_Abs1(int x)
{
	if(x >= 0)
		return x;
	else
		return -x;
}
uint8_t my_My_Abs(int x)
{
		if (x < 0)
				return (uint8_t)(-x);
		else
				return (uint8_t)x;
}
int My_Max(int x, int y)
{
		if (x > y)
				return x;
		else
				return y;
}

//舵机控制部分
void Get_AngleLine()//确定打角行
{
	//先确定环岛的基础打角行
	if((zuohuan == 2 || youhuan == 2)&&huanline_second>0)//入环
	{
		chose_dajiaoline = huan_line[huancount] - ruhuan_line_add;
	}
	else if(zuohuan == 2 || youhuan == 2||zuohuan == 6 || youhuan == 6 || zuohuan == 3 || youhuan == 3 || youhuan == 5 || zuohuan == 5)//环内和出环
	{
		chose_dajiaoline=huan_line[huancount];
	}
	//处理环岛动态打角行
	if(zuohuan == 2 || zuohuan == 3 || zuohuan == 5 || zuohuan == 6
	|| youhuan == 2 || youhuan == 3 || youhuan == 5 || youhuan == 6)//根据当前速度确定打角行
	{
		if(Add_Pulse<600){chose_dajiaoline+=0;}
		else if(Add_Pulse<620){chose_dajiaoline+=0;}
		else if(Add_Pulse<640){chose_dajiaoline+=1;}
		else if(Add_Pulse<660){chose_dajiaoline+=1;}
		else if(Add_Pulse<680){chose_dajiaoline+=2;}
		else if(Add_Pulse<700){chose_dajiaoline+=2;}
		else if(Add_Pulse<720){chose_dajiaoline+=3;}
		else if(Add_Pulse<740){chose_dajiaoline+=3;}
		else if(Add_Pulse<760){chose_dajiaoline+=4;}
		else if(Add_Pulse<780){chose_dajiaoline+=4;}
		else if(Add_Pulse<800){chose_dajiaoline+=5;}
		else if(Add_Pulse<820){chose_dajiaoline+=5;}
		else if(Add_Pulse<840){chose_dajiaoline+=6;}
		else if(Add_Pulse<860){chose_dajiaoline+=6;}
		else if(Add_Pulse<880){chose_dajiaoline+=7;}
		else if(Add_Pulse<900){chose_dajiaoline+=7;}
		else if(Add_Pulse<920){chose_dajiaoline+=8;}
		else if(Add_Pulse<940){chose_dajiaoline+=8;}
		else if(Add_Pulse<960){chose_dajiaoline+=9;}
		else if(Add_Pulse<980){chose_dajiaoline+=10;}
		else if(Add_Pulse<1000){chose_dajiaoline+=11;}
		else if(Add_Pulse<1020){chose_dajiaoline+=11;}
		else if(Add_Pulse<1040){chose_dajiaoline+=12;}
		else if(Add_Pulse<1060){chose_dajiaoline+=13;}
		else if(Add_Pulse<1080){chose_dajiaoline+=14;}
		else if(Add_Pulse<1100){chose_dajiaoline+=15;}
		else if(Add_Pulse<1120){chose_dajiaoline+=16;}
		else if(Add_Pulse<1140){chose_dajiaoline+=17;}
		else if(Add_Pulse<1160){chose_dajiaoline+=18;}
		else if(Add_Pulse<1180){chose_dajiaoline+=18;}
		else if(Add_Pulse<1200){chose_dajiaoline+=19;}
		else {chose_dajiaoline+=20;}
	}
//	else if(SS_dajiao_line<70&&my_Element == SS)
//	chose_dajiaoline =SS_dajiao_line;
	else //环岛之外的其他情况
	{
		//根据两轮和速度采用动态打脚行
		if(qi == 0 || (L_black[0]-R_black[0] > 150) || L_init >= 20 || R_init >= 20)//不是颠簸 或者颠簸使头已经低下来 或者左右起始高于20行
		{
			if(Add_Pulse<600){chose_dajiaoline=dajiao_line;}
			else if(Add_Pulse<620){chose_dajiaoline=dajiao_line;}
			else if(Add_Pulse<640){chose_dajiaoline=dajiao_line+1;}
			else if(Add_Pulse<660){chose_dajiaoline=dajiao_line+1;}
			else if(Add_Pulse<680){chose_dajiaoline=dajiao_line+2;}
			else if(Add_Pulse<700){chose_dajiaoline=dajiao_line+2;}
			else if(Add_Pulse<720){chose_dajiaoline=dajiao_line+3;}
			else if(Add_Pulse<740){chose_dajiaoline=dajiao_line+3;}
			else if(Add_Pulse<760){chose_dajiaoline=dajiao_line+4;}
			else if(Add_Pulse<780){chose_dajiaoline=dajiao_line+4;}
			else if(Add_Pulse<800){chose_dajiaoline=dajiao_line+5;}
			else if(Add_Pulse<820){chose_dajiaoline=dajiao_line+5;}
			else if(Add_Pulse<840){chose_dajiaoline=dajiao_line+6;}
			else if(Add_Pulse<860){chose_dajiaoline=dajiao_line+6;}
			else if(Add_Pulse<880){chose_dajiaoline=dajiao_line+7;}
			else if(Add_Pulse<900){chose_dajiaoline=dajiao_line+7;}
			else if(Add_Pulse<920){chose_dajiaoline=dajiao_line+8;}
			else if(Add_Pulse<940){chose_dajiaoline=dajiao_line+8;}
			else if(Add_Pulse<960){chose_dajiaoline=dajiao_line+9;}
			else if(Add_Pulse<980){chose_dajiaoline=dajiao_line+10;}
			else if(Add_Pulse<1000){chose_dajiaoline=dajiao_line+11;}
			else if(Add_Pulse<1020){chose_dajiaoline=dajiao_line+11;}
			else if(Add_Pulse<1040){chose_dajiaoline=dajiao_line+12;}
			else if(Add_Pulse<1060){chose_dajiaoline=dajiao_line+13;}
			else if(Add_Pulse<1080){chose_dajiaoline=dajiao_line+14;}
			else if(Add_Pulse<1100){chose_dajiaoline=dajiao_line+15;}
			else if(Add_Pulse<1120){chose_dajiaoline=dajiao_line+16;}
			else if(Add_Pulse<1140){chose_dajiaoline=dajiao_line+17;}
			else if(Add_Pulse<1160){chose_dajiaoline=dajiao_line+18;}
			else if(Add_Pulse<1180){chose_dajiaoline=dajiao_line+18;}
			else if(Add_Pulse<1200){chose_dajiaoline=dajiao_line+19;}
			else {chose_dajiaoline = dajiao_line+20;}
		}	
    else 		
		chose_dajiaoline = dajiao_line;	//其他情况打角行就用设定的打角行
//	 if(after_huiche_count<=50&&after_huiche_count>0)
//	 {
//		 chose_dajiaoline += Judge_Start;
//	 }
//	 else 
		if(L_init>0&&R_init>0&&podao_flag==0&&dianbo==0)//如果第0行左右全白 太高打角行
		chose_dajiaoline +=2;
		if(podao_flag==2)//坡道特殊处理打角行
		{
			if(Last_Line>=5)//如过截止行小于5 则在下面求偏差时 使偏差置零
			{
				uint8_t x=0,i=0,y=0;
				for(x=(uint8_t)(Last_Line-2);x>0;x--)
				{
						for (i = x; i > 0; i--)
								if (Pixels[i][ R_black[x]] == 0 || Pixels[i][L_black[x]] == 0)
										break;
						if (i == 0)
								break;
				}                    
				for (y = x; y > 0; y--)
				{
						if (Mabs(L_black[y], L_black[y - 1]) <= 3 && Mabs(R_black[y], R_black[y - 1]) <= 3)
								break;
				}
				if(chose_dajiaoline>y)//y是根据图像确定出来的最高的有效行 将打角行限制在这以下
				chose_dajiaoline=y;
			}
		}
		
   	if(dianboGeLi_count>0&&dianboGeLi_count<40&&dian_pan_huan==0)//颠簸特殊处理打角行 设定一定场数限制打角
		{
			 uint8_t x=0;
				 for(x=0;x<Last_Line*0.6;x++)
			{
					if(L_black[x]<185&&R_black[x]>0)//粗鲁判断这一行是不是真实行
					{
							double beishu = (double)(L_black[x] - R_black[x]) / fixvalue[x];
						  chose_dajiaoline=chose_dajiaoline*beishu;
							//setText用户自定义("颠簸打脚行所乘系数" + beishu);
							break;
					}
			}
   }
	 }
	 
	
	if(Last_Line<=chose_dajiaoline)//防止打脚行高于截止行
	 {
		 chose_dajiaoline=Last_Line;
	 }
}

void Get_Error()//确定偏差
{
	if(Last_Line > 5)//截断行比5高
		image_error = 92 - LCenter[chose_dajiaoline];
	else if(podao_flag==2||AI_State==1)//坡道2阶段 截止行过低则保持打脚
	  image_error = 0;
	else 
	  image_error = image_lasterror;
	if(youDaSi==1)//在中间会车区丢线时
	{
    image_error =92;		
	}
	

	//计算derror和记忆error
	image_derror=image_error-image_lasterror;
	image_lasterror=image_error;		
}

void Get_ServoPD()//根据不同元素确定舵机pd
{
	if((zuohuan==2||youhuan==2)&&huanline_second>0)//入环
	{
		chose_p=huan_ru_p[huancount]/10.0;	
    chose_d=D/10.0;		 
	}
	else if(zuohuan==6||youhuan==6||youhuan==5||zuohuan==5)//出环
	{
    chose_p=huan_chu_p[huancount]/10.0;	
    chose_d=D/10.0;		 
	}
	else if(zuohuan == 2 || youhuan == 2||zuohuan==3||youhuan==3)//环内
	{
		chose_p=huan_in_p[huancount]/10.0	;
		chose_d=D/10.0;
	}
	else if(zuohuan==7||youhuan==7)//出环的最后
	{
		chose_p = huan_chu_p[huancount]/10.0;
		chose_d = D/10.0; 
	} 
	else if(zuohuan==10||youhuan==10||mend_by_right==1||zuohuan==1||youhuan==1)
	{
	 	chose_p = 0.8;
		chose_d = 1; 	  
	}
	else if(zuohuan==8||youhuan==8)
	{
	 	chose_p = 0.8;
		chose_d = 0; 	  
	}
	else//其他赛道类型
	{
		if(my_Element == Long_Straight)//长直道
		{
			chose_p = Long_Straight_P/10.0;
			chose_d = Long_Straight_D/10.0;
		}
		else if(my_Element == Short_Straight)//短直道
		{
			chose_p = Short_Straight_P/10.0;
			chose_d = Short_Straight_D/10.0;
		}
		else//弯道
		{
			if(my_Element == Curve_First)//直入弯减速
			{
				chose_p = Curve_P/10.0 ;
				chose_d = Curve_D/10.0 ;
			}
			else if(my_Element == Curve_Second)//出弯
			{
				chose_p = Curve_P/10.0;
				chose_d = Curve_D/10.0;	
			}
			else//弯道
			{
				chose_p = Curve_P/10.0;
				chose_d = Curve_D/10.0;
			}
		}
		/*                 特殊元素                */
		if(my_Element == SS || my_Element == SS_Cut)//小s  
		{
			chose_p = SS_P/10.0;
			chose_d = SS_D/10.0;
		}	
		else if(my_Element == Ramp)//坡道
		{
			chose_p = Ramp_P/10.0;
			chose_d = Ramp_D/10.0;
		}		
		//会车
		if(mend_by_xuxian>0)//会车区补线之后
		{
			if(xuxianjilu!=1)
			{
				chose_p = Ahuiche_mend_p/10.0;
				chose_d = Ahuiche_mend_d/10.0;
			}
			else 
			{
				chose_p = Bhuiche_mend_p/10.0;
				chose_d = Bhuiche_mend_d/10.0;	
			}
		}	
		if(mend_recover==1)//会车补线恢复后6场
		{
			if(xuxianjilu!=1)
			{
				chose_p = Ahuiche_recover_p/10.0;
				chose_d = Ahuiche_recover_d/10.0;
			}
			else
			{
				chose_p = Bhuiche_recover_p/10.0;
				chose_d = Bhuiche_recover_d/10.0;
			}
		}
		if(podaojichang>0&&podaojichang<=10)//在坡道还未恢复正常时 图像可能会上下抖动 减小d 削弱影响
		{
			chose_d /=2;
		}
	}
}

void Servo_Control()//舵机打角计算和控制
{
	if (Last_Line>5||podao_flag==2)//截断行大于5
		duoji_InitalDuty=duojiZhongZhi - image_error * chose_p - image_derror * chose_d;//舵机pd公式
	else if(AI_State==1)
		duoji_InitalDuty=duojiZhongZhi;
	if(podao_flag==1&&Last_Line<30)//在坡道上坡过程限幅 前提是有绝对的丢线
	{
		if(duoji_InitalDuty>duojiZhongZhi+30)
			duoji_InitalDuty=duojiZhongZhi+30;
		else if(duoji_InitalDuty<duojiZhongZhi-30)
			duoji_InitalDuty=duojiZhongZhi-30;
	}
	else if(podao_flag==2&&Last_Line<10)//在中间过程限幅
	{
		if(duoji_InitalDuty>duojiZhongZhi+20)
			duoji_InitalDuty=duojiZhongZhi+20;
		else if(duoji_InitalDuty<duojiZhongZhi-20)
			duoji_InitalDuty=duojiZhongZhi-20;
	}
	else if(podao_flag==3&&Last_Line<50)//在坡道下坡过程限幅
	{	 
		if(duoji_InitalDuty>duojiZhongZhi+30)
			duoji_InitalDuty=duojiZhongZhi+30;
		else if(duoji_InitalDuty<duojiZhongZhi-30)
			duoji_InitalDuty=duojiZhongZhi-30; 		 
	}
//	if(dianboGeLi>0&&zuohuan==0&&youhuan==0&&sure_huan_after_dianbo==1&&dianboGeLi_count<=25)//图像起来后给出限制 颠簸限幅根据颠簸所在的实际位置来判断 如果在环岛前面则在判出环之前要限制打脚 如果在弯道前面则不需要
//	{
//		if(duoji_InitalDuty>duojiZhongZhi+20)
//			duoji_InitalDuty=duojiZhongZhi+20;
//		else if(duoji_InitalDuty<duojiZhongZhi-20)
//			duoji_InitalDuty=duojiZhongZhi-20;
//	} 
//	if(dianbo_zhichong==1)//颠簸可能要直冲
//	{
//		duoji_InitalDuty=duojiZhongZhi;
//	}
	 
	if(die_dangche_Flag==1&&CNT>Realbegin_Count&&dangche==2)//双车自爆直冲
	{
		duoji_InitalDuty=duojiZhongZhi;
	}
	servo_Flush();//更新舵机值
	angleChange = duoji_InitalDuty - duojiZhongZhi;//计算舵机转动的角速 用于差速控制
 
	
}

//电机控制部分
void Speed_Calculate()//根据不同元素给出不同的设定速度和电机pd
{
	if(my_Element == Long_Straight )//长直道
	{	
		image_speedset = speed_longstraight;
		MotorFuzzyLP = motor_str_I ;
		MotorFuzzyLD = motor_str_P;
		MotorFuzzyRP = motor_str_I ;
		MotorFuzzyRD = motor_str_P;
	}
	else if(my_Element == Short_Straight )//短直道
	{
		image_speedset = speed_shortstraight;
		MotorFuzzyLP = motor_str_I ;
		MotorFuzzyLD = motor_str_P;
		MotorFuzzyRP = motor_str_I ;
		MotorFuzzyRD = motor_str_P;
	}
	else if (my_Element == SS)//小S
	{
		image_speedset = speed_SS;
		if(small_s == 2)
		image_speedset = speed_normal;
		MotorFuzzyLP = motor_str_I ;
		MotorFuzzyLD = motor_str_P;
		MotorFuzzyRP = motor_str_I ;
		MotorFuzzyRD = motor_str_P;
	}
  else if(my_Element == Ramp)//坡道
	{
		image_speedset = speed_ramp ;
		MotorFuzzyLP = motor_str_I ;
		MotorFuzzyLD = motor_str_P;
		MotorFuzzyRP = motor_str_I ;
		MotorFuzzyRD = motor_str_P;
	}	
	else 
	{
		if(my_Element == Curve_First || my_Element == SS_Cut)//入弯 或 小S减速
		{
			//入弯减速检测思路
			//入弯减速是一个连续的过程 在这个过程中只能进行一段减速 减速完成之后将速度保持在（设定速度-3）的大小
			//入弯减速的起始条件在两轮中一轮速度到达（设定速度-40）
			//根据打角行确定减速的场数，根据场数的多少给出不同的目标速度
			//只有之前是长短直道、十字才是一次有效的入弯减速阶段
			
			//如果这是第一次判断到入弯 检测是不是有效的入弯
			if(Last_my_Element != Curve_First)
			{
				if(Last_my_Element == Short_Straight || Last_my_Element == Long_Straight || (Cross_Xie_Out_i > 0 || Cross_Zhong_Out_i > 0 || Cross_Zhi_Out_i > 0))//十字可能影响之前的判别 避免十字弯道直冲
				{
					RW_flag = 1;//代表这是一次有效的入弯减速
				}
				else if(Cut_AfterSS == 1 && Last_my_Element != Curve_First)//小S减速
				{
					RW_flag = 1;//有效入弯减速
				}
				else if(Cut_AfterIsland == 1 && Last_my_Element != Curve_First)//出环后减速
				{
					RW_flag = 1;//有效入弯减速
				}
				else if(CurveFirst_ByIsland == 1)//这次直入弯是根据环岛判断到的
				{
					RW_flag = 1;//有效入弯减速
				}
			}
			if(my_Element == SS_Cut && Last_my_Element != SS_Cut)//小S减速
				RW_flag = 1;//有效入弯减速
			
			//在这是有效的入弯减速的基础上处理 不然就给正常速度
			if(RW_flag == 1)//这是一次有效的入弯减速
			{
				if(Cut_Chang == 0 && Cut_finishFlag == 0)//当前不处于减速过程中 并且之前没有完成一次减速
				{
					//判断是不是可以开始减速
					if(Speed_Left + Speed_Right > speed_normal * 2 - 60)//可以开始减速
					{
						//确定减速场
						if(chose_dajiaoline < 19)
						{Cut_Chang = 2;}
						else if(chose_dajiaoline < 20)
						{Cut_Chang = 3;}
						else if(chose_dajiaoline < 21)
						{Cut_Chang = 4;}
						else if(chose_dajiaoline < 22)
						{Cut_Chang = 5;}
						else if(chose_dajiaoline < 23)
						{Cut_Chang = 6;}
						else if(chose_dajiaoline < 24)
						{Cut_Chang = 7;}
						else if(chose_dajiaoline < 25)
						{Cut_Chang = 8;}
						else if(chose_dajiaoline < 26)
						{Cut_Chang = 9;}
						else if(chose_dajiaoline < 27)
						{Cut_Chang = 10;}
						else if(chose_dajiaoline < 28)
						{Cut_Chang = 11;}
						else if(chose_dajiaoline < 29)
						{Cut_Chang = 12;}
						else if(chose_dajiaoline < 30)
						{Cut_Chang = 13;}
						else if(chose_dajiaoline < 31)
						{Cut_Chang = 14;}
						else 
						{Cut_Chang = 14;}
					}
				}
				
				//判断减速是否已经足够，就将减速跳出
				if(Speed_Left < (speed_normal - 40) && Speed_Right < (speed_normal - 40) && Cut_Chang > 1)//两轮转速都很小
				{
					Cut_Chang = 1;//再减速一场就好了
				}
				
				//根据确定的减速场进行减速
				if(Cut_Chang > 0)
				{
					if(Cut_Chang < 3)
					{
						image_speedset = speed_normal - 6;
						MotorFuzzyLP = motor_str_I ;
						MotorFuzzyLD = motor_str_P;
						MotorFuzzyRP = motor_str_I ;
						MotorFuzzyRD = motor_str_P;
					}
					else if(Cut_Chang < 6)
					{
						image_speedset = speed_normal - 12;
						MotorFuzzyLP = motor_str_I ;
						MotorFuzzyLD = motor_str_P;
						MotorFuzzyRP = motor_str_I ;
						MotorFuzzyRD = motor_str_P;
					}
					else if(Cut_Chang < 9)
					{
						image_speedset = speed_normal - 24;
						MotorFuzzyLP = motor_str_I ;
						MotorFuzzyLD = motor_str_P;
						MotorFuzzyRP = motor_str_I ;
						MotorFuzzyRD = motor_str_P;
					}
					else if(Cut_Chang < 12)
					{
						image_speedset = speed_normal - 36;
						MotorFuzzyLP = motor_str_I ;
						MotorFuzzyLD = motor_str_P;
						MotorFuzzyRP = motor_str_I ;
						MotorFuzzyRD = motor_str_P;
					}
					else if(Cut_Chang < 15)
					{
						image_speedset = speed_normal - 40;
						MotorFuzzyLP = motor_str_I ;
						MotorFuzzyLD = motor_str_P;
						MotorFuzzyRP = motor_str_I ;
						MotorFuzzyRD = motor_str_P;
					}
					else
					{
						image_speedset = speed_normal - 48;
						MotorFuzzyLP = motor_str_I ;
						MotorFuzzyLD = motor_str_P;
						MotorFuzzyRP = motor_str_I ;
						MotorFuzzyRD = motor_str_P;
					}
					Cut_Chang--;
					if(Cut_Chang == 0)//完成减速
						Cut_finishFlag = 1;
				}
				else if(Cut_finishFlag == 1)//已经完成减速 给设定速度-3
				{
					image_speedset = speed_normal - 3;
					MotorFuzzyLP = motor_cur_I;
					MotorFuzzyLD = motor_cur_P;
					MotorFuzzyRP = motor_cur_I;
					MotorFuzzyRD = motor_cur_P;
				}
				else//还没有开始入弯减速
				{
					image_speedset = speed_normal;
					MotorFuzzyLP = motor_cur_I;
					MotorFuzzyLD = motor_cur_P;
					MotorFuzzyRP = motor_cur_I;
					MotorFuzzyRD = motor_cur_P;
				}
			}
			else//这次入弯减速无效 给设定速度
			{
				image_speedset = speed_normal;
				MotorFuzzyLP = motor_cur_I;
				MotorFuzzyLD = motor_cur_P;
				MotorFuzzyRP = motor_cur_I;
				MotorFuzzyRD = motor_cur_P;
			}
		}
		else if(my_Element == Curve_Second)//出弯
		{
				image_speedset = speed_normal ;
				MotorFuzzyLP = motor_cur_I ;
				MotorFuzzyLD = motor_cur_P;
				MotorFuzzyRP = motor_cur_I ;
				MotorFuzzyRD = motor_cur_P;
		}			
		else
		{
			image_speedset = speed_normal;
			MotorFuzzyLP = motor_cur_I ;
			MotorFuzzyLD = motor_cur_P;
			MotorFuzzyRP = motor_cur_I ;
			MotorFuzzyRD = motor_cur_P;
		}
	}
	if(my_Element != Curve_First && my_Element != SS_Cut)//清楚入弯标志
	{
		Cut_Chang=0;
		RW_flag =0;
	}
	
	//处理特殊元素的速度
	if(xiaopo_jiansu>0)//缓坡速度
	{
		image_speedset=xiaopo_speed;								
	}
	else if(my_Element == Island//排除掉可以加速的环岛前直道和减速阶段
	   && ((zuohuan>=3&&zuohuan<=5)||(youhuan>=3&&youhuan<=5)))
	{
		//if(huancount<2)
		image_speedset=huan_size[huancount];
		//else 
		//	image_speedset-=huan_size[huancount];
	}
	/*********                  双车速度策略            **********/ 
  if(dangche==0)
	{			
		/*             起始发车不给差速              */		
		if(CNT<Realbegin_Count)				
		{
			image_speedset=speed_normal;
		}	
		if(xuxianjilu==1)//中间会车操作
		{
			if(CarFlag==0)//若为前车
			{
				if(already_huiche==0&&huiche!=2)//前车靠边停 后车还没到
				{
					double cutP=(double)(speed_normal)/MiddleStop_Init;//根据设定减场次确定p 设定减场稍微给大点以使前车能够稍微远
					huichecount++;
					if(huichecount<MiddleStop_Init)
					{
						image_speedset=speed_normal-cutP*huichecount;			
					}
					else
					{
						image_speedset=0;
					}
				}
				else if(already_huiche==0&&huiche==2)//后车已经到了  弯道会车 前车等待会车结束在启动
				{
					if(wandao_huiche_flag==1)
						image_speedset=0;           //弯道会车 前车等待会车结束在启动
				}	
			}
			else if(CarFlag==1)//后车全过程只作不给差速处理-car1_lowspeed
			{
				image_speedset=speed_normal;					
			}
			
			if(wandao_huiche_flag==0)//直道会车前车缓慢启动
			{
				if(zhidao_huiche_count>0&&zhidao_huiche_count<=15)//前车恢复后缓慢加速
				{	
					double addP=(double)(speed_normal)/15;//根据设定加场次确定p
					image_speedset=addP*zhidao_huiche_count;		
				}
			}
			else if(wandao_huiche_flag==1)//弯道会车
			{
				if(last_CarFlag==0)//复位前是前车
				{
					if(after_huiche_count>0&&after_huiche_count<=15)//前车恢复后缓慢加速
					{	
						double addP=(double)(speed_normal)/15;//根据设定加场次确定p
						image_speedset=addP*after_huiche_count;		
					}
				}
			}	
		}
		if(xuxianjilu==2)//最后会车操作
		{
			if(CarFlag==0)//若为前车
			{
				if(Stop_Line_Huiche==0&&huiche!=2)//前车靠边停 
				{
					double cutP=(double)(speed_normal)/EndStop_Init;//根据设定减场次EndStop_Init确定p 设定减场稍微小 以使前车停在起跑线前面
					tingchecount++;
					if(tingchecount<EndStop_Init)
					{
						image_speedset=speed_normal-cutP*tingchecount;
					}
					else
					{
						image_speedset=0;
					}
					if(LPulse+Dulse<100)
						have_stop=1;
				}	
				else if(Stop_Line_Huiche==0&&huiche==2)//后车已经到了 前车等待后车接近就启动
				{
					image_speedset=speed_normal;		

				}			
				else if(Stop_Line_Huiche==2)//开始停车
				{
					stop_count++;
					if(stop_count>RealStop_Count)
					{
						image_speedset=0;
					}
				}
			}
			else if(CarFlag==1)//后车全过程只作不给差速处理
			{
				if(Stop_Line_Huiche==1)//后车不给差速
				{
					image_speedset=speed_normal-car1_lowspeed;
				}		
				else if(Stop_Line_Huiche==2)//开始停车
				{
					stop_count++;
					if(stop_count>RealStop_Count)
					{
						image_speedset=0;
					}	
				}			
			}
		} 
	}		
	//双车会车 一车正常出赛道 另一辆单车跑完 判到第二次虚线停车
	if(longtime_huiche_Flag==1||die_dangche_Flag==2)
	{
		if(dangche==2)
		{
			if(xuxianjilu==2)
			{
				 stop_count++;
				 if(stop_count>25)
				 {
					 image_speedset=0;
				 }
			}
		}
	}
	
	//AI减速中
	if(AI_State == 1 && CarFlag == 2)//ai_state只在减速的时候会变1 别的时候不会变1
	{
		if(AI_Speed < image_speedset)
			image_speedset = AI_Speed;
	}
	else if(AI_State == 2 && CarFlag == 2)//减速恢复
	{
		AI_RecoveryCnt++;
		if((AI_Speed * (10 - AI_RecoveryCnt) / 10 + image_speedset * AI_RecoveryCnt / 10) < image_speedset)
			image_speedset = AI_Speed * (10 - AI_RecoveryCnt) / 10 + image_speedset * AI_RecoveryCnt / 10;
	}
	

	
	//各种保护
	if(motor_stop==1&&(protect_flag==1||die_dangche_Flag==1))
	{
		image_speedset=0;
    if(dangche==0)
		{
			NRF2401_SetMode(TX);
			Wireless(TX,10);			
		}
	}
	if(motor_stop==2)//双车模式下 另一辆车出赛道 这辆车也立即停
	{
		image_speedset=0;
	}	 
}

void Motor_Control()//计算电机差速 电机闭环控制
{
	//电机差速计算
	//根据需要的速度类型 确定是否给差速
	if(image_speedset == 0)//停车时不给差速
	{
		aimzuo = 0;
		aimyou = 0;
	}
	else if(my_Element == Short_Straight || my_Element == Long_Straight || my_Element == SS)//直道小S不给差速
	{
		aimzuo = image_speedset;
		aimyou = image_speedset;
	}
	else if(my_Element == Ramp)//坡道不给差速
	{
		aimzuo = image_speedset;
		aimyou = image_speedset;
	}
	else//其他给差速
	{
		if(my_My_Abs1(angleChange) < 18)
		{
			aimzuo = image_speedset;
			aimyou = image_speedset;
		}
		else if(my_My_Abs1(angleChange) < 24)
		{
			if(angleChange > 0)//左转
			{
				aimzuo = image_speedset / 1.5;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//右转
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.5;
			}
		}
		else if(my_My_Abs1(angleChange) < 32)
		{
			if(angleChange > 0)//左转
			{
				aimzuo = image_speedset / 1.55;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//右转
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.55;
			}
		}
		else if(my_My_Abs1(angleChange) < 43)
		{
			if(angleChange > 0)//左转
			{
				aimzuo = image_speedset / 1.6;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//右转
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.6;
			}
		}
		else if(my_My_Abs1(angleChange) < 53)
		{
			if(angleChange > 0)//左转
			{
				aimzuo = image_speedset / 1.65;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//右转
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.65;
			}
		}
		else if(my_My_Abs1(angleChange) < 67)
		{
			if(angleChange > 0)//左转
			{
				aimzuo = image_speedset / 1.7;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//右转
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.7;
			}
		}
		else if(my_My_Abs1(angleChange) < 75)
		{
			if(angleChange > 0)//左转
			{
				aimzuo = image_speedset / 1.75;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//右转
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.75;
			}
		}
		else
		{
			if(angleChange > 0)//左转
			{
				aimzuo = image_speedset / 1.8;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//右转
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.8;
			}
		}
	}
	
	if(Last_my_Element != Curve_First && my_Element == Curve_First/* && RW_flag == 1*/)//入弯限幅减速 要求是第一场判断为入弯减速、有效减速
	{
		if(motor_left > 8000 || motor_right > 8000)
		{
			motor_left = 5000;
			motor_right = 5000;
		}
	}	
	Encodercontrol_motor();//电机pid计算及限幅	
	if(guancha == 1)//双车当前车停下在启动时直冲
	{
		if(LPulse+Dulse<900&&have_stop==1)
		{
			motor_left = init_dianji_go;
			motor_right = init_dianji_go;
		}
	}
	if(podao_flag>=2)//中间过坡时给占空比 防止影响打脚
	{
		motor_left=3000;
		motor_right=3000;
	}	
	/*                          停车时防止倒退                    */
	if(aimzuo==0&&aimyou==0&&LPulse<80&&Dulse<80)//中间会车防倒退
	{	
		motor_left=0;
		motor_right=0;
	}
	if(aimzuo==0&&aimyou==0&&LPulse+Dulse<20&&xuxianjilu==2)//最后会车防倒退
	{	
		motor_left=0;
		motor_right=0;
	}
	//改变占空比
	motor_flush();
}



//根据图像确定赛道类型
uint8_t last_cutLine = 30;
void Set_Element()
{
	#define byte uint8_t
	
	uint8_t cutLine = 30;//减速行 短直道与直入弯的判断行
  if(chose_dajiaoline > 18)//提升减速行 能够有效减速
	{
		if(chose_dajiaoline < 20)
			cutLine += 1;
		else if(chose_dajiaoline < 22)
			cutLine += 2;
		else if(chose_dajiaoline < 24)
			cutLine += 3;
		else if(chose_dajiaoline < 26)
			cutLine += 4;
		else//28
			cutLine += 5;
	}
	if(my_Element == Curve_First)//如果上一场检测到直入弯 维持上一次的减速行 防止状态切换
	{
		if(cutLine < last_cutLine)
			cutLine = last_cutLine;
	}
	last_cutLine = cutLine;//记忆
	
//	Judge_LStart = L_init;
//	Judge_RStart = R_init;
	//显示用于标定
	//OLED_Write_Num2(0, 0, Judge_Memory);
	//OLED_Write_Num3(0, 2, (int)(Judge_Slope * 100));
	
	//初始化
	CurveFirst_ByIsland = 0;//环岛直入弯标记
	
	//赛道记忆
	Last_my_Element = my_Element;
	if (zuohuan > 0 || youhuan > 0)//环岛10阶段作为短直道 1阶段作为直入弯
	{
			if(zuohuan == 10 || youhuan == 10)//短直道
					my_Element = Short_Straight;
			else if(zuohuan == 1 || youhuan == 1)//直入弯
			{
					my_Element = Curve_First;
					CurveFirst_ByIsland =1;//环岛直入弯标记
			}
			else
					my_Element = Island;
			//text用户自定义("环岛");
	}
	else if (podao_flag != 0)
	{
			my_Element = Ramp;
			//text用户自定义("坡道" /*+ Ramp_Process*/);
	}
	else if (small_s > 0 && small_sCanGo == 1)
	{
			my_Element = SS;
			if(SS_needCut == 1)
			{
				my_Element = SS_Cut;//小S减速
			}
			//text用户自定义("小S");
	}
	else if (my_Curve_Flag == 0 && Judge_Memory > 35/*40*/ && Judge_Slope < 0.6 && ((Judge_LStart < 25 && Judge_RStart < 25) || (Cross_Xie_Out_i > 0 || Cross_Zhong_Out_i > 0 || Cross_Zhi_Out_i > 0)))//直道增加左右丢线起始行的限制
	{
			if (podaojichang == 0 || podaojichang > 10 || dianbo > 0)
					my_Element = Long_Straight;
			else
					my_Element = Curve_Third;
			//text用户自定义("长直道");
	}
	else if (my_Curve_Flag == 0 && Judge_Memory <= /*40*/35 && Judge_Memory > /*35*//*30*/cutLine && Judge_Slope < 0.8 && ((Judge_LStart < 25 && Judge_RStart < 25) || (Cross_Xie_Out_i > 0 || Cross_Zhong_Out_i > 0 || Cross_Zhi_Out_i > 0)) && (podaojichang == 0 || podaojichang > 10))
	{
			if (podaojichang == 0 || podaojichang > 10 || dianbo > 0)//清坡后10场不判直道
					my_Element = Short_Straight;
			else
					my_Element = Curve_Third;
			//text用户自定义("短直道");
	}
	else if (my_Curve_Flag == 0 && Judge_Memory <= /*35*//*30*/cutLine && Judge_Slope < 0.8)
	{
			my_Element = Curve_First;
			//text用户自定义("直入弯");
	}
	else if (my_Curve_Flag == 1 /*&& Judge_Slope >= 0.5*/ && Judge_Slope < 1.8 && Judge_Memory > 30)
	{
			my_Element = Curve_Second;
			//text用户自定义("出弯");
	}
	else
	{
			my_Element = Curve_Third;
			//text用户自定义("弯道");
	}	
	//小S之后减速的特殊处理
	if(my_Element == Curve_Third && Last_my_Element == SS)//进行小S减速判断
	{
		if(Speed_Left + Speed_Right > speed_normal * 2 - 60)
		{
			Cut_AfterSS = 1;
		}
	}
	//环岛之后减速的特殊处理
	if(my_Element == Curve_Third && Last_my_Element == Island)
	{
		if(Speed_Left + Speed_Right > speed_normal * 2 - 60)
		{
			Cut_AfterIsland = 1;
		}
	}
	
	if(Cut_AfterSS == 1 || Cut_AfterIsland == 1)//处于小S、环岛之后减速的过程中
	{
		if(Speed_Left < speed_normal - 20 && Speed_Right < speed_normal - 20)//如果速度已经减下来 可以给正常速度策略
		{
			Cut_AfterSS = 0;
			Cut_AfterIsland = 0;
		}
		else if(my_Element == Short_Straight || my_Element == Long_Straight || my_Element == SS)//如果是直道或者小S 跳出小S之后减速
		{
			Cut_AfterSS = 0;
			Cut_AfterIsland = 0;
		}
		else if(my_Element == Island || my_Element == Ramp)//环岛 坡道 特殊元素清楚
		{
			Cut_AfterSS = 0;
			Cut_AfterIsland = 0;
		}
		
		if(Cut_AfterSS == 1 || Cut_AfterIsland == 1)
			my_Element = Curve_First;//给直入弯减速
	}
	
	

	//弯道标志的清楚与判断
	if (my_Element == 11 || my_Element == 12)//这场是弯道或者出弯
	{
			my_Curve_Flag = 1;
	}
	if (my_Element == 11 && Judge_Slope > -0.3 && Judge_Slope < 0.3)//出弯的条件下检测弯道标志的清楚
	{
			byte judge_row = 0; float judge_k = 0;//检测清楚的系数
			judge_row = (byte)My_Max(Judge_LStart, Judge_RStart);
			if (Cross_Xie_Out_i > 0 || Cross_Zhong_Out_i > 0 || Cross_Zhi_Out_i > 0)
					judge_row = 0;
			if (judge_row < 20)
					judge_row = 20;
			judge_k = (float)my_My_Abs1(LCenter[judge_row] - 92) / (float)fixvalue[judge_row];
			if (judge_k < 0.1f)//清楚
			{
					my_Curve_Flag = 0;
					//如果将弯道标记清楚了 就改为短直道
					my_Element = Short_Straight;
					//settext用户自定义("清楚弯道记忆 短直道");
			}
			else if(judge_k < 0.2f)//再进行一行的判断
			{
					judge_row = 15;
					if(judge_row < Judge_LStart && judge_row > Judge_RStart)
					{
							judge_k = (float)my_My_Abs1(LCenter[judge_row] - 92) / (float)fixvalue[judge_row];
							if (judge_k < 0.1f)//清楚
							{
									my_Curve_Flag = 0;
									//如果将弯道标记清楚了 就改为短直道
									my_Element = Short_Straight;
									//settext用户自定义("清楚弯道记忆 短直道");
							}
					}
			}
	}
	else if (my_Element == 14 || my_Element == 15 || my_Element == 5)//特殊元素清楚弯道标记
	{
			my_Curve_Flag = 0;
	}
	//处理直入弯减速标志
	if (my_Element != Curve_First && my_Element != SS_Cut)
	{
	    RW_flag = 0;
	    Cut_Chang = 0;
	    Cut_finishFlag = 0;
	}
	if(small_s == 0)
		SS_needCut = 0;//清空标志位
}

//小车控制总函数
//会车逻辑
void huiche_state()
{
	if(die_dangche_Flag==2)
	{
		if(xuxianjilu==1&&huicheNum==0)
		{
			huicheNum++;
			already_huiche=2;
			Stop_Line_Huiche=2;
		}
		if(already_huiche==2)
		{
			after_huiche_count++;//用于第二次判断会车区的延时 防止在中间会车区出现二次判断		
		}	
	}
	if(dangche==0&&die_dangche_Flag==0)
	{			
		/*               中间会车                               */
		if(xuxianjilu==0||tongshi_huiche==1)//第一次区分前后车
		{
			if(Wireless(RX,0)==2 && already_huiche ==0)//收到2说明已有前车到达
			{
				already_huiche = 1;
			}				
		}				
		if(xuxianjilu==1)//进入中间会车区
		{
			if(CarFlag==2)//确定前后车 中间会车后already_huiche一直是2 不管是前车还是后车 一旦遇到虚线 就要给另一辆车发送消息
			{
				if(already_huiche==0&&tongshi_huiche==0)
				{
					NRF2401_SetMode(TX);
					CarFlag = 0;
					begin_stop=1;
				}
			  else if(already_huiche==1)
				{
					CarFlag = 1;
          NRF2401_SetMode(TX);					
				}
			}							
			if(CarFlag==0&&already_huiche==0)//前车
			{
				if(huiche==0)//前车停车发送会车信号
				{					
					Wireless(TX,2);							
					/******* 中间会车等待时间过长 判定此时后车已出赛道  ********/
					if(longtime_huiche_Flag==1)
					{
						Mid_huiche_Count++;
						if(Mid_huiche_Count>Menu_Mid_huiche_Count)
						{
							dangche=2;
							already_huiche=2;
							//Stop_Line_Huiche=2;
							CarFlag=2;
							Mid_huiche_Count=0;
							huicheNum++;
						}
				  }				
					if(NRF2401_SendData(NRF2401TXBuffer) == TX_OK)
					{
						NRF2401_SetMode(RX);
						huiche=1;						
					}					
				}
        if(huiche==0&&Car==3&&tongshi_huiche==0)		//如果前车没有发送成功 开时计场 当场数达到一定值时 看两车距离 如果辆车距离一直小于2米 那么判定此时两辆车同时到达
				{
				 send_fail_count++;
				}
        if(send_fail_count>5&&(uwb_Distance<=200&&last_distance[0]<=200&&last_distance[1]<=200)&&huiche==0)
				{
				   	NRF2401_SetMode(RX);
						huiche=1;		
            send_fail_flag=1;
            send_fail_count=0;					
				}					
        if(huiche	==1)//等待后车识别到虚线		
				{									
					/******* 中间会车等待时间过长 判定此时后车已出赛道  ********/
					if(longtime_huiche_Flag==1)
					{
						Mid_huiche_Count++;
						if(Mid_huiche_Count>Menu_Mid_huiche_Count)
						{
							dangche=2;
							already_huiche=2;
							//Stop_Line_Huiche=2;
							CarFlag=2;
							Mid_huiche_Count=0;
							huicheNum++;
						}
				  }					
					if(Wireless(RX,0)==4)//此时后车识别到虚线
					{
						huiche	= 2 ;
					}	
          else if(Wireless(RX,0)==2&&Car==3&&send_fail_flag==1)//同时识别到虚线
					{
						//NRF2401_SetMode(TX); 						
						tongshi_huiche=1;
						CarFlag=2;
						huiche=0;
					  send_fail_count=0;
					  send_fail_flag=0;
						OLED_Write_String(0,0,"tongshi");
					}		
					if(send_fail_flag==1&&CarFlag==0)
					{
					send_fail_count++;
					if(send_fail_count>5)	
					{
					 send_fail_count=0;
					 send_fail_flag=0;
					 huiche=0;
					 NRF2401_SetMode(TX);
					}
					}			
			  }	
				if(huiche==2&&tongshi_huiche==0)//后车已经识别出虚线
				{	
					send_fail_flag=0;
          send_fail_count=0;	
					if(test==0)
						houche_huiche_count++;	//这个参数用来防止超声波对不上 或者突然坏掉的情况 当两车都到达会车区且经过一定时间后会自动复位		
					
					/*  调试弯道会车  更改会车复位逻辑  */
					if(wandao_huiche_flag==1)//弯道会车
					{
						if(houche_huiche_count>houche_huiche_Menu_count||
							(last_distance[2]>=last_distance[1]&&last_distance[1]>=last_distance[0]&&last_distance[0]>=uwb_Distance&&last_distance[2]>uwb_Distance&&uwb_Distance<huiche_Menu_uwb))
								huichejuli_cut=1;
						if(huichejuli_cut==1)
						{
							if((last_distance[2]<=last_distance[1]&&last_distance[1]<=last_distance[0]&&last_distance[0]<=uwb_Distance&&last_distance[2]<uwb_Distance)
								||houche_huiche_count>houche_huiche_Menu_count)//uwb距离逐渐减小至菜单设定最小值后增大
							{
								huichejuli=1;
							}
							if(huichejuli==1)
							{	
								huicheyanshi_count++;	
								if(huicheyanshi_count>huicheyanshi)								
								{
										last_CarFlag=CarFlag;
										CarFlag=2;	
										already_huiche=2;					
										huichejuli=0;
										huiche=0;
										huicheNum++;
										NRF2401_SetMode(RX);	
										WirelessReceive=0;
										xueruo=0;
										houchexueruo=0;	
										huicheyanshi_count=0;		
										car0_stop_count=0;	
										houche_huiche_count=0;	
				            tongshi_huiche=0;									
									}	
							}	
						}
				  }
					else if(wandao_huiche_flag==0)//直道会车
					{	
						/*  直道会车会车复位逻辑  距离接近 会车复位  */
						if((huichejuli==0&&last_distance[2]>=last_distance[1]&&last_distance[1]>=last_distance[0]&&last_distance[0]>=uwb_Distance&&last_distance[2]>uwb_Distance
							&&last_distance[0]<100&&uwb_Distance<huiche_Menu_uwb)||houche_huiche_count>houche_huiche_Menu_count)//根据uwb来确定后车是否已靠近前车 三场连续减小
						{
							huichejuli=1;
						}
						if(huichejuli==1)
						{	
							huicheyanshi_count++;	
							if(huicheyanshi_count>huicheyanshi)								
							{		
								last_CarFlag=CarFlag;
								CarFlag=2;	
								already_huiche=2;					
								huichejuli=0;
								huiche=0;
								huicheNum++;
								NRF2401_SetMode(RX);	///
								WirelessReceive=0;
								xueruo=0;
								houchexueruo=0;	
								huicheyanshi_count=0;		
								car0_stop_count=0;	
								houche_huiche_count=0;		
				        tongshi_huiche=0;								
							}							
						}	
					}


		
				}		
			}			 
			if(CarFlag==1&&already_huiche==1)//后车
			{	
				if(huiche==0)//一开始识别到虚线 给前车发4 说明后车已判出虚线
				{					
					Wireless(TX,4);					
					if(NRF2401_SendData(NRF2401TXBuffer) == TX_OK)
					{
						NRF2401_SetMode(RX);
						huiche=2;					
					}					
				}
				if(huiche==2)
				{	
					if(test==0)
							houche_huiche_count++;	//这个参数用来防止超声波对不上 或者突然坏掉的情况 当两车都到达会车区且经过一定时间后会自动复位		
					if(wandao_huiche_flag==1)//弯道会车
					{		
						/*  调试弯道会车  更改会车复位逻辑  */
						if(houche_huiche_count>houche_huiche_Menu_count
							||(last_distance[2]>=last_distance[1]&&last_distance[1]>=last_distance[0]&&last_distance[0]>=uwb_Distance&&last_distance[2]>uwb_Distance&&uwb_Distance<huiche_Menu_uwb))
								huichejuli_cut=1;
						if(huichejuli_cut==1)
						{
							if((last_distance[2]<=last_distance[1]&&last_distance[1]<=last_distance[0]&&last_distance[0]<=uwb_Distance&&last_distance[2]<uwb_Distance)
								||houche_huiche_count>houche_huiche_Menu_count)//uwb距离逐渐减小至菜单设定最小值后增大
							{
								huichejuli=1;
							}
							if(huichejuli==1)
							{	
								last_CarFlag=CarFlag;
								already_huiche=2;
								CarFlag=2;
								WirelessReceive=0;
								huichejuli=0;
								huiche=0;
								huicheNum++;
								xueruo=0;
								houchexueruo=0;
								houche_huiche_count=0;
								hou_send=0;	
                NRF2401_SetMode(RX);
                tongshi_huiche=0;								
							}	
						}
					}
					else if(wandao_huiche_flag==0)//直道会车
					{
	
						/*  直道会车会车复位逻辑  距离接近 会车复位  */
						if((huichejuli==0&&last_distance[2]>=last_distance[1]&&last_distance[1]>=last_distance[0]&&last_distance[0]>=uwb_Distance&&last_distance[2]>uwb_Distance
							&&last_distance[0]<100&&uwb_Distance<huiche_Menu_uwb)||houche_huiche_count>houche_huiche_Menu_count)//根据uwb来确定后车是否已靠近前车 三场连续减小
						{
							huichejuli=1;
						}
						if(huichejuli==1)
						{	
							huicheyanshi_count++;	
							if(huicheyanshi_count>huicheyanshi)								
							{	
								NRF2401_SetMode(RX);///
								last_CarFlag=CarFlag;
								already_huiche=2;
								CarFlag=2;
								WirelessReceive=0;
								huichejuli=0;
								huiche=0;
								huicheNum++;
								xueruo=0;
								houchexueruo=0;
								houche_huiche_count=0;
								hou_send=0;
								tongshi_huiche=0;
							}
						
					}
				}
			}
		  }		
	  }
		if(wandao_huiche_flag==0)//直道会车缓慢启动计场
		{
			if(already_huiche==0&&huiche==2&&xuxianjilu==1)
				zhidao_huiche_count++;
		}
		if(already_huiche==2&&xuxianjilu==1)//也用于弯道会车前车缓慢启动计场
		{
			after_huiche_count++;//用于第二次判断会车区的延时 防止在中间会车区出现二次判断		
		}
		
		/*                最后回车                              */	
		if(already_huiche==2&&(xuxianjilu==1||tongshi_huiche==1)&&CarFlag ==2)//第二次区分前后车
		{
			if(Wireless(RX,0)==2)
				Stop_Line_Huiche = 1;
		}
		if(xuxianjilu==2)//进入最终会车区
		{
			if(CarFlag==2)//确定前后车
			{
				if(Stop_Line_Huiche==0&&tongshi_huiche==0)
				{
					CarFlag = 0;
					NRF2401_SetMode(TX);
					begin_stop=1;
				}
			  else if(Stop_Line_Huiche==1)
				{
					CarFlag = 1;
          NRF2401_SetMode(TX);					
				}
		  }			
			if(CarFlag==0&&Stop_Line_Huiche==0)
			{			
				if(huiche==0)//确定前后车
				{					
					Wireless(TX,2);	
					/******* 最后会车等待时间过长 判定此时后车已出赛道  ********/
					if(longtime_huiche_Flag==1)
					{
						End_huiche_count++;
						if(End_huiche_count>Menu_End_huiche_Count)
						{
							dangche=2;
							Stop_Line_Huiche=2;
							End_huiche_count=0;
							CarFlag=2;
						}
					}						
					if(NRF2401_SendData(NRF2401TXBuffer) == TX_OK)
					{
						NRF2401_SetMode(RX);
						huiche=1;	
					}					
				}	
				 if(huiche==0&&Car==3)		//如果前车没有发送成功 开时计场 当场数达到一定值时 看两车距离 如果辆车距离一直小于2米 那么判定此时两辆车同时到达
				{
				 send_fail_count++;
				}
        if(send_fail_count>5&&(uwb_Distance<=200&&last_distance[0]<=200&&last_distance[1]<=200)&&huiche==0)
				{
				   	NRF2401_SetMode(RX);
						huiche=1;		
            send_fail_flag=1;
            send_fail_count=0;					
				}	
        if(huiche	==1)//等待后车识别到虚线		
				{			
					/******* 最后会车等待时间过长 判定此时后车已出赛道  ********/
					if(longtime_huiche_Flag==1)
					{
						End_huiche_count++;
						if(End_huiche_count>Menu_End_huiche_Count)
						{
							dangche=2;
							Stop_Line_Huiche=2;
							End_huiche_count=0;
							CarFlag=2;
						}
					}	
					if(Wireless(RX,0)==4)//此时后车识别到虚线
					{
						huiche	= 2 ;
						guancha=1;
					}	
          else if(Wireless(RX,0)==2&&Car==3&&send_fail_flag==1)//同时识别到虚线
					{
						tongshi_huiche=1;
						CarFlag=2;
						huiche=0;
					  send_fail_count=0;
					  send_fail_flag=0;
						OLED_Write_String(0,0,"tongshi");
					}
					if(send_fail_flag==1&&CarFlag==0)
					{
					send_fail_count++;
					if(send_fail_count>5)	
					{
					 send_fail_count=0;
					 send_fail_flag=0;
					 huiche=0;
					 NRF2401_SetMode(TX);
					}
					}						
			  }			  
				if(huiche==2&&tongshi_huiche==0)//后车已经识别出虚线
				{	
					 send_fail_count=0;
					 send_fail_flag=0;
					if(test==0)
						houche_huiche_count++;	//这个参数用来防止超声波对不上 或者突然坏掉的情况 当两车都到达会车区且经过一定时间后会自动复位				
					if((huichejuli==0&&last_distance[2]>=last_distance[1]&&last_distance[1]>=last_distance[0]&&last_distance[0]>=uwb_Distance&&last_distance[2]>uwb_Distance
						&&last_distance[0]<100&&uwb_Distance<huiche_Menu_uwb)||houche_huiche_count>houche_huiche_Menu_count)//根据uwb来确定后车是否已靠近前车 三场连续减小
					{
						huichejuli=1;
					}
					if(huichejuli==1)
					{	
						huicheyanshi_count++;	
						if(huicheyanshi_count>last_huicheyanshi)								
						{
								Stop_Line_Huiche=2;	
								guancha=0;
								WirelessReceive=0;
								huichejuli=0;
								huiche=0;
								huicheNum++;
								NRF2401_SetMode(RX);	
								xueruo=0;
								houchexueruo=0;	
								huicheyanshi_count=0;		
								car0_stop_count=0;
								houche_huiche_count=0;						
							
						}								
					}			
				}											
			}			
			
					
			if(CarFlag==1&&Stop_Line_Huiche==1)
			{					
				if(huiche==0)//一开始识别到虚线 给前车发4 说明后车已判出虚线
				{					
					Wireless(TX,4);					
					if(NRF2401_SendData(NRF2401TXBuffer) == TX_OK)
					{
						NRF2401_SetMode(RX);
						huiche=2;	
					}					
				}
				if(huiche==2)
				{	
					if(test==0)
						houche_huiche_count++;	//这个参数用来防止超声波对不上 或者突然坏掉的情况 当两车都到达会车区且经过一定时间后会自动复位			
					if((huichejuli==0&&last_distance[2]>=last_distance[1]&&last_distance[1]>=last_distance[0]&&last_distance[0]>=uwb_Distance&&last_distance[2]>uwb_Distance
						&&last_distance[0]<100&&uwb_Distance<huiche_Menu_uwb)||houche_huiche_count>houche_huiche_Menu_count)//根据uwb来确定后车是否已靠近前车 三场连续减小
					{
						huichejuli=1;
					}
					if(huichejuli==1)
					{	
						huicheyanshi_count++;	
						if(huicheyanshi_count>last_huicheyanshi)								
						{
							Stop_Line_Huiche=2;
							WirelessReceive=0;
							huichejuli=0;
							huiche=0;
							huicheNum++;
							xueruo=0;
							houchexueruo=0;
							houche_huiche_count=0;
							hou_send=0;
						}
					}
				}
			}	  
	  }		
		if(Stop_Line_Huiche==2)
		{
			after_lasthuiche_count++;//用于第二次判断会车区的延时 防止在中间会车区出现二次判断		
		}
	}
	if(already_huiche==2&&xuxianjilu==1&&dangche==2)//也用于弯道会车前车缓慢启动计场
	{
		after_huiche_count++;//用于第二次判断会车区的延时 防止在中间会车区出现二次判断		
	}
	/*    没20ms周期继承一次距离      */
	last_distance[2] = last_distance[1];
	last_distance[1] = last_distance[0];
	last_distance[0] = uwb_Distance;
}

//设定一段距离减速
//设定一段距离减速
void AI_Init()//初始化记录环岛、坡道、会车区的数量
{
	for(uint8_t i = 0; i < 8; i++)
	{
		if(AI_Element_set[i] == 0)//环岛
			cutIsland_set[i] = AI_num_set[i];
		else if(AI_Element_set[i] == 1)//坡道
			cutRamp_set[i] = AI_num_set[i];
		else if(AI_Element_set[i] == 2)//会车区
			cutDashed_set[i] = AI_num_set[i];
	}
}

void Artificial_Intelligence()
{
	//计算环岛或者坡道之后的米数
	if(huancount == last_huancount + 1)//环岛结束
	{
		Island_distance[huancount] = encoderCM;//记录第几个环岛所在的米数
	}
	else if(PanPoJishu == last_PanPoJishu + 1)//坡道结束
	{
		Ramp_distance[PanPoJishu] = encoderCM;//记录第几个坡道所在的米数
	}
	else if(xuxianjilu == 1 && DashedField_distance == 100000)//检测到虚线会车区
	{
		DashedField_distance = encoderCM;//记录会车区的距离
	}
	
	
	
	//一直加 并计算实际的米数
	encoderCNT += Speed_Left + Speed_Right;
	encoderCM = encoderCNT / 116;
	
	//如果处在减速的过程中 并且有颠簸的清楚 检测以下是不是在颠簸减速的人工智能区间中 如果是就把这个区间作为使用过
	if((dianbo == 0 && last_dianbo > 0 && AI_State == 1) || (PanPoJishu == last_PanPoJishu + 1 && podaoF == 1))//颠簸清楚或者坡道清楚
	{
		//检测是不是在减速区间中
		for(uint8_t i = 0; i < 8; i++)
		{
			if(huancount == cutIsland_set[i])//是这个环岛 并且没有用过这次减速（颠簸清楚颠簸减速）
			{
				if(encoderCM > Island_distance[huancount] + cutCM_In[i] * 100 && encoderCM < Island_distance[huancount] + (cutCM_Out[i] + cutCM_In[i]) * 100)
				{
					if(AI_Element[i] == 2 && (dianbo == 0 && last_dianbo > 0 && AI_State == 1))//颠簸减速开关
						AI_Use[i] = 1;//用过这次的减速
					if(AI_Element[i] == 1 && (PanPoJishu == last_PanPoJishu + 1 && podaoF == 1))//坡道识别开关
						AI_Use[i] = 1;//用过这次坡道开关
				}
			} 
			else if(PanPoJishu == cutRamp_set[i])//是这个坡道 并且没有用过这次减速（颠簸清楚颠簸减速）
			{
				if(encoderCM > Ramp_distance[PanPoJishu] + cutCM_In[i] * 100 && encoderCM < Ramp_distance[PanPoJishu] + (cutCM_Out[i] + cutCM_In[i]) * 100)
				{
					if(AI_Element[i] == 2 && (dianbo == 0 && last_dianbo > 0 && AI_State == 1))//颠簸减速开关
						AI_Use[i] = 1;//用过这次的减速
					if(AI_Element[i] == 1 && (PanPoJishu == last_PanPoJishu + 1 && podaoF == 1))//坡道识别开关
						AI_Use[i] = 1;//用过这次坡道开关
				}
			}
			else if(xuxianjilu == cutDashed_set[i] && AI_Use[i] == 0)//处于这个会车区之后
			{
				if(encoderCM > DashedField_distance + cutCM_In[i] * 100 && encoderCM < DashedField_distance + (cutCM_Out[i] + cutCM_In[i]) * 100)
				{
					if(AI_Element[i] == 2 && (dianbo == 0 && last_dianbo > 0 && AI_State == 1))//颠簸减速开关
						AI_Use[i] = 1;//用过这次的减速
					if(AI_Element[i] == 1 && (PanPoJishu == last_PanPoJishu + 1 && podaoF == 1))//坡道识别开关
						AI_Use[i] = 1;//用过这次坡道开关
				}
			}
		}
	}
	
	
	
	last_dianbo = dianbo;//颠簸清楚记忆
	//更新记忆
	last_huancount = huancount;
	last_PanPoJishu = PanPoJishu;
	
	
	//计算是否处在人工智能的过程中
	//先清空人工智能减速状态
	uint8_t this_AIState = 0;
	podaoF = 0;//先关闭坡道开关
	dianboF = 0;//先关闭颠簸开关
	huichequF = 0;//先关闭会车区检测开关
	for(uint8_t i = 0; i < 8; i++)
	{
		if(huancount == cutIsland_set[i] && AI_Use[i] == 0)//是这个环岛 并且没有用过这次减速（颠簸清楚颠簸减速）
		{
			if(encoderCM > Island_distance[huancount] + cutCM_In[i] * 100 && encoderCM < Island_distance[huancount] + (cutCM_Out[i] + cutCM_In[i]) * 100)
			{
				//判断这段是颠簸还是坡道
				if(AI_Element[i] == 1)//检测坡道开关
					podaoF = 1;//打开坡道开关
				else if(AI_Element[i] == 2)//颠簸减速开关
				{
					dianboF = 1;//打开颠簸开关
					this_AIState = 1;//处于减速限制中
				}
				else if(AI_Element[i] == 3)//会车区检测开关
					huichequF = 1;//打开会车区检测开关
				else if(AI_Element[i] == 4)//纯粹减速开关
				{
					this_AIState = 1;//处于减速限制中
				}
			}
		} 
		else if(PanPoJishu == cutRamp_set[i] && AI_Use[i] == 0)//是这个坡道 并且没有用过这次减速（颠簸清楚颠簸减速）
		{
			if(encoderCM > Ramp_distance[PanPoJishu] + cutCM_In[i] * 100 && encoderCM < Ramp_distance[PanPoJishu] + (cutCM_Out[i] + cutCM_In[i]) * 100)
			{
				//判断这段是颠簸还是坡道
				if(AI_Element[i] == 1)//检测坡道开关
					podaoF = 1;//打开坡道开关
				else if(AI_Element[i] == 2)//颠簸减速开关
				{
					dianboF = 1;//打开颠簸开关
					this_AIState = 1;//处于减速限制中
				}
				else if(AI_Element[i] == 3)//会车区检测开关
					huichequF = 1;//打开会车区检测开关
				else if(AI_Element[i] == 4)//纯粹减速开关
				{
					this_AIState = 1;//处于减速限制中
				}
			}
		}
		else if(xuxianjilu == cutDashed_set[i] && AI_Use[i] == 0)//处于这个会车区之后
		{
			if(encoderCM > DashedField_distance + cutCM_In[i] * 100 && encoderCM < DashedField_distance + (cutCM_Out[i] + cutCM_In[i]) * 100)
			{
				//判断这段是颠簸还是坡道
				if(AI_Element[i] == 1)//检测坡道开关
					podaoF = 1;//打开坡道开关
				else if(AI_Element[i] == 2)//颠簸减速开关
				{
					dianboF = 1;//打开颠簸开关
					this_AIState = 1;//处于减速限制中
				}
				else if(AI_Element[i] == 3)//会车区检测开关
					huichequF = 1;//打开会车区检测开关
				else if(AI_Element[i] == 4)//纯粹减速开关
				{
					this_AIState = 1;//处于减速限制中
				}
			}
		}
	}
	
	//人工智能减速清楚的时候 防止猛加
	if(this_AIState == 0 && AI_State == 1)//这场不需要减速 上一场减速
	{
		AI_State = 2;//加速恢复阶段
		AI_RecoveryCnt = 0;//清空恢复
	}
	
	AI_State = this_AIState;//ai状态更新

	if(AI_State == 2)
	{
		if(AI_RecoveryCnt >= 10)
		{
			AI_State = 0;
		}
	}
	
	//大开关
	if(podaoF_set == 1)
		podaoF = 1;
	if(dianboF_set == 1)
		dianboF = 1;
	if(huichequF_set == 1)
		huichequF = 1;
}
uint8_t buzzer_num = 0;
void Buzzer_OnOff()//蜂鸣器处理
{
	//给几种不同的蜂鸣器叫的模式
	//0-不叫
	//编号：1  2  3  4  5  6  7  8  9  10
	//  1   1  1  1  1  1  1  1  1  1  1
	//  2   1  0  1  0  1  0  1  0  1  0
	//	3   1  1  0  1  1  1  1  0  1  1
	uint8_t buzzer_map[4][10] = 
	{
		{0,0,0,0,0,0,0,0,0,0},
		{1,1,1,1,1,1,1,1,1,1},
		{1,0,1,0,1,0,1,0,1,0},
		{1,1,1,1,1,0,0,0,0,0},
	};
	//蜂鸣器叫
	uint8_t buzzer_mode = 0;
	if(podao_flag>0 || podaoF > 0)//坡道
	{
		buzzer_mode = 1;
	}
	else if((dianboGeLi_count>0&&dianboGeLi_count<dianbofield) || AI_State > 0)//颠簸减速
		buzzer_mode = 2;
	else if(huichequF > 0)//会车区判断
		buzzer_mode = 3;
	else 
	{
		buzzer_mode = 0;
	}
	
	buzzer_num++;
	if(buzzer_num >= 10)
		buzzer_num = 0;	
	PAout(19) = buzzer_map[buzzer_mode][buzzer_num];
}
void SmartCar_Control()
{
	Get_AngleLine();//确定打角行
	
	Set_Element();//先确定赛道类型
	Artificial_Intelligence();//人工智能减速
	//舵机控制
	
	Get_Error();//确定偏差
	Get_ServoPD();//根据不同元素确定舵机pd
	Servo_Control();//舵机打角计算和控制
	//电机控制
	Speed_Calculate();//根据不同元素给出不同的设定速度和电机pd
	Motor_Control();//计算电机差速 电机闭环控制
	//调试
	Buzzer_OnOff();//蜂鸣器处理
}
