#include "control.h"
#include "NRF2401.H"
#include "Wireless.h"
#include "headfile.h"

//��������
uint8_t chose_dajiaoline = 0;//�����
uint8_t huan_line[8];//���������
uint8_t huancount=0;//��������
uint8_t ruhuan_line_add;//�뻷���ӵĴ����
uint8_t dajiao_line=0;//�趨�Ļ��������
uint8_t podao_flag=0;//�µ���־
int dianboGeLi_count=0;//��������
uint8_t dianbo_dajiaofield=0;//����һ��ʱ�����ƴ�ǳ����趨
uint8_t huan_ru_p[8];//�뻷p�趨
uint8_t huan_chu_p[8];//����p
uint8_t huan_in_p[8];//����p
double chose_p=0, chose_d=0;//���pd
float Long_Straight_P = 0.3, Long_Straight_D = 0.2;//��ֱ��pd
float Short_Straight_P = 0.6, Short_Straight_D = 0.5;//��ֱ��pd
float SS_P = 0.6, SS_D = 0.7;//СSpd
float Curve_P = 0.9, Curve_D = 1.9;//���pd
float Ramp_P = 0.6, Ramp_D = 0.4;//�µ�pd	
uint8_t Ahuiche_mend_p=0, Ahuiche_recover_p=0,Ahuiche_mend_d=0, Ahuiche_recover_d=0;//��ʼ�ᳵ������,�ָ���p d
uint8_t Bhuiche_mend_p=0, Bhuiche_recover_p=0,Bhuiche_mend_d=0, Bhuiche_recover_d=0;//�м�ᳵ������,�ָ���p d
uint8_t my_Element = 0, Last_my_Element = 0;//��������
uint8_t die_dangche_Flag=0;//�����ᳵ ֱ�ӳ��ȥ
uint8_t Realbegin_Count=0;//�����ᳵ �������֮����ȥ
int angleChange = 0;//���ֵ�仯�ĽǶ�
int image_speedset = 0;//���ݲ�ͬԪ�ظ������趨�ٶ�
int speed_longstraight = 450;//��ֱ���ٶ�
uint16_t speed_shortstraight = 420;
uint16_t speed_SS = 420, speed_ramp = 300, speed_normal = 360;//СS���µ������ �ٶ�
uint16_t motor_str_I = 12,motor_str_P  = 15;//���ֱ��pi
uint16_t motor_cur_I = 10,motor_cur_P = 10;//������pi
uint8_t RW_flag = 0;//�����������Ƿ���Ч
uint8_t Cut_Chang=0;//����ʣ�ೡ��
uint8_t Cut_finishFlag = 0;//������ɱ�־λ
uint8_t xiaopo_speed=0;//�����ٶ�
uint16_t huan_size[8];//��������
uint8_t CarFlag=2;//�ᳵʱǰ�� 0-ǰ�� 1-��
int MiddleStop_Init=0;//�м�ᳵ����ͣ����
int huichecount=0;//�м�ᳵͣ��������¼
int after_huiche_count=0;//�ᳵ֮��ǳ�
int EndStop_Init=0;//���ᳵ����ͣ����
int tingchecount=0;//���ᳵͣ��������¼
uint8_t have_stop=0;//���Ѿ�ͣ������
int stop_count=0;//���һ�λᳵ���֮��׼��ͣ���ǳ�
uint8_t RealStop_Count=0;//���ᳵ���֮�󼸳�ͣ���� �趨
uint8_t longtime_huiche_Flag=0;//˫��ģʽ һ����ʧ�ܱ��ɼ�����
uint8_t car1_lowspeed=0;//�ᳵ�󳵼��ٵĴ�С
uint8_t motor_stop=0;//ͣ����־λ
uint8_t dianbofield=0;//����֮��ռ�ձ�����޷������趨
int dianbospeed=0;//�����趨��ռ�ձ�
int init_dianji_go=0;//�������ʼռ�ձ�
uint8_t guancha=0;//�ᳵ����ʱֱ�� �ñ������ڼ��ǰ���Ƿ���ͣ��
uint8_t my_Curve_Flag = 0;//ȷ����������ʱʹ�õ�������
//�ᳵ��ر���
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

//���ñ���
//������
extern int Add_Pulse;//������ת�ٺ�
//����
extern uint8_t qi;//������
extern uint8_t dianbo ;//������־
extern int dianboGeLi;//��������
extern uint8_t sure_huan_after_dianbo;//ȷ�ϵ���֮��Ļ���
extern uint8_t dianbo_zhichong;//����ֱ���־λ
extern uint8_t dian_pan_huan;
//�µ�
extern int podaojichang;//�µ��ǳ�
extern int PanPoJishu;//�µ�����
extern uint8_t dian_pan_huan;//�����ȶ�״̬
//ͼ��
extern uint8_t RowNum,L_black[70],R_black[70],LCenter[70];//������
extern uint8_t L_init ,R_init ,R_init_record,L_init_record;//������ʼ��
extern uint8_t Last_Line;//�ض���
extern uint8_t mend_by_right;//���ƻ���������ȷ������
extern double Judge_Slope;//С�ݷ�б��
extern uint8_t Judge_Memory;//С�ݷ���Ч����
extern uint8_t fixvalue[70];//�����������
extern int CNT;//֡����¼
extern uint8_t Cross_Xie_Out_i, Cross_Zhong_Out_i, Cross_Zhi_Out_i;//ʮ�ֱ��
extern uint8_t xiaopo_jiansu;//����
extern uint8_t small_s;//СS
extern uint8_t SS_needCut;//СS��Ҫ����
extern uint8_t small_sCanGo;//СS���Կ�ʼ��־
//����
extern int image_error, image_derror, image_lasterror;//ƫ��
extern double MotorFuzzyLP, MotorFuzzyLD, MotorFuzzyRP, MotorFuzzyRD;//����ó��ĵ��pd
//�ᳵ��
extern uint8_t mend_by_xuxian;//�ᳵ������
extern uint8_t mend_recover;//�ᳵ���ڻָ��׶�
extern uint8_t xuxianjilu;//�ڼ����жϵ��ᳵ�� �м�ᳵ��1
extern uint8_t podaoF;//�µ�����
extern uint8_t dianboF;//��������
extern uint8_t huanline_second;
extern  uint8_t youDaSi;
uint8_t huichequF = 0;//�ᳵ����⿪��
uint8_t huichejuli_cut=0;
int zhidao_huiche_count=0; 
uint8_t last_CarFlag=2;
extern uint8_t Judge_Start;//��ʼ����
extern uint8_t SS_dajiao_line;
extern uint8_t Car;
//һ�ξ������
//����
//һ�ξ������
//�˵�
uint8_t AI_Element[8]={1,2,1,2,0,0,0,0};//1�����µ� 2������� 3�ᳵ�� 4�������� 0�ر�

uint8_t AI_Element_set[8] = {0,0,0,0,0,0,0,0};//�˹�������������Ԫ��֮���¼�� 0-���� 1-�µ� 2-�ᳵ��
uint8_t AI_num_set[8] = {0,0,0,0,0,0,0,0};//�˹����ܵ�Ԫ�������趨
uint8_t cutCM_In[8] = {2,1,7,0,0}, cutCM_Out[8] = {4,4,4,0,0};//���ٵ�·��

uint8_t podaoF_set = 0, dianboF_set = 0, huichequF_set = 0;//�󿪹� ��������AI����

uint8_t cutIsland_set[8] = {10,10,10,10,10,10,10,10}, cutRamp_set[8] = {10,10,10,10,10,10,10,10}, cutDashed_set[8] = {10,10,10,10,10,10,10,10};
uint8_t AI_CutCnt = 0;//�ڼ�������

uint8_t AI_Use[8] = {0,0,0,0,0,0,0,0};//�����Ƿ��Ѿ�ʹ�ù�����˹����ܼ���

//uint16_t AI_SpeedSet[5] = {100,100,100,10,10};//���ٵ��ٶ�
uint64_t Island_distance[9] = {0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000};//ÿ���������ڵľ���
uint64_t Ramp_distance[9] = {0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000};//ÿ���µ����ڵľ���
uint64_t DashedField_distance = 100000;//���ߵľ���
//״̬
uint8_t AI_State = 0;//0-���� 1-����������
uint8_t last_huancount = 0;//��¼��һ�λ������� �ж��ǲ����л��������ı仯
uint8_t last_PanPoJishu = 0;//��¼��һ���µ�����
uint8_t last_dianbo = 0;
int64_t encoderCNT = 0;//������������
uint16_t encoderCM = 0;//ʵ�ʻ�����������������ף�
uint16_t AI_Speed = 100;//���ٵ��ٶ�
uint8_t AI_RecoveryCnt = 0;//�ǳ��ָ� 10���ָ�������
uint8_t AI_Distance_By = 0;//0-���� 1-�µ�

uint8_t Cut_AfterSS = 0, Cut_AfterIsland = 0;//СS������ ֮�����
uint8_t CurveFirst_ByIsland = 0;//����ֱ������

uint8_t wandao_huiche_flag=0;

//����
extern int Mabs(int a,int b);//ȡ����ֵ���ĺ���
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

//������Ʋ���
void Get_AngleLine()//ȷ�������
{
	//��ȷ�������Ļ��������
	if((zuohuan == 2 || youhuan == 2)&&huanline_second>0)//�뻷
	{
		chose_dajiaoline = huan_line[huancount] - ruhuan_line_add;
	}
	else if(zuohuan == 2 || youhuan == 2||zuohuan == 6 || youhuan == 6 || zuohuan == 3 || youhuan == 3 || youhuan == 5 || zuohuan == 5)//���ںͳ���
	{
		chose_dajiaoline=huan_line[huancount];
	}
	//��������̬�����
	if(zuohuan == 2 || zuohuan == 3 || zuohuan == 5 || zuohuan == 6
	|| youhuan == 2 || youhuan == 3 || youhuan == 5 || youhuan == 6)//���ݵ�ǰ�ٶ�ȷ�������
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
	else //����֮����������
	{
		//�������ֺ��ٶȲ��ö�̬�����
		if(qi == 0 || (L_black[0]-R_black[0] > 150) || L_init >= 20 || R_init >= 20)//���ǵ��� ���ߵ���ʹͷ�Ѿ������� ����������ʼ����20��
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
		chose_dajiaoline = dajiao_line;	//�����������о����趨�Ĵ����
//	 if(after_huiche_count<=50&&after_huiche_count>0)
//	 {
//		 chose_dajiaoline += Judge_Start;
//	 }
//	 else 
		if(L_init>0&&R_init>0&&podao_flag==0&&dianbo==0)//�����0������ȫ�� ̫�ߴ����
		chose_dajiaoline +=2;
		if(podao_flag==2)//�µ����⴦������
		{
			if(Last_Line>=5)//�����ֹ��С��5 ����������ƫ��ʱ ʹƫ������
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
				if(chose_dajiaoline>y)//y�Ǹ���ͼ��ȷ����������ߵ���Ч�� �������������������
				chose_dajiaoline=y;
			}
		}
		
   	if(dianboGeLi_count>0&&dianboGeLi_count<40&&dian_pan_huan==0)//�������⴦������ �趨һ���������ƴ��
		{
			 uint8_t x=0;
				 for(x=0;x<Last_Line*0.6;x++)
			{
					if(L_black[x]<185&&R_black[x]>0)//��³�ж���һ���ǲ�����ʵ��
					{
							double beishu = (double)(L_black[x] - R_black[x]) / fixvalue[x];
						  chose_dajiaoline=chose_dajiaoline*beishu;
							//setText�û��Զ���("�������������ϵ��" + beishu);
							break;
					}
			}
   }
	 }
	 
	
	if(Last_Line<=chose_dajiaoline)//��ֹ����и��ڽ�ֹ��
	 {
		 chose_dajiaoline=Last_Line;
	 }
}

void Get_Error()//ȷ��ƫ��
{
	if(Last_Line > 5)//�ض��б�5��
		image_error = 92 - LCenter[chose_dajiaoline];
	else if(podao_flag==2||AI_State==1)//�µ�2�׶� ��ֹ�й����򱣳ִ��
	  image_error = 0;
	else 
	  image_error = image_lasterror;
	if(youDaSi==1)//���м�ᳵ������ʱ
	{
    image_error =92;		
	}
	

	//����derror�ͼ���error
	image_derror=image_error-image_lasterror;
	image_lasterror=image_error;		
}

void Get_ServoPD()//���ݲ�ͬԪ��ȷ�����pd
{
	if((zuohuan==2||youhuan==2)&&huanline_second>0)//�뻷
	{
		chose_p=huan_ru_p[huancount]/10.0;	
    chose_d=D/10.0;		 
	}
	else if(zuohuan==6||youhuan==6||youhuan==5||zuohuan==5)//����
	{
    chose_p=huan_chu_p[huancount]/10.0;	
    chose_d=D/10.0;		 
	}
	else if(zuohuan == 2 || youhuan == 2||zuohuan==3||youhuan==3)//����
	{
		chose_p=huan_in_p[huancount]/10.0	;
		chose_d=D/10.0;
	}
	else if(zuohuan==7||youhuan==7)//���������
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
	else//������������
	{
		if(my_Element == Long_Straight)//��ֱ��
		{
			chose_p = Long_Straight_P/10.0;
			chose_d = Long_Straight_D/10.0;
		}
		else if(my_Element == Short_Straight)//��ֱ��
		{
			chose_p = Short_Straight_P/10.0;
			chose_d = Short_Straight_D/10.0;
		}
		else//���
		{
			if(my_Element == Curve_First)//ֱ�������
			{
				chose_p = Curve_P/10.0 ;
				chose_d = Curve_D/10.0 ;
			}
			else if(my_Element == Curve_Second)//����
			{
				chose_p = Curve_P/10.0;
				chose_d = Curve_D/10.0;	
			}
			else//���
			{
				chose_p = Curve_P/10.0;
				chose_d = Curve_D/10.0;
			}
		}
		/*                 ����Ԫ��                */
		if(my_Element == SS || my_Element == SS_Cut)//Сs  
		{
			chose_p = SS_P/10.0;
			chose_d = SS_D/10.0;
		}	
		else if(my_Element == Ramp)//�µ�
		{
			chose_p = Ramp_P/10.0;
			chose_d = Ramp_D/10.0;
		}		
		//�ᳵ
		if(mend_by_xuxian>0)//�ᳵ������֮��
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
		if(mend_recover==1)//�ᳵ���߻ָ���6��
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
		if(podaojichang>0&&podaojichang<=10)//���µ���δ�ָ�����ʱ ͼ����ܻ����¶��� ��Сd ����Ӱ��
		{
			chose_d /=2;
		}
	}
}

void Servo_Control()//�����Ǽ���Ϳ���
{
	if (Last_Line>5||podao_flag==2)//�ض��д���5
		duoji_InitalDuty=duojiZhongZhi - image_error * chose_p - image_derror * chose_d;//���pd��ʽ
	else if(AI_State==1)
		duoji_InitalDuty=duojiZhongZhi;
	if(podao_flag==1&&Last_Line<30)//���µ����¹����޷� ǰ�����о��ԵĶ���
	{
		if(duoji_InitalDuty>duojiZhongZhi+30)
			duoji_InitalDuty=duojiZhongZhi+30;
		else if(duoji_InitalDuty<duojiZhongZhi-30)
			duoji_InitalDuty=duojiZhongZhi-30;
	}
	else if(podao_flag==2&&Last_Line<10)//���м�����޷�
	{
		if(duoji_InitalDuty>duojiZhongZhi+20)
			duoji_InitalDuty=duojiZhongZhi+20;
		else if(duoji_InitalDuty<duojiZhongZhi-20)
			duoji_InitalDuty=duojiZhongZhi-20;
	}
	else if(podao_flag==3&&Last_Line<50)//���µ����¹����޷�
	{	 
		if(duoji_InitalDuty>duojiZhongZhi+30)
			duoji_InitalDuty=duojiZhongZhi+30;
		else if(duoji_InitalDuty<duojiZhongZhi-30)
			duoji_InitalDuty=duojiZhongZhi-30; 		 
	}
//	if(dianboGeLi>0&&zuohuan==0&&youhuan==0&&sure_huan_after_dianbo==1&&dianboGeLi_count<=25)//ͼ��������������� �����޷����ݵ������ڵ�ʵ��λ�����ж� ����ڻ���ǰ�������г���֮ǰҪ���ƴ�� ��������ǰ������Ҫ
//	{
//		if(duoji_InitalDuty>duojiZhongZhi+20)
//			duoji_InitalDuty=duojiZhongZhi+20;
//		else if(duoji_InitalDuty<duojiZhongZhi-20)
//			duoji_InitalDuty=duojiZhongZhi-20;
//	} 
//	if(dianbo_zhichong==1)//��������Ҫֱ��
//	{
//		duoji_InitalDuty=duojiZhongZhi;
//	}
	 
	if(die_dangche_Flag==1&&CNT>Realbegin_Count&&dangche==2)//˫���Ա�ֱ��
	{
		duoji_InitalDuty=duojiZhongZhi;
	}
	servo_Flush();//���¶��ֵ
	angleChange = duoji_InitalDuty - duojiZhongZhi;//������ת���Ľ��� ���ڲ��ٿ���
 
	
}

//������Ʋ���
void Speed_Calculate()//���ݲ�ͬԪ�ظ�����ͬ���趨�ٶȺ͵��pd
{
	if(my_Element == Long_Straight )//��ֱ��
	{	
		image_speedset = speed_longstraight;
		MotorFuzzyLP = motor_str_I ;
		MotorFuzzyLD = motor_str_P;
		MotorFuzzyRP = motor_str_I ;
		MotorFuzzyRD = motor_str_P;
	}
	else if(my_Element == Short_Straight )//��ֱ��
	{
		image_speedset = speed_shortstraight;
		MotorFuzzyLP = motor_str_I ;
		MotorFuzzyLD = motor_str_P;
		MotorFuzzyRP = motor_str_I ;
		MotorFuzzyRD = motor_str_P;
	}
	else if (my_Element == SS)//СS
	{
		image_speedset = speed_SS;
		if(small_s == 2)
		image_speedset = speed_normal;
		MotorFuzzyLP = motor_str_I ;
		MotorFuzzyLD = motor_str_P;
		MotorFuzzyRP = motor_str_I ;
		MotorFuzzyRD = motor_str_P;
	}
  else if(my_Element == Ramp)//�µ�
	{
		image_speedset = speed_ramp ;
		MotorFuzzyLP = motor_str_I ;
		MotorFuzzyLD = motor_str_P;
		MotorFuzzyRP = motor_str_I ;
		MotorFuzzyRD = motor_str_P;
	}	
	else 
	{
		if(my_Element == Curve_First || my_Element == SS_Cut)//���� �� СS����
		{
			//������ټ��˼·
			//���������һ�������Ĺ��� �����������ֻ�ܽ���һ�μ��� �������֮���ٶȱ����ڣ��趨�ٶ�-3���Ĵ�С
			//������ٵ���ʼ������������һ���ٶȵ���趨�ٶ�-40��
			//���ݴ����ȷ�����ٵĳ��������ݳ����Ķ��ٸ�����ͬ��Ŀ���ٶ�
			//ֻ��֮ǰ�ǳ���ֱ����ʮ�ֲ���һ����Ч��������ٽ׶�
			
			//������ǵ�һ���жϵ����� ����ǲ�����Ч������
			if(Last_my_Element != Curve_First)
			{
				if(Last_my_Element == Short_Straight || Last_my_Element == Long_Straight || (Cross_Xie_Out_i > 0 || Cross_Zhong_Out_i > 0 || Cross_Zhi_Out_i > 0))//ʮ�ֿ���Ӱ��֮ǰ���б� ����ʮ�����ֱ��
				{
					RW_flag = 1;//��������һ����Ч���������
				}
				else if(Cut_AfterSS == 1 && Last_my_Element != Curve_First)//СS����
				{
					RW_flag = 1;//��Ч�������
				}
				else if(Cut_AfterIsland == 1 && Last_my_Element != Curve_First)//���������
				{
					RW_flag = 1;//��Ч�������
				}
				else if(CurveFirst_ByIsland == 1)//���ֱ�����Ǹ��ݻ����жϵ���
				{
					RW_flag = 1;//��Ч�������
				}
			}
			if(my_Element == SS_Cut && Last_my_Element != SS_Cut)//СS����
				RW_flag = 1;//��Ч�������
			
			//��������Ч��������ٵĻ����ϴ��� ��Ȼ�͸������ٶ�
			if(RW_flag == 1)//����һ����Ч���������
			{
				if(Cut_Chang == 0 && Cut_finishFlag == 0)//��ǰ�����ڼ��ٹ����� ����֮ǰû�����һ�μ���
				{
					//�ж��ǲ��ǿ��Կ�ʼ����
					if(Speed_Left + Speed_Right > speed_normal * 2 - 60)//���Կ�ʼ����
					{
						//ȷ�����ٳ�
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
				
				//�жϼ����Ƿ��Ѿ��㹻���ͽ���������
				if(Speed_Left < (speed_normal - 40) && Speed_Right < (speed_normal - 40) && Cut_Chang > 1)//����ת�ٶ���С
				{
					Cut_Chang = 1;//�ټ���һ���ͺ���
				}
				
				//����ȷ���ļ��ٳ����м���
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
					if(Cut_Chang == 0)//��ɼ���
						Cut_finishFlag = 1;
				}
				else if(Cut_finishFlag == 1)//�Ѿ���ɼ��� ���趨�ٶ�-3
				{
					image_speedset = speed_normal - 3;
					MotorFuzzyLP = motor_cur_I;
					MotorFuzzyLD = motor_cur_P;
					MotorFuzzyRP = motor_cur_I;
					MotorFuzzyRD = motor_cur_P;
				}
				else//��û�п�ʼ�������
				{
					image_speedset = speed_normal;
					MotorFuzzyLP = motor_cur_I;
					MotorFuzzyLD = motor_cur_P;
					MotorFuzzyRP = motor_cur_I;
					MotorFuzzyRD = motor_cur_P;
				}
			}
			else//������������Ч ���趨�ٶ�
			{
				image_speedset = speed_normal;
				MotorFuzzyLP = motor_cur_I;
				MotorFuzzyLD = motor_cur_P;
				MotorFuzzyRP = motor_cur_I;
				MotorFuzzyRD = motor_cur_P;
			}
		}
		else if(my_Element == Curve_Second)//����
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
	if(my_Element != Curve_First && my_Element != SS_Cut)//��������־
	{
		Cut_Chang=0;
		RW_flag =0;
	}
	
	//��������Ԫ�ص��ٶ�
	if(xiaopo_jiansu>0)//�����ٶ�
	{
		image_speedset=xiaopo_speed;								
	}
	else if(my_Element == Island//�ų������Լ��ٵĻ���ǰֱ���ͼ��ٽ׶�
	   && ((zuohuan>=3&&zuohuan<=5)||(youhuan>=3&&youhuan<=5)))
	{
		//if(huancount<2)
		image_speedset=huan_size[huancount];
		//else 
		//	image_speedset-=huan_size[huancount];
	}
	/*********                  ˫���ٶȲ���            **********/ 
  if(dangche==0)
	{			
		/*             ��ʼ������������              */		
		if(CNT<Realbegin_Count)				
		{
			image_speedset=speed_normal;
		}	
		if(xuxianjilu==1)//�м�ᳵ����
		{
			if(CarFlag==0)//��Ϊǰ��
			{
				if(already_huiche==0&&huiche!=2)//ǰ������ͣ �󳵻�û��
				{
					double cutP=(double)(speed_normal)/MiddleStop_Init;//�����趨������ȷ��p �趨������΢�������ʹǰ���ܹ���΢Զ
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
				else if(already_huiche==0&&huiche==2)//���Ѿ�����  ����ᳵ ǰ���ȴ��ᳵ����������
				{
					if(wandao_huiche_flag==1)
						image_speedset=0;           //����ᳵ ǰ���ȴ��ᳵ����������
				}	
			}
			else if(CarFlag==1)//��ȫ����ֻ���������ٴ���-car1_lowspeed
			{
				image_speedset=speed_normal;					
			}
			
			if(wandao_huiche_flag==0)//ֱ���ᳵǰ����������
			{
				if(zhidao_huiche_count>0&&zhidao_huiche_count<=15)//ǰ���ָ���������
				{	
					double addP=(double)(speed_normal)/15;//�����趨�ӳ���ȷ��p
					image_speedset=addP*zhidao_huiche_count;		
				}
			}
			else if(wandao_huiche_flag==1)//����ᳵ
			{
				if(last_CarFlag==0)//��λǰ��ǰ��
				{
					if(after_huiche_count>0&&after_huiche_count<=15)//ǰ���ָ���������
					{	
						double addP=(double)(speed_normal)/15;//�����趨�ӳ���ȷ��p
						image_speedset=addP*after_huiche_count;		
					}
				}
			}	
		}
		if(xuxianjilu==2)//���ᳵ����
		{
			if(CarFlag==0)//��Ϊǰ��
			{
				if(Stop_Line_Huiche==0&&huiche!=2)//ǰ������ͣ 
				{
					double cutP=(double)(speed_normal)/EndStop_Init;//�����趨������EndStop_Initȷ��p �趨������΢С ��ʹǰ��ͣ��������ǰ��
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
				else if(Stop_Line_Huiche==0&&huiche==2)//���Ѿ����� ǰ���ȴ��󳵽ӽ�������
				{
					image_speedset=speed_normal;		

				}			
				else if(Stop_Line_Huiche==2)//��ʼͣ��
				{
					stop_count++;
					if(stop_count>RealStop_Count)
					{
						image_speedset=0;
					}
				}
			}
			else if(CarFlag==1)//��ȫ����ֻ���������ٴ���
			{
				if(Stop_Line_Huiche==1)//�󳵲�������
				{
					image_speedset=speed_normal-car1_lowspeed;
				}		
				else if(Stop_Line_Huiche==2)//��ʼͣ��
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
	//˫���ᳵ һ������������ ��һ���������� �е��ڶ�������ͣ��
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
	
	//AI������
	if(AI_State == 1 && CarFlag == 2)//ai_stateֻ�ڼ��ٵ�ʱ����1 ���ʱ�򲻻��1
	{
		if(AI_Speed < image_speedset)
			image_speedset = AI_Speed;
	}
	else if(AI_State == 2 && CarFlag == 2)//���ٻָ�
	{
		AI_RecoveryCnt++;
		if((AI_Speed * (10 - AI_RecoveryCnt) / 10 + image_speedset * AI_RecoveryCnt / 10) < image_speedset)
			image_speedset = AI_Speed * (10 - AI_RecoveryCnt) / 10 + image_speedset * AI_RecoveryCnt / 10;
	}
	

	
	//���ֱ���
	if(motor_stop==1&&(protect_flag==1||die_dangche_Flag==1))
	{
		image_speedset=0;
    if(dangche==0)
		{
			NRF2401_SetMode(TX);
			Wireless(TX,10);			
		}
	}
	if(motor_stop==2)//˫��ģʽ�� ��һ���������� ������Ҳ����ͣ
	{
		image_speedset=0;
	}	 
}

void Motor_Control()//���������� ����ջ�����
{
	//������ټ���
	//������Ҫ���ٶ����� ȷ���Ƿ������
	if(image_speedset == 0)//ͣ��ʱ��������
	{
		aimzuo = 0;
		aimyou = 0;
	}
	else if(my_Element == Short_Straight || my_Element == Long_Straight || my_Element == SS)//ֱ��СS��������
	{
		aimzuo = image_speedset;
		aimyou = image_speedset;
	}
	else if(my_Element == Ramp)//�µ���������
	{
		aimzuo = image_speedset;
		aimyou = image_speedset;
	}
	else//����������
	{
		if(my_My_Abs1(angleChange) < 18)
		{
			aimzuo = image_speedset;
			aimyou = image_speedset;
		}
		else if(my_My_Abs1(angleChange) < 24)
		{
			if(angleChange > 0)//��ת
			{
				aimzuo = image_speedset / 1.5;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//��ת
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.5;
			}
		}
		else if(my_My_Abs1(angleChange) < 32)
		{
			if(angleChange > 0)//��ת
			{
				aimzuo = image_speedset / 1.55;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//��ת
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.55;
			}
		}
		else if(my_My_Abs1(angleChange) < 43)
		{
			if(angleChange > 0)//��ת
			{
				aimzuo = image_speedset / 1.6;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//��ת
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.6;
			}
		}
		else if(my_My_Abs1(angleChange) < 53)
		{
			if(angleChange > 0)//��ת
			{
				aimzuo = image_speedset / 1.65;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//��ת
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.65;
			}
		}
		else if(my_My_Abs1(angleChange) < 67)
		{
			if(angleChange > 0)//��ת
			{
				aimzuo = image_speedset / 1.7;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//��ת
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.7;
			}
		}
		else if(my_My_Abs1(angleChange) < 75)
		{
			if(angleChange > 0)//��ת
			{
				aimzuo = image_speedset / 1.75;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//��ת
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.75;
			}
		}
		else
		{
			if(angleChange > 0)//��ת
			{
				aimzuo = image_speedset / 1.8;
				aimyou = image_speedset;
			}
			else if(angleChange < 0)//��ת
			{
				aimzuo = image_speedset;
				aimyou = image_speedset / 1.8;
			}
		}
	}
	
	if(Last_my_Element != Curve_First && my_Element == Curve_First/* && RW_flag == 1*/)//�����޷����� Ҫ���ǵ�һ���ж�Ϊ������١���Ч����
	{
		if(motor_left > 8000 || motor_right > 8000)
		{
			motor_left = 5000;
			motor_right = 5000;
		}
	}	
	Encodercontrol_motor();//���pid���㼰�޷�	
	if(guancha == 1)//˫����ǰ��ͣ��������ʱֱ��
	{
		if(LPulse+Dulse<900&&have_stop==1)
		{
			motor_left = init_dianji_go;
			motor_right = init_dianji_go;
		}
	}
	if(podao_flag>=2)//�м����ʱ��ռ�ձ� ��ֹӰ����
	{
		motor_left=3000;
		motor_right=3000;
	}	
	/*                          ͣ��ʱ��ֹ����                    */
	if(aimzuo==0&&aimyou==0&&LPulse<80&&Dulse<80)//�м�ᳵ������
	{	
		motor_left=0;
		motor_right=0;
	}
	if(aimzuo==0&&aimyou==0&&LPulse+Dulse<20&&xuxianjilu==2)//���ᳵ������
	{	
		motor_left=0;
		motor_right=0;
	}
	//�ı�ռ�ձ�
	motor_flush();
}



//����ͼ��ȷ����������
uint8_t last_cutLine = 30;
void Set_Element()
{
	#define byte uint8_t
	
	uint8_t cutLine = 30;//������ ��ֱ����ֱ������ж���
  if(chose_dajiaoline > 18)//���������� �ܹ���Ч����
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
	if(my_Element == Curve_First)//�����һ����⵽ֱ���� ά����һ�εļ����� ��ֹ״̬�л�
	{
		if(cutLine < last_cutLine)
			cutLine = last_cutLine;
	}
	last_cutLine = cutLine;//����
	
//	Judge_LStart = L_init;
//	Judge_RStart = R_init;
	//��ʾ���ڱ궨
	//OLED_Write_Num2(0, 0, Judge_Memory);
	//OLED_Write_Num3(0, 2, (int)(Judge_Slope * 100));
	
	//��ʼ��
	CurveFirst_ByIsland = 0;//����ֱ������
	
	//��������
	Last_my_Element = my_Element;
	if (zuohuan > 0 || youhuan > 0)//����10�׶���Ϊ��ֱ�� 1�׶���Ϊֱ����
	{
			if(zuohuan == 10 || youhuan == 10)//��ֱ��
					my_Element = Short_Straight;
			else if(zuohuan == 1 || youhuan == 1)//ֱ����
			{
					my_Element = Curve_First;
					CurveFirst_ByIsland =1;//����ֱ������
			}
			else
					my_Element = Island;
			//text�û��Զ���("����");
	}
	else if (podao_flag != 0)
	{
			my_Element = Ramp;
			//text�û��Զ���("�µ�" /*+ Ramp_Process*/);
	}
	else if (small_s > 0 && small_sCanGo == 1)
	{
			my_Element = SS;
			if(SS_needCut == 1)
			{
				my_Element = SS_Cut;//СS����
			}
			//text�û��Զ���("СS");
	}
	else if (my_Curve_Flag == 0 && Judge_Memory > 35/*40*/ && Judge_Slope < 0.6 && ((Judge_LStart < 25 && Judge_RStart < 25) || (Cross_Xie_Out_i > 0 || Cross_Zhong_Out_i > 0 || Cross_Zhi_Out_i > 0)))//ֱ���������Ҷ�����ʼ�е�����
	{
			if (podaojichang == 0 || podaojichang > 10 || dianbo > 0)
					my_Element = Long_Straight;
			else
					my_Element = Curve_Third;
			//text�û��Զ���("��ֱ��");
	}
	else if (my_Curve_Flag == 0 && Judge_Memory <= /*40*/35 && Judge_Memory > /*35*//*30*/cutLine && Judge_Slope < 0.8 && ((Judge_LStart < 25 && Judge_RStart < 25) || (Cross_Xie_Out_i > 0 || Cross_Zhong_Out_i > 0 || Cross_Zhi_Out_i > 0)) && (podaojichang == 0 || podaojichang > 10))
	{
			if (podaojichang == 0 || podaojichang > 10 || dianbo > 0)//���º�10������ֱ��
					my_Element = Short_Straight;
			else
					my_Element = Curve_Third;
			//text�û��Զ���("��ֱ��");
	}
	else if (my_Curve_Flag == 0 && Judge_Memory <= /*35*//*30*/cutLine && Judge_Slope < 0.8)
	{
			my_Element = Curve_First;
			//text�û��Զ���("ֱ����");
	}
	else if (my_Curve_Flag == 1 /*&& Judge_Slope >= 0.5*/ && Judge_Slope < 1.8 && Judge_Memory > 30)
	{
			my_Element = Curve_Second;
			//text�û��Զ���("����");
	}
	else
	{
			my_Element = Curve_Third;
			//text�û��Զ���("���");
	}	
	//СS֮����ٵ����⴦��
	if(my_Element == Curve_Third && Last_my_Element == SS)//����СS�����ж�
	{
		if(Speed_Left + Speed_Right > speed_normal * 2 - 60)
		{
			Cut_AfterSS = 1;
		}
	}
	//����֮����ٵ����⴦��
	if(my_Element == Curve_Third && Last_my_Element == Island)
	{
		if(Speed_Left + Speed_Right > speed_normal * 2 - 60)
		{
			Cut_AfterIsland = 1;
		}
	}
	
	if(Cut_AfterSS == 1 || Cut_AfterIsland == 1)//����СS������֮����ٵĹ�����
	{
		if(Speed_Left < speed_normal - 20 && Speed_Right < speed_normal - 20)//����ٶ��Ѿ������� ���Ը������ٶȲ���
		{
			Cut_AfterSS = 0;
			Cut_AfterIsland = 0;
		}
		else if(my_Element == Short_Straight || my_Element == Long_Straight || my_Element == SS)//�����ֱ������СS ����СS֮�����
		{
			Cut_AfterSS = 0;
			Cut_AfterIsland = 0;
		}
		else if(my_Element == Island || my_Element == Ramp)//���� �µ� ����Ԫ�����
		{
			Cut_AfterSS = 0;
			Cut_AfterIsland = 0;
		}
		
		if(Cut_AfterSS == 1 || Cut_AfterIsland == 1)
			my_Element = Curve_First;//��ֱ�������
	}
	
	

	//�����־��������ж�
	if (my_Element == 11 || my_Element == 12)//�ⳡ��������߳���
	{
			my_Curve_Flag = 1;
	}
	if (my_Element == 11 && Judge_Slope > -0.3 && Judge_Slope < 0.3)//����������¼�������־�����
	{
			byte judge_row = 0; float judge_k = 0;//��������ϵ��
			judge_row = (byte)My_Max(Judge_LStart, Judge_RStart);
			if (Cross_Xie_Out_i > 0 || Cross_Zhong_Out_i > 0 || Cross_Zhi_Out_i > 0)
					judge_row = 0;
			if (judge_row < 20)
					judge_row = 20;
			judge_k = (float)my_My_Abs1(LCenter[judge_row] - 92) / (float)fixvalue[judge_row];
			if (judge_k < 0.1f)//���
			{
					my_Curve_Flag = 0;
					//���������������� �͸�Ϊ��ֱ��
					my_Element = Short_Straight;
					//settext�û��Զ���("���������� ��ֱ��");
			}
			else if(judge_k < 0.2f)//�ٽ���һ�е��ж�
			{
					judge_row = 15;
					if(judge_row < Judge_LStart && judge_row > Judge_RStart)
					{
							judge_k = (float)my_My_Abs1(LCenter[judge_row] - 92) / (float)fixvalue[judge_row];
							if (judge_k < 0.1f)//���
							{
									my_Curve_Flag = 0;
									//���������������� �͸�Ϊ��ֱ��
									my_Element = Short_Straight;
									//settext�û��Զ���("���������� ��ֱ��");
							}
					}
			}
	}
	else if (my_Element == 14 || my_Element == 15 || my_Element == 5)//����Ԫ�����������
	{
			my_Curve_Flag = 0;
	}
	//����ֱ������ٱ�־
	if (my_Element != Curve_First && my_Element != SS_Cut)
	{
	    RW_flag = 0;
	    Cut_Chang = 0;
	    Cut_finishFlag = 0;
	}
	if(small_s == 0)
		SS_needCut = 0;//��ձ�־λ
}

//С�������ܺ���
//�ᳵ�߼�
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
			after_huiche_count++;//���ڵڶ����жϻᳵ������ʱ ��ֹ���м�ᳵ�����ֶ����ж�		
		}	
	}
	if(dangche==0&&die_dangche_Flag==0)
	{			
		/*               �м�ᳵ                               */
		if(xuxianjilu==0||tongshi_huiche==1)//��һ������ǰ��
		{
			if(Wireless(RX,0)==2 && already_huiche ==0)//�յ�2˵������ǰ������
			{
				already_huiche = 1;
			}				
		}				
		if(xuxianjilu==1)//�����м�ᳵ��
		{
			if(CarFlag==2)//ȷ��ǰ�� �м�ᳵ��already_huicheһֱ��2 ������ǰ�����Ǻ� һ���������� ��Ҫ����һ����������Ϣ
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
			if(CarFlag==0&&already_huiche==0)//ǰ��
			{
				if(huiche==0)//ǰ��ͣ�����ͻᳵ�ź�
				{					
					Wireless(TX,2);							
					/******* �м�ᳵ�ȴ�ʱ����� �ж���ʱ���ѳ�����  ********/
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
        if(huiche==0&&Car==3&&tongshi_huiche==0)		//���ǰ��û�з��ͳɹ� ��ʱ�Ƴ� �������ﵽһ��ֵʱ ���������� �����������һֱС��2�� ��ô�ж���ʱ������ͬʱ����
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
        if(huiche	==1)//�ȴ���ʶ������		
				{									
					/******* �м�ᳵ�ȴ�ʱ����� �ж���ʱ���ѳ�����  ********/
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
					if(Wireless(RX,0)==4)//��ʱ��ʶ������
					{
						huiche	= 2 ;
					}	
          else if(Wireless(RX,0)==2&&Car==3&&send_fail_flag==1)//ͬʱʶ������
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
				if(huiche==2&&tongshi_huiche==0)//���Ѿ�ʶ�������
				{	
					send_fail_flag=0;
          send_fail_count=0;	
					if(test==0)
						houche_huiche_count++;	//�������������ֹ�������Բ��� ����ͻȻ��������� ������������ᳵ���Ҿ���һ��ʱ�����Զ���λ		
					
					/*  ��������ᳵ  ���Ļᳵ��λ�߼�  */
					if(wandao_huiche_flag==1)//����ᳵ
					{
						if(houche_huiche_count>houche_huiche_Menu_count||
							(last_distance[2]>=last_distance[1]&&last_distance[1]>=last_distance[0]&&last_distance[0]>=uwb_Distance&&last_distance[2]>uwb_Distance&&uwb_Distance<huiche_Menu_uwb))
								huichejuli_cut=1;
						if(huichejuli_cut==1)
						{
							if((last_distance[2]<=last_distance[1]&&last_distance[1]<=last_distance[0]&&last_distance[0]<=uwb_Distance&&last_distance[2]<uwb_Distance)
								||houche_huiche_count>houche_huiche_Menu_count)//uwb�����𽥼�С���˵��趨��Сֵ������
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
					else if(wandao_huiche_flag==0)//ֱ���ᳵ
					{	
						/*  ֱ���ᳵ�ᳵ��λ�߼�  ����ӽ� �ᳵ��λ  */
						if((huichejuli==0&&last_distance[2]>=last_distance[1]&&last_distance[1]>=last_distance[0]&&last_distance[0]>=uwb_Distance&&last_distance[2]>uwb_Distance
							&&last_distance[0]<100&&uwb_Distance<huiche_Menu_uwb)||houche_huiche_count>houche_huiche_Menu_count)//����uwb��ȷ�����Ƿ��ѿ���ǰ�� ����������С
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
			if(CarFlag==1&&already_huiche==1)//��
			{	
				if(huiche==0)//һ��ʼʶ������ ��ǰ����4 ˵�������г�����
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
							houche_huiche_count++;	//�������������ֹ�������Բ��� ����ͻȻ��������� ������������ᳵ���Ҿ���һ��ʱ�����Զ���λ		
					if(wandao_huiche_flag==1)//����ᳵ
					{		
						/*  ��������ᳵ  ���Ļᳵ��λ�߼�  */
						if(houche_huiche_count>houche_huiche_Menu_count
							||(last_distance[2]>=last_distance[1]&&last_distance[1]>=last_distance[0]&&last_distance[0]>=uwb_Distance&&last_distance[2]>uwb_Distance&&uwb_Distance<huiche_Menu_uwb))
								huichejuli_cut=1;
						if(huichejuli_cut==1)
						{
							if((last_distance[2]<=last_distance[1]&&last_distance[1]<=last_distance[0]&&last_distance[0]<=uwb_Distance&&last_distance[2]<uwb_Distance)
								||houche_huiche_count>houche_huiche_Menu_count)//uwb�����𽥼�С���˵��趨��Сֵ������
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
					else if(wandao_huiche_flag==0)//ֱ���ᳵ
					{
	
						/*  ֱ���ᳵ�ᳵ��λ�߼�  ����ӽ� �ᳵ��λ  */
						if((huichejuli==0&&last_distance[2]>=last_distance[1]&&last_distance[1]>=last_distance[0]&&last_distance[0]>=uwb_Distance&&last_distance[2]>uwb_Distance
							&&last_distance[0]<100&&uwb_Distance<huiche_Menu_uwb)||houche_huiche_count>houche_huiche_Menu_count)//����uwb��ȷ�����Ƿ��ѿ���ǰ�� ����������С
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
		if(wandao_huiche_flag==0)//ֱ���ᳵ���������Ƴ�
		{
			if(already_huiche==0&&huiche==2&&xuxianjilu==1)
				zhidao_huiche_count++;
		}
		if(already_huiche==2&&xuxianjilu==1)//Ҳ��������ᳵǰ�����������Ƴ�
		{
			after_huiche_count++;//���ڵڶ����жϻᳵ������ʱ ��ֹ���м�ᳵ�����ֶ����ж�		
		}
		
		/*                ���س�                              */	
		if(already_huiche==2&&(xuxianjilu==1||tongshi_huiche==1)&&CarFlag ==2)//�ڶ�������ǰ��
		{
			if(Wireless(RX,0)==2)
				Stop_Line_Huiche = 1;
		}
		if(xuxianjilu==2)//�������ջᳵ��
		{
			if(CarFlag==2)//ȷ��ǰ��
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
				if(huiche==0)//ȷ��ǰ��
				{					
					Wireless(TX,2);	
					/******* ���ᳵ�ȴ�ʱ����� �ж���ʱ���ѳ�����  ********/
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
				 if(huiche==0&&Car==3)		//���ǰ��û�з��ͳɹ� ��ʱ�Ƴ� �������ﵽһ��ֵʱ ���������� �����������һֱС��2�� ��ô�ж���ʱ������ͬʱ����
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
        if(huiche	==1)//�ȴ���ʶ������		
				{			
					/******* ���ᳵ�ȴ�ʱ����� �ж���ʱ���ѳ�����  ********/
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
					if(Wireless(RX,0)==4)//��ʱ��ʶ������
					{
						huiche	= 2 ;
						guancha=1;
					}	
          else if(Wireless(RX,0)==2&&Car==3&&send_fail_flag==1)//ͬʱʶ������
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
				if(huiche==2&&tongshi_huiche==0)//���Ѿ�ʶ�������
				{	
					 send_fail_count=0;
					 send_fail_flag=0;
					if(test==0)
						houche_huiche_count++;	//�������������ֹ�������Բ��� ����ͻȻ��������� ������������ᳵ���Ҿ���һ��ʱ�����Զ���λ				
					if((huichejuli==0&&last_distance[2]>=last_distance[1]&&last_distance[1]>=last_distance[0]&&last_distance[0]>=uwb_Distance&&last_distance[2]>uwb_Distance
						&&last_distance[0]<100&&uwb_Distance<huiche_Menu_uwb)||houche_huiche_count>houche_huiche_Menu_count)//����uwb��ȷ�����Ƿ��ѿ���ǰ�� ����������С
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
				if(huiche==0)//һ��ʼʶ������ ��ǰ����4 ˵�������г�����
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
						houche_huiche_count++;	//�������������ֹ�������Բ��� ����ͻȻ��������� ������������ᳵ���Ҿ���һ��ʱ�����Զ���λ			
					if((huichejuli==0&&last_distance[2]>=last_distance[1]&&last_distance[1]>=last_distance[0]&&last_distance[0]>=uwb_Distance&&last_distance[2]>uwb_Distance
						&&last_distance[0]<100&&uwb_Distance<huiche_Menu_uwb)||houche_huiche_count>houche_huiche_Menu_count)//����uwb��ȷ�����Ƿ��ѿ���ǰ�� ����������С
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
			after_lasthuiche_count++;//���ڵڶ����жϻᳵ������ʱ ��ֹ���м�ᳵ�����ֶ����ж�		
		}
	}
	if(already_huiche==2&&xuxianjilu==1&&dangche==2)//Ҳ��������ᳵǰ�����������Ƴ�
	{
		after_huiche_count++;//���ڵڶ����жϻᳵ������ʱ ��ֹ���м�ᳵ�����ֶ����ж�		
	}
	/*    û20ms���ڼ̳�һ�ξ���      */
	last_distance[2] = last_distance[1];
	last_distance[1] = last_distance[0];
	last_distance[0] = uwb_Distance;
}

//�趨һ�ξ������
//�趨һ�ξ������
void AI_Init()//��ʼ����¼�������µ����ᳵ��������
{
	for(uint8_t i = 0; i < 8; i++)
	{
		if(AI_Element_set[i] == 0)//����
			cutIsland_set[i] = AI_num_set[i];
		else if(AI_Element_set[i] == 1)//�µ�
			cutRamp_set[i] = AI_num_set[i];
		else if(AI_Element_set[i] == 2)//�ᳵ��
			cutDashed_set[i] = AI_num_set[i];
	}
}

void Artificial_Intelligence()
{
	//���㻷�������µ�֮�������
	if(huancount == last_huancount + 1)//��������
	{
		Island_distance[huancount] = encoderCM;//��¼�ڼ����������ڵ�����
	}
	else if(PanPoJishu == last_PanPoJishu + 1)//�µ�����
	{
		Ramp_distance[PanPoJishu] = encoderCM;//��¼�ڼ����µ����ڵ�����
	}
	else if(xuxianjilu == 1 && DashedField_distance == 100000)//��⵽���߻ᳵ��
	{
		DashedField_distance = encoderCM;//��¼�ᳵ���ľ���
	}
	
	
	
	//һֱ�� ������ʵ�ʵ�����
	encoderCNT += Speed_Left + Speed_Right;
	encoderCM = encoderCNT / 116;
	
	//������ڼ��ٵĹ����� �����е�������� ��������ǲ����ڵ������ٵ��˹����������� ����ǾͰ����������Ϊʹ�ù�
	if((dianbo == 0 && last_dianbo > 0 && AI_State == 1) || (PanPoJishu == last_PanPoJishu + 1 && podaoF == 1))//������������µ����
	{
		//����ǲ����ڼ���������
		for(uint8_t i = 0; i < 8; i++)
		{
			if(huancount == cutIsland_set[i])//��������� ����û���ù���μ��٣���������������٣�
			{
				if(encoderCM > Island_distance[huancount] + cutCM_In[i] * 100 && encoderCM < Island_distance[huancount] + (cutCM_Out[i] + cutCM_In[i]) * 100)
				{
					if(AI_Element[i] == 2 && (dianbo == 0 && last_dianbo > 0 && AI_State == 1))//�������ٿ���
						AI_Use[i] = 1;//�ù���εļ���
					if(AI_Element[i] == 1 && (PanPoJishu == last_PanPoJishu + 1 && podaoF == 1))//�µ�ʶ�𿪹�
						AI_Use[i] = 1;//�ù�����µ�����
				}
			} 
			else if(PanPoJishu == cutRamp_set[i])//������µ� ����û���ù���μ��٣���������������٣�
			{
				if(encoderCM > Ramp_distance[PanPoJishu] + cutCM_In[i] * 100 && encoderCM < Ramp_distance[PanPoJishu] + (cutCM_Out[i] + cutCM_In[i]) * 100)
				{
					if(AI_Element[i] == 2 && (dianbo == 0 && last_dianbo > 0 && AI_State == 1))//�������ٿ���
						AI_Use[i] = 1;//�ù���εļ���
					if(AI_Element[i] == 1 && (PanPoJishu == last_PanPoJishu + 1 && podaoF == 1))//�µ�ʶ�𿪹�
						AI_Use[i] = 1;//�ù�����µ�����
				}
			}
			else if(xuxianjilu == cutDashed_set[i] && AI_Use[i] == 0)//��������ᳵ��֮��
			{
				if(encoderCM > DashedField_distance + cutCM_In[i] * 100 && encoderCM < DashedField_distance + (cutCM_Out[i] + cutCM_In[i]) * 100)
				{
					if(AI_Element[i] == 2 && (dianbo == 0 && last_dianbo > 0 && AI_State == 1))//�������ٿ���
						AI_Use[i] = 1;//�ù���εļ���
					if(AI_Element[i] == 1 && (PanPoJishu == last_PanPoJishu + 1 && podaoF == 1))//�µ�ʶ�𿪹�
						AI_Use[i] = 1;//�ù�����µ�����
				}
			}
		}
	}
	
	
	
	last_dianbo = dianbo;//�����������
	//���¼���
	last_huancount = huancount;
	last_PanPoJishu = PanPoJishu;
	
	
	//�����Ƿ����˹����ܵĹ�����
	//������˹����ܼ���״̬
	uint8_t this_AIState = 0;
	podaoF = 0;//�ȹر��µ�����
	dianboF = 0;//�ȹرյ�������
	huichequF = 0;//�ȹرջᳵ����⿪��
	for(uint8_t i = 0; i < 8; i++)
	{
		if(huancount == cutIsland_set[i] && AI_Use[i] == 0)//��������� ����û���ù���μ��٣���������������٣�
		{
			if(encoderCM > Island_distance[huancount] + cutCM_In[i] * 100 && encoderCM < Island_distance[huancount] + (cutCM_Out[i] + cutCM_In[i]) * 100)
			{
				//�ж�����ǵ��������µ�
				if(AI_Element[i] == 1)//����µ�����
					podaoF = 1;//���µ�����
				else if(AI_Element[i] == 2)//�������ٿ���
				{
					dianboF = 1;//�򿪵�������
					this_AIState = 1;//���ڼ���������
				}
				else if(AI_Element[i] == 3)//�ᳵ����⿪��
					huichequF = 1;//�򿪻ᳵ����⿪��
				else if(AI_Element[i] == 4)//������ٿ���
				{
					this_AIState = 1;//���ڼ���������
				}
			}
		} 
		else if(PanPoJishu == cutRamp_set[i] && AI_Use[i] == 0)//������µ� ����û���ù���μ��٣���������������٣�
		{
			if(encoderCM > Ramp_distance[PanPoJishu] + cutCM_In[i] * 100 && encoderCM < Ramp_distance[PanPoJishu] + (cutCM_Out[i] + cutCM_In[i]) * 100)
			{
				//�ж�����ǵ��������µ�
				if(AI_Element[i] == 1)//����µ�����
					podaoF = 1;//���µ�����
				else if(AI_Element[i] == 2)//�������ٿ���
				{
					dianboF = 1;//�򿪵�������
					this_AIState = 1;//���ڼ���������
				}
				else if(AI_Element[i] == 3)//�ᳵ����⿪��
					huichequF = 1;//�򿪻ᳵ����⿪��
				else if(AI_Element[i] == 4)//������ٿ���
				{
					this_AIState = 1;//���ڼ���������
				}
			}
		}
		else if(xuxianjilu == cutDashed_set[i] && AI_Use[i] == 0)//��������ᳵ��֮��
		{
			if(encoderCM > DashedField_distance + cutCM_In[i] * 100 && encoderCM < DashedField_distance + (cutCM_Out[i] + cutCM_In[i]) * 100)
			{
				//�ж�����ǵ��������µ�
				if(AI_Element[i] == 1)//����µ�����
					podaoF = 1;//���µ�����
				else if(AI_Element[i] == 2)//�������ٿ���
				{
					dianboF = 1;//�򿪵�������
					this_AIState = 1;//���ڼ���������
				}
				else if(AI_Element[i] == 3)//�ᳵ����⿪��
					huichequF = 1;//�򿪻ᳵ����⿪��
				else if(AI_Element[i] == 4)//������ٿ���
				{
					this_AIState = 1;//���ڼ���������
				}
			}
		}
	}
	
	//�˹����ܼ��������ʱ�� ��ֹ�ͼ�
	if(this_AIState == 0 && AI_State == 1)//�ⳡ����Ҫ���� ��һ������
	{
		AI_State = 2;//���ٻָ��׶�
		AI_RecoveryCnt = 0;//��ջָ�
	}
	
	AI_State = this_AIState;//ai״̬����

	if(AI_State == 2)
	{
		if(AI_RecoveryCnt >= 10)
		{
			AI_State = 0;
		}
	}
	
	//�󿪹�
	if(podaoF_set == 1)
		podaoF = 1;
	if(dianboF_set == 1)
		dianboF = 1;
	if(huichequF_set == 1)
		huichequF = 1;
}
uint8_t buzzer_num = 0;
void Buzzer_OnOff()//����������
{
	//�����ֲ�ͬ�ķ������е�ģʽ
	//0-����
	//��ţ�1  2  3  4  5  6  7  8  9  10
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
	//��������
	uint8_t buzzer_mode = 0;
	if(podao_flag>0 || podaoF > 0)//�µ�
	{
		buzzer_mode = 1;
	}
	else if((dianboGeLi_count>0&&dianboGeLi_count<dianbofield) || AI_State > 0)//��������
		buzzer_mode = 2;
	else if(huichequF > 0)//�ᳵ���ж�
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
	Get_AngleLine();//ȷ�������
	
	Set_Element();//��ȷ����������
	Artificial_Intelligence();//�˹����ܼ���
	//�������
	
	Get_Error();//ȷ��ƫ��
	Get_ServoPD();//���ݲ�ͬԪ��ȷ�����pd
	Servo_Control();//�����Ǽ���Ϳ���
	//�������
	Speed_Calculate();//���ݲ�ͬԪ�ظ�����ͬ���趨�ٶȺ͵��pd
	Motor_Control();//���������� ����ջ�����
	//����
	Buzzer_OnOff();//����������
}
