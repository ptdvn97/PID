#include <main.h>
#include <lcd.h>
#include <math.h>
#byte    QEICON= 0xFB6    // Thanh ghi dieu khien
#byte DFLTCON  = 0xf60
int16    POSCNT; 
#byte    POSCNT= 0xF66  
#byte    POSCNTH= 0xF67
#byte    POSCNTL= 0xF66
int16    MAXCNT;
#byte    MAXCNT= 0xF64  
#byte    MAXCNTH= 0xF65
#byte    MAXCNTL= 0xF64

#define Kp        10
#define Ki        0.1
#define Kd        0.01
#define max_pwm   1000
#define timer2_value 0.5

signed int16 PID, new_err = 0, de_Pos=0, re_Pos=0,I=0,e=0,pwm=0;
int8 ini_vl[4] = {0,0,0,0}; //1444

int16 xuatSo(int16 so)
{
   unsigned int16 bien;
   
   bien = so;  // 1444
   ini_vl[3] = bien %10;  //4
   
   bien = bien/10;
   ini_vl[2] = bien%10;  //4
   
   bien = bien/10; 
   ini_vl[1] = bien%10; // 4
   
   bien = bien/10;
   ini_vl[0] = bien; // 1
}

void PID_Conf()
{
   new_err = de_Pos - re_Pos;
   PID = Kp*new_err;
   I  = e + new_err;
   PID = PID + Ki*I;
   PID = PID + Kd*(new_err-e)/timer2_value;
   PWM = PID;
   if (PWM > max_pwm) PWM = max_pwm;
   e = new_err;
   set_pwm1_duty((int16)PWM);
}

void main()
{
   QEICON   = 0x18;
   POSCNT = 0;
   MAXCNT  = 65536;
   de_Pos = 700;
   setup_timer_2(T2_DIV_BY_16,255,1);// 244hz
   setup_ccp1(CCP_PWM);
   set_pwm1_duty(0);
   setup_ccp2(CCP_PWM);
   lcd_init();
   xuatSo(de_Pos);
   lcd_gotoxy(1,1);
   lcd_putc('\f');
   printf(lcd_putc,"Inital: %d%d%d%d",ini_vl[0],ini_vl[1],ini_vl[2],ini_vl[3]);
   while(TRUE)
   {
      re_Pos = POSCNT/4;
      xuatSo(re_Pos);
      lcd_gotoxy(1,2);
      printf(lcd_putc,"Respond: %d%d%d%d",ini_vl[0],ini_vl[1],ini_vl[2],ini_vl[3]);
      PID_Conf();
   }
}





