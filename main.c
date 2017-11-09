//
//
// Luu y:
//
// Lap trinh DK Encoder = PID
// Nhap so tu Keypad roi nhan # de bat dau chay.
// copy file lcd.h vào C:\Program Files (x86)\PICC\Devices
// Nhan Button sau khi re_Pos = de_Pos de nhap lai gia tri de_Pos
// re_Pos: gia tri that cua encoder.
// de_Pos: gia tri xac lap.
// Chinh tan so chip 18F4431 la 4MHz, encoder 360 pulse/rev
// Su dung file Simulation trong folder PID.
//
//
#include <main.h>
#include <lcd.h>
#include <math.h>
//Config QEI
#byte    QEICON= 0xFB6
#byte DFLTCON  = 0xf60
int16    POSCNT; 
#byte    POSCNT= 0xF66  
#byte    POSCNTH= 0xF67
#byte    POSCNTL= 0xF66
int16    MAXCNT;
#byte    MAXCNT= 0xF64  
#byte    MAXCNTH= 0xF65
#byte    MAXCNTL= 0xF64
//Config PID
#define Kp        10
#define Ki        0.1
#define Kd        0.01
#define max_pwm   1023
#define timer2_value 0.5
//Config matrix 4x3
#define COT_1 PIN_D4
#define COT_2 PIN_D5
#define COT_3 PIN_D6
#define HANG_1 input(PIN_D0)
#define HANG_2 input(PIN_D1)
#define HANG_3 input(PIN_D2)
#define HANG_4 input(PIN_D3)

char phimnhan[12]=
               {
                  '1','2','3', 
                  '4','5','6',
                  '7','8','9',
                  '*','0','#',
               };

void controlCol(int cot, int state)
{
   switch(cot)
   {
      case 1:
      {
         output_bit(COT_1,state);
         break;
      }
      case 2:
      {
         output_bit(COT_2,state);
         break;
      }
      case 3:
      {
         output_bit(COT_3,state);
         break;
      }
      
   }
}

void scanCol() // turn on all column
{
   int i;
   for(i==1;i<=3;i++)
   {
      controlCol(i,0);
   }
}

char scanKey()
{
   int8 Key = 0;
   controlCol(1,1);
   if(HANG_1==1)
   {
      Key = 1;
   }
   if(HANG_2==1)
   {
      Key = 4;
   }
   if(HANG_3==1)
   {
      Key = 7;
   }
   if(HANG_4==1)
   {
      Key = 10;
   }
   controlCol(1,0);
   controlCol(2,1);
   if(HANG_1==1)
   {
      Key = 2;
   }
   if(HANG_2==1)
   {
      Key = 5;
   }
   if(HANG_3==1)
   {
      Key = 8;
   }
   if(HANG_4==1)
   {
      Key = 11;
   }
   controlCol(2,0);
   controlCol(3,1);
   if(HANG_1==1)
   {
      Key = 3;
   }
   if(HANG_2==1)
   {
      Key = 6;
   }
   if(HANG_3==1)
   {
      Key = 9;
   }
   if(HANG_4==1)
   {
      Key = 12;
   }
   controlCol(3,0);
   return Key;
}

signed int16 PID, new_err = 0, de_Pos=0, re_Pos=0,I=0,e=0,pwm=0;
int8 value[4] = {0,0,0,0}; //stores single numbers
int16 so = 0;



void xuatSo(int16 number)
{
   unsigned int16 temp;
   
   temp = number;  // ex: number = 1234 
   value[3] = temp %10;  //  4
   
   temp = temp/10;
   value[2] = temp%10;  // 3
   
   temp = temp/10; 
   value[1] = temp%10; //  2
   
   temp = temp/10;
   value[0] = temp; //  1
}

void PID_Config()
{
   new_err = de_Pos - re_Pos;
   new_err = abs(new_err);
   PID = Kp*new_err;
   I  = e + new_err;
   PID = PID + Ki*I;
   PID = PID + Kd*(new_err-e)/timer2_value;
   PWM = PID;
   if (PWM > max_pwm) PWM = max_pwm;
   e = new_err;
   if(de_Pos > re_Pos)
   {
      set_pwm1_duty(PWM);
      set_pwm2_duty(0);
   }
   else
   {
    set_pwm2_duty(PWM);
    set_pwm1_duty(0);
   }
}

void nhapSo()
{  
   int8 Key=0, tam, index=15;
   int16 so=0;
   lcd_init();
   lcd_putc('\f');
   printf(lcd_putc,"Desired value: ");
   scanCol();
   while(TRUE)
   {
      Key=scanKey();
      if(Key!=0)
      {
         if(Key == 12)
         {
            break;
         }
         tam=Key;
         if(Key != 11)
         so = so*10 + Key;
         else
         so = so*10;
         lcd_gotoxy(index,1);
         printf(lcd_putc,"%c",phimnhan[tam-1]); // hien thi phim nhan
         index++;
         while(scanKey()!=0)
         {delay_ms(5);}
         de_Pos = so;
      }
   }
}

void main()
{
   output_bit(PIN_B0,1);
   nhapSo();
   QEICON   = 0x18;
   POSCNT = 0;
   MAXCNT  = 65536;
   scanCol();
   setup_timer_2(T2_DIV_BY_4,255,1);// 244hz
   setup_ccp1(CCP_PWM);
   set_pwm1_duty(0);
   setup_ccp2(CCP_PWM);
   set_pwm2_duty(0);
   lcd_init();
   xuatSo(de_Pos);
   lcd_gotoxy(1,1);
   lcd_putc('\f');
   printf(lcd_putc,"Initial: %d%d%d%d",value[0],value[1],value[2],value[3]);
   while(TRUE)
   {  
      re_Pos = POSCNT/4;
      xuatSo(re_Pos);
      lcd_gotoxy(1,2);
      printf(lcd_putc,"Respond: %d%d%d%d",value[0],value[1],value[2],value[3]);
      PID_Config();
      while(new_err==0)
      {
         if(input(PIN_C3)==0)
         {
            nhapSo();
            new_err = 0,I=0,e=0,pwm=0;
            PID_Config();
         }
      }
   }
}



