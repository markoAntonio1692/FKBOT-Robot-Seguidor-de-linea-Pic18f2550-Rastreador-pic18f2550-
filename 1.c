//ROBOT SEGUIDOR DE LINEAS FK-BOT 
////version final 2013-2014
// Programado Por: MARKO ANTONIO CABALLERO MORENO  Ing Electro. y Telec. (UNTELS)
//
// Programado para primera pista de competencia  untelstronics 2014
//
//Hardware utilizado: pic 18f2550 (con bootloader neoteo),modulo driver de motor l298
//                    8 sensores cny70,4 opamps lm358, motoreductores dc de 12v, 
//                    4 celdas de litio de 3.7 volts,cristal de 20 MHz, estructura de mecano
// distribucion de sensores:
//                        parte delantera
//                   _______________________
//                   |s1                  s9| 
//                   |  s2 s3 s4 s5 s6 s7   | 
//                   |______________________|
#include <18F2550.h>
#fuses HS,NOWDT,NOPROTECT,NOLVP,NODEBUG,NOUSBDIV,CPUDIV1,NOVREGEN,PUT,NOBROWNOUT,NOSTVREN, pll1
#use delay(clock=48000000)
#build (reset=0x1000,interrupt=0x1008)
#org 0x0000,0x0FFF{}

#use STANDARD_IO(B)
#USE STANDARD_IO(A)  
#USE STANDARD_IO(C)
//definiciones de las salidas
#define motor_da  pin_a1
#define motor_dr  pin_a2
#define motor_ia  pin_a4
#define motor_ir  pin_a3
#define led_r     pin_c0
#define led_a     pin_c7
#define trig      pin_c6
#define eco       pin_a5

int s1=0,s2=0,s3=0,s4=1,s5=1,s6=0,s7=0,s8=0,btn1,btn2; //variables de sensores y botones
int comp=-3; //////////// aumenta o disminuye pwm general del robot(velocidad)
int16 contador=0,clock=0,sensado=1,k=0;
int m=0;
int16 distancia,valor,contador3=0,ultra=0; //variable ultra para la lectura de ultrasonido , 1 hay obstaculo, 0 no hay obstaculo
float posicion=0,derivativo=0,proporcional=0,integral=0,power_diferencia,proporcional_pasado=0;
float kp=16,kd=40,ki=0.001;
int z=6;
int16 t=0;
int n_cuenta=0;

//fuciones para ultrasonido
void PULSO();
void distancia_filtro();
void cambio(int cm); //funcion para cambio de la  distancia (cm) de sensado ultrsonido
////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#INT_timer0
void timer0()
{
   if(clock==1) //clock para menu de inicio
   {  
      contador++; //variable para contador de fucnio
       IF (contador == 93)
       {     
         output_toggle (led_a) ;
        k++; //variable para dar tiempos en las opciones de menu
        contador=0;
       }
   }
   
   // cambiar variable sensado para elegir el tipo de sensado
   IF (sensado == 1)  //linea negra
   {
      s1 = ~input(pin_b0) ;
      s2 = ~input(pin_b1) ;
      s3 = ~input(pin_b2) ;
      s4 = ~input(pin_b3) ;
      s5 = ~input(pin_b4) ;
      s6 = ~input(pin_b5) ;
      s7 = ~input(pin_b6) ;
      s8 = ~input(pin_b7) ; 
   }
   IF (sensado == 0) //linea blanca
   {
      s1 = input (pin_b0) ;
      s2 = input (pin_b1) ;
      s3 = input (pin_b2) ;
      s4 = input (pin_b3) ;
      s5 = input (pin_b4) ;
      s6 = input (pin_b5) ;
      s7 = input (pin_b6) ;
      s8 = input (pin_b7) ;
   } 
   set_timer0(170);
}
#INT_TIMER1                        
void var()  ///variable tiempo aumenta cada 1 ms //para funciones de ratreos  con tiempo 
{  
      t++; //variable para contar el tiempo 
      //l++;
     if(t==5000) t=0;  
     set_timer1 (64036);
}

#INT_TIMER3    
void Interrupcion() // interrupcion para lectura de ultrasonido, si se utiliza, descomentar  aqui y descomentar la configuracion en el void main()
{
/*
PULSO(); // Mandamos el pulso de 10ƒÊs para iniciar una nueva lectura
while(!input(eco)); // Esperamos que el pic eco este en 1
delay_us(distancia); // Hacemos un retardo usando el tiempo de espera
distancia_filtro(); // Llamamos a la funcion de lectura explicada anteriormente
set_timer0(65536 - distancia);
*/
}

void motores(signed int16 izquierdo,signed int16  derecho); //funcion para manejo de motores 0-255
//rastreos basicos
void r_f(int16 pwm,int16 reduccion);
void r_l(int16 pwm,int16 reduccion);
void r_f2(int16 pwm,int16 reduccion);   
void r_bd(int16 pwm,int16 reduccion); 
void r_bd2(int16 pwm,int16 reduccion);
void r_bi(int16 pwm,int16 reduccion);
//rastreos basicos con tiempo
void r_bi_t(int16 tiempo_ms,int16 pwm, int16 reduccion);
void r_bd_t(int16 tiempo_ms,int16 pwm, int16 reduccion);
void r_f2_t(int16 tiempo_ms,int16 pwm, int16 reduccion);
void r_f_t(int16 tiempo_ms,int16 pwm, int16 reduccion);
void r_f_t(int16 tiempo_ms,int16 pwm, int16 reduccion);
// rastreos pid con tiempo
void pid2_t(int16 tiempo_ms,int16 pwm);
void pid_t(int16 tiempo_ms,int16 pwm);
//patrones: movimientos predefinidos (patrones mi mejor aporte a la programacion del robot)
void patron_i(int tipo,int16 pwm);
void patron_d(int tipo,int16 pwm);
void patron_a(int tipo,int16 pwm);
void patron_cruz_i(int tipo,int16 pwm);
void patron_cruz_d(int tipo,int16 pwm);
//rastreos pid
void pid(int16 pwm);
void pid2(int16 pwm);
void pid_cont(int tipo,int16 pwm, int n_veces); //rastreo pid contador para esquinas (n_veces)
void pid_a(int16 pwm);

void error(); //funcion para lectura de error

//funciones generales
void luz();// funcion para leds
void botones();
void activar_t1();
void desactivar_t1();
void p (void);   ///parar con while (true)
void parar(void); //parar simple
void f0_1(); //funcion de preinicio
void f0();// funcion de inicio
// funciones que se utilizaran para los casos (programacion de pista)
void f1();
void f2();
void f3(); 
void f4();
void f5();
void f6();
void f7();
void f8();
void f9();
void f10();
void f11(); 
void f12();
void f13();
void f14();
void f15();
void f16();
void f17();
void f18();
void f19(); 
void f20();
void f21();
void f22();
void f23();
void f24();
void f25();
void f26();
void f27(); 
void f28();
void f29();
void f30();
void f31();
void f32();
void f33();
void f34();
void f35(); 
void f36();
void f37();
void f38();
void f39();
void f40();
void f41();
void f42();
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
void main()
{
   delay_ms (50);
   set_tris_b (0xff);
   p () ; //motores parados
   
   //////conf pwm
   setup_ccp1 (CCP_PWM);
   setup_ccp2 (CCP_PWM);
   setup_timer_2 (T2_DIV_BY_1, 255, 1);
   set_pwm1_duty (0);
   set_pwm2_duty (0);
   
   //conf de lectura aalogica (para las baterias)
   //setup_adc_ports (an0) ;
   //setup_adc (adc_clock_internal);
   //configuracion de timers
   setup_timer_0 (rtcc_internal|rtcc_div_1);
   set_timer0 (200) ;
   
   setup_timer_1 (T1_INTERNAL|T1_DIV_BY_8) ;
   set_timer1 (64036);
   
    setup_timer_3 (T1_INTERNAL|T1_DIV_BY_1) ;
   set_timer3 (65536 - distancia);
   enable_interrupts (INT_timer0);
   enable_interrupts (INT_timer1);
   enable_interrupts (INT_timer3);
   delay_ms(1);
   
   disable_interrupts (global);//desabilitacion de interrupciones
   luz () ;//indicador leds
   f0 () ; //funcion para el pre inicio
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void PULSO()
{
output_high(trig);
delay_us(10);
output_low(trig);
}

void cambio(int cm) //funcion para cambio de la  distancia (cm) de sensado ultrasonido
{
valor=cm;
distancia=58*valor;
}

void distancia_filtro() 
{
if(!input(eco)) // Si el pin ECO esta en 0
{
contador3++; // Aumentamos en uno la variable contador
if(contador3==2) // Si contador es igual a 2
{
ultra=1; // Aseguramos que encontró un objeto
contador=0; // Inicializamos la variable contador
output_toggle(led_r);
}}
else // Si el pin ECO esta en 1
{
ultra=0;
contador3=0; // Inicializamos la variable contador
output_low(led_r);
}}

void motores(signed int16 izquierdo,signed int16 derecho) 
{
int v1=0,v2=0; 
   if(derecho >0)
   {
      output_high(motor_da);
      output_low(motor_dr);
      v2=derecho-3+comp;
   }
    if(derecho <0)
   {
      output_low(motor_da);
      output_high(motor_dr);
      v2=((derecho*-1))-2+comp;
   }
     if(derecho==0)
   {
      output_high(motor_da);
      output_high(motor_dr);
      v2=0;
   }
   ///////////////////////////
     if(izquierdo >0)
   {
      output_high(motor_ia);
      output_low(motor_ir);
      v1=izquierdo-6+comp;
   }
   if(izquierdo <0)
   {
      output_low(motor_ia);
      output_high(motor_ir);
      v1=((izquierdo*-1))+comp;
   }
    if(izquierdo==0)
   {
      output_high(motor_ia);
      output_high(motor_ir);
       v1=0;
   }
   set_pwm1_duty (v1);  set_pwm2_duty (v2);
}

//funciones para los diversos patrones derecha, izquierda, avanzar, cruz derecha, cruz izquierda, 
// un patron no debe ir en un bucle infinto solo debe ser llamado en una sola vez
void patron_cruz_i(int tipo,int16 pwm)
{
   IF (tipo == 1)
   {
     motores(pwm,pwm);
      WHILE (true)
      {
         IF (s1==0 && s2==0 && s7==0 && s8==0)
         {
            delay_ms(2);
            motores(-140,-140); delay_ms(10);
            motores(-140,pwm);
            WHILE (true)
            {
               IF (s6 == 0&&s7 == 1)
               {
                  pwm=pwm-5;   motores(-pwm,pwm);
                  WHILE (true)
                  {
                     IF (s4 == 1 && s6==0 )
                     {
                     motores(pwm,0);   delay_ms(30);
                        BREAK;
                     }
                  }
                  BREAK;
               }
            }
            BREAK;
         }
      }
   }
   p();
}
void patron_cruz_d(int tipo,int16 pwm)///
{
   if (tipo == 1)
   {
      motores(pwm,pwm);
      while(true)
      {
         if(s1==0 && s2 == 0 && s7==0&&s8 == 0)
         {
            delay_ms(2);
            motores(-140,-140); delay_ms(10); 
            motores(pwm,-140);
            while(true)
            {
               if (s2==1 && s3==0)
               {
                  pwm=pwm-5;
                  motores(pwm,-pwm);
                  while (true)
                  {
                     IF (s3==0 && s5==1)
                     {
                        motores(0,pwm); delay_ms(30);
                        BREAK;
                     }
                  }
                  BREAK;
               }
            }
            BREAK;
         }
      }
   }
   p();
}

void patron_a(int tipo,int16 pwm)
{
   IF(tipo==1)
   {
      motores(pwm,pwm);
      WHILE (true)
      {
         IF (s1==0 && s2==0 && s7==0 && s8==0)
         {  
            delay_ms(2);
            motores(-130,-130);delay_ms(10); 
            BREAK;
         }
      }
   }
   
   if(tipo==2)
   {
      motores(pwm,pwm);
      while(true)
      {
         if(s2==0 && s3==0 &&  s4==0 && s5==0 && s6==0 && s7==0)
         {  
            delay_ms(2);
            motores(-140,-140);  delay_ms(10);  
            BREAK;
         }
      }
   }
    if(tipo==3)
   {
      motores(pwm,pwm);
      while(true)
      {
         if(s1==0 && s2==0 && s3==0 &&  s4==0 && s5==0 && s6==0 && s7==0 && s8==0)
         {
            delay_ms(2);
            motores(-140,-140);  delay_ms(10);  
            BREAK;
         }
      }
   }
   p () ;
}

void patron_i( int tipo,int16 pwm)
{
   IF(tipo==1)
   {
      WHILE (true)
      {
         motores(pwm,pwm);
         IF (s2==0 && s3==0 && s4==0 &&  s5==0 && s6==0 && s7==0 )
         {
            delay_ms(2);
            motores(-140,-140); delay_ms(15); 
            WHILE (true)
            {
               motores(-pwm,pwm);
               IF (s5==1 && s6==0)
               {
                  motores(pwm,-145);       delay_ms(15);
                  BREAK;
               }
            }

            BREAK;
         }
      }
   }

   IF(tipo==2)
   {
      WHILE (true)
      {
         motores(pwm,pwm);
         IF ( s2==0 && s3==0 && s4==0 && s5==0 && s6==0 && s7==0 )
         {
            delay_ms(2);
            motores(-140,-140); delay_ms(20);
            WHILE (true)
           {
               motores(-140,pwm);
               IF (s4 == 1 && s6 == 0)
               {
                  motores(pwm,-pwm); delay_ms(20);
                  BREAK;
               }
            }
            BREAK;
         }
      }
   }
   p();
}
  
void patron_d(int tipo,int16 pwm)
{
   IF(tipo==1)
   {
      WHILE (true)
      {
         motores(pwm,pwm);
         IF (s2==0 && s3==0 && s4==0 && s5==0 && s6==0 && s7==0)
         {
          delay_ms(2);     
          motores(-140,-140); delay_ms(15);
          WHILE (true)
            {
               motores(pwm,-pwm);
               IF (s3==0 && s4 == 1)
               {
                  motores(-145,pwm); delay_ms(15);
                  BREAK;
               }
            }
            BREAK;
         }
      }
   }

   IF(tipo==2)
   {
      WHILE (true)
      {
         motores(pwm,pwm);
         IF (s2==0 && s3==0 && s4==0 && s5==0 && s6==0 && s7==0)
         {
            delay_ms(2);
            motores(-140,-140); delay_ms(20);
            WHILE (true)
            {
               motores(pwm,-140);
               IF (s3 ==0 &&s5 == 1)
               {
                  motores(-pwm,pwm); delay_ms(20);
                  BREAK;
               }
            }
            BREAK;
         }
      }
   }
   p () ;
}

//funciones de rastreos basicos
// pwm: velocidad, reduccion: permit reducor la brusquedad de los cambio de sentido de motor
void r_f(INT16 pwm,int16 reduccion)//rastreo flanco  sensan sesores 3y 6
{
   if (s3==0 && s6==0)
   {
      motores(pwm,pwm);
   }
   if (s3==1 && s6==0)
   {
      pwm=pwm-reduccion;    motores(-pwm,pwm);
   }
   if (s3==0 && s6==1)
   {
      pwm=pwm-reduccion;   motores(pwm,-pwm);
   }
}

void r_l(int16 pwm,int16 reduccion)
{
   if (s4==1 && s5==1)
   {
      motores(pwm,pwm);
   }
   if (s4==1 && s5==0)
   {
    pwm=pwm-reduccion;    motores(0,pwm);
   }
   if (s4==0 && s5==1)
   {
      pwm=pwm-reduccion;   motores(pwm,0);
   }
}
void r_bd(int16 pwm,int16 reduccion)// rastreo borde derecho
{
   if (s5==1 && s6==0)
   {
     motores(pwm,pwm);
   }
   if (s5==0 && s6==0)
   {
     pwm=pwm-reduccion;    motores(-pwm,pwm);
   }
   if (s5==1 && s6==1)
   {
      pwm=pwm-reduccion;    motores(pwm,-pwm);
   }
}
void r_bd2(int16 pwm,int16 reduccion)//rastreo borde izquierdp
{
 if (s3==1 && s4==0)
   {
     motores(pwm,pwm);
   }
   if (s3==0 && s4==0)
   {
     pwm=pwm-reduccion;    motores(-pwm,pwm);
   }
   if (s3==1 && s3==1)
   {
      pwm=pwm-reduccion;    motores(pwm,-pwm);
   }
}

void r_bi(int16 pwm,int16 reduccion)
{
   if (s3==0 && s4==1)
   {
      motores(pwm,pwm);
   }
   if (s3 == 1&&s4 == 1) 
   {
      pwm=pwm-reduccion;    motores(-pwm,pwm); 
   }
   if (s3==0 && s4==0)
   {
      pwm=pwm-reduccion;    motores(pwm,-pwm);
   }
}
void r_f2(int16 pwm,int16 reduccion)  //rastreo flancos 2 sensan s2, s3 , s6, s7
{
   if(s3 == 0&&s6 == 0)
   {
    motores(pwm,pwm);
   }
   if(s3 == 1&&s6 == 0)
   {
   pwm=pwm-reduccion;    motores(-pwm,pwm);
   }
   if(s3 == 0&&s6 == 1)
   {
    pwm=pwm-reduccion;   motores(pwm,-pwm);  
   }
   if(s2 == 1&&s6 == 0)
   {
     pwm=pwm-reduccion;    motores(-pwm,pwm);
   }

   if(s2 == 0&&s7 == 1)
   {
      pwm=pwm-reduccion;  motores(pwm,-pwm);
   }
}
//rastreos basicos con tiempo 
void r_bi_t(int16 tiempo_ms,int16 pwm, int16 reduccion)
{
    t=0;
   while(t<=tiempo_ms)
   {
      r_bi(pwm,reduccion);
   }
}

void r_bd_t(int16 tiempo_ms,int16 pwm, int16 reduccion)
{
    t=0;
   while(t<=tiempo_ms)
   {
      r_bd(pwm,reduccion);
   }
}

void r_f_t(int16 tiempo_ms,int16 pwm, int16 reduccion)
{
    t=0;
   while(t<=tiempo_ms)
   {
      r_f(pwm,reduccion);
   }
}

void r_f2_t(int16 tiempo_ms,int16 pwm, int16 reduccion)
{
    t=0;
   while(t<=tiempo_ms)
   {
      r_f2(pwm,reduccion);
   }
}

void r_l_t(int16 tiempo_ms,int16 pwm, int16 reduccion)
{
   t=0;
   while(t<=tiempo_ms)
   {
      r_l(pwm,reduccion);
   }
}

//rastreo s pid con tiempo
void pid2_t(int16 tiempo_ms,int16 pwm)
{
   t=0;
   while(t<=tiempo_ms)
   {
      pid2(pwm);
   }
}

void pid_t(int16 tiempo_ms,int16 pwm)
{
   t=0;
   while(t<=tiempo_ms)
   {
      pid(pwm);
   }
}


void error(void)//funcion que recoge el error de sensado y ademas corrige si se ha salido de linea
{                 // utilzacion de tecnica de logica difusa para saber el error (que tan lejo sse esta), ademas se 
                  //varian consatantes segun sea el conjunto en que este
                  //
  //if(s1==1)            z=1;
  kp=14;
  ki=0.001;
  
   IF(s2==1 && s3==0 &&  s4==0&& s7==0) ////caso en el que se haya salido por la derecha
   {
   z=11;
   ki= 0.01;
   kp=15;
    delay_ms(5);
      if(s2==0 && s3==0 &&s5==0 && s4==0 && s6==0 &&s7==0)
      {
         while(true)
         {
              motores(-151,190);
              if(s3==1 || (s4==1&& s3==1)) break;
         }   
   break;
      }
   }  
   IF(s2==1 && s3==1 && s7==0)    {        z=2; kp=12;ki=0.01;} //
   IF(s1==0 && s3==1 && s7==0)            {z=3; kp=12;ki=0.01;}
   IF(s1==0 && s3==1 && s4==1 && s7==0)   {z=4; kp=12;ki=0.005; }
   IF(s2==0 && s4==1 && s7==0)            {z=5; kp=12;ki=0.001;}
   IF(s2==0 && s4==1 && s5==1 && s7==0)   {z=6; kp=12;ki= 0.001;}///////// casos en eque se encuentre en linea
   IF(s2==0 && s5==1 && s7==0)            {z=7; kp=12;ki= 0.001;}
   IF(s2==0 && s5==1 && s6==1 && s8==0)   {z=8; kp=12;ki= 0.005;}
   IF(s2==0 && s6==1 && s8==0)            {z=9; kp=12;ki= 0.01;}
   IF(s2==0 && s6==1 && s7==1 )          { z=10;kp=12;ki= 0.01;}
   IF(s2==0 && s5==0 && s6==0 && s7==1)   //caso en el que se haya salido por la izquierda
   { 
         z=11;
        kp=15;
        ki= 0.01;
       delay_ms(5);
      if(s7==0 && s6==0 && s5==0 &&s4==0 && s3==0 &&s2==0)
      {
            while(true)
            {      
               motores(190,-151);
               if(s6==1 ||( s5==1&&s6==1)) break;
            }
      }
      break;
   } //  if(s8==1)                              z=11;
}

void pid(int16 pwm) /// funcion PID principal, tiene como parametro el pwm
{ 
float max=pwm;
error(); // lama a la funcion error 
posicion=z;
proporcional=posicion-6;
integral=integral+proporcional_pasado;
derivativo=proporcional-proporcional_pasado;
if(integral>255)  integral=255;// limitamos integral para evitar problemas
if(integral<255) integral=-255;
power_diferencia=(proporcional*kp) + (derivativo*kd)+(integral*ki);
if ( power_diferencia > max ) power_diferencia = max;  //limitamos segun sea necesario la velocidad que pongamos en pwm
if ( power_diferencia < -max ) power_diferencia = -max;
if (power_diferencia < 0)
    {
       motores(max+power_diferencia, max);
    }
    if (power_diferencia >0)
    {
       motores(max, max-power_diferencia);     
    }
proporcional_pasado=proporcional;
}


void pid2(int16) // funcion pid secundaria(pid alternativo) con otros parametros
{
kp=18;
ki=0.001;
   IF(s2==1 && s3==0 &&  s4==0&& s7==0)   {z=1; kp=18;ki=0.01;} 
   IF(s2==1 && s3==1 && s7==0)    {        z=2; kp=18;ki=0.01;}
   IF(s1==0 && s3==1 && s7==0)            {z=3; kp=18;ki=0.005;}
   IF(s1==0 && s3==1 && s4==1 && s7==0)   {z=4; kp=18; ki=0.005;}
   IF(s2==0 && s4==1 && s7==0)            {z=5; kp=18;ki=0.001;}
   IF(s2==0 && s4==1 && s5==1 && s7==0)   {z=6; kp=18; ki=0.001; }/////////
   IF(s2==0 && s5==1 && s7==0)            {z=7; kp=18;ki=0.001;}
   IF(s2==0 && s5==1 && s6==1 && s8==0)   {z=8; kp=18;ki=0.005;}
   IF(s2==0 && s6==1 && s8==0)            {z=9; kp=18;ki=0.005;}
   IF(s2==0 && s6==1 && s7==1 )          { z=10;kp=18;ki=0.01;}
   IF(s2==0 && s5==0 && s6==0 && s7==1)   {z=11;kp=18;ki=0.01;}

  float max=pwm;
posicion=z;
proporcional=posicion-6;
integral=integral+proporcional_pasado;
derivativo=proporcional-proporcional_pasado;
if(integral>1500)  integral=1500;
if(integral<-1500) integral=-1500;
power_diferencia=(proporcional*kp) + (derivativo*kd)+(integral*ki);
if ( power_diferencia > max ) power_diferencia = max; 
if ( power_diferencia < -max ) power_diferencia = -max;
if (power_diferencia < 0)
    {
       motores(max+power_diferencia, max);
    }
    if (power_diferencia >0)
    {
       motores(max, max-power_diferencia);     
    }
proporcional_pasado=proporcional;
}



void pid_cont(int tipo, int16 pwm,int n_veces) // pid con contador para casos en que halla esquinas en la pista
{
   n_cuenta=0;
  if(tipo==1)
  {
  while(true)
   {
      pid(pwm);
      while(s1==1 && s8==0)
      {
         patron_i(1,pwm);
        
       n_cuenta++;
      }
      while(s1==0 && s8==1)
      {
         patron_d(1,pwm);
           
         n_cuenta++;
      }
      if(n_cuenta==n_veces) break;    
   }
  }
  
  if(tipo==2)
  {
  while(true)
   {
      pid(pwm);
      while(s1==1 && s8==0)
      {
         patron_a(1,pwm);
        
       n_cuenta++;
      }
      while(s1==0 && s8==1)
      {
         patron_a(1,pwm);
           
         n_cuenta++;
      }
      while(s1==1 && s8==1)
      {
         patron_a(1,pwm);
           
         n_cuenta++;
      }
      if(n_cuenta==n_veces) break;    
   }
  }
  
 }

void pid_a(int16 pwm){ // funcion para acelerar , pwm es el tope al que acelera ( acelera y ratrea con pid)
m=pwm;
    if(m==150){t=0;
   while(t<=10) pid(140);
   t=0;
   while(t<=30) pid(145);
   t=0;
   while(t<=30) pid(148);
   t=0;
   while(t<=30) pid(150);}
   if(m==160){
   t=0;
   while(t<=30) pid(140);
   t=0;
   while(t<=30) pid(145);
   t=0;
   while(t<=30) pid(148);
   t=0;
   while(t<=30) pid(150);
   t=0;
   while(t<=20) pid(155);
   t=0;
   while(t<=10) pid(158);
   t=0;
   while(t<=10) pid(160);}
   if(m==170){  
   t=0;
   while(t<=10) pid(140);
   t=0;
   while(t<=30) pid(145);
   t=0;
   while(t<=30) pid(148);
   t=0;
   while(t<=30) pid(150);
   t=0;
   while(t<=20) pid(155);
   t=0;
   while(t<=10) pid(158);
   t=0;
   while(t<=10) pid(160);
   t=0;
   while(t<=10)pid(162);
   t=0;
   while(t<=10)pid(165);
   t=0;
   while(t<=10)pid(168);
   t=0;
   while(t<=10)pid(170);}
     if(m==180){  
   t=0;
   while(t<=10) pid(140);
   t=0;
   while(t<=30) pid(145);
   t=0;
   while(t<=30) pid(148);
   t=0;
   while(t<=30) pid(150);
   t=0;
   while(t<=30) pid(155);
   t=0;
   while(t<=10) pid(158);
   t=0;
   while(t<=10) pid(160); 
      t=0;
      while(t<=10)pid(162);
      t=0;
      while(t<=10)pid(165);
      t=0;
      while(t<=10)pid(168);
      t=0;
      while(t<=10)pid(170);    
      t=0;
      while(t<=10)pid(172);
      t=0;
      while(t<=10)pid(175);
      t=0;
      while(t<=10)pid(178);
      t=0;
      while(t<=10)pid(180);}  
    if(m==190 || m==200){  
   t=0;
   while(t<=10) pid(140);
   t=0;
   while(t<=30) pid(145);
   t=0;
   while(t<=30) pid(148);
   t=0;
   while(t<=30) pid(150);
   t=0;
   while(t<=30) pid(155);
   t=0;
   while(t<=10) pid(158);
   t=0;
   while(t<=10) pid(160); 
      t=0;
      while(t<=10)pid(162);
      t=0;
      while(t<=10)pid(165);
      t=0;
      while(t<=10)pid(168);
      t=0;
      while(t<=10)pid(170);    
      t=0;
      while(t<=10)pid(172);
      t=0;
      while(t<=10)pid(175);
      t=0;
      while(t<=10)pid(178);
      t=0;
      while(t<=10)pid(180);
      t=0;
      while(t<=10)pid(190);
      t=0;
      while(t<=10)pid(200);} 
   p();}
void luz() // funciones de led
{
   delay_ms (200) ;
   output_high(led_r) ;
   output_high(led_a) ;
    delay_ms (200) ;
   output_low(led_r) ;
   output_low(led_a) ;
     delay_ms (200) ;
}
void botones() // lectura de botones
{
   btn2 = input (pin_c6) ;
   btn1 = input (pin_a5) ;
}
void p(void)
{
   output_low(motor_da);
   output_low(motor_dr);
   output_low(motor_ia);
   output_low(motor_ir);
}
void parar(void)
{
   while(true)
   {
      motores(0,0);  luz();
   }
}

////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
void f0()//funcion de preinicio 
{
   WHILE(true)
   {     
      botones () ;
      output_high (pin_c7) ;    
      
      IF (btn2==0) ///arranque normal boton2 para arranque normal
      {
         delay_ms (20) ;
         luz ();
         enable_interrupts (global);//habilita interrupciones ya para iniciar 
         delay_ms (40) ;
         
         f1();//////*************************************************+///cambiar aca para empezar/////////////////
      }

      if (btn1==0) //arranque con opciones boton 2 para elegir opciones de velocidad
      {
         delay_ms(200);
         clock=1;
         enable_interrupts (global);
         disable_interrupts (INT_timer1);  
         btn1=1;
         btn2=1;
         while(true)
         {
            botones();
            if(btn2==0)// si se presiona otrves  para salir de menu y  arranque normalmente
            {
            delay_ms(10);
            luz();
            f1();
            }
            if(btn1==0) // si se vuelve a presionar entra al menu para elegir  las velocidades de inicio
            {
               delay_ms(10);
               output_high(led_r); 
               f0_1(); //llama a la funcion  del menu donde estan las velocidades        
            }   
         }
      }}}        
      
void f0_1(){      
k=0;// variable del clck en timer0 ( if clock1==...) para cambiar de estado en el menu 
        // se cambia de opcion cada cierto timepo  automaticamente
        // tenemos que elecgir la opcion que necesitemos pesionando el boton 1
        while(true)
         {
            if(k==1)//opcion 1 de velocidad 
            {      
               botones();
               output_low(motor_dr);
               output_low(motor_da);
               output_low(motor_ia);
               output_high(motor_ir);
               if(btn2==0)
               {
                  ////compensacion=-15;   ///////////////////podemos descomentar aqui para poner las velocidad que queramos elegir  ahora estan desactivadas todas                              
                  delay_ms(10);           //las demas estan tambien comentadas, si queremos utilizar la opcion de menu, podemos descomentar todas la variable compensacion
                  while(true)
                  {botones();
                     if(btn1==0)// si se eligio presionando boton 1
                     {                      
                       luz();
                       clock=0; //variable para desactivar  el clock para menu
                       delay_ms(20);
                       f1();       //salta a la programacion de la pista             
                     }
                  }
               }               
            }   
            if(k==3)  // segunda opcion
            {
               botones();
               output_low(motor_dr);
               output_low(motor_da);
               output_high(motor_ia);
               output_high(motor_ir); 
               if(btn2==0)
               {               
                  //compensacion=-10;                                 
                  delay_ms(10);
                  while(true)
                  {botones();
                     if(btn1==0)
                     {
                       luz();
                       clock=0;
                       delay_ms(40);
                       f1();                    //salta a la programacion de la pista
                     }
                  }
               } 
            }  
            if(k==5) //tercera opcion
            {
               botones();
               output_low(motor_dr);
               output_high(motor_da);
               output_high(motor_ia);
               output_high(motor_ir); 
               if(btn2==0)
               {                 
                 // compensacion=10;                                 
                  delay_ms(10);
                  while(true)
                  {botones();
                     if(btn1==0)
                     {
                       
                       luz();
                       clock=0;             
                       delay_ms(40);
                       f1();                       //salta a la programacion de la pista
                     }
               }
               } 
            }
            if(k==7)// cuarta opcion
            {
               botones();
               output_high(motor_dr);
               output_high(motor_da);
               output_high(motor_ia);
               output_high(motor_ir); 
               if(btn2==0)
               {               
                 // compensacion=15;                                 
                  delay_ms(40);
                  while(true)
                  {botones();
                     if(btn1==0)
                     {
                       luz();
                       clock=0;
                       delay_ms(40);
                       f1();   //salta a la programacion de la pista
                     
                     }
                   }
               }                
            }          
            if(k==9) // apaga las opcione sy reinicia el todo para repetir las opciones
            {              
               output_low(motor_dr);
               output_low(motor_da);
               output_low(motor_ia);
               output_low(motor_ir); 
               k=1;
            }         
         }                 
      }  
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void f1()  ///
{
clock=0;  //no mover //desactivo clock de menu
//cambio(8); //funcion para cambiar ultrasonido

sensado=1; // 0 para fondo negr0 , 1 para fondo blanco
delay_ms(20);
pid_a(190); //funcion para acelerar 
pid_cont(1,190,1); // funcion para la primer a esquina de pista
pid_t(1800,190);// pid con tiempo para la parte donde estan las curva
while(true)
{
pid(190);
if(s8==1) f2(); //si choco s8 la linea  //lama a la siguiente funcion
}}

void f2(){
patron_d(1,180); //patron para la parte derecha
while(true){
pid(190);
if(s8==1) f3();

}}

void f3(){
patron_d(1,185);
while(true){
pid(190);
if(s1==1 &&s8==1 ) f4();
}}
 
 void f4(){
 patron_d(1,180);
 pid_cont(1,180,1);
 while(true){
 pid(180);
if( s8==1) f5();
 }}
 
 void f5(){
 patron_i(1,180);
 while(true){
 r_l(185,20);
 if(s1==1 &&s7==1) f6();
 }}
 void f6(){
 motores(-170,170);
 delay_ms(200);
 patron_i(1,170);
 while(true)
 {
 r_l(170,10);
 if(s2==1 &&s7==1)f7();
 }}
 
 void f7(){
 sensado=0;
 delay_ms(5);
 motores(180,180);
 delay_ms(100);
 while(true){
 r_f2(185,20);
 if(s1==1 && s2==1 && s7==1 &&s8==1) f8();
 }}
 
 void f8(){ //camobio sensado
 sensado=1;
 delay_ms(50);
 motores(180,187);
 delay_ms(300);
 while(true){
 pid2(175);
 if(s1==0 &&  s2 ==0 &&s3==0 &&s4==0 &&s5==0 &&s6==0 &&s7==0 &&s8==0 ) f9();
 }}
 
 void f9(){
 patron_d(1,185);
 while(true)
 {
pid(180);
if(s8==1 ) f10();
 }
 }
 
void f10(){
patron_a(2,180);
motores(180,-170);
delay_ms(200);
patron_d(2,190);
while(true){
pid(190);
if(s8==1){
patron_cruz_d(1,180);
 f11();}
}}
//////////////////pic
void f11(){
while(true){
pid(180);
if(s2==1 &&s7==1) f12();
}}

void f12(){
sensado=0;
delay_ms(50);
while(true){
pid(190);
if(s1==1) f13();
}}
void f13(){
patron_a(1,180);
patron_d(1,185);
while(true){
r_l(185,20);
if(s7==1) f14();
}}

void f14(){
patron_cruz_i(1,185);
while(true)
{
r_l(180,15);
if(s8==1 &&s7==1)f15();
}}

void f15(){
sensado=1;
delay_ms(30);
patron_a(1,180);
while(true)
{
r_l(180,15);
if(s2==1) f16();
}}

void f16(){
patron_cruz_d(1,180);
pid_a(190);
while(true)
{
pid(200);
if(s1==1 &&s8==1) f17();
}}

void f17(){
r_l_t(300,185,15);
while(true){
pid(180);
if(s3==1  && s6==1 ) f18();
}}

void f18(){
motores(190,0);
while(true){
if(s8==0 &&s7==0 && s6==0 &&s5==0 &&s4==0 &&s3==0 && s4==0 ) f19();
}}

void f19(){
patron_i(2,190);
pid2_t(1900,180);
while(true){
pid2(180);
if(s1==1 &&s8==1) 
{
patron_cruz_i(1,180);
f20();
}}}

void f20(){
r_bd_t(1000,170,15);
while(true){
r_bd(170,15);
if(s8==1) f21();
}}

void f21(){
motores(-175,175);
while(true){
if(s1==0 &&s2==0 &&s3==0 &&s4==0 &&s5==0 &&s6==0 && s8==0 ) f22();
}}

void f22(){
patron_d(2,175);
r_f2_t(200,170,10);
motores(160,175);
delay_ms(400);
sensado=0;
delay_ms(20);
while(true) {
  r_bi(170,15);
  if(s1==1&&s2==1&&s3==1) f23();
}}

void f23(){
sensado=1;
delay_ms(20);
while(true){
r_bd(170,10);
if(s1==1) f24();
}}

void f24(){
patron_a(1,170);
delay_ms(10);
pid_cont(2,178,2);
patron_i(1,175);
while(true){
r_l(180,15);
if(s1==0 &&s2==0 &&s3==0 &&s4==0 &&s5==0 &&s6==0 &&s7==0 &&s8==0) f25();
}}

void f25(){
patron_d(1,175);
while(true){
r_bd(175,10);
if(s8==1) f26();
}}

void f26(){
patron_d(1,175);

while(true){
r_l(180,15);
if(s4==0 &&s5==0) f27();
}}

void f27(){
motores(180,180);
delay_ms(200);
while(true){
r_l(180,15);
if( s4==0 &&s5==0  ) f28();
}}

void f28(){
motores(180,180);
while(true){
r_f2(175,10);
if(s8==1 && s7==1) f29();
}}

void f29(){
patron_a(1,170);
patron_d(1,175);
while(true){
pid(180);
if(s1==1 ) f30();
}}

void f30(){ //final
patron_i(1,178);
while(true){
pid(190);
if(s2==1 &&s3==1 &&s4==1 &&s5==1 &&s6==1 &&s7==1) parar();
}}



//"En la simplesa esta la Genialidad"
