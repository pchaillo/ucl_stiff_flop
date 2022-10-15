//#include <stdio.h>

uint8_t fed_pressure[7];
float cmd_pressure[6];
uint8_t analog_pin[6] = {2,3,5,6,7,8};
char start_byte;
int a;
unsigned long serial_true_time, serial_false_time;


// New variables :
float FromPC1, FromPC2, FromPC3, FromPC4, FromPC5, FromPC6;
float PMax = 180.0 ; // Max pressure kPa

void setup() {
  SerialUSB.begin(115200);
 // Serial.setTimeout(1);

 // SerialUSB.begin(115200); 
  // set PWM mode, 0-5 correspond to pressure regulator 1-6
  pinMode(analog_pin[0],OUTPUT);
  pinMode(analog_pin[1],OUTPUT);
  pinMode(analog_pin[2],OUTPUT);
  pinMode(analog_pin[3],OUTPUT);
  pinMode(analog_pin[4],OUTPUT);
  pinMode(analog_pin[5],OUTPUT);
  serial_true_time = 0;
  serial_false_time = 0;
 /* while (!Serial)
  {;
    } */
//  delay(1000);
}

void loop() {
  
  /* Version JIALEI
  // put your main code here, to run repeatedly:
  if(SerialUSB.available())
  {
    Receive_cmd_Pressure();
  }
  // convert the cmd pressure to PWM and send out the PWM 
  analogWrite(0,int(cmd_pressure[0]/2.772*255));
  analogWrite(1,int(cmd_pressure[1]/2.772*255));
  analogWrite(2,int(cmd_pressure[2]/2.772*255));
  analogWrite(3,int(cmd_pressure[3]/2.772*255));
  analogWrite(4,int(cmd_pressure[4]/2.772*255));
  analogWrite(5,int(cmd_pressure[5]/2.772*255));
  // ms
  delay(5); */

  /*
 while (!Serial.available());
  Serial.print(100); */
  
  //Serial.println(loop_nb);
  
  // NEW ONE

    if (SerialUSB.available()>0)
  {
    serial_true_time = millis();
    FromPC1 = SerialUSB.parseFloat();
    FromPC2 = SerialUSB.parseFloat();   
    FromPC3 = SerialUSB.parseFloat();
    FromPC4 = SerialUSB.parseFloat();
    FromPC5 = SerialUSB.parseFloat();   
    FromPC6 = SerialUSB.parseFloat();
 
    if (SerialUSB.read()!= '\n')
    {
      SerialUSB.print("Warning parsing numbers!");
      SerialUSB.print("got: ");
      SerialUSB.print(FromPC1);
      SerialUSB.print(",");
      SerialUSB.print(FromPC2);
      SerialUSB.print(",");
      SerialUSB.println(FromPC3); 
      SerialUSB.print(",");    
      SerialUSB.print(FromPC4);
      SerialUSB.print(",");
      SerialUSB.print(FromPC5);
      SerialUSB.print(",");
      SerialUSB.println(FromPC6);     
    }    
    else
    {
    // SerialUSB.print("Test1");
        if (FromPC1 > PMax || FromPC2 > PMax || FromPC3 > PMax)
        {
          SerialUSB.print("Warning, some pressures too high! Something wrong with: ");          
        }
        else
        {
      //    SerialUSB.println("testla");
        /*  SerialUSB.print("Good values : "); 
          SerialUSB.print("got: PC1 = ");
        SerialUSB.print(FromPC1);
        SerialUSB.print(", PC 2 = ");
        SerialUSB.print(FromPC2);
        SerialUSB.print(", PC3 = ");
        SerialUSB.print(FromPC3);    
                  SerialUSB.print("got: PC4 = ");
        SerialUSB.print(FromPC4);
        SerialUSB.print(", PC 5 = ");
        SerialUSB.print(FromPC5);
        SerialUSB.print(", PC6 = ");
        SerialUSB.println(FromPC6);    */
          setBar(0,FromPC1);  
          setBar(1,FromPC2);  
          setBar(2,FromPC3);  
          setBar(3,FromPC4);  
          setBar(4,FromPC5);  
          setBar(5,FromPC6);         
        }
    }
  }
  else
  { serial_false_time = millis();
    if (serial_false_time-serial_true_time > 500)
    {
      setBar(0,0);  
      setBar(1,0);  
      setBar(2,0);  
      setBar(3,0);  
      setBar(4,0);  
      setBar(5,0);   
   }
    
  }
}

void setBar(int chan, float value)
{ 
  // 10V = 3bar
  // 1V = 0.3 bar
  float pressure = value/100;
 // int v = constrain(PWM,0,MAX_PWM);
  
  // analogWrite(ports[chan-1],v);
  //SerialUSB.println(pressure);
  analogWrite(analog_pin[chan],int(pressure/2.78*255));
}


/*
// OLD NON USED FUNCTIONS
void Send_Pressure()
{
  float pressure_temp;
  fed_pressure[0] = 250;
  for(int i=0; i++;i<5)
  {
    pressure_temp = (analogRead(analog_pin[i])*10.098/1023-0.35)/3;
    fed_pressure[i+1] = uint8_t(pressure_temp*100); // 1000 scale
  }
  // send the pressure out
  SerialUSB.write(fed_pressure,sizeof(fed_pressure));   
}

void Receive_cmd_Pressure()
{
  start_byte = Serial.read();
  if (start_byte == 's'){
    for (int i=0; i++;i<5)
    {
      cmd_pressure[i] = Serial.read()/100;
    }
  }  
}*/
