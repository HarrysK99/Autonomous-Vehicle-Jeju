#include <ros.h>
#include <std_msgs/Int8MultiArray.h>

/*=====ROS 통신을 위한 변수========*/
  ros::NodeHandle nh;
  std_msgs::Int8MultiArray msg;
  
  int8_t _speed=0; //0 - 10 km/h
  int8_t angle=0; //-30 - 30 degree
  int8_t gear=0;
/*==============================*/


/*=====하드웨어 제어를 위한 변수========*/
  int encoder_pot = A8;    // 조향모터 가변저항 및 조향모터의 PID제어를 위한 변수
  int val;
  int encoder_val;
  float kp = 0.1;
  float ki = 0.00000 ;
  float kd = 2.00;
  float Theta, Theta_d;
  int dt;
  unsigned long t;
  unsigned long t_prev = 0;
  int val_prev =0;
  float e, e_prev = 0, inte, inte_prev = 0;
  float Vmax = 24;
  float Vmin = -24;
  float V = 0.1;
  void SteerCon(); // 조향 모터의 PID제어 함수
  int St_M = 512;// 직진 조향값
  int steering_angle = 300;
  
  const byte interruptPinA = 2;  // 구동모터의 엔코더센서 핀번호
  const byte interruptPinB = 3;
  volatile long EncoderCount = 0; // 엔코더 값 변수
  
  const byte PWMPin = 12; // 조향 모터의 모터드라이브 PWM,DIR 핀번호
  const byte DirPin1 = 13;
  
  int PWMone = 10; // 구동 모터의 모터드라이브 PWM1,PWM2,DIR1,DIR2 핀번호
  int Dirone = 11;
  int PWMtwo = 8;
  int Dirtwo = 9;
  void mDrive(); // 구동 모터의 OPEN LOOP 제어 함수
  
  int Speed, Steer,HSteer=0,HSpeed=0; // 상위 제어기와 통신을 위한 변수
  byte State, ESTOP, Mode,HGear=0;
  byte ALIVE = 0;
  char SendMessage[17];
  
  int Thro,Aux,Gear,ALIE,T,A,G,AL; // 조종기를 위한 변수
  void controller(); // 조종기 통신 함수
/*==============================*/

/*=====ROS 통신을 위한 함수========*/
void msgCallback(const std_msgs::Int8MultiArray& msg) {
  _speed=msg.data[0];
  angle=msg.data[1];
  gear=msg.data[2];
}

ros::Subscriber<std_msgs::Int8MultiArray> sub("SpeedAngleGear", &msgCallback);
/*==============================*/

void setup(){
  /*======ROS 통신========*/
  nh.initNode();
  nh.subscribe(sub);
  /*=====================*/
  pinMode(DirPin1, OUTPUT); // 조향 모터
  pinMode(PWMone,OUTPUT);
  pinMode(Dirone,OUTPUT); // 구동 모터
  pinMode(PWMtwo,OUTPUT);
  pinMode(Dirone,OUTPUT);
  pinMode(interruptPinA, INPUT_PULLUP); // 엔코더 핀
  pinMode(interruptPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE); // 엔코더 값을 읽기 위한 인터럽트 서비스 루틴
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE); 

  //State=1;
}

void loop(){
  /*=====for Debugging=======*/
  char ch[20]={0};
  if(angle>0){
    ch[0]=angle/10+48;
    ch[1]=angle%10+48;
  }
  else{
    ch[0]='-';
    ch[1]=-1*angle/10+48;
    ch[2]=-1*angle%10+48;
  }
  nh.loginfo(ch);
  /*============================*/
  HSpeed=_speed;
  HSteer=angle;
  HGear=gear;
 
  HSpeed = map(HSpeed, 0, 10, 0, 255);
  HSteer = map(HSteer, -30, 30, 0, 1000);
  Mdrive(HSpeed,HGear);    //SPEED값 제어
  SteerCon(HSteer);    //STEER값 제어
  
  nh.spinOnce();
  delay(1);
}

void SteerCon(float Q) { // 조향모터의 제어를 위한 함수, Q = 조향 목표값  
    val = Q;                          
    if(val > St_M + steering_angle){  // 조향 최대값 제한
      val = St_M + steering_angle;
    }
    else if(val < St_M - steering_angle){ // 조향 최소값 제한
      val = St_M - steering_angle;
    }
    encoder_val = analogRead(encoder_pot);               // Read V_out from Feedback Pot
    t = millis();
    
    dt = (t - t_prev);                                  // Time step
    Theta = val;                                        // Theta= Actual Angular Position of the Motor
    Theta_d = encoder_val;                              // Theta_d= Desired Angular Position of the Motor

    e = Theta_d - Theta;                                // Error -> 목표 값에서 현재 값을 빼야하는 거 아닌가?
    inte = inte_prev + (dt * (e + e_prev) / 2);         // Integration of Error
    V = kp * e + ki * inte + (kd * (e - e_prev) / dt) ; // Controlling Function

    if (V > Vmax) {
      V = Vmax;
      inte = inte_prev;
    }
    if (V < Vmin) {
      V = Vmin;
      inte = inte_prev;
      val_prev= val;
    }
    int PWMval = int(255 * abs(V) / Vmax);
    if (PWMval > 200) {
      PWMval = 200;
    }
    if (V > 0.5) {
      digitalWrite(DirPin1, HIGH);
      analogWrite(PWMPin, PWMval);
    }
    else if (V < -0.5) {
      digitalWrite(DirPin1, LOW);
      analogWrite(PWMPin, PWMval);
    }
    else {
      digitalWrite(DirPin1, LOW);
      analogWrite(PWMPin, 0);
    }
    t_prev = t;
    inte_prev = inte;
    e_prev = e;
} 

void Mdrive(int q,int w){ //구동모터의 제어를 위한 함수, q = Speed, w = 정,역방향 지정
  // 정방향
  if(w == 0){
    digitalWrite(Dirtwo,1);
    analogWrite(PWMtwo,q);
    digitalWrite(Dirone,0);
    analogWrite(PWMone,q);
  }

  // 역방향
  else if(w == 2){
    digitalWrite(Dirtwo,0);
    analogWrite(PWMtwo,q);
   digitalWrite(Dirone,1);
   analogWrite(PWMone,q);
  }

  // e-stop
  else if(w == 1){
    digitalWrite(Dirtwo,0);
    analogWrite(PWMtwo,0);
    digitalWrite(Dirone,0);
    analogWrite(PWMone,0);
  }
}

void ISR_EncoderA() { // 엔코더 값을 저장하기 위한 내부 인터럽트 서비스 함수
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount++;
    }
    else {
      EncoderCount--;
    }
  }

  else {
    if (PinA == HIGH) {
      EncoderCount--;
    }
    else {
      EncoderCount++;
    }
  }
}
void ISR_EncoderB() { // 엔코더 값을 저장하기 위한 내부 인터럽트 서비스 함수
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCount--;
    }
    else {
      EncoderCount++;
    }
  }

  else {
    if (PinB == HIGH) {
      EncoderCount++;
    }
    else {
      EncoderCount--;
    }
  }
}

void controller() {
    ALIE = pulseIn(A0, HIGH, 50000);  //CH1,좌우 //ALIE는 STEER값임.
    // 아날로그3번 핀의 pwm입력신호가 LOW로 바뀐 순간부터 HIGH로 바뀌는 순간까지의 경과시간을 마이크로초 단위로 Rudd변수에 저장.
    // 만약, 50ms시간동안 펄스의 변화가 없다면, 0값이 Rudd변수에 저장된다.
    Thro = pulseIn(A1, HIGH, 50000);  //CH2, 앞뒤  
    // 아날로그5번 핀의 pwm입력신호가 LOW로 바뀐 순간부터 HIGH로 바뀌는 순간까지의 경과시간을 마이크로초 단위로 Thro변수에 저장. 
    // 만약, 50ms시간동안 펄스의 변화가 없다면, 0값이 Thro변수에 저장된다.
    Gear = pulseIn(A2, HIGH, 50000);  //SWA,CH5
    // 아날로그2번 핀의 pwm입력신호가 LOW로 바뀐 순간부터 HIGH로 바뀌는 순간까지의 경과시간을 마이크로초 단위로 Gear변수에 저장. 
    // 만약, 50ms시간동안 펄스의 변화가 없다면, 0값이 Gear변수에 저장된다.
    Aux = pulseIn(A3, HIGH, 50000);   //VRA,CH6 +1 전진, -1 후진 
    // 아날로그1번 핀의 pwm입력신호가 LOW로 바뀐 순간부터 HIGH로 바뀌는 순간까지의 경과시간을 마이크로초 단위로 Aux변수에 저장. 
    // 만약, 50ms시간동안 펄스의 변화가 없다면, 0값이 Aux변수에 저장된다.
    
    
    T = Thro/50; Thro = T*50; A = Aux/50; Aux = A*50; G = Gear/50; Gear = G*50; AL = ALIE/50; ALIE = AL*50; // 조종기 신호 안정화
    
    if(ALIE>1700){ ALIE=1700; } else if(ALIE<1000){ ALIE = 1000; } // 조종기 ALIE(조향) 신호 최대 최소값 제한
    if(Thro>1600){ Thro=1600; }  else if(Thro<1000){ ESTOP = 1;} else{ESTOP=0;} // 조종기 THRO(구동) 신호 최대값 제한, Throttle Cut할때 ESTOP신호 주도록 설정 
    //1850 대신에 1600으로 제한함
    Speed = map(Thro, 1050, 1850, -255, 255); // 조종기 THRO 신호를 Speed값으로 변환 
    if(Speed>255){ Speed = 255; } else if(Speed<10 or ESTOP == 1 ){ Speed = 0; } // Speed값 최대 최소값 제한 및 ESTOP상황에서 멈추도록 설정
    Steer = map(ALIE, 1200, 1700,1023, 0); // 조종기 ALIE 신호를 Steer값으로 변환 //초속 3.6
    if(Steer>St_M+400){ Steer = St_M+400; } else if(Steer<St_M-400){ Steer = St_M-400; } // Steer값 최대 최소값 제한
    else if(Steer>St_M-20 and Steer<St_M+20){ Steer = St_M;} // 조향 유지를 위한 DeadZone
    
    if(Aux>1700){ Mode = 0;} else if(Aux<1700 and Aux>1300){ Mode = 1; Speed = 0; } else if(Aux<1300){ Mode = 2; }
    // 조종기 Aux신호를 주행 상태결정하는 Mode변수로 치환, Mode=0:직진, Mode=1:중립, Mode=2:후진 
    
    if(Gear>500 and Aux > 500){ // 조종기가 On일때 
     if(Gear<1200){ State = 1; } else{ State = 0; } 
    // 조종기 Gear신호를 통해 자율주행모드, 수동주행모드 설정. State = 1:Auto mode, State = 0: Manual mode
     }
    else{ Speed = 0; Steer = 500; }  // 조종기가 Off일때 속도 0 조향 0
}
