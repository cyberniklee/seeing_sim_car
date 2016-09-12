#define CMD_READ_ENCODER      'e'
#define CMD_RESET_ENCODER    'r'
#define CMD_UPDATE_PID      'u'
#define CMD_MOTOR_SPEEDS    'm'
#define CMD_GET_BAUDRATE    'b'


int pid_kp = 20;
int pid_kd = 12;
int pid_ki = 0;
int pid_ko = 50;

int right_rear_pwm = 9;
int left_rear_pwm = 11;

int left_rear_forward_pin = 7;
int left_rear_backward_pin = 6;
int right_rear_forward_pin = 4;
int right_rear_backward_pin = 5;

int right_roll_count_int = 0;
int left_roll_count_int = 1;

long right_enc = 0;
long left_enc = 0;

#define MAX_PWM 75
#define PID_RATE 30
#define PID_INTERVAL 1000/PID_RATE
unsigned long next_pid_time = PID_INTERVAL;

#define AUTO_STOP_INTERVAL 2000
long last_motor_cmd_time = AUTO_STOP_INTERVAL;

char chr;
int arg = 0;
int index = 0;
char argv1[16];
char argv2[16];
char cmd;
long arg1;
long arg2;

void resetCommand()
{
  cmd = NULL; 
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = arg2 = 0;
  arg = 0;
  index = 0;
}

void setMotorSpeed(int left_speed, int right_speed)
{
  if(left_speed > 0)
  {
    digitalWrite(left_rear_forward_pin, 1);
    digitalWrite(left_rear_backward_pin, 0);
    analogWrite(left_rear_pwm, left_speed);
  }else if(left_speed < 0)
  {
    digitalWrite(left_rear_forward_pin, 0);
    digitalWrite(left_rear_backward_pin, 1);
    analogWrite(left_rear_pwm, -left_speed);
  }else{
    digitalWrite(left_rear_forward_pin, 0);
    digitalWrite(left_rear_backward_pin, 0);
    analogWrite(left_rear_pwm, 0);
  }
  
  if(right_speed > 0)
  {
    digitalWrite(right_rear_forward_pin, 1);
    digitalWrite(right_rear_backward_pin, 0);
    analogWrite(right_rear_pwm, right_speed);
  }else if(right_speed < 0)
  {
    digitalWrite(right_rear_forward_pin, 0);
    digitalWrite(right_rear_backward_pin, 1);
    analogWrite(right_rear_pwm, -right_speed);
  }else{
    digitalWrite(right_rear_forward_pin, 0);
    digitalWrite(right_rear_backward_pin, 0);
    analogWrite(right_rear_pwm, 0);
  }
}

unsigned long next_pid_cycle = PID_INTERVAL;

struct set_point_info
{
  double target_ticks_per_frame;
  long encode;
  long pre_encode;
  int pre_input;
  int int_term;
  long output;
};

void resetEncoder()
{
  right_enc = 0;
  left_enc = 0;
}

struct set_point_info left_pid, right_pid;

int is_moving = 0;

void resetPID()
{
  left_pid.target_ticks_per_frame = 0.0;
  left_pid.encode = left_enc;
  left_pid.pre_encode = left_pid.encode;
  left_pid.output = 0;
  left_pid.pre_input = 0;
  left_pid.int_term = 0;
  
  right_pid.target_ticks_per_frame = 0.0;
  right_pid.encode = right_enc;
  right_pid.pre_encode = right_pid.encode;
  right_pid.output = 0;
  right_pid.pre_input = 0;
  right_pid.int_term = 0;
}

void calcPID(struct set_point_info *p)
{
  long perror;
  long output;
  int input;
 
  input = p->encode - p->pre_encode;
  perror = p->target_ticks_per_frame - input;
  
  output = (pid_kp*perror - pid_kd*(input - p->pre_input) + p->int_term)/pid_ko;
  
  p->pre_encode = p->encode;
  
  output += p->output;
  
  if(output >= MAX_PWM)
    output = MAX_PWM;
  else if(output <= -MAX_PWM)
    output = -MAX_PWM;
  else p->int_term += pid_ki*perror;
  
  p->output = output;
  p->pre_input = input;
}

void execPID()
{
  left_pid.encode = left_enc;
  right_pid.encode = right_enc;
  
  if(!is_moving)
  {
    if(left_pid.pre_input != 0 || right_pid.pre_input != 0)
    {
      resetPID();
    } 
    return;
  }
  
  calcPID(&left_pid);
  calcPID(&right_pid);

  setMotorSpeed(left_pid.output, right_pid.output);
}

void execCommand()
{
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd)
  {
    case CMD_READ_ENCODER:
      Serial.print(left_enc);
      Serial.print(" ");
      Serial.println(right_enc);
      break;
    case CMD_RESET_ENCODER:
      resetPID();
      resetEncoder();
      Serial.println("OK");
      break;
    case CMD_UPDATE_PID:
      while((str = strtok_r(p, ":", &p)) != '\0')
      {
        pid_args[i] = atoi(str);
        i++; 
      }
      pid_kp = pid_args[0];
      pid_kd = pid_args[1];
      pid_ki = pid_args[2];
      pid_ko = pid_args[3];
      Serial.println("OK");
      break;
    case CMD_MOTOR_SPEEDS:
      last_motor_cmd_time = millis();
      if(arg1 == 0 && arg2 == 0)
      {
        setMotorSpeed(0, 0);
        is_moving = 0;
      }else{
        is_moving = 1; 
      }
      left_pid.target_ticks_per_frame = arg1;
      right_pid.target_ticks_per_frame = arg2;
      Serial.println("OK");
      break;
    case CMD_GET_BAUDRATE:
      Serial.println("115200");
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(right_rear_pwm, OUTPUT);
  pinMode(left_rear_pwm, OUTPUT);
  
  attachInterrupt(right_roll_count_int, rightCountRoutine, FALLING);
  attachInterrupt(left_roll_count_int, leftCountRoutine, FALLING);
  
  Serial.begin(115200);
  
  resetPID();
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available())
  {
    chr = Serial.read();
    if(chr == 13)
    {
      if(arg == 1)
        argv1[index] = NULL;
      else if(arg == 2)
        argv2[index] = NULL;
      execCommand();
      resetCommand();
    }else if(chr == ' ')
    {
      if(arg == 0)
      {
        arg = 1;
      }
      else if(arg == 1)
      {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }else{
      if(arg == 0)
      {
        cmd = chr;
      }else if(arg == 1)
      {
        argv1[index] = chr;
        index++; 
      }else if(arg == 2)
      {
        argv2[index] = chr;
        index++; 
      }
    }
  }
  if(millis() > next_pid_time)
  {
    execPID();
    next_pid_time += PID_INTERVAL;
  }
  
  if((millis() - last_motor_cmd_time) > AUTO_STOP_INTERVAL)
  {
    setMotorSpeed(0, 0);
    is_moving = 0; 
  }
}

void rightCountRoutine()
{
/*
  if((millis() - time1) > 1)
    right_roll_count++;
  time1 = millis();
  */
  if(right_pid.output > 0)
    right_enc++;
  else if(right_pid.output < 0) 
    right_enc--;
}

void leftCountRoutine()
{
  /*
  if((millis() - time2) > 1)
    left_roll_count++;
  time2 = millis();
  */
  if(left_pid.output > 0)
    left_enc++;
  else if(left_pid.output < 0) 
    left_enc--;
}
