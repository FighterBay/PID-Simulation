#include <iostream>
#include <ctime>
#include <cstdlib>
#include <unistd.h>

#include<string.h>    
#include<sys/socket.h>
#include<arpa/inet.h> 



using namespace std;

class PID
{

private:
  float kp;
  float ki;
  float kd;
  float i_error;
  float last_error;
  int max;
public:
  PID(float k1, float k2, float k3)
  {
    kp = k1;
    ki = k2;
    k3 = kd;
    i_error = 0;
    last_error = 0;
    max = 400;
  }
  void update_c(float k1, float k2, float k3)
  {
    kp = k1;
    ki = k2;
    k3 = kd;

  }
  int compute(float input, float target, float dt)
  {
    float error = target - input;

    i_error += (error + last_error) * dt;

    float d_error = (error-last_error)/dt;
    last_error = error;
    //cout<<"Integral error:"<<  (int)(error*kp + i_error*ki + d_error*kd)<<"\n";
    if((int)(error*kp + i_error*ki + d_error*kd) > max )
      return max;
    if(-1 * (int)(error*kp + i_error*ki + d_error*kd) > max )
      return -1*max;
    return (int)(error*kp + i_error*ki + d_error*kd);

  }
};


float ang_dis(float tor,float moi,float theta,float dt){
    float ang_acc = tor/moi;

    theta =  theta * 3.141592653589793 / 180.0;
    theta =  theta + (0.5*ang_acc*dt*dt);

    return theta * 180.0 / 3.141592653589793;
  }


  float valmap(float value, float istart, float istop, float ostart, float ostop){
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));

  }

int main()
{

  PID roll_pid(1,1,1);

  float start_pitch = 50;
  float start_roll = 50;
  float start_yaw = 0;
  int max_pwm = 2000;
  int min_pwm = 1000;
  float MOI = 0.07;
  float max_thrust = 0.8; //in kg per rotor
  int thrust_ini = 1400; // Inital thrust PWM
  float arm_length = 0.05; //arm length in meters
  float dt = 0.2;
  float e_roll = 0.0;
  float e_pitch = 0.0;
  float e_yaw = 0.0;
  unsigned long int sec= time(NULL);

  int M1_PWM = 0; //thrust_ini - roll + rnn
  int M2_PWM = 0; //thrust_ini + roll + rnn
  int M3_PWM = 0; //thrust_ini + roll + rnn
  int M4_PWM = 0; //thrust_ini - roll + rnn
  int cur_roll = 0;

  float F1,F2,net_f; //Forces on both the side of the copter.

    int socket_desc;
    struct sockaddr_in server;
    string msg; //guru ram rahim

    //Create socket
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);

    if (socket_desc == -1)
    {
        cout<<"Couldn't create socket";
    }

    server.sin_addr.s_addr = inet_addr("127.0.0.1");
    server.sin_family = AF_INET;
    server.sin_port = htons( 5002 );

    //Connect to remote server
    if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        cout<<"Connection error with the server";
        return 1;
    }

    cout<<"Connected successfully";

  srand(time(NULL));
while(1)
{
  cur_roll = roll_pid.compute(start_roll,e_roll,dt);


  M1_PWM = thrust_ini - cur_roll; //+ rand() % 10;
  M2_PWM = thrust_ini + cur_roll; //+ rand() % 10;
  M3_PWM = thrust_ini + cur_roll;// + rand() % 10;
  M4_PWM = thrust_ini - cur_roll;// + rand() % 10;

  //cout<<M1_PWM<< " " << M2_PWM << " " << M3_PWM << " " << M4_PWM;
  F1 = (valmap(M2_PWM,min_pwm,max_pwm,0,max_thrust) + valmap(M3_PWM,min_pwm,max_pwm,0,max_thrust))*0.70710678118; // 0.70710678118 is cos(45 degrees)
  F2 = (valmap(M1_PWM,min_pwm,max_pwm,0,max_thrust) + valmap(M4_PWM,min_pwm,max_pwm,0,max_thrust))*0.70710678118;  // 0.70710678118 is cos(45 degrees)



  net_f = (F1-F2) * arm_length;

  //cout<<"\n"<<net_f<<" forces "<<net_f<<endl;

  start_roll = ang_dis(net_f,MOI,start_roll,dt);
cout<<start_roll<<"\n";

msg = to_string(start_roll) + "\n";
    if( send(socket_desc , msg.data() , msg.size() , 0) < 0)
    {
        cout<<"Sending failed. :(";
        return 1;
    }
    cout<<"Data Sent\n";

usleep(dt * 1000000);
}




cout<<"Yolo";
return 0;
}
