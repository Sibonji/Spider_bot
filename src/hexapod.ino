// Библиотека для работы с Multiservo Shield
// https://github.com/amperka/Multiservo
#include "Multiservo.h"
#include <math.h>
#include <IRremote.hpp>        
#include <Servo.h>
Servo myservo;

#define IR_RECEIVE_PIN 11        // Сигнальный вывод, подключенный к Ардуино
#define SPEED_1      5 
#define DIR_1        4
 
#define SPEED_2      6
#define DIR_2        7

#define GUN_PIN      2

int SPEED_MAX = 255;

int curr_speed = SPEED_MAX;
auto comand_old = 0xB54AFF00;

// Задаём количество сервоприводов
constexpr uint8_t MULTI_SERVO_COUNT = 18;
// Создаём массив объектов для работы с сервомоторами
Multiservo multiservo[MULTI_SERVO_COUNT];


// Задаём имя пина к которому подключён сервопривод
constexpr uint8_t MULTI_SERVO_PIN = 18;

// Переменная для хранения текущей позиции сервомотора
int pos = 0;
double J1L = 48.5;
double J2L = 50;
double J3L = 112.2;
double j3_all_offset = 3;

int offset[MULTI_SERVO_COUNT];
int direction[MULTI_SERVO_COUNT];

void setup() {
  myservo.attach(10);
  pinMode(GUN_PIN, OUTPUT);
  for (int i = 4; i < 8; i++) {     
    pinMode(i, OUTPUT);
  }
  Serial.begin(9600); 
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // 
  // direction[3] = -1;
  // direction[4] = -1;
  // direction[5] = -1;
  // direction[6] = -1;
  // direction[7] = -1;
  // right side
  int count = 0;
  for (count = 0; count < 3; count++) {
    multiservo[count*3].attach(count*3);
    multiservo[count*3].write(90);
    multiservo[count*3+1].attach(count*3+1);
    multiservo[count*3+1].write(80);
    multiservo[count*3+2].attach(count*3+2);
    multiservo[count*3+2].write(70);
  }

  // left side
  for (count = 3; count < 6; count++) {
    multiservo[count*3].attach(count*3);
    multiservo[count*3].write(90);
    multiservo[count*3+1].attach(count*3+1);
    multiservo[count*3+1].write(100);
    multiservo[count*3+2].attach(count*3+2);
    multiservo[count*3+2].write(110);
  }

  // multiservo[MULTI_SERVO_PIN].attach(MULTI_SERVO_PIN);
  // multiservo[MULTI_SERVO_PIN].write(179);

    // Перебираем значения моторов от 0 до 17
    // offset[0] = 90;
    // offset[1] = 50;
    // offset[2] = 90;
    // for (int count = 0; count < 3; count++) {
    //     // Подключаем сервомотор
    //     multiservo[count].attach(count);
    //     multiservo[count].write(offset[count]);
    // }
    // for (int count = 0; count < 5; count++) {
    //     // Подключаем сервомотор
    //     multiservo[count*2 + 1].attach(count);
    //     multiservo[count*2 + 1].write(0);
    //     // Подключаем сервомотор
    //     multiservo[count*2 + 2].attach(count);
    //     multiservo[count*2 + 2].write(180);
    // }
}

void count_2d_pos(double Y, double Z ) {
  double j3 = acos((J2L*J2L + J3L*J3L - Y*Y - Z*Z)/(2*J2L*J3L));
  double j2 = acos((J2L*J2L - J3L*J3L + Y*Y + Z*Z)/(2*J2L*sqrt(Y*Y + Z*Z))) - atan(Z/Y);

  int servo_j2 = 90 - (int) (j2/3.1415*180);
  int servo_j3 = (int) (j3/3.1415*180);
  multiservo[1].write(servo_j2);
  multiservo[2].write(servo_j3 + 3);
}

void count_3d_pos_right(double X, double Y, double Z, int leg_num) {
  double H = sqrt(Y*Y + X*X) - J1L;
  double j1 = atan(X/Y);
  double j3 = acos((J2L*J2L + J3L*J3L - H*H - Z*Z)/(2*J2L*J3L));
  double j2 = acos((J2L*J2L - J3L*J3L + H*H + Z*Z)/(2*J2L*sqrt(H*H + Z*Z))) - atan(Z/H);

  int servo_j1 = 90 + (int) (j1/3.1415*180);
  int servo_j2 = 90 - (int) (j2/3.1415*180);
  int servo_j3 = (int) (j3/3.1415*180);
  multiservo[leg_num*3].write(servo_j1);
  multiservo[leg_num*3+1].write(servo_j2);
  multiservo[leg_num*3+2].write(servo_j3 + 3);
}

void count_3d_pos_left(double X, double Y, double Z, int leg_num) {
  double H = sqrt(Y*Y + X*X) - J1L;
  double j1 = atan(X/Y);
  double j3 = acos((J2L*J2L + J3L*J3L - H*H - Z*Z)/(2*J2L*J3L));
  double j2 = acos((J2L*J2L - J3L*J3L + H*H + Z*Z)/(2*J2L*sqrt(H*H + Z*Z))) - atan(Z/H);
  int servo_j1 = 90 + (int) (j1/3.1415*180);
  int servo_j2 = 90 + (int) (j2/3.1415*180);
  int servo_j3 = 180 - (int) (j3/3.1415*180);
  multiservo[leg_num*3].write(servo_j1);
  multiservo[leg_num*3+1].write(servo_j2);
  multiservo[leg_num*3+2].write(servo_j3 + 3);
}

double z_count(double z_offset, double x) {
  double parabola = -30 + x*x*3/160;
  return z_offset + parabola;
}

void loop(){
  double start = -25.0;
  double end = 25.0;
  double z_offset = 100.0;
  double z = 0;
  double i_other;
  for (double i = start; i <= end; i = i + 5.0) {
    z = z_count(z_offset, i);
    count_3d_pos_left(0-i, 100.0, z, 3);
    count_3d_pos_left(0-i, 100.0, z, 5);
    count_3d_pos_right(i, 100.0, z, 1);
    count_3d_pos_right(0-i, 100.0, z_offset, 0);
    count_3d_pos_right(0-i, 100.0, z_offset, 2);
    count_3d_pos_left(i, 100.0, z_offset, 4);
    delay(50);
  }
  for (double i = end; i >= start; i = i - 5.0) {
    z = z_count(z_offset, 0-i);
    count_3d_pos_left(0-i, 100.0, z_offset, 3);
    count_3d_pos_left(0-i, 100.0, z_offset, 5);
    count_3d_pos_right(i, 100.0, z_offset, 1);
    count_3d_pos_right(0-i, 100.0, z, 0);
    count_3d_pos_right(0-i, 100.0, z, 2);
    count_3d_pos_left(i, 100.0, z, 4);
    delay(50);
  }
}

