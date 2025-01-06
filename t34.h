#pragma once
#include <chrono>
#include "depthai/depthai.hpp"
#include <iostream>
#include <unistd.h>
#include <pigpio.h>
#include <string>
#include <wiringPi.h>
#include <stdlib.h>
#include <cmath>
#include <wiringPiI2C.h> 

// Function prototypes with comments explaining their functionality
void setup_PWM(int pin); // Funkcja odpowiedzialna za ustawienie czestotliwosci, maksymalnego wypelnienia oraz wyjscia dla sygnalu PWM na danym pinie.

void set_DIR_PIN(int pin, int state); // Funkcja odpowiedzialna za ustawienie stanu wysokiego lub niskiego na danym pinie (uzywana aby okreslic kierunek silnika).

void rotating_idle(); // Funkcja ktora powoduje ze pojazd kreci sie dookola wlasnej osi dopoki stan nie zostanie nadpisany przez inne funkcje.

int i2C_init(); // Funkcja odpowiedzialna za inicjalizacje urzadzen podpietych do magistrali I2C, w tym przypadku ukladu IMU.

void i2C_deinit(int fd1); // Funkcja odpowiedzialna za deinicjalizacje urzadzen podpietych do magistrali I2C.

void i2c_read_gyro(int fd, float *angleX, float *angleY, float *angleZ, float dt, float alpha); // Funkcja odpowiedzialna za odczyt danych z rejestrow ukladu IMU.

double distance_sensor_measure(int PIN); // Funkcja odpowiedzialna za pomiar odleglosci z sensora ultradzwiekowego podpietego do danego pinu.

float compute_pid(float error, float &prev_error, float &integral, float kp, float ki, float kd, float integralLimit); // Funkcja odpowiedzialna za regulacje PID. Na podstawie bledu oblicza sygnal zwrotny.

void reset_error_integral(float &prevErrorR, float &integralR, float &prevErrorL, float &integralL, float &prevErrorX, float &integralX, float &prevErrorO, float &integralO, float &prevErrorC, float &integralC, float &prevErrordiff, float &integraldiff); // Funkcja odpowiedzialna za zerowanie wszystkich bledow i calek.

void adjust_movement(float output, float error, int max_pwm); // Funkcja odpowiedzialna za obrocenie pojazdu tak, by blad byl ponizej danej wartosci. Wykorzystywana np. do namierzania obiektow.

int camera_turning(int &posX, int &centerX, int &countx, int active_search, uint32_t &last_time); // Funkcja odpowiedzialna za ustawienie pojazdu w kierunku wykrytego obiektu. Potrzebuje pozycji wykrytego obiektu za pomoca kamery.

void turning_better(float angle_to_achive, int &fd, float &dt); // Funkcja odpowiedzialna za obrocenie pojazdu, az do osiagniecia danego kata. Potrzebuje pozycji pojazdu odczytanej z IMU.

void drive_straight(int target_rpm, double current_rpmL, double current_rpmR, int r_dir, int l_dir); // Funkcja odpowiedzialna za jazde na wprost z zadana predkoscia. Pozwala na jazde do przodu i tylu. Wyrownuje rowniez prace silnikow w razie pojawienia sie oporu na jednej ze stron.

double calculateRPM(int impulsy, double czas_s); // Funkcja odpowiedzialna za obliczanie ilosci obrotow na minute dla silnika.

void sensors_measure(double time, double &distance1, double &distance2, double &distance3, double &distance4, double &rpmR, double &rpmL, uint32_t &ex_start_time); // Funkcja odpowiedzialna za pomiar ilosci obrotow na minute dla silnikow oraz odleglosci z sensorow ultradzwiekowych.

void square_test(); // Funkcja odpowiedzialna za wykonywanie testu kwadratu.

void detect_cone(cv::Mat& image, cv::Mat& imagewykryty, int& posX, double& dArea, int& centerX); // Funkcja odpowiedzialna za wykrywanie pacholkow przy uzyciu kamerki internetowej. Wykorzystuje wykrywanie kolorow oraz stosunek dlugosci krawedzi by okreslic wykrycie.
