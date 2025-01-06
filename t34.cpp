#include "t34.h"

//Adres i status czujnika
#define BMI160_ADDRESS 0x69  
#define BMI160_STATUS_REG 0x1B
//Rejestry konfiguracji akcelerometru
#define ACC_CONF 0x40
#define ACC_RANGE 0x41
// Rejestry konfiguracji żyroskopu
#define GYRO_CONF 0x42
#define GYRO_RANGE 0x43
#define CMD 0x7E
// Rejestry danych żyroskopu
#define GYRO_X_L 0x0C
#define GYRO_X_H 0x0D
#define GYRO_Y_L 0x0E
#define GYRO_Y_H 0x0F
#define GYRO_Z_L 0x10
#define GYRO_Z_H 0x11

#define ACC_X_L 0x12
#define ACC_X_H 0x13
#define ACC_Y_L 0x14
#define ACC_Y_H 0x15
#define ACC_Z_L 0x16
#define ACC_Z_H 0x17

int fd;
int posR = 0;
int posL = 0;
double rpmR = 0;
double rpmL = 0;
int dir1 = 13;
int dir2 = 19;
int pin_R = 20;
int pin_L = 21;
double distance1 = 500;
double distance2 = 500;
double distance3 = 500;
double distance4 = 500;
int countx = 0;
float alpha = 0.8;
float angleX = 0;
float angleY = 0;
float angleZ = 0;
float dt = 0.0132;
float dt1 = 0.0137;
const double IMPULSY_NA_OBROT = 1633.25;
// PID od jechania na wprost
float Kp_speed = 3; // 3.0
float Ki_speed = 0.15; // 0.15
float Kd_speed = 0.3;   // 0.3

float Kp_sync = 0.1; //1
float Ki_sync = 0.03;  //0,005
float Kd_sync = 0.0; 
//Limity całek
float integralLimit = 800;
float integralLimit1 = 500;
// PID od skręcaniac
float Kp1 = 0.1;
float Ki1 = 0.08;
float Kd1 = 0.02;
// PID od lokalizacji pachołka
float Kp2 = 0.3;
float Ki2 = 0.095;
float Kd2 = 0.9;
double dArea = 0;
int centerX = 208;
int posX = 0;
float prevErrorR = 0;
float integralR = 0;
float prevErrorL = 0;
float integralL = 0;
float prevErrorX = 0;
float integralX = 0;
float prevErrorO = 0;
float integralO = 0;
float prevErrorC = 0;
float integralC = 0;
float prevErrordiff = 0;
float integraldiff = 0;
bool detect_flag = false;
uint32_t last_time = gpioTick();

void setup_PWM(int pin){
    // 10kHz from 0 to 255
    gpioSetMode(pin, PI_OUTPUT);
    gpioSetPWMfrequency(pin, 10000);
    gpioSetPWMrange(pin, 255);
}

void set_DIR_PIN(int pin, int state){
    gpioSetMode(pin, PI_OUTPUT);
    gpioWrite(pin, state);
}

void rotating_idle(){

    set_DIR_PIN(dir1, 0); // R
    set_DIR_PIN(dir2, 0); // L
    gpioPWM(pin_L, 50);   // Silnik lewy
    gpioPWM(pin_R, 50);   // Silnik prawy
}

float compute_pid(float error, float &prev_error, float &integral, float kp, float ki, float kd, float integralLimit){
    integral += error;
    if (integral > integralLimit)
        integral = integralLimit;
    if (integral < -integralLimit)
        integral = -integralLimit;
    float derivative = error - prev_error;
    float output = kp * error + ki * integral + kd * derivative;
    prev_error = error;
    return output;
}

void reset_error_integral(float &prevErrorR, float &integralR, float &prevErrorL, float &integralL, float &prevErrorX, float &integralX, float &prevErrorO, float &integralO, float &prevErrorC, float &integralC, float &prevErrordiff, float &integraldiff){
    prevErrorR = 0;
    integralR = 0;
    prevErrorL = 0;
    integralL = 0;
    prevErrorX = 0;
    integralX = 0;
    prevErrorO = 0;
    integralO = 0;
    prevErrorC = 0;
    integralC = 0;
    prevErrordiff = 0;
    integraldiff = 0;
}

void adjust_movement(float output, float error, int max_pwm){
    int pwm_left = fabs(output);  // PWM lewego silnika
    int pwm_right = fabs(output); // PWM prawego silnika

    pwm_left = std::clamp(pwm_left, 0, max_pwm);
    pwm_right = std::clamp(pwm_right, 0, max_pwm);

    if (error > 0)
    {                         // Jeśli obiekt jest po lewej stronie
        set_DIR_PIN(dir1,0); // Ustaw kierunek dla lewego silnika
        set_DIR_PIN(dir2, 0); // Ustaw kierunek dla prawego silnika
        std::cout << "Skrecam w lewo " << std::endl;
    }
    else
    {                         // Jeśli obiekt jest po prawej stronie
        set_DIR_PIN(dir1, 1); // Ustaw kierunek dla lewego silnika
        set_DIR_PIN(dir2, 1); // Ustaw kierunek dla prawego silnika
        std::cout << "Skrecam w prawo " << std::endl;
    }
    std::cout << "pwm_lewo " << pwm_left << std::endl;
    std::cout << "pwm_prawo " << pwm_right << std::endl;
    gpioPWM(pin_L, pwm_left);
    gpioPWM(pin_R, pwm_right);
}

int camera_turning(int &posX, int &centerX, int &countx, int active_search, uint32_t& last_time){
    float error = centerX - posX;
    std::cout << "ERROR: " << error << std::endl;
    uint32_t current_time_camera =gpioTick();
    double elapsed_time_camera =(current_time_camera - last_time)/1e6;
    static bool first_time=true;
    if(active_search){
        if(posX==0){
            if(first_time == true){
            if(elapsed_time_camera<3){
                adjust_movement(100, 1, 100); // Przesuwamy w prawo z niską prędkością
            }else{
                gpioPWM(pin_L, 0);  
                gpioPWM(pin_R, 0);
                last_time=gpioTick();  
                first_time = false;
            }
            }

            if(elapsed_time_camera<6){
                adjust_movement(100, -1, 100); // Przesuwamy w prawo z niską prędkością
            }else{
                adjust_movement(100, 1, 100); // Przesuwamy w lewo z niską prędkością
            }

            //zerowanie
            if(elapsed_time_camera>12){
                gpioPWM(pin_L, 0);  
                gpioPWM(pin_R, 0);
                last_time=gpioTick();  
            }
        countx = 0;
        return 0;
    }
    }
    if (fabs(error) <= 9)
    {
        countx++;
        std::cout << "count: " << countx << std::endl;
    }
    if (countx >= 30)
    {
        countx = 0;
        return 1;
    }
    else
    {
        error = centerX - posX;
        float output = compute_pid(error, prevErrorC, integralC, Kp2, Ki2, Kd2, integralLimit1);
        adjust_movement(output, error, 100);
        return 0;
    }
}

void turning_better(float angle_to_achive, int &fd, float &dt){
    i2c_read_gyro(fd, &angleX, &angleY, &angleZ, dt, alpha);

    // Ustal kąt docelowy względem zakresu 0–360
    angle_to_achive = fmod(angle_to_achive, 360.0f);
    if (angle_to_achive < 0)
    {
        angle_to_achive += 360.0f;
    }

    float error = angle_to_achive - angleZ;

    if (error > 180)
    {
        error -= 360;
    }
    else if (error < -180)
    {
        error += 360;
    }

    // Pętla sterująca

    while (fabs(error) > 5)
    {
        i2c_read_gyro(fd, &angleX, &angleY, &angleZ, dt, alpha);

        error = angle_to_achive - angleZ;

        if (error > 180)
        {
            error -= 360;
        }
        else if (error < -180)
        {
            error += 360;
        }

        std::cout << "Error: " << error << std::endl;

        float output = compute_pid(error, prevErrorO, integralO, Kp1, Ki1, Kd1, integralLimit);
        std::cout << "Output: " << output << std::endl;

        adjust_movement(output, error, 255);

        gpioDelay(10000);
    }
    while (fabs(error) > 0.1)
    {
        i2c_read_gyro(fd, &angleX, &angleY, &angleZ, dt, alpha);

        error = angle_to_achive - angleZ;

        if (error > 180)
        {
            error -= 360;
        }
        else if (error < -180)
        {
            error += 360;
        }

        std::cout << "Error: " << error << std::endl;

        float output = compute_pid(error, prevErrorO, integralO, 0.1, 0.05, 0, integralLimit);
        std::cout << "Output: " << output << std::endl;

        adjust_movement(output, error, 255);

        gpioDelay(10000);
    }
}

void drive_straight2(int pwm,int r_dir, int l_dir){
    set_DIR_PIN(dir1, l_dir); // Ustaw kierunek dla lewego silnika
    set_DIR_PIN(dir2, r_dir); // Ustaw kierunek dla prawego silnika
    gpioPWM(pin_L, pwm);  
    gpioPWM(pin_R, pwm);

}


void drive_straight(int target_rpm, double current_rpmL, double current_rpmR, int r_dir, int l_dir){

    float errorL = target_rpm - fabs(current_rpmL);
    float errorR = target_rpm - fabs(current_rpmR);
    float error_diff = (fabs(current_rpmL) - fabs(current_rpmR))/2.0;

    int correctionL = compute_pid(errorL, prevErrorL, integralL, Kp_speed, Ki_speed, Kd_speed, integralLimit1);
    int correctionR = compute_pid(errorR, prevErrorR, integralR, Kp_speed, Ki_speed, Kd_speed, integralLimit1);
    int sync_correction = compute_pid(error_diff, prevErrordiff, integraldiff, Kp_sync, Ki_sync, Kd_sync, integralLimit1);

    int dutyL = target_rpm + correctionL - sync_correction;
    int dutyR = target_rpm + correctionR + sync_correction;
    dutyL = std::clamp(dutyL, 30, 250);
    dutyR = std::clamp(dutyR, 30, 250);

    std::cout << "ErrorL: " << errorL << ", ErrorR: " << errorR << std::endl;
    std::cout << "DutyL: " << dutyL << ", DutyR: " << dutyR << std::endl;
    set_DIR_PIN(dir1, r_dir); // Silnik prawy
    set_DIR_PIN(dir2, l_dir); // Silnik lewy
    gpioPWM(pin_L, dutyL);    // Silnik lewy
    gpioPWM(pin_R, dutyR);    // Silnik prawy
}

double calculateRPM(int impulsy, double czas_s){
    double obroty_na_sekunde = impulsy / IMPULSY_NA_OBROT / czas_s;
    return obroty_na_sekunde * 60.0; // Konwersja na obroty na minutę (RPM)
}

void sensors_measure(double time, double &distance1, double &distance2, double &distance3, double &distance4, double &rpmR, double &rpmL, uint32_t &ex_start_time){

    uint32_t current_timex = gpioTick();
    double elapsed_s = (current_timex - ex_start_time) / 1e6; // Przeliczenie z mikrosekund na sekundy
    rpmR = calculateRPM(posR, elapsed_s);
    rpmL = calculateRPM(posL, elapsed_s);
    std::cout << "RPM R=" << rpmR << std::endl;
    std::cout << "RPM L=" << rpmL << std::endl;
    if (elapsed_s >= time)
    { // czas co jaki pomiar

        distance1 = distance_sensor_measure(17);
        std::cout << "Odleglosc1: " << distance1 << std::endl;
        posR = 0;
        posL = 0;
        ex_start_time = gpioTick();
    }
}
void square_test(){
    uint32_t start_time_square;
    double elapsed_time_square;
    uint32_t square_ex_time;
    float start_angle=360;
    for(int j=0;j<4;j++){
    start_angle-=90;
    elapsed_time_square=0;
    start_time_square = gpioTick();
    square_ex_time = gpioTick();
    while (elapsed_time_square<12) {
        uint32_t current_time_square = gpioTick();
        elapsed_time_square = (current_time_square - start_time_square) / 1e6; 
            //std::cout << "Elapsed: " << elapsed_time_square << std::endl;
        sensors_measure(0.1,distance1, distance2, distance3, distance4, rpmR, rpmL, square_ex_time);
        drive_straight(50,rpmL,rpmR,0,1);
    }
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    gpioDelay(500000); 
    turning_better(start_angle, fd,dt);
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    gpioDelay(500000);
    reset_error_integral(prevErrorR,integralR,prevErrorL,integralL,prevErrorX,integralX,prevErrorO,integralO,prevErrorC,integralC,prevErrordiff,integraldiff);
    }

    gpioDelay(1000000);

    for(int j=0;j<4;j++){
    start_angle+=90;
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    gpioDelay(500000); 
    turning_better(start_angle, fd,dt1);
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    gpioDelay(500000);
    reset_error_integral(prevErrorR,integralR,prevErrorL,integralL,prevErrorX,integralX,prevErrorO,integralO,prevErrorC,integralC,prevErrordiff,integraldiff);
    elapsed_time_square=0;
    start_time_square = gpioTick();
    square_ex_time = gpioTick();
    while (elapsed_time_square<12) {
        uint32_t current_time_square = gpioTick();
        elapsed_time_square = (current_time_square - start_time_square) / 1e6; 
            //std::cout << "Elapsed: " << elapsed_time_square << std::endl;
        sensors_measure(0.1,distance1, distance2, distance3, distance4, rpmR, rpmL, square_ex_time);
        drive_straight(50,rpmL,rpmR,1,0);
    }
    }
}

void square_test2(){
    uint32_t start_time_square;
    double elapsed_time_square;
    uint32_t square_ex_time;
    float start_angle=360;
    for(int j=0;j<4;j++){
    start_angle-=90;
    elapsed_time_square=0;
    start_time_square = gpioTick();
    square_ex_time = gpioTick();
    while (elapsed_time_square<12) {
        uint32_t current_time_square = gpioTick();
        elapsed_time_square = (current_time_square - start_time_square) / 1e6; 
            //std::cout << "Elapsed: " << elapsed_time_square << std::endl;
        drive_straight2(150,1,0);
    }
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    gpioDelay(500000); 
    turning_better(start_angle, fd,dt);
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    gpioDelay(500000);
    reset_error_integral(prevErrorR,integralR,prevErrorL,integralL,prevErrorX,integralX,prevErrorO,integralO,prevErrorC,integralC,prevErrordiff,integraldiff);
    }

    gpioDelay(1000000);

    for(int j=0;j<4;j++){
    start_angle+=90;
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    gpioDelay(500000); 
    turning_better(start_angle, fd,dt1);
    gpioPWM(pin_L, 0);  
    gpioPWM(pin_R, 0);
    gpioDelay(500000);
    reset_error_integral(prevErrorR,integralR,prevErrorL,integralL,prevErrorX,integralX,prevErrorO,integralO,prevErrorC,integralC,prevErrordiff,integraldiff);
    elapsed_time_square=0;
    start_time_square = gpioTick();
    square_ex_time = gpioTick();
    while (elapsed_time_square<12) {
        uint32_t current_time_square = gpioTick();
        elapsed_time_square = (current_time_square - start_time_square) / 1e6; 
            //std::cout << "Elapsed: " << elapsed_time_square << std::endl;
        drive_straight2(150,0,1);
    }
    }
}


int i2C_init() {
    // Inicjalizacja I2C
    int fd = i2cOpen(1, BMI160_ADDRESS, 0);  
    if (fd < 0) {
        std::cerr << "Nie udało się zainicjować I2C" << std::endl;
        return -1;
    }
    i2cWriteByteData(fd, CMD, 0xB6);  
    gpioDelay(100000);  
    i2cWriteByteData(fd, CMD, 0xB0);
    gpioDelay(50000);               
    i2cWriteByteData(fd, CMD, 0x15);  
    gpioDelay(50000);
    i2cWriteByteData(fd, CMD, 0x11);
    gpioDelay(50000);
    i2cWriteByteData(fd, CMD, 0x03);
    gpioDelay(50000);                
    i2cWriteByteData(fd, GYRO_CONF, 0x0C);  
    i2cWriteByteData(fd, GYRO_RANGE, 0x00);
    gpioDelay(50000); 
    i2cWriteByteData(fd, ACC_CONF, 0x08);  
    i2cWriteByteData(fd, ACC_RANGE, 0x03);                
    gpioDelay(50000);  // Czekamy 80 ms na zakończenie konfiguracji
    return fd;
}

void i2C_deinit(int fd1) {
    i2cClose(fd1);
}

void i2c_read_gyro(int fd ,float *angleX, float *angleY, float *angleZ, float dt, float alpha){
    const float threshold = 1;     // Próg spoczynku w dps
    uint8_t status = i2cReadByteData(fd, BMI160_STATUS_REG);
    std::cout << "Status: " << (int)status << std::endl;
//if(status == 208){
    int16_t gyroX = (i2cReadByteData(fd, GYRO_X_H) << 8) | i2cReadByteData(fd, GYRO_X_L);
    int16_t gyroY = (i2cReadByteData(fd, GYRO_Y_H) << 8) | i2cReadByteData(fd, GYRO_Y_L);
    int16_t gyroZ = (i2cReadByteData(fd, GYRO_Z_H) << 8) | i2cReadByteData(fd, GYRO_Z_L);

    //int16_t accX = (i2cReadByteData(fd, ACC_X_H) << 8) | i2cReadByteData(fd, ACC_X_L);
    //int16_t accY = (i2cReadByteData(fd, ACC_Y_H) << 8) | i2cReadByteData(fd, ACC_Y_L);
    //int16_t accZ = (i2cReadByteData(fd, ACC_Z_H) << 8) | i2cReadByteData(fd, ACC_Z_L);

        // Przeliczanie wartości na stopnie na sekundę (dps) przy założeniu zakresu ±2000°/s
        float scale = 2000 / 32768.0;
        float gyroX_dps = gyroX * scale;
        float gyroY_dps = gyroY * scale;
        float gyroZ_dps = gyroZ * scale;

        //float acc_scale = 8.0 / 32768.0;
       // float accX_g = accX * acc_scale;
        //float accY_g = accY * acc_scale;
        //float accZ_g = accZ * acc_scale;
    
    //float angle_accX = atan2(accY_g, sqrt(accX_g * accX_g + accZ_g * accZ_g)) * 180 / M_PI;
    //float angle_accY = atan2(-accX_g, sqrt(accY_g * accY_g + accZ_g * accZ_g)) * 180 / M_PI;

     if (abs(gyroZ_dps) < threshold) {
            gyroZ_dps = 0;
        }



    *angleX += gyroX_dps * dt;  
    *angleY += gyroY_dps * dt;  
    *angleZ += gyroZ_dps * dt;  

        if (*angleX >= 360.0) {
        *angleX -= 360.0;
        } else if (*angleX < 0) {
            *angleX += 360.0;
        }

        if (*angleY >= 360.0) {
            *angleY -= 360.0;
        } else if (*angleY < 0) {
            *angleY += 360.0;
        }

        if (*angleZ >= 360.0) {
            *angleZ -= 360.0;
        } else if (*angleZ < 0) {
            *angleZ += 360.0;
        }
        //std::cout << "ACCX: " << angle_accX << "ACCY: " << angle_accY << std::endl;
        std::cout << "AngleX: " << *angleX << "AngleY: " << *angleY << "AngleZ: " << *angleZ << std::endl;
}

double distance_sensor_measure(int PIN) {
    // Ustawienie pinu jako wyjście, aby wysłać impuls „trigger”
    gpioSetMode(PIN, PI_OUTPUT);
    gpioWrite(PIN, PI_LOW);
    gpioDelay(2);  // Czekamy 2 mikrosekundy
    gpioTrigger(PIN, 10, PI_HIGH);  // 10-mikrosekundowy impuls triggera

    // Zmiana trybu na wejście, aby odebrać echo
    gpioSetMode(PIN, PI_INPUT);

    uint32_t startTick, endTick;
    uint32_t timeout = 50000;  // 30 ms timeout (dla maksymalnej odległości ~500 cm)

    // Czekaj na sygnał HIGH
    uint32_t startWait = gpioTick();
    while (gpioRead(PIN) == PI_LOW) {
        if (gpioTick() - startWait > timeout) {
            std::cerr << "Timeout waiting for HIGH signal" << std::endl;
            return -1;  // Timeout — brak sygnału HIGH
        }
    }
    startTick = gpioTick();

    // Czekaj na sygnał LOW
    startWait = gpioTick();
    while (gpioRead(PIN) == PI_HIGH) {
        if (gpioTick() - startWait > timeout) {
            std::cerr << "Timeout waiting for LOW signal" << std::endl;
            return -1;  // Timeout — brak sygnału LOW
        }
    }
    endTick = gpioTick();

    uint32_t pulseDuration = endTick - startTick;

    double distance = (pulseDuration * 0.0343) / 2;  // Przeliczenie na cm
    return distance;
}

/*
void detect_cone(cv::Mat& image, cv::Mat& imagewykryty, int& posX, double& dArea, int& centerX) {
    cv::Mat imageHSV;
    cv::cvtColor(image, imageHSV, cv::COLOR_BGR2HSV);
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    cv::inRange(imageHSV, lower, upper, imagewykryty);

    // Filtracja szumów
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::erode(imagewykryty, imagewykryty, kernel, cv::Point(-1, -1), 3);
    cv::dilate(imagewykryty, imagewykryty, kernel, cv::Point(-1, -1), 2);
    cv::dilate(imagewykryty, imagewykryty, kernel, cv::Point(-1, -1), 11);
    cv::erode(imagewykryty, imagewykryty, kernel, cv::Point(-1, -1), 7);

    // Wykrywanie konturów
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(imagewykryty, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);


    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);

        if (area < 2000 || area > 12000) {
            continue;
        }


        cv::Rect boundingRect = cv::boundingRect(contour);
        float aspectRatio = static_cast<float>(boundingRect.height) / boundingRect.width;

        // Proporcje charakterystyczne dla pachołka
        if (aspectRatio < 1.5 || aspectRatio > 2.0) {
            continue;
        }

        cv::Moments oMoments = cv::moments(contour);
        if (oMoments.m00 > 0) {
            double dM01 = oMoments.m01;
            double dM10 = oMoments.m10;

            posX = static_cast<int>(dM10 / oMoments.m00);
            int posY = static_cast<int>(dM01 / oMoments.m00);
            dArea = area;
            cv::ellipse(image, cv::Point(posX, posY), cv::Size(50, 50), 0, 0, 360, cv::Scalar(255, 0, 0), 6);
            cv::line(image, cv::Point(posX - 50, posY), cv::Point(posX + 50, posY), cv::Scalar(255, 0, 0), 3);
            cv::line(image, cv::Point(posX, posY - 50), cv::Point(posX, posY + 50), cv::Scalar(255, 0, 0), 3);
            cv::putText(image, "Cone", cv::Point(posX, posY - 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        }
    }
    centerX = image.cols / 2;
}
*/