#include "navigation.h"
#include "motor.h"
#include "mpu6050.h"
#include "encoder.h"
#include "control.h"
#include "delay.h"
#include "math.h"
#include "Servo.h"
#include "VL53L0X.h"
// ===================================================CONFIG CAC GIA TRI ======================================================================


uint16_t lidarDistance = 0; // Mang luu gia tri VL53L0X o cac goc quay
uint16_t Lidar_Map[181] = {0}; // Ban do luu gia tri VL53L0X o cac goc quay tu 0 den 180 do

/* Cờ "lái thủ công": khi =1, motorcontrol_pid() ở control.c sẽ KHÔNG
 * ghi vào TIM4 nữa, để Nav_MPU_Turn() được toàn quyền điều khiển động cơ
 * bằng motor_run() trong lúc xoay. */
volatile u8 nav_manual = 0;


//Ham lay gia tri khoang cach hien tai va luu vao bien toan cuc (de sau nay hien thi ra OLED)
uint16_t Lidar_GetDist() {
    uint16_t current_distance = readRangeContinuousMillimeters(0); // Ham doc gia tri tu VL53L0X
    lidarDistance = current_distance; // Cap nhat gia tri vao bien toan cuc
    return current_distance;
}

// ===================================================NHUNG HAM DI CHUYEN CO BAN=====================================================================

/* Chạy thẳng: chốt hướng hiện tại làm hướng chuẩn, đặt vận tốc mục tiêu.
 * Vòng lặp nền (HeadingHold_Task + motorcontrol_pid, chạy mỗi 20ms trong
 * ngắt MPU6050) sẽ tự bơm PWM ra TIM4 và giữ xe đi thẳng theo yaw. */

void Nav_Motor_Forward(void) {  /* Bật cầu H chạy thẳng */ 

    nav_manual = 0;

    PID_Reset(&pid_l);
    PID_Reset(&pid_r);

    heading_target = yaw;   // "khóa" hướng đi hiện tại
    g_sp_w = 0.0f;
    g_sp_v = NAV_FWD_SPEED;


}


void Nav_Motor_Stop(void)    { /* Phanh động cơ */ 

    nav_manual = 0;

    g_sp_v = 0.0f;
    g_sp_w = 0.0f;

    PID_Reset(&pid_l);
    PID_Reset(&pid_r);

    motor_stop();

}

/* Xoay xe tại chỗ một góc `angle` (độ) so với hướng hiện tại, dùng
 * vòng P đơn giản trên sai số yaw (mpu6050_angleDiff) và motor_run()
 * trực tiếp (bypass vòng PID vận tốc). */

void Nav_MPU_Turn(int angle) { /* Xoay xe theo PID & MPU6050 (+90 / -90) */ 

    nav_manual = 1;          // "rút" motorcontrol_pid ra khỏi cuộc

    g_sp_v = 0.0f;
    g_sp_w = 0.0f;
    PID_Reset(&pid_l);
    PID_Reset(&pid_r);
    motor_stop();

    float target = yaw + (float)angle;
    if (target >  180.0f) target -= 360.0f;
    if (target < -180.0f) target += 360.0f;

    u32 settle_ms = 0;

    while (1)
    {
        float err = mpu6050_angleDiff(target, yaw);

        if (fabsf(err) <= TURN_DONE_DEG)
        {
            motor_stop();
            settle_ms += dt_ms;
            if (settle_ms >= TURN_SETTLE_MS) break;
        }
        else
        {
            settle_ms = 0;

            float pwm_f = TURN_KP * err;
            pwm_f = limit(pwm_f, -(float)MAX_TURN_PWM, (float)MAX_TURN_PWM);

            if (pwm_f > 0.0f && pwm_f <  MIN_TURN_PWM) pwm_f =  MIN_TURN_PWM;
            if (pwm_f < 0.0f && pwm_f > -MIN_TURN_PWM) pwm_f = -MIN_TURN_PWM;

            i16 pwm = (i16)pwm_f;
            // err > 0  =>  cần yaw TĂNG  =>  bánh trái lùi, bánh phải tiến
            motor_run(-pwm, pwm);
        }

        HAL_Delay(dt_ms);
    }

    motor_stop();
    heading_target = yaw;    // đồng bộ hướng "thẳng" mới cho HeadingHold_Task
    nav_manual = 0;          // trả quyền điều khiển lại cho vòng lặp nền
}


void Nav_Encoder_Reset(void) { /* Xóa biến đếm số vòng bánh xe */ 
    ec_l.dist  = 0.0f;
    ec_r.dist  = 0.0f;
    ec_l.total = 0;
    ec_r.total = 0;

    last_dist_l = 0.0f;
    last_dist_r = 0.0f;
}


float Nav_Encoder_Get_Dist(void) {  
    return (ec_l.dist + ec_r.dist) * 0.5f;
}

//==================================================== Bo loc khong gian==============================================================================

//Quet hanh lang phia truoc (60-120 do)
bool Check_Front_Corridor(void) {
    // Quét từ 60 đến 120 độ, kiểm tra nếu tất cả đều > SAFE_DISTANCE thì trả về true
    for (int angle = 60; angle <= 120; angle += 10) { 
        Servo_WriteAngle(angle); // Quay servo đến góc cần quét
        HAL_Delay(30); // Chờ servo ổn định

        uint16_t dist = Lidar_GetDist(); // Đọc khoảng cách tại góc này
        Lidar_Map[angle] = dist; // Cap nhat vao mang 

        if(dist > 0 && dist < SAFE_DISTANCE) {
            return true; // Phát hiện vật cản trong hành lang
        }
    }
    return false; // Hành lang phía trước an toàn
}

//Phat hien vat can, quet toan dai tu 0-180 do va dua ra quyet dinh di chuyen
Nav_Direction Scan_and_Decide(void) {
    uint16_t left_clearance = 0, right_clearance = 0;
    int left_edge_angle = 0, right_edge_angle = 0;

    //RESET Lidar_Map truoc khi quet lai
    for(int i=0; i<=180; i++) {
        Lidar_Map[i] = 0;
    }

    for (int angle = 0; angle <= 180; angle += 10) {
        Servo_WriteAngle(angle);
        HAL_Delay(30); // Chờ servo ổn định
        uint16_t dist = Lidar_GetDist();
        Lidar_Map[angle] = dist; // Cap nhat vao mang

        //Phan tich diem sau nhat
        //check ben phai
        if (angle >=0 && angle <= 80) {
            if(dist > left_clearance) {
                right_clearance = dist;
                right_edge_angle = angle; 
            }
        }
        //check ben trai
        else if(angle >=100 && angle <= 180) {
            if(dist > left_clearance) {
                left_clearance = dist;
                left_edge_angle = angle;
            }
        }
    }
    Servo_WriteAngle(90);
    HAL_Delay(50);

    /*Ap dung mang loc an toan: ket hop giua khoang cach va goc*/
    bool can_turn_left = ((left_clearance >= MIN_CLEARANCE) && (left_edge_angle >= 140));
    bool can_turn_right = ((right_clearance >= MIN_CLEARANCE) && (right_edge_angle <= 40));

    if(can_turn_left && can_turn_right) {
        return (left_clearance >= right_clearance) ? DIR_LEFT : DIR_RIGHT;
    } 
    else if (can_turn_left) {
        return DIR_LEFT;
    }
    else if (can_turn_right) {
        return DIR_RIGHT;
    }
    return DIR_ABORT; //Khong the vuot qua vat can, quay dau
}



//THUC THI DIEU HUONG NE VAT CAN
bool Execute_Obstacle_Bypass(void) {
    Nav_Motor_Stop();
    
    Servo_WriteAngle(90);
    HAL_Delay(100);
    uint16_t center_dist = Lidar_GetDist();

    Nav_Direction turn_dir = Scan_and_Decide();
    
    if (turn_dir == DIR_ABORT) {
        return false; // Ép gọi U-Turn trên hàm main
    }

    Nav_MPU_Turn(turn_dir); // Bẻ vô lăng toàn thân xe

    // PHÂN LOẠI VẬT CẢN (Tâm vs Lệch biên)
    if (center_dist > 0 && center_dist < SAFE_DISTANCE) {
        
        // --- KỊCH BẢN 1: BÁM SƯỜN ---
        // Nếu rẽ Trái -> Vật cản nằm bên Phải -> Servo quay góc 0 để nhìn Phải
        // Nếu rẽ Phải -> Vật cản nằm bên Trái -> Servo quay góc 180 để nhìn Trái
        int side_angle = (turn_dir == DIR_LEFT) ? 0 : 180;
        float offset_dist = 0.0f;
        
        // [Pha 1: Dò ngang]
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        
        bool edge_found = false;
        while (!edge_found) {
            Servo_WriteAngle(side_angle);
            HAL_Delay(50);
            if (Lidar_GetDist() > (SAFE_DISTANCE + EDGE_JUMP_THRESHOLD)) {
                edge_found = true;
            }
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                Nav_MPU_Turn(-turn_dir); 
                return false; 
            }
        }
        
        // Rướn 28cm
        float target_dist = Nav_Encoder_Get_Dist() + CLEARANCE_RUN_M;
        while (Nav_Encoder_Get_Dist() < target_dist) {
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                Nav_MPU_Turn(-turn_dir); 
                return false; 
            }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();
        offset_dist = Nav_Encoder_Get_Dist(); 
        
        // [Pha 2: Bám dọc]
        Nav_MPU_Turn(-turn_dir); 
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        
        edge_found = false;
        while (!edge_found) {
            Servo_WriteAngle(side_angle);
            HAL_Delay(50);
            if (Lidar_GetDist() > (SAFE_DISTANCE + EDGE_JUMP_THRESHOLD)) {
                edge_found = true;
            }
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                return false; 
            }
        }
        
        // Rướn 28cm lần hai
        target_dist = Nav_Encoder_Get_Dist() + CLEARANCE_RUN_M;
        while (Nav_Encoder_Get_Dist() < target_dist) {
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                return false; 
            }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();
        
        // [Pha 3: Lùi về trục đường cũ]
        Nav_MPU_Turn(-turn_dir);
        Servo_WriteAngle(90); 
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        while (Nav_Encoder_Get_Dist() < offset_dist) {
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                return false; 
            }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();
        
        Nav_MPU_Turn(turn_dir);
        return true; 
        
    } else {
        
        // --- KỊCH BẢN 2: LƯỚI CỐ ĐỊNH (Hộp mù) ---
        Servo_WriteAngle(90); 
        
        // [Dạt 40cm]
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        while (Nav_Encoder_Get_Dist() < 0.40f) {
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                Nav_MPU_Turn(-turn_dir); 
                return false; 
            }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();
        
        // [Tiến 50cm]
        Nav_MPU_Turn(-turn_dir);
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        while (Nav_Encoder_Get_Dist() < 0.50f) {
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                return false; 
            }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();
        
        // [Lùi 40cm về luống]
        Nav_MPU_Turn(-turn_dir);
        Nav_Encoder_Reset();
        Nav_Motor_Forward();
        while (Nav_Encoder_Get_Dist() < 0.40f) {
            if (Check_Front_Corridor()) {
                Nav_Motor_Stop();
                return false; 
            }
            HAL_Delay(10);
        }
        Nav_Motor_Stop();
        
        Nav_MPU_Turn(turn_dir);
        return true;
    }
}