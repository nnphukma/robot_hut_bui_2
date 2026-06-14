#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "types.h"	
#include <stdint.h>
#include <stdbool.h>
#include "VL53L0X.h" // Thêm thư viện cảm biến Lidar VL53L0X để lấy giá trị khoảng cách
#include "Servo.h" // Thêm thư viện điều khiển servo nếu cần thiết cho việc quay lái

#define SAFE_DISTANCE 200        // Khoảng cách an toàn để di chuyển thẳng (20cm)
#define EDGE_JUMP_THRESHOLD 150  // Khoảng cách bước nhảy để phát hiện cạnh (15cm)
#define MIN_CLEARANCE 300        // Khe ho phai sau > 30cm moi lot xe
#define ROBOT_WIDTH_M 0.27f //Coi robot la hinh vuong co canh 27cm de tinh toan khoang cach toi mep trai va phai
#define CLEARANCE_RUN_M 0.28f //Quãng đường chạy rướn thoát thân 27cm đuôi xe + 8cm bảo hiểm (Do cảm biến ở mũi)


// Giữ nguyên Enum chuẩn của bạn để quản lý Máy trạng thái (State Machine)
typedef enum {
    ROBOT_FORWARD,          
    ROBOT_BACKWARD_30CM,    
    ROBOT_TURN_RIGHT_90,    
    ROBOT_SIDE_FORWARD_30,  
    ROBOT_TURN_LEFT_90,     
} RobotState_t;

typedef enum {
    DIR_LEFT = -90,
    DIR_RIGHT = 90,
    DIR_ABORT = 0
} Nav_Direction;

// Cấu hình tham số điều khiển xoay bằng MPU6050
#define TURN_KP          5.5f   // Hệ số P điều khiển góc xoay
#define MAX_TURN_PWM     400    // Giới hạn PWM lớn nhất khi xoay
#define MIN_TURN_PWM     220    // Khử ma sát tĩnh (Dead-zone) sàn nhà

// --- Tham số cho 5 hàm Nav_xxx ---
#define NAV_FWD_SPEED     0.15f  // m/s - tốc độ tịnh tiến khi Nav_Motor_Forward()
#define TURN_DONE_DEG     1.5f   // sai số góc (độ) coi như đã quay xong
#define TURN_SETTLE_MS    150u   // giữ ổn định bao lâu trước khi dừng hẳn (ms)

extern uint16_t lidarDistance; // Biến toàn cục lưu giá trị khoảng cách đo được từ Lidar
extern uint16_t Lidar_Map[181]; // Bản đồ khoảng cách từ 0 đến 180 độ, mỗi phần tử lưu khoảng cách tại góc tương ứng


// Khai báo các hàm di chuyển cơ bản (Primitive API)
//Ham tra ve gia tri distance cua VL53L0X
uint16_t Lidar_GetDist(void);

//Phan di chuyen
void Nav_Motor_Forward(void); //Chay tien
void Nav_Motor_Stop(void);  //Phanh
void Nav_MPU_Turn(int angle); //Quay xe
void Nav_Encoder_Reset(void); //Reset encoder ve 0
float Nav_Encoder_Get_Dist(void); //Lay gia tri encoder da di duoc tinh tu luc reset

//Phan ra quyet dinh
bool Check_Front_Corridor(void);
Nav_Direction Scan_and_Decide(void);
bool Execute_Obstacle_Bypass(void);

#endif /* __NAVIGATION_H */