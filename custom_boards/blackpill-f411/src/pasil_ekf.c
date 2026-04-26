#include <nuttx/config.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <sys/ioctl.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/i2c/i2c_master.h> 

#define ARM_MATH_CM4
#include <arm_math.h>

/* STMicroelectronics VL53L0X API */
#include "vl53l0x_api.h"

#define LOOP_PERIOD_NS 2000000 /* 500Hz */
#define SEC_TO_NS      1000000000
#define DT_SEC         0.002f

#define KP 15.0f
#define KI 0.0f
#define KD 0.5f
#define PWM_NEUTRAL 4915
#define PWM_MIN     3276
#define PWM_MAX     6553
#define STATE_DIM 7

#define QMC5883P_ADDR 0x2C
#define BMP280_ADDR   0x76

/* Global File Descriptor for ST API Wrapper */
int pasil_fd_i2c = -1;

/* BMP280 Calibration State */
static uint16_t dig_T1, dig_P1;
static int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t  t_fine;

static float32_t P_data[49], F_data[49], F_T_data[49], Q_data[49], temp_data[49], new_P_data[49];
static arm_matrix_instance_f32 P, F, F_T, Q, TempMat, NewP;

static void init_ekf_matrices(void) {
    arm_mat_init_f32(&P, STATE_DIM, STATE_DIM, P_data); arm_mat_init_f32(&F, STATE_DIM, STATE_DIM, F_data);
    arm_mat_init_f32(&F_T, STATE_DIM, STATE_DIM, F_T_data); arm_mat_init_f32(&Q, STATE_DIM, STATE_DIM, Q_data);
    arm_mat_init_f32(&TempMat, STATE_DIM, STATE_DIM, temp_data); arm_mat_init_f32(&NewP, STATE_DIM, STATE_DIM, new_P_data);
    for (int i = 0; i < 49; i++) { P_data[i] = 0.0f; F_data[i] = 0.0f; Q_data[i] = 0.0f; }
    for (int i = 0; i < 7; i++) { P_data[i * 8] = 1.0f; F_data[i * 8] = 1.0f; Q_data[i * 8] = 0.001f; }
}

static void timespec_add_ns(struct timespec *ts, long ns) {
    ts->tv_nsec += ns;
    if (ts->tv_nsec >= SEC_TO_NS) { ts->tv_sec++; ts->tv_nsec -= SEC_TO_NS; }
}

static void i2c_write_reg(int fd, uint8_t addr, uint8_t reg, uint8_t value) {
    struct i2c_msg_s msg; struct i2c_transfer_s xfer;
    uint8_t tx_data[2] = {reg, value};
    msg.frequency = 400000; msg.addr = addr; msg.flags = 0;    
    msg.buffer = tx_data; msg.length = 2; xfer.msgv = &msg; xfer.msgc = 1;
    ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&xfer);
}

static void bmp280_extract_cal(int fd) {
    struct i2c_msg_s msg[2]; struct i2c_transfer_s xfer;
    uint8_t reg = 0x88; uint8_t cal[24];
    msg[0].frequency = 400000; msg[0].addr = BMP280_ADDR; msg[0].flags = 0; msg[0].buffer = &reg; msg[0].length = 1;
    msg[1].frequency = 400000; msg[1].addr = BMP280_ADDR; msg[1].flags = I2C_M_READ; msg[1].buffer = cal; msg[1].length = 24;
    xfer.msgv = msg; xfer.msgc = 2;
    if(ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&xfer) >= 0) {
        dig_T1 = (cal[1] << 8) | cal[0];  dig_T2 = (cal[3] << 8) | cal[2];  dig_T3 = (cal[5] << 8) | cal[4];
        dig_P1 = (cal[7] << 8) | cal[6];  dig_P2 = (cal[9] << 8) | cal[8];  dig_P3 = (cal[11] << 8) | cal[10];
        dig_P4 = (cal[13] << 8) | cal[12]; dig_P5 = (cal[15] << 8) | cal[14]; dig_P6 = (cal[17] << 8) | cal[16];
        dig_P7 = (cal[19] << 8) | cal[18]; dig_P8 = (cal[21] << 8) | cal[20]; dig_P9 = (cal[23] << 8) | cal[22];
    }
}

static float bmp280_calculate_pressure(int32_t adc_T, int32_t adc_P) {
    int32_t var1, var2; int64_t p_var1, p_var2, p;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    p_var1 = ((int64_t)t_fine) - 128000;
    p_var2 = p_var1 * p_var1 * (int64_t)dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3) >> 8) + ((p_var1 * (int64_t)dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)dig_P1) >> 33;
    if (p_var1 == 0) return 0; 
    p = 1048576 - adc_P;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (float)p / 256.0f;
}

int pasil_imu_task(int argc, char *argv[]) {
    struct timespec wakeup_time;
    int fd_imu, fd_pwm;
    struct pwm_info_s pwm_info;
    uint32_t cycle_count = 0;
    
    float pitch = 0.0f, pitch_accel = 0.0f;
    float pid_error = 0.0f, pid_integral = 0.0f, pid_derivative = 0.0f, last_error = 0.0f;
    int32_t new_duty_cycle = PWM_NEUTRAL;

    int16_t mag_x = 0, mag_y = 0, mag_z = 0;
    float true_pressure_pa = 0.0f;
    uint16_t laser_dist_mm = 0;

    VL53L0X_Dev_t tof_dev_struct;
    VL53L0X_DEV tof_dev = &tof_dev_struct;
    tof_dev->I2cDevAddr = 0x29; /* 7-bit standard address */

    printf("[PASIL-MASTER] Finalizing Fleet Assault...\n");
    init_ekf_matrices();

    fd_imu = open("/dev/imu0", O_RDONLY);
    fd_pwm = open("/dev/pwm0", O_RDONLY);
    pasil_fd_i2c = open("/dev/i2c1", O_RDWR);

    if (pasil_fd_i2c >= 0) {
        printf("[PASIL-MASTER] Waking Barometer & Magnetometer...\n");
        bmp280_extract_cal(pasil_fd_i2c);
        //i2c_write_reg(pasil_fd_i2c, BMP280_ADDR, 0xE0, 0xB6);
        i2c_write_reg(pasil_fd_i2c, BMP280_ADDR, 0xF4, 0x55);
        i2c_write_reg(pasil_fd_i2c, QMC5883P_ADDR, 0x0B, 0x01); usleep(50000); 
        i2c_write_reg(pasil_fd_i2c, QMC5883P_ADDR, 0x20, 0x40);
        i2c_write_reg(pasil_fd_i2c, QMC5883P_ADDR, 0x21, 0x01);
        i2c_write_reg(pasil_fd_i2c, QMC5883P_ADDR, 0x0A, 0x1D); usleep(50000);

        printf("[PASIL-MASTER] Uploading ST Firmware to Laser...\n");
        VL53L0X_DataInit(tof_dev);
        VL53L0X_StaticInit(tof_dev);
        uint32_t refSpadCount; uint8_t isApertureSpads;
        VL53L0X_PerformRefSpadManagement(tof_dev, &refSpadCount, &isApertureSpads);
        VL53L0X_PerformRefCalibration(tof_dev, NULL, NULL);
        VL53L0X_SetDeviceMode(tof_dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        VL53L0X_StartMeasurement(tof_dev);
    }

    pwm_info.frequency = 50; pwm_info.duty = PWM_NEUTRAL;
    ioctl(fd_pwm, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm_info));
    ioctl(fd_pwm, PWMIOC_START, 0);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    for(int i = 0; i < 2500; i++) {
        cycle_count++;
        timespec_add_ns(&wakeup_time, LOOP_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        /* 500Hz: IMU */
        int16_t raw_mem[16] = {0}; 
        if (read(fd_imu, raw_mem, sizeof(raw_mem)) >= 12) {
            float gy = (float)raw_mem[1]; 
            float ax = (float)raw_mem[3]; float ay = (float)raw_mem[4]; float az = (float)raw_mem[5];
            pitch_accel = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
            pitch = 0.98f * (pitch + gy * DT_SEC) + (0.02f) * pitch_accel;
        }

        arm_mat_trans_f32(&F, &F_T); arm_mat_mult_f32(&F, &P, &TempMat);
        arm_mat_mult_f32(&TempMat, &F_T, &NewP); arm_mat_add_f32(&NewP, &Q, &P);

        /* 50Hz: MAGNETOMETER */
        if (pasil_fd_i2c >= 0 && (cycle_count % 10 == 5)) {
            struct i2c_msg_s msg[2]; struct i2c_transfer_s xfer; uint8_t mag_status = 0;
            uint8_t reg_status = 0x09;
            msg[0].frequency = 400000; msg[0].addr = QMC5883P_ADDR; msg[0].flags = 0; msg[0].buffer = &reg_status; msg[0].length = 1;
            msg[1].frequency = 400000; msg[1].addr = QMC5883P_ADDR; msg[1].flags = I2C_M_READ; msg[1].buffer = &mag_status; msg[1].length = 1;
            xfer.msgv = msg; xfer.msgc = 2;
            
            if (ioctl(pasil_fd_i2c, I2CIOC_TRANSFER, (unsigned long)&xfer) >= 0 && (mag_status & 0x01)) {
                uint8_t reg_data = 0x01; uint8_t rx_data[6] = {0};
                msg[0].buffer = &reg_data; msg[1].buffer = rx_data; msg[1].length = 6;
                if (ioctl(pasil_fd_i2c, I2CIOC_TRANSFER, (unsigned long)&xfer) >= 0) {
                    mag_x = (int16_t)(rx_data[0] | (rx_data[1] << 8));
                    mag_y = (int16_t)(rx_data[2] | (rx_data[3] << 8));
                    mag_z = (int16_t)(rx_data[4] | (rx_data[5] << 8));
                }
            }
        }

        /* 10Hz: BAROMETER */
        if (pasil_fd_i2c >= 0 && (cycle_count % 50 == 8)) {
            struct i2c_msg_s msg[2]; struct i2c_transfer_s xfer;
            uint8_t reg_press = 0xF7, reg_status = 0xF3; uint8_t rx_data[6] = {0}, status = 0;

            i2c_write_reg(pasil_fd_i2c, BMP280_ADDR, 0xF4, 0x55);

            /* BUSY WAIT: Poll Status Register 0xF3 Bit 3 (im_update) and Bit 0 (measuring) */
            /* We only try 5 times to avoid hanging the 500Hz loop */
            for(int retry = 0; retry < 5; retry++) {
                msg[0].frequency = 400000; msg[0].addr = BMP280_ADDR; msg[0].flags = 0;
                msg[0].buffer = &reg_status; msg[0].length = 1;
                msg[1].frequency = 400000; msg[1].addr = BMP280_ADDR; msg[1].flags = I2C_M_READ;
                msg[1].buffer = &status; msg[1].length = 1;
                xfer.msgv = msg; xfer.msgc = 2;
                ioctl(pasil_fd_i2c, I2CIOC_TRANSFER, (unsigned long)&xfer);
                
                if (!(status & 0x09)) break; // Exit if not busy
            }

            msg[0].frequency = 400000; msg[0].addr = BMP280_ADDR; msg[0].flags = 0; msg[0].buffer = &reg_press; msg[0].length = 1;
            msg[1].frequency = 400000; msg[1].addr = BMP280_ADDR; msg[1].flags = I2C_M_READ; msg[1].buffer = rx_data; msg[1].length = 6;
            xfer.msgv = msg; xfer.msgc = 2;
            
            if (ioctl(pasil_fd_i2c, I2CIOC_TRANSFER, (unsigned long)&xfer) >= 0) {
                int32_t adc_P = (rx_data[0] << 12) | (rx_data[1] << 4) | (rx_data[2] >> 4);
                int32_t adc_T = (rx_data[3] << 12) | (rx_data[4] << 4) | (rx_data[5] >> 4);
                true_pressure_pa = bmp280_calculate_pressure(adc_T, adc_P);
            }
        }

        /* 10Hz: LASER TELEMETRY (TDM Offset 2 to avoid collisions) */
        if (pasil_fd_i2c >= 0 && (cycle_count % 50 == 2)) {
            uint8_t dataReady = 0;
            VL53L0X_GetMeasurementDataReady(tof_dev, &dataReady);
            if(dataReady == 1) {
                VL53L0X_RangingMeasurementData_t r_data;
                VL53L0X_GetRangingMeasurementData(tof_dev, &r_data);
                laser_dist_mm = r_data.RangeMilliMeter;
                VL53L0X_ClearInterruptMask(tof_dev, 0); /* Clears the interrupt to fetch next reading */
            }
        }

        pid_error = 0.0f - pitch; 
        pid_integral += pid_error * DT_SEC; pid_derivative = (pid_error - last_error) / DT_SEC;
        new_duty_cycle = PWM_NEUTRAL + (int32_t)((KP * pid_error) + (KI * pid_integral) + (KD * pid_derivative));
        last_error = pid_error;

        if (new_duty_cycle > PWM_MAX) new_duty_cycle = PWM_MAX;
        if (new_duty_cycle < PWM_MIN) new_duty_cycle = PWM_MIN;
        pwm_info.duty = (uint16_t)new_duty_cycle;
        ioctl(fd_pwm, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm_info));

        /* --- TACTICAL TELEMETRY (2Hz) --- */
        if (i % 250 == 0) {
            /* Added mag_z to the printout to satisfy the compiler */
            printf("ATT:[%4d] | MAG:[%5d %5d %5d] | BARO:[%8.1f Pa] | LASER:[%4d mm] | PWM:[%5d]\n", 
                   (int)pitch, mag_x, mag_y, mag_z, true_pressure_pa, laser_dist_mm, (int)new_duty_cycle);
        }
    }

    /* System Disarm */
    pwm_info.duty = PWM_MIN;
    ioctl(fd_pwm, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm_info));
    ioctl(fd_pwm, PWMIOC_STOP, 0);
    
    /* Cleaned up shutdown logic to satisfy GNU GCC strict indentation rules */
    if (fd_pwm >= 0) { close(fd_pwm); }
    if (fd_imu >= 0) { close(fd_imu); }
    if (pasil_fd_i2c >= 0) { close(pasil_fd_i2c); }
    
    printf("[PASIL-MASTER] Stack Disarmed. Fleet Assault Complete.\n");
    return OK;
}
