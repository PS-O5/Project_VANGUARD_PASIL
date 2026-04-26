#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <sys/ioctl.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/i2c/i2c_master.h> /* CRITICAL: Raw I2C Bus Control */

#define ARM_MATH_CM4
#include <arm_math.h>

#define LOOP_PERIOD_NS 2000000 /* 500Hz */
#define SEC_TO_NS      1000000000
#define DT_SEC         0.002f

/* PID & Matrix Definitions (Keep your existing ones) */
#define KP 15.0f
#define KI 0.0f
#define KD 0.5f
#define PWM_NEUTRAL 4915
#define PWM_MIN     3276
#define PWM_MAX     6553
#define STATE_DIM 7

static float32_t P_data[49], F_data[49], F_T_data[49], Q_data[49], temp_data[49], new_P_data[49];
static arm_matrix_instance_f32 P, F, F_T, Q, TempMat, NewP;

static void init_ekf_matrices(void) {
    arm_mat_init_f32(&P, STATE_DIM, STATE_DIM, P_data);
    arm_mat_init_f32(&F, STATE_DIM, STATE_DIM, F_data);
    arm_mat_init_f32(&F_T, STATE_DIM, STATE_DIM, F_T_data);
    arm_mat_init_f32(&Q, STATE_DIM, STATE_DIM, Q_data);
    arm_mat_init_f32(&TempMat, STATE_DIM, STATE_DIM, temp_data);
    arm_mat_init_f32(&NewP, STATE_DIM, STATE_DIM, new_P_data);

    for (int i = 0; i < 49; i++) { P_data[i] = 0.0f; F_data[i] = 0.0f; Q_data[i] = 0.0f; }
    for (int i = 0; i < 7; i++) { P_data[i * 8] = 1.0f; F_data[i * 8] = 1.0f; Q_data[i * 8] = 0.001f; }
}

static void timespec_add_ns(struct timespec *ts, long ns) {
    ts->tv_nsec += ns;
    if (ts->tv_nsec >= SEC_TO_NS) { ts->tv_sec++; ts->tv_nsec -= SEC_TO_NS; }
}

/* Raw I2C Write Helper */
static void qmc_write_reg(int fd, uint8_t reg, uint8_t value) {
    struct i2c_msg_s msg[1];
    struct i2c_transfer_s xfer;
    uint8_t tx_data[2] = {reg, value};

    msg[0].frequency = 400000;
    msg[0].addr      = 0x0D; /* QMC5883L I2C Address */
    msg[0].flags     = 0;    /* Write Flag */
    msg[0].buffer    = tx_data;
    msg[0].length    = 2;

    xfer.msgv = msg;
    xfer.msgc = 1;
    ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&xfer);
}

int pasil_imu_task(int argc, char *argv[]) {
    struct timespec wakeup_time;
    int fd_imu, fd_pwm, fd_baro, fd_i2c;
    struct pwm_info_s pwm_info;
    uint32_t cycle_count = 0;
    
    float pitch = 0.0f, pitch_accel = 0.0f;
    float pid_error = 0.0f, pid_integral = 0.0f, pid_derivative = 0.0f, last_error = 0.0f;
    int32_t new_duty_cycle = PWM_NEUTRAL;

    printf("[PASIL-MASTER] Arming Fleet (Direct Silicon Protocol)...\n");
    init_ekf_matrices();

    fd_imu  = open("/dev/imu0", O_RDONLY);
    fd_pwm  = open("/dev/pwm0", O_RDONLY);
    fd_baro = open("/dev/baro0", O_RDONLY);
    
    /* Open the Raw I2C Bus for the Magnetometer */
    fd_i2c  = open("/dev/i2c1", O_RDWR);
    if (fd_i2c >= 0) {
        printf("[PASIL-MASTER] Waking QMC5883L Silicon...\n");
        qmc_write_reg(fd_i2c, 0x0B, 0x01); /* Set/Reset Period */
        qmc_write_reg(fd_i2c, 0x09, 0x1D); /* Continuous Mode, 200Hz, 8G, 512 OSR */
    }

    pwm_info.frequency = 50; 
    pwm_info.duty = PWM_NEUTRAL;
    ioctl(fd_pwm, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm_info));
    ioctl(fd_pwm, PWMIOC_START, 0);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    for(int i = 0; i < 2500; i++) {
        cycle_count++;
        timespec_add_ns(&wakeup_time, LOOP_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        /* --- 500Hz: IMU INGESTION --- */
        int16_t raw_mem[16] = {0}; 
        if (read(fd_imu, raw_mem, sizeof(raw_mem)) >= 12) {
            float gx = (float)raw_mem[0]; float gy = (float)raw_mem[1]; float gz = (float)raw_mem[2];
            float ax = (float)raw_mem[3]; float ay = (float)raw_mem[4]; float az = (float)raw_mem[5];

            pitch_accel = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
            pitch = 0.98f * (pitch + gy * DT_SEC) + (0.02f) * pitch_accel;
        }

        /* EKF Math */
        arm_mat_trans_f32(&F, &F_T);
        arm_mat_mult_f32(&F, &P, &TempMat);
        arm_mat_mult_f32(&TempMat, &F_T, &NewP);
        arm_mat_add_f32(&NewP, &Q, &P);

        /* --- 50Hz: RAW I2C MAGNETOMETER (Address 0x0D) --- */
        int16_t mag_x = 0, mag_y = 0, mag_z = 0;
        if (fd_i2c >= 0 && (cycle_count % 10 == 5)) {
            struct i2c_msg_s msg[2];
            struct i2c_transfer_s xfer;
            uint8_t reg = 0x00; /* Start reading from X LSB */
            uint8_t rx_data[6] = {0};

            /* Write Register Address */
            msg[0].frequency = 400000; msg[0].addr = 0x0D; msg[0].flags = 0; 
            msg[0].buffer = &reg; msg[0].length = 1;
            /* Read 6 Bytes */
            msg[1].frequency = 400000; msg[1].addr = 0x0D; msg[1].flags = I2C_M_READ; 
            msg[1].buffer = rx_data; msg[1].length = 6;

            xfer.msgv = msg; xfer.msgc = 2;
            if (ioctl(fd_i2c, I2CIOC_TRANSFER, (unsigned long)&xfer) >= 0) {
                /* QMC5883L is Little-Endian (LSB first) */
                mag_x = (int16_t)(rx_data[0] | (rx_data[1] << 8));
                mag_y = (int16_t)(rx_data[2] | (rx_data[3] << 8));
                mag_z = (int16_t)(rx_data[4] | (rx_data[5] << 8));
            }
        }

        /* --- 10Hz: BAROMETER INGESTION --- */
        if (fd_baro >= 0 && (cycle_count % 50 == 8)) {
            uint8_t baro_buffer[32] = {0};
            read(fd_baro, baro_buffer, sizeof(baro_buffer));
        }

        /* PID & Actuator Update */
        pid_error = 0.0f - pitch; 
        pid_integral += pid_error * DT_SEC;
        pid_derivative = (pid_error - last_error) / DT_SEC;
        new_duty_cycle = PWM_NEUTRAL + (int32_t)((KP * pid_error) + (KI * pid_integral) + (KD * pid_derivative));
        last_error = pid_error;

        if (new_duty_cycle > PWM_MAX) new_duty_cycle = PWM_MAX;
        if (new_duty_cycle < PWM_MIN) new_duty_cycle = PWM_MIN;

        pwm_info.duty = (uint16_t)new_duty_cycle;
        ioctl(fd_pwm, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm_info));

        /* Telemetry Update */
        if (i % 100 == 5) { /* Print exactly on the Mag TDM slot */
            printf("PITCH: %4d | MAG[X:%5d Y:%5d Z:%5d] | PWM: %5d\n", 
                   (int)pitch, mag_x, mag_y, mag_z, new_duty_cycle);
        }
    }

    /* Shutdown */
    pwm_info.duty = PWM_MIN;
    ioctl(fd_pwm, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm_info));
    ioctl(fd_pwm, PWMIOC_STOP, 0);
    close(fd_pwm); close(fd_imu); close(fd_baro); close(fd_i2c);
    
    printf("[PASIL-MASTER] Test Complete.\n");
    return OK;
}
