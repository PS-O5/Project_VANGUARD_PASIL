#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <nuttx/sensors/sensor.h>
#include <sys/ioctl.h>
#include <nuttx/timers/pwm.h>


/* ARM CMSIS-DSP Header */
#define ARM_MATH_CM4
#include <arm_math.h>

#define LOOP_PERIOD_NS 2000000 
#define SEC_TO_NS      1000000000

/* EKF Constants */
#define STATE_DIM 7

/* Allocate Matrix Data Arrays (Static RAM) */
static float32_t P_data[STATE_DIM * STATE_DIM];
static float32_t F_data[STATE_DIM * STATE_DIM];
static float32_t F_T_data[STATE_DIM * STATE_DIM]; /* F Transpose */
static float32_t Q_data[STATE_DIM * STATE_DIM];
static float32_t temp_data[STATE_DIM * STATE_DIM];
static float32_t new_P_data[STATE_DIM * STATE_DIM];

/* CMSIS-DSP Matrix Instances */
static arm_matrix_instance_f32 P;
static arm_matrix_instance_f32 F;
static arm_matrix_instance_f32 F_T;
static arm_matrix_instance_f32 Q;
static arm_matrix_instance_f32 TempMat;
static arm_matrix_instance_f32 NewP;

static void init_ekf_matrices(void) {
    /* Initialize Matrix Structures */
    arm_mat_init_f32(&P, STATE_DIM, STATE_DIM, P_data);
    arm_mat_init_f32(&F, STATE_DIM, STATE_DIM, F_data);
    arm_mat_init_f32(&F_T, STATE_DIM, STATE_DIM, F_T_data);
    arm_mat_init_f32(&Q, STATE_DIM, STATE_DIM, Q_data);
    arm_mat_init_f32(&TempMat, STATE_DIM, STATE_DIM, temp_data);
    arm_mat_init_f32(&NewP, STATE_DIM, STATE_DIM, new_P_data);

    /* Zero out arrays */
    for (int i = 0; i < STATE_DIM * STATE_DIM; i++) {
        P_data[i] = 0.0f;
        F_data[i] = 0.0f;
        Q_data[i] = 0.0f;
    }
    
    /* Seed Identity/Initial Values */
    for (int i = 0; i < STATE_DIM; i++) {
        P_data[i * STATE_DIM + i] = 1.0f; /* Initial Covariance */
        F_data[i * STATE_DIM + i] = 1.0f; /* Base Identity for Jacobian */
        Q_data[i * STATE_DIM + i] = 0.001f; /* Process Noise */
    }
}

static void timespec_add_ns(struct timespec *ts, long ns) {
    ts->tv_nsec += ns;
    if (ts->tv_nsec >= SEC_TO_NS) {
        ts->tv_sec++;
        ts->tv_nsec -= SEC_TO_NS;
    }
}

int pasil_imu_task(int argc, char *argv[]) {
    struct timespec wakeup_time;
    int fd_pwm;
    struct pwm_info_s info;
    
    printf("[PASIL-EKF] Initializing CMSIS-DSP Matrices...\n");
    init_ekf_matrices();

    /* --- PHASE 4: HARDWARE ACTUATION --- */
    printf("[PASIL-CTRL] Engaging TIM2 Hardware PWM...\n");
    fd_pwm = open("/dev/pwm0", O_RDONLY);
    if (fd_pwm < 0) {
        printf("[PASIL-CTRL] FATAL: Cannot open /dev/pwm0\n");
        return ERROR;
    }

    /* Configure PWM: 50Hz Frequency, 1500us Duty Cycle (Neutral) */
    info.frequency = 50; 
    info.duty      = 32768; /* Duty cycle is represented as a 16-bit fraction (0-65535). 
                             * 1500us / 20000us = 0.075. 
                             * 0.075 * 65536 = 4915. Wait, 32768 is 50%. Let's set 1500us exactly.
                             */
    info.duty = 4915; /* 7.5% Duty cycle -> 1.5ms pulse on a 20ms period */
    
    ioctl(fd_pwm, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
    ioctl(fd_pwm, PWMIOC_START, 0);
    printf("[PASIL-CTRL] PWM Signal LIVE on PA0: 50Hz, 1.5ms pulse.\n");

    printf("[PASIL-EKF] Engaging 500Hz DSP Covariance Engine...\n");
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    for(int i = 0; i < 500; i++) {
        timespec_add_ns(&wakeup_time, LOOP_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        /* 1. Transpose F -> F_T */
        arm_mat_trans_f32(&F, &F_T);
        /* 2. Multiply F * P -> TempMat */
        arm_mat_mult_f32(&F, &P, &TempMat);
        /* 3. Multiply TempMat * F_T -> NewP */
        arm_mat_mult_f32(&TempMat, &F_T, &NewP);
        /* 4. Add Process Noise: NewP + Q -> P */
        arm_mat_add_f32(&NewP, &Q, &P);

        /* In a real stack, we would read the IMU, update the EKF, 
         * calculate the PID error, and update the info.duty via ioctl here. 
         */
    }

    /* Safety Cutoff */
    ioctl(fd_pwm, PWMIOC_STOP, 0);
    close(fd_pwm);

    printf("[PASIL-EKF] 500 Cycles Complete. DSP Matrix P[0][0] = %f\n", P_data[0]);
    printf("[PASIL-CTRL] PWM Signal Terminated.\n");
    
    return OK;
}
