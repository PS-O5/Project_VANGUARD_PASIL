#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <nuttx/sensors/sensor.h>

/* Phase 3: 500Hz EKF Hardware Math Stress Test */
#define LOOP_PERIOD_NS 2000000 
#define SEC_TO_NS      1000000000
#define DT_SEC         0.002f

struct imu_6dof_data {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
};

/* Telemetry Buffer */
struct ekf_log {
    long dt_us;
    float q0, q1, q2, q3; /* Quaternion states */
};

static struct ekf_log telemetry_buffer[500];
static struct imu_6dof_data raw_imu;

static void timespec_add_ns(struct timespec *ts, long ns) {
    ts->tv_nsec += ns;
    if (ts->tv_nsec >= SEC_TO_NS) {
        ts->tv_sec++;
        ts->tv_nsec -= SEC_TO_NS;
    }
}

static long calc_dt_us(struct timespec *start, struct timespec *end) {
    long sec_diff = end->tv_sec - start->tv_sec;
    long nsec_diff = end->tv_nsec - start->tv_nsec;
    return (sec_diff * 1000000) + (nsec_diff / 1000);
}

int pasil_imu_task(int argc, char *argv[]) {
    int fd;
    struct timespec wakeup_time, cycle_start, cycle_end;
    
    /* Dummy EKF State Vector */
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    
    printf("[PASIL-PHASE3] Booting EKF FPU Stress Test. Delaying 3s...\n");
    sleep(3);
    printf("[PASIL-PHASE3] Engaging 500Hz Hardware Math Load (SILENT)...\n");

    fd = open("/dev/imu0", O_RDONLY);
    if (fd < 0) return ERROR;

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    cycle_start = wakeup_time;

    for(int i = 0; i < 500; i++) {
        timespec_add_ns(&wakeup_time, LOOP_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        read(fd, &raw_imu, sizeof(struct imu_6dof_data));
        
        /* --- EKF COMPUTATIONAL STRESS LOAD --- */
        /* We simulate the quaternion derivative update and normalization.
         * If the FPU is inactive, this block will cause massive latency.
         */
        float gx = raw_imu.gyro_x;
        float gy = raw_imu.gyro_y;
        float gz = raw_imu.gyro_z;

        /* Quaternion Kinematics: q_dot = 0.5 * q \otimes omega */
        float dq0 = 0.5f * (-q[1]*gx - q[2]*gy - q[3]*gz) * DT_SEC;
        float dq1 = 0.5f * ( q[0]*gx + q[2]*gz - q[3]*gy) * DT_SEC;
        float dq2 = 0.5f * ( q[0]*gy - q[1]*gz + q[3]*gx) * DT_SEC;
        float dq3 = 0.5f * ( q[0]*gz + q[1]*gy - q[2]*gx) * DT_SEC;

        q[0] += dq0; q[1] += dq1; q[2] += dq2; q[3] += dq3;

        /* Heavy Math: Normalization requires Inverse Square Root */
        float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if (norm > 0.0f) {
            q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
        }

        /* Simulate a heavy trigonometric Jacobian evaluation */
        volatile float dummy_stress = sinf(q[1]) * cosf(q[2]) + tanf(q[3]);
        (void)dummy_stress; /* Prevent compiler optimization */

        clock_gettime(CLOCK_MONOTONIC, &cycle_end);
        long dt_us = calc_dt_us(&cycle_start, &cycle_end);

        telemetry_buffer[i].dt_us = dt_us;
        telemetry_buffer[i].q0 = q[0];
        telemetry_buffer[i].q1 = q[1];
        telemetry_buffer[i].q2 = q[2];
        telemetry_buffer[i].q3 = q[3];

        cycle_start = cycle_end;
    }

    printf("[PASIL-PHASE3] Collection Complete. Draining Telemetry:\n");
    for(int i = 0; i < 500; i++) {
        printf("T+%03d | dt: %4ld us | Q: [%5.3f, %5.3f, %5.3f, %5.3f]\n", 
               i, telemetry_buffer[i].dt_us, 
               telemetry_buffer[i].q0, telemetry_buffer[i].q1, 
               telemetry_buffer[i].q2, telemetry_buffer[i].q3);
    }

    close(fd);
    return OK;
}
