#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"
#include <sys/ioctl.h>
#include <nuttx/i2c/i2c_master.h>

/* The global I2C file descriptor from pasil_ekf.c */
extern int pasil_fd_i2c; 

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
    struct i2c_msg_s msg; 
    struct i2c_transfer_s xfer;
    uint8_t tx_data[count + 1];
    
    tx_data[0] = index;
    for(uint32_t i = 0; i < count; i++) tx_data[i+1] = pdata[i];

    msg.frequency = 400000; msg.addr = Dev->I2cDevAddr; msg.flags = 0;
    msg.buffer = tx_data; msg.length = count + 1;
    xfer.msgv = &msg; xfer.msgc = 1;
    
    return (ioctl(pasil_fd_i2c, I2CIOC_TRANSFER, (unsigned long)&xfer) >= 0) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
    struct i2c_msg_s msg[2]; 
    struct i2c_transfer_s xfer;

    msg[0].frequency = 400000; msg[0].addr = Dev->I2cDevAddr; msg[0].flags = 0;
    msg[0].buffer = &index; msg[0].length = 1;
    msg[1].frequency = 400000; msg[1].addr = Dev->I2cDevAddr; msg[1].flags = I2C_M_READ;
    msg[1].buffer = pdata; msg[1].length = count;
    
    xfer.msgv = msg; xfer.msgc = 2;
    return (ioctl(pasil_fd_i2c, I2CIOC_TRANSFER, (unsigned long)&xfer) >= 0) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data) {
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data) {
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data) {
    uint8_t buf[2] = { (data >> 8) & 0xFF, data & 0xFF };
    return VL53L0X_WriteMulti(Dev, index, buf, 2);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data) {
    uint8_t buf[2];
    VL53L0X_Error status = VL53L0X_ReadMulti(Dev, index, buf, 2);
    if(status == VL53L0X_ERROR_NONE) *data = ((uint16_t)buf[0] << 8) | buf[1];
    return status;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data) {
    uint8_t buf[4] = { (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF };
    return VL53L0X_WriteMulti(Dev, index, buf, 4);
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data) {
    uint8_t buf[4];
    VL53L0X_Error status = VL53L0X_ReadMulti(Dev, index, buf, 4);
    if(status == VL53L0X_ERROR_NONE) *data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
    return status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData) {
    uint8_t data;
    VL53L0X_Error status = VL53L0X_RdByte(Dev, index, &data);
    if (status == VL53L0X_ERROR_NONE) {
        data = (data & AndData) | OrData;
        status = VL53L0X_WrByte(Dev, index, data);
    }
    return status;
}

/* Required by ST API to prevent slamming the I2C bus during status polling */
#include <unistd.h>

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) {
    usleep(2000); /* 2 millisecond thread yield */
    return VL53L0X_ERROR_NONE;
}
