# Coding Guidelines

## Sensor Drivers

Do not use magic numbers. Use `#define` for constants, including register addresses and bit masks.

```c
/*
 * Bad: Magic number is used directly.  
 */

if (who_am_i_read_val != 0b00110011) {
    return HAL_ERROR;
} 

/*
 * Good: Constant is defined and used.
 */

// Define constants at the start of the .c file
#define WHO_AM_I_EXPECTED 0b00110011

// ...

// Use defined constant in code
if (who_am_i_read_val != WHO_AM_I_EXPECTED) {
    return HAL_ERROR;
}
```

`#define` shared constants in `.h` files so other `.c` files can include them. `#define` private constants directly in `.c` files.

```c
// Only the driver code in lis3dh.c needs to see these defines, place them in lis3dh.c
#define CTRL_REG1 0x20
#define CTRL_REG1_ODR_10HZ 0b00100000   // 10 Hz
#define CTRL_REG1_ODR_100HZ 0b01010000  // 100 Hz
#define CTRL_REG1_ODR_1344HZ 0b10010000 // 1344 Hz
#define CTRL_REG1_ENABLE_ALL_AXES 0b111

// These constants could be useful to other .c files, place them in lis3dh.h
// Prefix shared constants properly (with LIS3DH in this case) so it is clear which module they came from.
#define LIS3DH_AXIS_MAX_RAW 32767 
#define LIS3DH_AXIS_MIN_RAW -32768
#define LIS3DH_AXIS_MAX_G 8.0 
#define LIS3DH_AXIS_MIN_G -8.0
```

Return a status value from any function that can fail. 

```c
HAL_StatusTypeDef lis3dh_initialize(void) {
    if (!callback) {
        // No callback is registered.
        return HAL_ERROR;
    }
    
    // ...

    return HAL_OK;
}
```

Check all return values of functions that can fail. **Improperly handled sensor failures could result in serious consequences, such as failure to deploy parachute and loss of vehicle.**

```c
HAL_StatusTypeDef status;

status = read(WHO_AM_I, &read_buf, sizeof(read_buf));
if (status != HAL_OK) return status;
```

If a function won't be called from a C file other than the one it is defined in, declare it `static` so that other C files **cannot** call it.

```c
/*
 * This function is inside the lis3dh.c sensor driver file.
 * It is for reading registers from the LIS3DH accelerometer and is not part of
 * the public interface for that driver so it is marked static. 
 */
static HAL_StatusTypeDef read(const uint8_t addr, uint8_t *data,
                              const uint16_t data_len) {

    // ...

}
```