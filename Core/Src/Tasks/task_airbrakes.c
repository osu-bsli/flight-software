#include "flight_software.h"
#include "stm32h7xx_hal.h"
#include "SEGGER_RTT.h"
#include "main.h"
#include <FreeRTOS.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <task.h>
#include <arm_math.h>

static TaskHandle_t handle;
static StaticTask_t tcb;

#define STACK_SIZE 32768
static StackType_t stack[STACK_SIZE];

/*
void matrixMultiplyd(double *A, double *B, double *C,
                     int rowsA, int colsA, int colsB)
{
  // C = A * B
  for (int i = 0; i < rowsA; i++)
  {
    for (int j = 0; j < colsB; j++)
    {
      double sum = 0.0;
      for (int k = 0; k < colsA; k++)
      {
        sum += A[i * colsA + k] * B[k * colsB + j];
      }
      C[i * colsB + j] = sum;
    }
  }
}

void matrixMultiplyf(float *A, float *B, float *C,
                     int rowsA, int colsA, int colsB)
{
  // C = A * B
  for (int i = 0; i < rowsA; i++)
  {
    for (int j = 0; j < colsB; j++)
    {
      float sum = 0.0;
      for (int k = 0; k < colsA; k++)
      {
        sum += A[i * colsA + k] * B[k * colsB + j];
      }
      C[i * colsB + j] = sum;
    }
  }
}
*/

// Matrix dimensions
#define N       57
#define SIZE    (N * N)

static void task_airbrakes(void *argument)
{
  UNUSED(argument);

  float Ad[SIZE], Bd[SIZE], Cd[SIZE];
  arm_matrix_instance_f32 A, B, C;

  arm_mat_init_f32(&A, N, N, Ad);
  arm_mat_init_f32(&B, N, N, Bd);
  arm_mat_init_f32(&C, N, N, Cd);

  // Seed the random number generator
  srand(1337);

  // Initialize matrices with random numbers.
  for (int i = 0; i < SIZE; i++)
  {
    Ad[i] = (float)rand() / 0x7FFF;
    Bd[i] = (float)rand() / 0x7FFF;
    Cd[i] = (float)rand() / 0x7FFF;
  }

  uint32_t time_ms = HAL_GetTick();
  uint32_t num_matmuls = 0;

  while (true)
  {
    // Measure matrix multiplications.
    arm_mat_mult_f32(&A, &B, &C);

    num_matmuls++;

    if (HAL_GetTick() >= time_ms + 1000)
    {
      time_ms += 1000;

      SEGGER_RTT_printf(0, "A second has passed on the airbrakes task: %d matmuls\n", num_matmuls);
      num_matmuls = 0;
    }
  }
}

void task_airbrakes_start(void)
{
  ASSERT(handle == NULL);

  handle = xTaskCreateStatic(
      task_airbrakes,   /* Function that implements the task. */
      "airbrakes",      /* Text name for the task. */
      STACK_SIZE,       /* Number of indexes in the xStack array. */
      NULL,             /* Parameter passed into the task. */
      2, /* Priority at which the task is created. */
      stack,            /* Array to use as the task's stack. */
      &tcb);            /* Variable to hold the task's data structure. */
}
