#include "freertos_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "main.h"           // for extern handles (I2C, SPI, TIM, GPIO)
#include <stdio.h>

// Include prototypes for your sensor functions (from original main.c)
extern void MPU6050_Init(void);
extern void MPU6050_Read_All(void);
extern int MAX30102_Init(I2C_HandleTypeDef *hi2c);
extern BMP280_HandleTypedef bmp280;
extern int bmp280_init(BMP280_HandleTypedef *dev);
extern int bmp280_read_all(BMP280_HandleTypedef *dev);
extern float bmp270_calculate_altitude_stub(float pressure, float sea_level_pressure);
extern float bmp280_calculate_altitude(float pressure, float sea_level_pressure);

// LCD functions from your code
extern void LCD_DisplayHeader(void);
extern void LCD_DisplaySensorStatus(void);
extern void LCD_DisplayMAX30102Data(uint32_t red, uint32_t ir, float ratio);
extern void LCD_DisplayBMP280Data(float temp, float pressure, float altitude);
extern void LCD_DisplayMPU6050Data(float ax, float ay, float az, float temp);
extern void LCD_DisplayReadingNumber(uint32_t reading_num);
extern void LCD_ClearDataArea(void);

// FreeRTOS objects (exported)
TaskHandle_t xMPUTaskHandle = NULL;
TaskHandle_t xMAX30102TaskHandle = NULL;
TaskHandle_t xBMP280TaskHandle = NULL;
TaskHandle_t xLCDTaskHandle = NULL;
TaskHandle_t xHeartbeatTaskHandle = NULL;

QueueHandle_t qMPU = NULL;
QueueHandle_t qMAX30102 = NULL;
QueueHandle_t qBMP280 = NULL;

static uint32_t reading_counter = 0;

/* ----- MPU6050 Task: reads IMU at 1 Hz ----- */
static void vMPUTask(void *pvParameters)
{
    const TickType_t xDelay = pdMS_TO_TICKS(1000);
    mpu_data_t out;
    for(;;)
    {
        // read using your existing function which fills global Ax,Ay,Az,... 
        MPU6050_Read_All();

        // copy to local struct (global variables from original code)
        extern float Ax, Ay, Az, Gx, Gy, Gz, MPU_Temperature;
        out.ax = Ax;
        out.ay = Ay;
        out.az = Az;
        out.gx = Gx;
        out.gy = Gy;
        out.gz = Gz;
        out.temp = MPU_Temperature;

        // send to LCD queue (non-blocking, drop on full)
        if(xQueueSend(qMPU, &out, 0) != pdPASS) {
            // queue full; you might log or handle overflow
        }

        vTaskDelay(xDelay);
    }
}

/* ----- MAX30102 Task: reads HR/SpO2 FIFO every 1s ----- */
static void vMAX30102Task(void *pvParameters)
{
    const TickType_t xDelay = pdMS_TO_TICKS(1000);
    max30102_data_t out;
    HAL_StatusTypeDef status;

    for(;;)
    {
        // Read FIFO (use same register reads you used in main.c)
        uint8_t fifo_data[6] = {0};
        status = HAL_I2C_Mem_Read(&hi2c1, 0xAE, 0x07, 1, fifo_data, 6, 100);

        if(status == HAL_OK)
        {
            uint32_t red = ((uint32_t)fifo_data[0] << 16) |
                           ((uint32_t)fifo_data[1] << 8) |
                           fifo_data[2];
            red &= 0x3FFFF;

            uint32_t ir = ((uint32_t)fifo_data[3] << 16) |
                          ((uint32_t)fifo_data[4] << 8) |
                          fifo_data[5];
            ir &= 0x3FFFF;

            out.red = red;
            out.ir = ir;
            out.ratio = (ir > 0) ? ((float)red / (float)ir) : 0.0f;

            xQueueSend(qMAX30102, &out, 0);
        } else {
            // I2C error; optional: send zeros or skip
            out.red = 0; out.ir = 0; out.ratio = 0.0f;
            xQueueSend(qMAX30102, &out, 0);
        }

        vTaskDelay(xDelay);
    }
}

/* ----- BMP280 Task: read environment every 3s ----- */
static void vBMP280Task(void *pvParameters)
{
    const TickType_t xDelay = pdMS_TO_TICKS(3000);
    bmp_data_t out;

    for(;;)
    {
        if(bmp280_read_all(&bmp280) == BMP280_OK)
        {
            out.temp = bmp280.temperature;
            out.pressure = bmp280.pressure;
            out.altitude = bmp280_calculate_altitude(bmp280.pressure, 1013.25f);
        } else {
            out.temp = 0.0f; out.pressure = 0.0f; out.altitude = 0.0f;
        }

        xQueueSend(qBMP280, &out, 0);
        vTaskDelay(xDelay);
    }
}

/* ----- LCD Task: collects latest data and refreshes display ----- */
static void vLCDTask(void *pvParameters)
{
    mpu_data_t mpu_last = {0};
    max30102_data_t max_last = {0};
    bmp_data_t bmp_last = {0};

    uint32_t loop_tick = 0;

    // initial header
    LCD_DisplayHeader();
    LCD_DisplaySensorStatus();

    for(;;)
    {
        // Try to receive without blocking (use latest data only)
        if(xQueueReceive(qMPU, &mpu_last, 0) == pdPASS) {
            // update reading counter for display tracking
        }
        if(xQueueReceive(qMAX30102, &max_last, 0) == pdPASS) { }
        if(xQueueReceive(qBMP280, &bmp_last, 0) == pdPASS) { }

        // cycle display every second: MPU -> MAX -> BMP
        loop_tick++;
        uint8_t display_mode = loop_tick % 3; // 1,2,0 sequence
        reading_counter++;
        LCD_DisplayReadingNumber(reading_counter);

        if(display_mode == 1) {
            if(mpu6050_ready) {
                LCD_DisplayMPU6050Data(mpu_last.ax, mpu_last.ay, mpu_last.az, mpu_last.temp);
            }
        } else if(display_mode == 2) {
            if(max30102_ready) {
                LCD_DisplayMAX30102Data(max_last.red, max_last.ir, max_last.ratio);
            }
        } else {
            if(bmp280_ready) {
                LCD_DisplayBMP280Data(bmp_last.temp, bmp_last.pressure, bmp_last.altitude);
            }
        }

        // update status bar occasionally
        if ((loop_tick % 5) == 0) {
            LCD_DisplaySensorStatus();
        }

        // refresh rate ~1s (cooperate with sensor tasks)
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ----- Heartbeat Task ----- */
static void vHeartbeatTask(void *pvParameters)
{
    const TickType_t xDelay = pdMS_TO_TICKS(500);
    for(;;)
    {
        HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        vTaskDelay(xDelay);
    }
}

/* ----- Initialization: create queues & tasks ----- */
void FREERTOS_TasksInit(void)
{
    // create queues (length small because tasks send latest only)
    qMPU = xQueueCreate(2, sizeof(mpu_data_t));
    qMAX30102 = xQueueCreate(2, sizeof(max30102_data_t));
    qBMP280 = xQueueCreate(2, sizeof(bmp_data_t));

    // safety: check queue creation
    configASSERT(qMPU);
    configASSERT(qMAX30102);
    configASSERT(qBMP280);

    // Create tasks
    xTaskCreate(vMPUTask, "MPU", 256, NULL, tskIDLE_PRIORITY + 2, &xMPUTaskHandle);
    xTaskCreate(vMAX30102Task, "MAX30102", 256, NULL, tskIDLE_PRIORITY + 2, &xMAX30102TaskHandle);
    xTaskCreate(vBMP280Task, "BMP280", 256, NULL, tskIDLE_PRIORITY + 1, &xBMP280TaskHandle);
    xTaskCreate(vLCDTask, "LCD", 384, NULL, tskIDLE_PRIORITY + 3, &xLCDTaskHandle);
    xTaskCreate(vHeartbeatTask, "HB", 128, NULL, tskIDLE_PRIORITY + 1, &xHeartbeatTaskHandle);
}
