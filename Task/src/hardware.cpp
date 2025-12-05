#include "hardware.h"
// spline interpolator for IMU
#include "spline.h"
#include "kalman.h"

// new
#include "SEGGER_RTT.h"

uint8_t can1TxData[8] = {0};
static uint8_t can1RxData[8] = {0};

uint8_t can2TxData[8] = {0};
static uint8_t can2RxData[8] = {0};

static CAN_RxHeaderTypeDef rx_header;
static CAN_TxHeaderTypeDef tx_header;

static int can_error = 0;
uint32_t spi_error = 0;

int debug_time = 0;
int uart_time = 0;

float raw_X = 0;
float raw_Y = 0;

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef canfilter;

    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;

    // filtrate any ID you want here
    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;

    canfilter.FilterActivation = ENABLE;
    canfilter.SlaveStartFilterBank = 14;

    // use different filter for can1&can2
    if (hcan == &hcan1)
    {
        canfilter.FilterBank = 0;
        canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
    }
    if (hcan == &hcan2)
    {
        canfilter.FilterBank = 14;
        canfilter.FilterFIFOAssignment = CAN_FilterFIFO1;
    }
    if (HAL_CAN_ConfigFilter(hcan, &canfilter) != HAL_OK)
    {
        Error_Handler();
    }
}

void sendCan(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t tx_data[8])
{
    uint32_t mail_box = 0;

    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &mail_box) != HAL_OK)
    {
        can_error++;
    }
}

float last_imu_yaw = 0;
float last_X = 0;
float last_Y = 0;

// Sensor offsets relative to robot rotation center (robot body frame),
// units must match optical-flow delta (e.g., millimeters)
// X forward, Y left
static const float OF_OFFSET_X = 115.0f; // optical flow sensor x
static const float OF_OFFSET_Y = 10.5f;  // optical flow sensor y
static const float IMU_OFFSET_X = 28.0f; // IMU x (not used for yaw, reserved for future accel fusion)
static const float IMU_OFFSET_Y = 50.0f; // IMU y

float global_X = 0;
float global_Y = 0;

float global_X_t = 0;
float global_Y_t = 0;

// 【新添加】机器人坐标系下的线速度 (vx, vy)
float body_vx = 0;
float body_vy = 0;

// 【新添加】Z轴角速度 (omega_z)
float omega_z = 0;

float delta_yaw = 0;
float delta_X = 0;
float delta_Y = 0;

float raw_vx = 0;
float raw_vy = 0;
float raw_omega = 0;

// optical/mouse timestamps (milliseconds from HAL_GetTick)
static uint32_t mouse_time_ms = 0;
static uint32_t last_mouse_time_ms = 0;

float delta_yaw_t = 0;
float angle_t = 0;

float f = 0;
float e = 0;

float et = 0;
float ft = 0;

float all_t = 0;

// Kalman filters for body velocities and angular rate
static Kalman1D kf_vx;
static Kalman1D kf_vy;
static Kalman1D kf_omega;

// IMU spline interpolation buffer (timestamps in seconds)
#define IMU_SPLINE_N 8
static float imu_time_buf[IMU_SPLINE_N];
static float imu_gz_buf[IMU_SPLINE_N];
static int imu_buf_count = 0; // number of valid samples
static int imu_buf_head = 0;  // next insert index
static spline::CubicSpline imu_gz_spline;
static bool imu_spline_ready = false;

// Simple robustness for optical-flow spikes: median-of-3 on raw samples
#define OF_MEDIAN_WIN 3
static float of_x_hist[OF_MEDIAN_WIN] = {0};
static float of_y_hist[OF_MEDIAN_WIN] = {0};
static int of_hist_count = 0;
static int of_hist_head = 0;

static inline float median3(float a, float b, float c)
{
    // branchless-ish median of three
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b; // now b is median
}

// (no DWT/micros helper here) use HAL_GetTick() which returns ms

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1) return;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, can1RxData) != HAL_OK)
    {
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
        return;
    }

    if (rx_header.StdId != 0x300)
    {
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
        return;
    }

    // read raw optical flow deltas (sensor-specific ordering)
    float raw_X_t = 0.0f, raw_Y_t = 0.0f;
    memcpy(&raw_X_t, can1RxData, sizeof(float));
    memcpy(&raw_Y_t, &(can1RxData[4]), sizeof(float));

    // update timestamp (milliseconds)
    mouse_time_ms = HAL_GetTick();

    // push into small median window to reject single-sample spikes
    of_x_hist[of_hist_head] = raw_Y_t;
    of_y_hist[of_hist_head] = -raw_X_t; // keep original sign convention
    of_hist_head = (of_hist_head + 1) % OF_MEDIAN_WIN;
    if (of_hist_count < OF_MEDIAN_WIN) of_hist_count++;

    if (of_hist_count == OF_MEDIAN_WIN)
    {
        int a = (of_hist_head + OF_MEDIAN_WIN - 1) % OF_MEDIAN_WIN;
        int b = (of_hist_head + OF_MEDIAN_WIN - 2) % OF_MEDIAN_WIN;
        int c = (of_hist_head + OF_MEDIAN_WIN - 3) % OF_MEDIAN_WIN;
        raw_X = median3(of_x_hist[a], of_x_hist[b], of_x_hist[c]);
        raw_Y = median3(of_y_hist[a], of_y_hist[b], of_y_hist[c]);
    }
    else
    {
        raw_X = raw_Y_t;
        raw_Y = -raw_X_t;
    }

    // First-sample guard
    if (last_mouse_time_ms == 0u)
    {
        last_X = raw_X;
        last_Y = raw_Y;
        last_mouse_time_ms = mouse_time_ms;
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
        return;
    }

    // delta in sensor units
    delta_X = raw_X - last_X;
    delta_Y = raw_Y - last_Y;
    all_t += sqrtf(delta_X * delta_X + delta_Y * delta_Y);

    // compute dt in seconds, protect against wraps
    uint32_t t0 = last_mouse_time_ms;
    uint32_t t1 = mouse_time_ms;
    uint32_t dt_ms = (t1 >= t0) ? (t1 - t0) : (t1 + (0xFFFFFFFFu - t0) + 1u);
    float dt_s = (float)dt_ms / 1000.0f;

    // clamp dt to avoid divide-by-small and missed-sample extremes
    const float min_dt_s = 0.001f; // 0.5 ms
    const float max_dt_s = 0.1f;    // 100 ms
    if (dt_s < min_dt_s) dt_s = min_dt_s;
    if (dt_s > max_dt_s) dt_s = max_dt_s;

    // compute delta yaw using spline if available, otherwise fallback to instantaneous gyro
    if (imu_spline_ready)
    {
        float t0s = (float)last_mouse_time_ms / 1000.0f;
        float t1s = (float)mouse_time_ms / 1000.0f;
        float gz0 = imu_gz_spline.eval(t0s);
        float gz1 = imu_gz_spline.eval(t1s);
        delta_yaw_t = 0.5f * (gz0 + gz1) * (t1s - t0s) / 180.0f * PI;
    }
    else
    {
        delta_yaw_t = dt_s * (robot.imu->getData(imu::kOmegaZ)) / 180.0f * PI;
    }

    // rigid-body correction
    float cosdt = cosf(delta_yaw_t);
    float sindt = sinf(delta_yaw_t);
    float dx_rot = (cosdt - 1.0f) * OF_OFFSET_X - sindt * OF_OFFSET_Y;
    float dy_rot =  sindt * OF_OFFSET_X + (cosdt - 1.0f) * OF_OFFSET_Y;
    e = delta_X - dx_rot;
    f = delta_Y - dy_rot;

    // --- 开始修改速度计算与滤波部分 ---

    // 1. 计算原始速度 (Raw Velocity)
    // 使用临时变量保存，而不是直接赋值给全局变量
    raw_vx = e / dt_s;
    raw_vy = f / dt_s;
    raw_omega = delta_yaw_t / dt_s;

    // 2. 【先限幅】 (Clamping)
    // 这里的 max_body_speed 是物理限制，超过这个值肯定是不正常的噪音
    // 我们把 raw_vx 变成 "限幅后的 Raw"
    const float max_body_speed = 3000.0f; 
    
    if (raw_vx > max_body_speed) raw_vx = max_body_speed;
    else if (raw_vx < -max_body_speed) raw_vx = -max_body_speed;

    if (raw_vy > max_body_speed) raw_vy = max_body_speed;
    else if (raw_vy < -max_body_speed) raw_vy = -max_body_speed;

    // 3. 【后滤波】 (Filtering)
    // 将 "限幅后的 Raw" 喂给卡尔曼，得到 "Filtered"
    // 这样对比的就是纯粹的滤波效果，而不包含去尖峰的效果
    body_vx = kf_vx.update(raw_vx);
    body_vy = kf_vy.update(raw_vy);
    omega_z = kf_omega.update(raw_omega);

    angle_t = atan2f(e, f);
    // 2. 限幅 (Clamp) - 针对原始值进行限幅，防止异常值破坏滤波器的内部状态
    //const float max_body_speed = 3000.0f;
    //if (fabsf(raw_vx) > max_body_speed) raw_vx = (raw_vx > 0) ? max_body_speed : -max_body_speed;
    //if (fabsf(raw_vy) > max_body_speed) raw_vy = (raw_vy > 0) ? max_body_speed : -max_body_speed;

    //angle_t = atan2f(e, f);

    // 3. 卡尔曼滤波 (Raw -> Filtered) 并更新全局变量
    // update() 接收原始值，返回滤波后的平滑值
    //body_vx = kf_vx.update(raw_vx);
    //body_vy = kf_vy.update(raw_vy);
    //omega_z = kf_omega.update(raw_omega);

    // 4. 【RTT 打印输出】
    // 使用 snprintf 格式化为 CSV 风格：时间, raw_vx, filt_vx, raw_vy, filt_vy
    // 缓冲区 128 字节通常足够
    char rtt_buf[128]; 
    // 注意：%.2f 保留两位小数，既节省传输带宽又能满足调试需求
    snprintf(rtt_buf, sizeof(rtt_buf), "%lu,%.2f,%.2f,%.2f,%.2f\n", 
             mouse_time_ms, raw_vx, body_vx, raw_vy, body_vy);
    
    // 发送到 RTT 通道 0
    SEGGER_RTT_WriteString(0, rtt_buf);

    // --- 修改结束 ---

    global_X_t += e;
    global_Y_t += f;

    float yaw_now_rad = (robot.imu->getData(imu::kAngleZ) - (-108.67f)) / 180.0f * PI;
    float yaw_mid_rad = yaw_now_rad + 0.5f * delta_yaw_t;
    global_X += e * cosf(yaw_mid_rad) + f * sinf(yaw_mid_rad);
    global_Y += -e * sinf(yaw_mid_rad) + f * cosf(yaw_mid_rad);

    last_X = raw_X;
    last_Y = raw_Y;
    last_mouse_time_ms = mouse_time_ms;

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan2)
    {

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, can2RxData) == HAL_OK)
        {
            for (int i = 0; i < 4; i++)
            {
                if (rx_header.StdId == robot.wheelMotor[i]->rx_id())
                {
                    robot.wheelMotor[i]->decode(can2RxData);
                }
            }

            if (rx_header.StdId == robot.dribbler->rx_id())
            {
                robot.dribbler->decode(can2RxData);
            }
        }
    }
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // huart1 <---> linux
    if (huart == &huart1)
    {
        // uart_time = HAL_GetTick();
        HAL_UART_Transmit(&huart1, robot.piTxDataUart, piTxDataUartLength, 0x1);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, robot.piRxDataUart, piRxDataUartLength);
    }

    if (huart == &huart4)
    {
        uart_time = HAL_GetTick();

        /* Size parameter not used in this callback */
        (void)Size;

        /* read DR to clear RXNE; don't create an unused variable */
        (void)huart->Instance->DR;

        delta_yaw = (robot.imu->getData(imu::kAngleZ) - last_imu_yaw) / 180.0 * PI;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, robot.imuRxData, imu::imuRxDataLength);

        robot.imu->decode(robot.imuRxData);

        // record timestamped gyro-z sample (deg/s) for spline interpolation
        {
            float t_s = uart_time / 1000.0f; // seconds
            float gz = robot.imu->getData(imu::kOmegaZ);

            imu_time_buf[imu_buf_head] = t_s;
            imu_gz_buf[imu_buf_head] = gz;
            imu_buf_head = (imu_buf_head + 1) % IMU_SPLINE_N;
            if (imu_buf_count < IMU_SPLINE_N) imu_buf_count++;

            // build ordered arrays (oldest..newest) for spline
            if (imu_buf_count >= 2)
            {
                float xt[IMU_SPLINE_N];
                float yt[IMU_SPLINE_N];
                int start = (imu_buf_head - imu_buf_count + IMU_SPLINE_N) % IMU_SPLINE_N;
                for (int i = 0; i < imu_buf_count; ++i)
                {
                    int idx = (start + i) % IMU_SPLINE_N;
                    xt[i] = imu_time_buf[idx];
                    yt[i] = imu_gz_buf[idx];
                }
                // setPoints expects strictly increasing x; it's usually satisfied because timestamps increase
                if (imu_gz_spline.setPoints(xt, yt, (std::size_t)imu_buf_count))
                {
                    imu_spline_ready = true;
                }
                else
                {
                    imu_spline_ready = false;
                }
            }
            else
            {
                imu_spline_ready = false;
            }
        }

        last_imu_yaw = robot.imu->getData(imu::kAngleZ);

    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1)
    {
        // HAL_IWDG_Refresh(&hiwdg);
        debug_time = HAL_GetTick();

        HAL_SPI_TransmitReceive_DMA(&hspi1, robot.spiTxData, robot.spiRxData, spi_length);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    spi_error = HAL_SPI_GetError(hspi);
    if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_MODF))
    {
        __HAL_SPI_CLEAR_MODFFLAG(hspi);
        __HAL_RCC_SPI1_FORCE_RESET();
        __HAL_RCC_SPI1_RELEASE_RESET();
        MX_SPI1_Init();
        HAL_SPI_TransmitReceive_IT(hspi, robot.spiTxData, robot.spiRxData, spi_length);
    }
}

void getADC(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
    {
        robot.INFRA_ADC1_Value = HAL_ADC_GetValue(&hadc1) / 4095.0 * 3.3;
    }

    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 1);
    if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC))
    {
        robot.BATVOL_ADC2_Value = HAL_ADC_GetValue(&hadc2) / 4095.0 * 3.3 * bat_k;
    }

    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 1);
    if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc3), HAL_ADC_STATE_REG_EOC))
    {
        robot.CAPVOL_ADC3_Value = HAL_ADC_GetValue(&hadc3) / 4095.0 * 3.3 * cap_k;
    }
}

void hardware_init(void)
{

    HAL_CAN_Start(&hcan1);
    CanFilter_Init(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_CAN_Start(&hcan2);
    CanFilter_Init(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, robot.piRxDataUart, piRxDataUartLength);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, robot.imuRxData, imu::imuRxDataLength);
}
