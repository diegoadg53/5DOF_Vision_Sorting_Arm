// Real firmware functions; only clocks, SPI/UART transactions, and GPIO are mocked.
#define main firmware_main
#include "../Core/Src/main.c"
#undef main

typedef struct {
    uint32_t checks, failures, first_failure;
} TestResult;
volatile TestResult test_result;
volatile uint32_t test_failure_codes[32];
volatile uint8_t uart_rx_stream[32];
volatile uint32_t uart_rx_count, uart_rx_index, uart_rx_active;
volatile uint8_t servo_rx_stream[3][16];
volatile uint32_t servo_rx_count[3], servo_rx_index[3];
static int servo_reply = 1;
static int servo_corrupt_reply;
static uint32_t direction_errors;
static uint32_t mock_ms, mock_subtick, position_writes, nonzero_motor_writes;
static uint8_t position_id;
static uint16_t selected_pin, last_spi_command;
static uint16_t last_spi_write, spi_bank[5][16];
static int corrupt_reads, corrupt_index = -1, rx_restart_fail;
static unsigned rx_start_calls, rx_abort_calls;
static int rx_error_during_start;
static char calibration_status_tx[160];
volatile JointDiagnosticPacket test_diag_sample;
static JointDiagnosticPacket diagnostic_packet_tx;
static unsigned usb_blocking_writes;
static uint32_t unsafe_autocal_changes;
static uint16_t spi_flags = 0xF000;
static uint16_t spi_status_injection;
static uint32_t wrist_motor_commands;
static HAL_StatusTypeDef motor_it_status;
static int spi_fail, corrupt_spi, uart_fail, wrist_reply = 1;
static int stop_during_mode;
static float wrist_raw = 206.64f;
static float spi_raw[5] = {230, 211.4f, 225, 38, 274.6f};
uint32_t SystemCoreClock = 84000000;
static int mock_flash_enabled;
static unsigned mock_flash_writes;
HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return mock_flash_enabled ? HAL_OK : HAL_ERROR; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t value) {
    (void)type;
    if (!mock_flash_enabled || address < 0x08040000U || address >= 0x08080000U) return HAL_ERROR;
    uint32_t *p = (uint32_t *)address;
    if ((*p & (uint32_t)value) != (uint32_t)value) return HAL_ERROR;
    *p &= (uint32_t)value; ++mock_flash_writes; return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *error) {
    (void)error;
    if (!mock_flash_enabled || erase->Sector < 6 || erase->Sector > 7) return HAL_ERROR;
    memset((void *)(0x08040000U + (erase->Sector - 6) * 0x20000U), 255, 0x20000);
    return HAL_OK;
}
static int ssc_active, ssc_clock, ssc_output, ssc_data, ssc_reply_ready;
static unsigned ssc_tx_bits, ssc_rx_bits, ssc_edges, ssc_bad_edges, spi3_hal_calls;
static uint16_t ssc_tx[2], ssc_rx[9];
static unsigned ssc_rx_count;
static int ssc_shift_reply, ssc_cancel_at_bit, ssc_stuck_data;
volatile uint32_t test_dwt_stopped;
volatile uint32_t test_dwt_step = 84;
static HAL_StatusTypeDef MockSpiReply(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);

// Newlib's float scanner uses a heap; this exists only in the emulator fixture.
void *_sbrk(ptrdiff_t increment) {
    static char heap[32768];
    static ptrdiff_t used;
    if (increment < 0 || used + increment > (ptrdiff_t)sizeof(heap))
        return (void *)-1;
    void *p = heap + used;
    used += increment;
    return p;
}
int printf(const char *format, ...) {
    (void)format;
    return 0;
}
uint32_t HAL_GetTick(void) {
    if (++mock_subtick >= 16) {
        mock_subtick = 0;
        ++mock_ms;
    }
    return mock_ms;
}
void HAL_Delay(uint32_t ms) {
    mock_ms += ms;
    if (stop_during_mode) command_queue.emergency = 1;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
    if (state == GPIO_PIN_SET) port->ODR |= pin;
    else port->ODR &= ~pin;
    if (port == GPIOB && pin == CS_JointAxle_DOF3_Pin) {
        if (state == GPIO_PIN_RESET) {
            ssc_active = 1;
            ssc_reply_ready = 0;
            ssc_tx_bits = ssc_rx_bits = ssc_edges = 0;
            memset(ssc_tx, 0, sizeof(ssc_tx));
        } else ssc_active = 0;
    }
    if (port == DOF3_Sensor_MOSI_GPIO_Port && pin == DOF3_Sensor_MOSI_Pin)
        ssc_data = state == GPIO_PIN_SET;
    if (port == DOF3_Sensor_SCK_GPIO_Port && pin == DOF3_Sensor_SCK_Pin) {
        int high = state == GPIO_PIN_SET;
        if (ssc_active && high != ssc_clock) {
            ++ssc_edges;
            if (!high && ssc_output) {
                if (ssc_tx_bits < 32) {
                    unsigned w = ssc_tx_bits / 16;
                    ssc_tx[w] = (uint16_t)((ssc_tx[w] << 1) | ssc_data);
                } else ++ssc_bad_edges;
                ++ssc_tx_bits;
            }
        }
        ssc_clock = high;
    }
    if (state == GPIO_PIN_RESET)
        selected_pin = pin;
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) {
    if (port != DOF3_Sensor_MOSI_GPIO_Port || init->Pin != DOF3_Sensor_MOSI_Pin) return;
    ssc_output = init->Mode == GPIO_MODE_OUTPUT_PP;
    if (!ssc_output && ssc_active && ssc_tx_bits && !ssc_reply_ready) {
        last_spi_command = ssc_tx[0];
        unsigned expected_tx = last_spi_command & 0x8000 ? 16 : 32;
        if (ssc_clock || ssc_tx_bits != expected_tx) ++ssc_bad_edges;
        if (!(last_spi_command & 0x8000)) {
            last_spi_write = ssc_tx[1];
            unsigned address = (last_spi_command >> 4) & 0x3F;
            if (!spi_fail) {
                if (address == 8 && (spi_bank[4][8] & 3) &&
                    ((last_spi_write ^ spi_bank[4][8]) & 0xFFFC)) ++unsafe_autocal_changes;
                spi_bank[4][address] = last_spi_write;
            }
        }
        ssc_rx_count = last_spi_command & 0x8000 ? (last_spi_command & 15) + 1U : 1U;
        memset(ssc_rx, 0xFF, sizeof(ssc_rx));
        MockSpiReply(&hspi3, (uint8_t *)ssc_rx, ssc_rx_count, SENSOR_SPI_TIMEOUT_MS);
        if (ssc_shift_reply && last_spi_command == 0x8021) {
            uint32_t frame = ((uint32_t)ssc_rx[0] << 16) | ssc_rx[1];
            frame = (frame << 1) | 1U;
            ssc_rx[0] = frame >> 16; ssc_rx[1] = frame;
        }
        ssc_reply_ready = 1;
    }
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
    if (port != DOF3_Sensor_MOSI_GPIO_Port || pin != DOF3_Sensor_MOSI_Pin ||
        !ssc_active || ssc_output || ssc_clock || !ssc_reply_ready ||
        ssc_rx_bits >= ssc_rx_count * 16) {
        ++ssc_bad_edges; return GPIO_PIN_RESET;
    }
    unsigned bit = ssc_rx_bits++;
    if (ssc_cancel_at_bit && ssc_rx_bits == (unsigned)ssc_cancel_at_bit)
        command_queue.emergency = 1;
    if (ssc_stuck_data) return ssc_stuck_data > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
    return (ssc_rx[bit / 16] & (1U << (15 - bit % 16))) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, const uint8_t *bytes, uint16_t n,
                                   uint32_t timeout) {
    if (h == &hspi3) ++spi3_hal_calls;
    (void)n;
    (void)timeout;
    last_spi_command = *(uint16_t *)bytes;
    if (!spi_fail && n == 2 && !(last_spi_command & 0x8000U)) {
        int index = TleDeviceIndex(h, selected_pin);
        last_spi_write = ((const uint16_t *)bytes)[1];
        unsigned address = (last_spi_command >> 4) & 0x3F;
        if (address == 8 && (spi_bank[index][8] & 3) &&
            ((last_spi_write ^ spi_bank[index][8]) & 0xFFFC)) ++unsafe_autocal_changes;
        spi_bank[index][(last_spi_command >> 4) & 0x3F] = last_spi_write;
    }
    return spi_fail ? HAL_TIMEOUT : HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *h, uint8_t *bytes, uint16_t n,
                                  uint32_t timeout) {
    if (h == &hspi3) ++spi3_hal_calls;
    return MockSpiReply(h, bytes, n, timeout);
}
static HAL_StatusTypeDef MockSpiReply(SPI_HandleTypeDef *h, uint8_t *bytes, uint16_t n,
                                      uint32_t timeout) {
    (void)timeout;
    if (spi_fail)
        return HAL_TIMEOUT;
    int index = h == &hspi1   ? (selected_pin == CS_MotorAxle_DOF1_Pin ? 0 : 1)
                : h == &hspi2 ? (selected_pin == CS_MotorAxle_DOF2_Pin ? 2 : 3)
                              : 4;
    int bad_config = spi_bank[index][15] != Feedback_TleParameterCrc(&spi_bank[index][8]);
    uint16_t flags = bad_config ? spi_flags & ~0x4000U : spi_flags;
    uint8_t crc_bytes[18] = {(uint8_t)(last_spi_command >> 8), (uint8_t)last_spi_command};
    uint16_t *response = (uint16_t *)bytes;
    if (!(last_spi_command & 0x8000U)) {
        crc_bytes[2] = (uint8_t)(last_spi_write >> 8);
        crc_bytes[3] = (uint8_t)last_spi_write;
        response[0] = flags | 0xB00U | Feedback_Crc8(crc_bytes, 4);
    } else {
        unsigned count = n - 1;
        unsigned address = (last_spi_command >> 4) & 0x3F;
        for (unsigned i = 0; i < count; ++i) {
            response[i] = address + i == 2 ?
                (uint16_t)(spi_raw[index] * 32768.0f / 360.0f) | 0x8000U :
                address + i == 0 ? ((bad_config ? 8 : 0) | spi_status_injection) : spi_bank[index][address+i];
            crc_bytes[2+2*i] = (uint8_t)(response[i] >> 8);
            crc_bytes[3+2*i] = (uint8_t)response[i];
        }
        response[count] = flags | 0xB00U | Feedback_Crc8(crc_bytes, 2+2*count);
        if (corrupt_spi || (last_spi_command == 0x8021 && corrupt_reads &&
                            (corrupt_index < 0 || corrupt_index == index))) {
            response[count] ^= 1;
            if (corrupt_reads) --corrupt_reads;
        }
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *bytes, uint16_t n,
                                    uint32_t timeout) {
    (void)timeout;
    if (h==&huart2) ++usb_blocking_writes;
    int direction_bus = MotorBusIndex(h);
    if (direction_bus >= 0) {
        GPIO_TypeDef *ports[4] = {GPIOA,GPIOB,GPIOA,GPIOC};
        uint16_t pins[4] = {GPIO_PIN_9,GPIO_PIN_10,GPIO_PIN_0,GPIO_PIN_12};
        if ((h->Instance->CR1 & (USART_CR1_TE | USART_CR1_RE)) != USART_CR1_TE ||
            (ports[direction_bus]->OTYPER & pins[direction_bus])) {
            ++direction_errors; return HAL_ERROR;
        }
    }
    if (uart_fail == 1 || (uart_fail == 2 && h == &huart5 && n >= 6 && bytes[4] == 1))
        return HAL_ERROR;
    if (h == &huart5 && n >= 6 && bytes[4] == 1) {
        ++position_writes;
        position_id = bytes[2];
    }
    if (h == &huart5 && n == 6 && bytes[4] == 28 && wrist_reply) {
        int16_t position = (int16_t)roundf(wrist_raw / .24f);
        uint8_t frame[8] = {
            0x55, 0x55, bytes[2], 5, 28, (uint8_t)position, (uint8_t)(position >> 8), 0};
        frame[7] = (uint8_t)~(uint8_t)(frame[2] + frame[3] + frame[4] + frame[5] + frame[6]);
        for (int i = 0; i < 8; ++i)
            uart_rx_stream[i] = frame[i];
        uart_rx_index = 0;
        uart_rx_count = 8;
        uart_rx_active = 1;
    }
    int bus = MotorBusIndex(h);
    if (servo_reply && bus >= 0 && bus < 3 && n == 6) {
        uint8_t cmd = bytes[4], id = (uint8_t)(bus+1), data[4] = {0};
        uint8_t count = cmd == 14 || cmd == 26 || cmd == 32 || cmd == 25 ? 1 :
                        cmd == 27 || cmd == 28 ? 2 : 4;
        if (cmd == 14) data[0] = id;
        else if (cmd == 26) data[0] = 39;
        else if (cmd == 25) data[0] = 85;
        else if (cmd == 27) { data[0] = 11500 & 255; data[1] = 11500 >> 8; }
        else if (cmd == 28) { data[0] = 500 & 255; data[1] = 500 >> 8; }
        else if (cmd == 32) data[0] = bus != 0; // stopped left internal drive disabled
        else if (cmd == 30) {
            data[0] = 1;
            int16_t speed = motor_tx_stats[bus].requested;
            data[2] = (uint8_t)speed; data[3] = (uint8_t)((uint16_t)speed >> 8);
        } else if (cmd == 23) {
            data[0] = 9000 & 255; data[1] = 9000 >> 8;
            data[2] = 12000 & 255; data[3] = 12000 >> 8;
        }
        uint8_t frame[10] = {0x55,0x55,id,(uint8_t)(count+3),cmd};
        memcpy(frame+5,data,count);
        uint8_t sum = 0;
        for (int i = 2; i < count+5; ++i) sum += frame[i];
        frame[count+5] = (uint8_t)~sum;
        if (servo_corrupt_reply) frame[count+5] ^= 1;
        for (int i = 0; i < count+6; ++i) servo_rx_stream[bus][i] = frame[i];
        servo_rx_index[bus] = 0; servo_rx_count[bus] = count+6;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, const uint8_t *bytes, uint16_t n) {
    if (h==&huart2 && n==sizeof(JointDiagnosticPacket)) memcpy(&diagnostic_packet_tx,bytes,n);
    if (h == &huart6 && n < sizeof(calibration_status_tx)) {
        memcpy(calibration_status_tx, bytes, n); calibration_status_tx[n] = 0;
    }
    if (h == &huart5 && n == 10 && bytes[4] == 29) ++wrist_motor_commands;
    if (n == 10 && bytes[4] == 29 && (bytes[7] || bytes[8]))
        ++nonzero_motor_writes;
    return motor_it_status;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *bytes, uint16_t n) {
    ++rx_start_calls;
    if (rx_error_during_start) {
        h->ErrorCode = HAL_UART_ERROR_NE; HAL_UART_ErrorCallback(h);
    }
    (void)bytes;
    (void)n;
    return rx_restart_fail ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) {
    (void)h;
    ++rx_abort_calls;
    return HAL_OK;
}

static void check(int ok, uint32_t code) {
    ++test_result.checks;
    if (!ok) {
        if (test_result.failures < 32) test_failure_codes[test_result.failures] = code;
        ++test_result.failures;
        if (!test_result.first_failure)
            test_result.first_failure = code;
    }
}
static void reset(void) {
    memset(&joint_diagnostic,0,sizeof(joint_diagnostic));
    memset(&diagnostic_packet_tx,0,sizeof(diagnostic_packet_tx));
    usb_blocking_writes=0;
    const JointOffsets initial = {JC_DEFAULT_OFFSETS};
    JC_Init(&joint_calibration, &initial);
    calibration_source = JC_DEFAULTS; calibration_last_request = 0;
    calibration_reply = calibration_preview = calibration_commit = calibration_stop_flushed = 0;
    calibration_last_phase = calibration_generation = 0;
    ssc_active = ssc_clock = ssc_output = ssc_data = ssc_reply_ready = 0;
    ssc_shift_reply = ssc_cancel_at_bit = ssc_stuck_data = 0;
    ssc_tx_bits = ssc_rx_bits = ssc_edges = ssc_bad_edges = spi3_hal_calls = 0;
    test_dwt_stopped = 0;
    test_dwt_step = 84;
    memset(motor_tx_stats, 0, sizeof(motor_tx_stats));
    memset(drive_status_reader, 0, sizeof(drive_status_reader));
    memset((void *)servo_rx_count, 0, sizeof(servo_rx_count));
    memset((void *)servo_rx_index, 0, sizeof(servo_rx_index));
    servo_reply = 1;
    servo_corrupt_reply = 0;
    direction_errors = 0;
    motor_it_status = HAL_OK;
    memset(&command_queue, 0, sizeof(command_queue));
    memset(feedback_samples, 0, sizeof(feedback_samples));
    memset(tle_state, 0, sizeof(tle_state));
    latest_spi_mask = motion_feedback_hold = 0;
    portenta_rx_position = 0;
    portenta_uart_error = portenta_rx_errors = portenta_queue_peak = 0;
    portenta_noise_errors = portenta_frame_errors = portenta_overruns = 0;
    portenta_parity_errors = portenta_dma_errors = 0;
    portenta_rx_running = 0;
    portenta_last_restart_ms = portenta_rx_restarts = portenta_restart_failures = 0;
    rx_start_calls = rx_abort_calls = rx_error_during_start = 0;
    calibration_status_tx[0] = 0;
    rx_restart_fail = 0;
    unsafe_autocal_changes = 0;
    for (int i = 0; i < 5; ++i) {
        memset(spi_bank[i], 0, sizeof(spi_bank[i]));
        spi_bank[i][6] = 0xC001;
        spi_bank[i][8] = 0x0804;
        spi_bank[i][10] = (uint16_t)(0x1200 + i);
        spi_bank[i][15] = Feedback_TleParameterCrc(&spi_bank[i][8]) ^ 1U;
    }
    memset(move_dirs, 0, sizeof(move_dirs));
    wrist_address = (WristAddress){WRIST_FIRST_ID, 0, 0};
    a1m = a1j = a2m = a2j = a3j = a4 = NAN;
    feedback_fault_latched = position_control_active = 0;
    system_state = STATE_MODES;
    mock_ms = 1000;
    mock_subtick = 0;
    last_wrist_read = 0;
    position_writes = nonzero_motor_writes = 0;
    spi_fail = corrupt_spi = uart_fail = 0;
    corrupt_reads = 0; corrupt_index = -1;
    stop_during_mode = 0;
    spi_flags = 0xF000;
    spi_status_injection = 0; wrist_motor_commands = 0;
    wrist_reply = 1;
    wrist_raw = 206.64f;
    // HOME follows the user's current mechanical calibration offsets.
    spi_raw[1] = -OFFSET_L;
    spi_raw[3] = -OFFSET_R;
    spi_raw[4] = -OFFSET_BASE;
    hspi1.Instance = SPI1;
    hspi2.Instance = SPI2;
    hspi3.Instance = SPI3;
    huart1.Instance = USART1;
    huart2.Instance = USART2;
    huart2.gState = huart6.gState = HAL_UART_STATE_READY;
    huart3.Instance = USART3;
    huart4.Instance = UART4;
    huart5.Instance = UART5;
    huart6.Instance = USART6;
    huart1.gState = huart3.gState = huart4.gState = huart5.gState = HAL_UART_STATE_READY;
    huart1.Instance->CR1 = huart3.Instance->CR1 = huart4.Instance->CR1 = USART_CR1_RE;
    GPIOA->OTYPER = GPIO_PIN_9 | GPIO_PIN_0;
    GPIOB->OTYPER = GPIO_PIN_10;
    GPIOC->OTYPER = GPIO_PIN_12;
    huart5.Instance->CR1 = USART_CR1_RE;
    huart5.Instance->SR = UART_FLAG_TXE | UART_FLAG_TC;
    htim2.Instance = TIM2;
    hdma_usart6_rx.Instance = DMA2_Stream1;
    uart_rx_active = uart_rx_count = uart_rx_index = 0;
    pwm_pos_5 = 1560;
    pwm_pos_6 = 500;
}
static void warm(void) {
    SampleFastFeedback();
    ServiceWristFeedback(mock_ms);
    for (int i = 0; i < 6; ++i) {
        mock_ms += 10;
        SampleFastFeedback();
        ServiceWristFeedback(mock_ms);
    }
}
static void command(const char *line) {
    char copy[COMMAND_LINE_SIZE];
    strcpy(copy, line);
    ProcessPortentaCommand(copy);
}

static void advance_feedback(uint32_t elapsed) {
    while (elapsed) {
        uint32_t step = elapsed > 10 ? 10 : elapsed;
        mock_ms += step;
        elapsed -= step;
        SampleFastFeedback();
        ServiceWristFeedback(mock_ms);
    }
}

static void startup_and_commands(void) {
    reset();
    float q[6];
    GetFeedbackAngles(mock_ms, q);
    for (int i = 0; i < 5; ++i)
        check(isnan(q[i]), 101);
    command("S:AUTO");
    command("P:0,0,0,0,0");
    check(feedback_fault_latched && !position_control_active && system_state == STATE_ESTOP, 102);
    check(position_writes == 0 && nonzero_motor_writes == 0, 103);
    command("S:AUTO");
    check(system_state == STATE_ESTOP, 104);
    command("S:MODES");
    check(!feedback_fault_latched && system_state == STATE_MODES, 105);
    warm();
    check(PositionFeedbackReady(mock_ms), 106);
    GetFeedbackAngles(mock_ms, q);
    check(fabsf(q[0]) < .01f && fabsf(q[1]) < .01f && fabsf(q[2]) < .02f, 107);
    command("S:AUTO");
    command("P:0,0,0,0,0");
    check(position_control_active && position_writes == 1 && !feedback_fault_latched, 108);
    check(position_id == wrist_address.id && position_id != 254, 113);
    check(fabsf(slew_L_cal) < .02f && fabsf(slew_R_cal) < .02f && fabsf(slew_3_cal) < .02f, 109);

    reset();
    warm();
    command("S:AUTO");
    command("P:nan,0,0,0,0");
    check(feedback_fault_latched && position_writes == 0 && !position_control_active, 110);
    reset();
    warm();
    a1j = 0;
    a2j = 0;
    a3j = 0;
    command("S:AUTO");
    command("P:0,0,0,0,0");
    check(feedback_fault_latched && position_writes == 0, 111);
    reset();
    warm();
    command("S:AUTO");
    uart_fail = 1;
    command("P:0,0,0,0,0");
    check(feedback_fault_latched && !position_control_active && position_writes == 0, 112);
    reset();
    warm();
    command("S:AUTO");
    uart_fail = 2;
    command("P:0,0,0,0,-90");
    check(feedback_fault_latched && !position_control_active && pwm_pos_5 == 1560, 114);
}

static void wrist_feedback(void) {
    reset();
    uint32_t before = mock_ms;
    float value = Read_LX225_Angle(&huart5, 1);
    check(fabsf(value - 206.64f) < .01f && mock_ms - before < WRIST_RESPONSE_TIMEOUT_MS, 201);
    warm();
    check(wrist_address.locked && wrist_address.id == 1, 202);
    wrist_reply = 0;
    for (int i = 0; i < 4; ++i) {
        advance_feedback(50);
    }
    check(wrist_address.id == 1 && wrist_address.locked, 203);
    check(!FeedbackReady(mock_ms) && isnan(RawFeedbackValue(SENSOR_WRIST, mock_ms)), 204);
    float q[6];
    GetFeedbackAngles(mock_ms, q);
    check(isnan(q[3]), 205);
    command("S:AUTO");
    command("P:0,0,0,0,0");
    check(feedback_fault_latched && !position_control_active && position_writes == 0, 206);
    command("S:MODES");
    wrist_reply = 1;
    wrist_raw = 173.84f;
    advance_feedback(50);
    check(!FeedbackReady(mock_ms), 221);
    advance_feedback(50);
    GetFeedbackAngles(mock_ms, q);
    check(FeedbackReady(mock_ms) && fabsf(q[3] + 33.26f) < .25f, 207);
    WristAddress discover = {1, 0, 0};
    for (int i = 0; i < 2; ++i)
        Wrist_RecordAddress(&discover, 0, 1);
    check(discover.id == 1, 208);
    Wrist_RecordAddress(&discover, 0, 1);
    check(discover.id == 2, 209);
    Wrist_RecordAddress(&discover, 1, 1);
    for (int i = 0; i < 100; ++i)
        Wrist_RecordAddress(&discover, 0, 1);
    check(discover.id == 2 && discover.locked, 210);

    reset();
    warm();
    command("S:AUTO");
    command("P:0,0,0,0,0");
    check(position_control_active, 217);
    wrist_reply = 0;
    mock_ms += FEEDBACK_WRIST_MAX_AGE_MS + 1;
    SampleFastFeedback();
    SuperviseMotionFeedback(mock_ms);
    check(feedback_fault_latched && system_state == STATE_ESTOP && !position_control_active, 218);
    command("S:AUTO");
    check(system_state == STATE_ESTOP, 219);
    command("S:MODES");
    check(system_state == STATE_MODES && !position_control_active, 220);

    uint8_t p[8] = {0x55, 0x55, 1, 5, 28, 100, 0, 0};
    p[7] = (uint8_t)~(uint8_t)(1 + 5 + 28 + 100);
    check(fabsf(Wrist_ParsePosition(p, 8, 1) - 24) < .01f, 211);
    check(isnan(Wrist_ParsePosition(p, 8, 2)), 212);
    p[3] = 7;
    check(isnan(Wrist_ParsePosition(p, 8, 1)), 213);
    p[3] = 5;
    p[7] ^= 1;
    check(isnan(Wrist_ParsePosition(p, 8, 1)), 214);
    p[7] ^= 1;
    check(isnan(Wrist_ParsePosition(p, 7, 1)), 215);
    reset();
    huart5.gState = HAL_UART_STATE_BUSY_TX;
    before = mock_ms;
    check(isnan(Read_LX225_Angle(&huart5, 1)) && mock_ms - before <= 3, 216);
}

static void spi_validation_and_wrap(void) {
    reset();
    warm();
    check(FeedbackReady(mock_ms), 301);
    corrupt_spi = 1;
    SampleFastFeedback();
    check(!FeedbackReady(mock_ms) && isnan(RawFeedbackValue(SENSOR_JOINT_L, mock_ms)) &&
          isfinite(a1j), 302);
    corrupt_spi = 0;
    SampleFastFeedback();
    check(FeedbackReady(mock_ms), 303);
    SampleFastFeedback();
    check(FeedbackReady(mock_ms), 304);
    spi_fail = 1;
    check(isnan(Read_TLE5012_Angle(&hspi1, GPIOB, CS_JointAxle_DOF1_Pin, NULL, NULL)), 305);
    spi_fail = 0;
    spi_flags = 0xE000;
    check(isnan(Read_TLE5012_Angle(&hspi1, GPIOB, CS_JointAxle_DOF1_Pin, NULL, NULL)), 306);
    spi_flags = 0x7000;
    check(isnan(Read_TLE5012_Angle(&hspi1, GPIOB, CS_JointAxle_DOF1_Pin, NULL, NULL)), 307);
    check(Feedback_Crc8((const uint8_t *)"123456789", 9) == 0x4B, 308);
    // Fixed arithmetic examples, independent of operator calibration changes.
    check(fabsf(Feedback_CalibrateSpi(359.99f, -38) + 38.01f) < .01f, 309);
    check(fabsf(Feedback_CalibrateSpi(4.6f, -274.6f) - 90) < .01f, 310);
    check(fabsf(Feedback_CalibrateSpi(21.4f, -211.4f) - 170) < .01f, 311);
    check(fabsf(Feedback_CalibrateSpiSector(218.2f, -38, -110, 180) - 180.2f) < .01f, 313);
    FeedbackSample sample = {1, 0xFFFFFFF0U, 2};
    check(Feedback_Fresh(&sample, 20, 50) && !Feedback_Fresh(&sample, 100, 50), 312);
}

static void command_fifo(void) {
    reset(); StartPortentaReceiver();
    const char *a = "S:AU";
    memcpy(rx_buffer, a, 4);
    HAL_UARTEx_RxEventCallback(&huart6, 4);
    const char *b = "TO\nP:0,0,0,0,0\nG:0\n";
    memcpy(rx_buffer + 4, b, strlen(b));
    HAL_UARTEx_RxEventCallback(&huart6, 4 + strlen(b));
    char line[COMMAND_LINE_SIZE];
    check(CommandQueue_Pop(&command_queue, line) && !strcmp(line, "S:AUTO"), 401);
    check(CommandQueue_Pop(&command_queue, line) && !strcmp(line, "P:0,0,0,0,0"), 402);
    check(CommandQueue_Pop(&command_queue, line) && !strcmp(line, "G:0"), 403);
    check(!CommandQueue_Pop(&command_queue, line), 404);
    for (int i = 0; i < COMMAND_QUEUE_SIZE; ++i)
        CommandQueue_Push(&command_queue, (const uint8_t *)"P:0,0,0,0,0\n", 12);
    check(command_queue.overflow && !CommandQueue_Pop(&command_queue, line), 405);
    CommandQueue_Push(&command_queue, (const uint8_t *)"S:ESTOP\n", 8);
    check(command_queue.emergency, 406);
    reset();
    warm();
    CommandQueue_Push(&command_queue, (const uint8_t *)"S:AUTO\nP:0,0,0,0,0\nS:ESTOP\n", 27);
    check(command_queue.emergency && !CommandQueue_Pop(&command_queue, line), 407);
    command("S:AUTO");
    command("P:0,0,0,0,0");
    check(position_writes == 0 && !position_control_active, 408);
    reset();
    char long_line[COMMAND_LINE_SIZE + 2];
    memset(long_line, 'P', sizeof(long_line));
    long_line[sizeof(long_line) - 1] = '\n';
    CommandQueue_Push(&command_queue, (uint8_t *)long_line, sizeof(long_line));
    check(command_queue.overflow && !CommandQueue_Pop(&command_queue, line), 409);

    reset(); warm(); command("S:AUTO"); command("P:0,0,0,0,0");
    command_queue.emergency = 1;
    SuperviseMotionFeedback(mock_ms);
    check(system_state == STATE_ESTOP && !position_control_active, 410);
    Hiwonder_SetMotorSpeed_IT(&huart1, 254, 500);
    check(nonzero_motor_writes == 0, 411);

    reset(); warm(); command("S:AUTO"); stop_during_mode = 1;
    command("P:0,0,0,0,-90");
    check(command_queue.emergency && position_writes == 0 && pwm_pos_5 == 1560, 412);
    SuperviseMotionFeedback(mock_ms);
    check(system_state == STATE_ESTOP && !position_control_active && !feedback_fault_latched, 413);
}

static void hot_and_late_sensor_power(void) {
    reset();
    check(Feedback_TleCrcValid(0x8021, 0xF013, 0xBBDB) &&
          isnan(Feedback_TleAngle(0x8021, 0xF013, 0xBBDB)), 601);
    uint16_t before[5][8];
    for (int i = 0; i < 5; ++i) memcpy(before[i], &spi_bank[i][8], sizeof(before[i]));
    check(isnan(Read_TLE5012_Angle(&hspi1, GPIOB, CS_JointAxle_DOF1_Pin, NULL, NULL)) &&
          (tle_state[SENSOR_JOINT_L].status & 8), 602);
    warm();
    check(FeedbackReady(mock_ms), 603);
    for (int i = 0; i < 5; ++i) {
        check(tle_state[i].configured && spi_bank[i][15] == Feedback_TleParameterCrc(&spi_bank[i][8]), 604);
        check(!memcmp(before[i], &spi_bank[i][8], 7*sizeof(uint16_t)) &&
              (before[i][7] & 0xFF00) == (spi_bank[i][15] & 0xFF00), 605);
    }
    reset(); spi_fail = 1; SampleFastFeedback();
    check(!FeedbackReady(mock_ms) && !tle_state[0].configured, 606);
    spi_fail = 0;
    for (int i = 0; i < 5; ++i) {
        spi_bank[i][6] = 1; spi_bank[i][8] = 0x0801;
        spi_bank[i][15] = Feedback_TleParameterCrc(&spi_bank[i][8]);
    }
    mock_ms += SENSOR_CONFIG_RETRY_MS;
    warm();
    check(FeedbackReady(mock_ms), 607);
    for (int i = 0; i < 5; ++i)
        check(spi_bank[i][6] == 0xC001 && spi_bank[i][8] == 0x0804 &&
              spi_bank[i][15] == Feedback_TleParameterCrc(&spi_bank[i][8]), 608);
    check(unsafe_autocal_changes == 0, 609);
}

static void transient_spi_during_manual(void) {
    reset(); warm(); command("S:MANUAL"); command("P:0,0,0,0,0");
    corrupt_index = SENSOR_JOINT_R; corrupt_reads = 1;
    SampleFastFeedback(); SuperviseMotionFeedback(mock_ms);
    check(FeedbackReady(mock_ms) && position_control_active && !feedback_fault_latched, 701);
    check(tle_state[SENSOR_JOINT_R].crc_errors == 1 && tle_state[SENSOR_JOINT_R].retries == 1, 702);
    uint32_t wrist_before = wrist_motor_commands;
    corrupt_reads = SENSOR_SPI_ATTEMPTS;
    SampleFastFeedback(); SuperviseMotionFeedback(mock_ms);
    check(motion_feedback_hold && position_control_active && !feedback_fault_latched, 703);
    check(isnan(RawFeedbackValue(SENSOR_JOINT_R, mock_ms)) && FeedbackWithinAge(mock_ms), 704);
    check(wrist_motor_commands == wrist_before, 705);
    Hiwonder_SetMotorSpeed_IT(&huart1, 254, 500);
    check(nonzero_motor_writes == 0, 706);
    mock_ms += 10; SampleFastFeedback(); SuperviseMotionFeedback(mock_ms);
    check(!motion_feedback_hold && !feedback_fault_latched && position_control_active, 707);

    corrupt_spi = 1;
    for (int i = 0; i < 7; ++i) { mock_ms += 10; SampleFastFeedback(); SuperviseMotionFeedback(mock_ms); }
    check(feedback_fault_latched && system_state == STATE_ESTOP && !position_control_active, 708);
    corrupt_spi = 0; mock_ms += SENSOR_CONFIG_RETRY_MS; warm();
    check(feedback_fault_latched && !position_control_active, 709);

    reset(); warm(); command("S:MANUAL"); command("M:1,0,0,0,0,0");
    corrupt_index = SENSOR_JOINT_R; corrupt_reads = SENSOR_SPI_ATTEMPTS;
    SampleFastFeedback(); SuperviseMotionFeedback(mock_ms);
    command("M:-1,0,0,0,0,0");
    check(motion_feedback_hold && move_dirs[0] == -1 && !feedback_fault_latched, 710);
    mock_ms += 10; SampleFastFeedback(); SuperviseMotionFeedback(mock_ms);
    check(!motion_feedback_hold && !feedback_fault_latched && move_dirs[0] == -1, 711);

    reset(); warm(); command("S:MANUAL"); command("P:0,0,0,0,0");
    spi_flags = 0xB000; spi_status_injection = 4;
    SampleFastFeedback(); SuperviseMotionFeedback(mock_ms);
    check(feedback_fault_latched && !position_control_active && (tle_state[0].status & 4), 712);
}

static void circular_dma_events(void) {
    reset();
    check(StartPortentaReceiver(), 801);
    memcpy(rx_buffer, "M:1,0,0,0,0,0\n", 14);
    HAL_UARTEx_RxEventCallback(&huart6, 14);
    HAL_UARTEx_RxEventCallback(&huart6, 14); // duplicate IDLE position
    char line[COMMAND_LINE_SIZE];
    check(CommandQueue_Pop(&command_queue, line) && !strcmp(line, "M:1,0,0,0,0,0"), 802);
    check(!CommandQueue_Pop(&command_queue, line), 803);
    // Long region of complete harmless lines spans HT then TC then wrap.
    memset(&command_queue, 0, sizeof(command_queue)); portenta_rx_position = 0;
    for (int i = 0; i < 256; ++i) rx_buffer[i] = (i % 2) ? '\n' : 'Z';
    HAL_UARTEx_RxEventCallback(&huart6, 40);
    for (int i = 0; i < 20; ++i) check(CommandQueue_Pop(&command_queue, line), 804);
    HAL_UARTEx_RxEventCallback(&huart6, 80);
    for (int i = 0; i < 20; ++i) check(CommandQueue_Pop(&command_queue, line), 805);
    portenta_rx_position = 250; // previous region already consumed
    memcpy(rx_buffer + 250, "S:AUTO", 6);
    HAL_UARTEx_RxEventCallback(&huart6, 256);
    memcpy(rx_buffer, "\nG:0\n", 5);
    HAL_UARTEx_RxEventCallback(&huart6, 5);
    check(CommandQueue_Pop(&command_queue, line) && !strcmp(line, "S:AUTO"), 806);
    check(CommandQueue_Pop(&command_queue, line) && !strcmp(line, "G:0"), 807);
    check(!command_queue.overflow, 808);
    huart6.ErrorCode = HAL_UART_ERROR_ORE;
    HAL_UART_ErrorCallback(&huart6);
    check((command_queue.overflow & RX_FAULT_UART) && portenta_uart_error == HAL_UART_ERROR_ORE &&
          portenta_rx_errors == 1, 809);

    reset(); StartPortentaReceiver();
    for (int i = 0; i < 20; ++i) memcpy(rx_buffer + 6*i, "V:500\n", 6);
    HAL_UARTEx_RxEventCallback(&huart6, 120);
    check(!command_queue.overflow && portenta_queue_peak == 20, 810);
    ProcessQueuedCommands();
    check((command_queue.head + COMMAND_QUEUE_SIZE - command_queue.tail) % COMMAND_QUEUE_SIZE == 12, 811);
    ProcessQueuedCommands(); ProcessQueuedCommands();
    check(command_queue.head == command_queue.tail && global_velocity == 500, 812);
    rx_restart_fail = 1;
    check(!StartPortentaReceiver() && (command_queue.overflow & RX_FAULT_RESTART), 813);
}

static void uart_fault_recovery(void) {
    reset(); warm(); command("S:MANUAL"); command("T:0");
    command("J:81,1,BEGIN"); StartPortentaReceiver();
    // An unsaved temporary Apply must be rolled back if reception is damaged.
    joint_calibration.active.offset[0] += 2;
    joint_calibration.dirty = 1; joint_calibration.phase = JC_APPLIED;
    huart6.Instance->CR1 |= USART_CR1_IDLEIE | USART_CR1_RXNEIE | USART_CR1_PEIE;
    huart6.Instance->CR3 |= USART_CR3_EIE;
    huart6.ErrorCode = HAL_UART_ERROR_NE;
    HAL_UART_ErrorCallback(&huart6);
    check(!portenta_rx_running && rx_abort_calls == 0 && rx_start_calls == 1, 1400);
    check(portenta_noise_errors == 1 && portenta_overruns == 0 &&
          (command_queue.overflow & RX_FAULT_UART), 1401);
    check(!(huart6.Instance->CR1 & (USART_CR1_IDLEIE | USART_CR1_RXNEIE | USART_CR1_PEIE)) &&
          !(huart6.Instance->CR3 & USART_CR3_EIE), 1402);
    memcpy(rx_buffer, "P:20,20,20,20,20\n", 17);
    HAL_UARTEx_RxEventCallback(&huart6, 17);
    check(command_queue.head == command_queue.tail, 1403); // No late DMA commands.
    LatchReceiveFault(command_queue.overflow);
    check(system_state == STATE_ESTOP && feedback_fault_latched && !joint_calibration.open &&
          joint_calibration.phase == JC_FAULT && joint_calibration.reason == JC_LINK_ERROR, 1404);
    check(!joint_calibration.dirty && OFFSET_L == joint_calibration.baseline.offset[0] &&
          motor_tx_stats[0].requested == 0 && motor_tx_stats[1].requested == 0, 1405);
    huart6.gState = HAL_UART_STATE_READY; CalibrationTx();
    check(!strcmp(calibration_status_tx, "J:81,0,13,0,13\n"), 1406);
    // Main clears the damaged FIFO, then recovers RX; neither action resets ESTOP.
    memset(&command_queue, 0, sizeof(command_queue));
    ServicePortentaReceiver(99);
    check(rx_abort_calls == 0 && !portenta_rx_running, 1407);
    ServicePortentaReceiver(100);
    check(rx_abort_calls == 1 && rx_start_calls == 2 && portenta_rx_restarts == 1 &&
          portenta_rx_running && command_queue.discard, 1408);
    check(feedback_fault_latched && system_state == STATE_ESTOP && !joint_calibration.open, 1409);
    memcpy(rx_buffer, "20,0,0\nS:AUTO\n", 14);
    HAL_UARTEx_RxEventCallback(&huart6, 14);
    char line[COMMAND_LINE_SIZE];
    check(CommandQueue_Pop(&command_queue, line) && !strcmp(line, "S:AUTO") &&
          !CommandQueue_Pop(&command_queue, line), 1410); // Ignore damaged suffix.
    command("S:AUTO"); check(system_state == STATE_ESTOP, 1411);
    calibration_reply = 0; calibration_ping_reply_ms = mock_ms - 250;
    command("J:81,0,PING");
    check(calibration_reply && joint_calibration.reason == JC_LINK_ERROR &&
          !joint_calibration.open && system_state == STATE_ESTOP, 1417);
    command("J:81,2,STATUS");
    check(joint_calibration.phase == JC_FAULT && joint_calibration.reason == JC_LINK_ERROR, 1418);
    command_queue.discard = 1;
    CommandQueue_Push(&command_queue, (const uint8_t *)"\nS:MODES\n", 9);
    ProcessQueuedCommands();
    check(system_state == STATE_MODES && !feedback_fault_latched && !joint_calibration.open, 1419);
    reset(); rx_restart_fail = 1; ServicePortentaReceiver(100);
    check(!portenta_rx_running && portenta_restart_failures == 1 &&
          (command_queue.overflow & RX_FAULT_RESTART), 1412);
    ServicePortentaReceiver(150); check(rx_abort_calls == 1, 1413); // Bounded retry.
    rx_restart_fail = 0; ServicePortentaReceiver(200);
    check(portenta_rx_running && portenta_rx_restarts == 1, 1414);
    reset(); rx_error_during_start = 1;
    check(!StartPortentaReceiver() && portenta_noise_errors == 1 &&
          (command_queue.overflow & RX_FAULT_UART) && rx_abort_calls == 0, 1415);
    // A genuine sensor fault also terminates the session with an error.
    reset(); warm(); command("S:MANUAL"); command("T:0"); command("J:82,1,BEGIN");
    LatchFeedbackFault("test feedback fault");
    check(!joint_calibration.open && joint_calibration.phase == JC_FAULT &&
          joint_calibration.reason == JC_BAD_FEEDBACK, 1416);
}

__attribute__((used, noinline)) void test_done(void) {
    for (;;)
        __asm volatile("nop");
}
static void motor_transport_diagnostics(void) {
    reset();
    Hiwonder_SetMotorSpeed_IT(&huart1, 254, -800);
    check(motor_tx_stats[0].requested == -800 && motor_tx_stats[0].submitted == 1 &&
          motor_tx_stats[1].submitted == 0, 901);
    int16_t packed = (int16_t)((uint16_t)tx_buf_1[7] | ((uint16_t)tx_buf_1[8] << 8));
    check(packed == -800, 902);
    HAL_UART_TxCpltCallback(&huart1);
    check(motor_tx_stats[0].completed == 1, 903);
    huart1.gState = HAL_UART_STATE_BUSY_TX;
    Hiwonder_SetMotorSpeed_IT(&huart1, 254, 500);
    check(motor_tx_stats[0].requested == 500 && motor_tx_stats[0].busy == 1 &&
          motor_tx_stats[0].submitted == 1 && motor_tx_stats[0].last_status == HAL_BUSY, 904);
    huart1.gState = HAL_UART_STATE_READY;
    motor_it_status = HAL_ERROR;
    Hiwonder_SetMotorSpeed_IT(&huart1, 254, 500);
    check(motor_tx_stats[0].failed == 1 && motor_tx_stats[0].submitted == 1 &&
          motor_tx_stats[0].last_status == HAL_ERROR, 905);
    motor_it_status = HAL_OK;
    Hiwonder_SetMotorSpeed_IT(&huart3, 254, 500);
    check(motor_tx_stats[1].submitted == 1 && motor_tx_stats[0].submitted == 1, 906);
    huart1.ErrorCode = HAL_UART_ERROR_FE;
    HAL_UART_ErrorCallback(&huart1);
    check(motor_tx_stats[0].uart_errors == 1 &&
          motor_tx_stats[0].last_uart_error == HAL_UART_ERROR_FE && !command_queue.overflow, 907);
}
static void servo_status_readback(void) {
    reset();
    uint8_t data[4], source;
    check(ReadServoRegister(&huart1,254,14,1,data,&source) && source == 1 && data[0] == 1,1001);
    check(ReadServoRegister(&huart1,1,27,2,data,&source) &&
          ((uint16_t)data[0] | ((uint16_t)data[1] << 8)) == 11500,1002);
    check(ReadServoRegister(&huart1,1,26,1,data,&source) && data[0] == 39,1003);
    check(ReadServoRegister(&huart1,1,32,1,data,&source) && data[0] == 0,1004);
    motor_tx_stats[0].requested = -500;
    check(ReadServoRegister(&huart1,1,30,4,data,&source) && data[0] == 1 &&
          (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8)) == -500,1005);
    check(!ReadServoRegister(&huart1,2,26,1,data,&source),1006); // wrong ID reply
    servo_reply = 0;
    uint32_t start = mock_ms;
    check(!ReadServoRegister(&huart1,1,26,1,data,&source) && mock_ms-start <= 12,1007);
    servo_reply = 1;
    for (int i = 0; i < 24; ++i) { mock_ms += 100; ServiceDriveStatus(mock_ms); }
    for (int i = 0; i < 3; ++i)
        check(drive_status_reader[i].locked && drive_status_reader[i].id == i+1 &&
              drive_status_reader[i].successes == 8,1008);
    uint32_t successes = drive_status_reader[0].successes;
    system_state = STATE_MANUAL; mock_ms += 100;
    ServiceDriveStatus(mock_ms);
    check(drive_status_reader[0].successes == successes,1009);
    uint8_t reply[7] = {0x55,0x55,1,4,26,39,0};
    reply[6] = (uint8_t)~(uint8_t)(1+4+26+39);
    check(Servo_ParseReply(reply,7,1,26,1,data,&source),1010);
    reply[6] ^= 1;
    check(!Servo_ParseReply(reply,7,1,26,1,data,&source),1011);
    check(!Servo_ParseReply(reply,6,1,26,1,data,&source),1012);
    servo_corrupt_reply = 1;
    check(!ReadServoRegister(&huart1,1,26,1,data,&source) &&
          servo_read_trace[0].transmitted && servo_read_trace[0].count == 7 &&
          servo_read_trace[0].bytes[0] == 0x55,1013);
    servo_corrupt_reply = 0;
    servo_reply = 0;
    check(!ReadServoRegister(&huart1,1,26,1,data,&source) &&
          servo_read_trace[0].transmitted && servo_read_trace[0].count == 0,1014);
    check(direction_errors == 0,1015);
}
static void halfduplex_turnaround(void) {
    reset();
    UART_HandleTypeDef *buses[4] = {&huart1,&huart3,&huart4,&huart5};
    GPIO_TypeDef *ports[4] = {GPIOA,GPIOB,GPIOA,GPIOC};
    const uint16_t pins[4] = {GPIO_PIN_9,GPIO_PIN_10,GPIO_PIN_0,GPIO_PIN_12};
    for (int i = 0; i < 4; ++i) {
        Hiwonder_SetMotorSpeed_IT(buses[i],254,-500);
        check((buses[i]->Instance->CR1 & 12U) == USART_CR1_TE &&
              !(ports[i]->OTYPER & pins[i]),1101);
        HAL_UART_TxCpltCallback(buses[i]);
        check((buses[i]->Instance->CR1 & 12U) == USART_CR1_RE &&
              (ports[i]->OTYPER & pins[i]) && ports[i]->BSRR == pins[i],1102);
    }
    Hiwonder_SetMotorSpeed(&huart1,254,0);
    check((huart1.Instance->CR1 & 12U) == USART_CR1_RE &&
          (GPIOA->OTYPER & GPIO_PIN_9),1103);
    Hiwonder_UnlockAngleLimits(&huart5,254);
    check((huart5.Instance->CR1 & 12U) == USART_CR1_RE &&
          (GPIOC->OTYPER & GPIO_PIN_12) && direction_errors == 0,1104);
    motor_it_status = HAL_ERROR;
    Hiwonder_SetMotorSpeed_IT(&huart1,254,500);
    check((huart1.Instance->CR1 & 12U) == USART_CR1_RE &&
          (GPIOA->OTYPER & GPIO_PIN_9),1105);
}
static void elbow_clocked_transport(void) {
    reset(); warm();
    check(FeedbackReady(mock_ms) && spi3_hal_calls == 0 && ssc_bad_edges == 0, 1200);
    uint16_t data = 0, safety = 0, bank[8];
    check(ReadTleWord(&hspi3, GPIOB, CS_JointAxle_DOF3_Pin, 0x8021, &data, &safety) &&
          Feedback_TleCrcValid(0x8021, data, safety), 1201);
    check(ssc_tx_bits == 16 && ssc_rx_bits == 32 && ssc_edges == 96 &&
          ssc_tx[0] == 0x8021 && !ssc_active && !ssc_clock && !ssc_output, 1202);
    check(tle_state[4].last_transfer_us > 0 && tle_state[4].last_transfer_us < 2000 &&
          tle_state[4].max_transfer_us >= tle_state[4].last_transfer_us, 1219);
    check(__get_PRIMASK() == 0, 1220);
    check(ReadTleRegisters(&hspi3, GPIOB, CS_JointAxle_DOF3_Pin, 0x8088, bank, 8, &safety) &&
          !memcmp(bank, &spi_bank[4][8], sizeof(bank)), 1203);
    check(ssc_tx_bits == 16 && ssc_rx_bits == 144 && ssc_edges == 320, 1204);
    check(WriteTleWord(&hspi3, CS_JointAxle_DOF3_Pin, 6, 0xC001) &&
          ssc_tx_bits == 32 && ssc_rx_bits == 16 && ssc_edges == 96 &&
          ssc_tx[0] == 0x5061 && ssc_tx[1] == 0xC001, 1205);

    uint32_t good = tle_state[4].good_angles;
    ssc_shift_reply = 1;
    check(isnan(Read_TLE5012_Angle(&hspi3, GPIOB, CS_JointAxle_DOF3_Pin, &data, &safety)), 1206);
    check(tle_state[4].crc_errors == SENSOR_SPI_ATTEMPTS &&
          tle_state[4].shifted_frames == SENSOR_SPI_ATTEMPTS &&
          tle_state[4].good_angles == good && !ssc_active && !ssc_clock && !ssc_output, 1207);
    ssc_shift_reply = 0;
    check(isfinite(Read_TLE5012_Angle(&hspi3, GPIOB, CS_JointAxle_DOF3_Pin, &data, &safety)), 1208);
    for (int level = -1; level <= 1; level += 2) {
        ssc_stuck_data = level;
        check(isnan(Read_TLE5012_Angle(&hspi3, GPIOB, CS_JointAxle_DOF3_Pin, &data, &safety)) &&
              !ssc_active && !ssc_clock && !ssc_output, 1209);
    }
    ssc_stuck_data = 0;

    ssc_cancel_at_bit = 5;
    check(!ReadTleWord(&hspi3, GPIOB, CS_JointAxle_DOF3_Pin, 0x8021, &data, &safety) &&
          tle_transfer_result == TLE_TRANSFER_IO && ssc_rx_bits == 16 &&
          !ssc_active && !ssc_clock && !ssc_output, 1210);
    command_queue.emergency = 0;
    ssc_cancel_at_bit = 0;
    check(ReadTleWord(&hspi3, GPIOB, CS_JointAxle_DOF3_Pin, 0x8021, &data, &safety), 1211);
    test_dwt_stopped = 1;
    check(!ReadTleWord(&hspi3, GPIOB, CS_JointAxle_DOF3_Pin, 0x8021, &data, &safety) &&
          tle_transfer_result == TLE_TRANSFER_IO && !ssc_active && !ssc_output && !ssc_clock, 1212);
    test_dwt_stopped = 0;
    test_dwt_step = 84000; // Delayed execution must hit the transaction deadline.
    check(!ReadTleWord(&hspi3, GPIOB, CS_JointAxle_DOF3_Pin, 0x8021, &data, &safety) &&
          !ssc_active && !ssc_output && !ssc_clock, 1213);
    test_dwt_step = 84;
    DWT->CYCCNT = 0xFFFFFF00U;
    check(ReadTleWord(&hspi3, GPIOB, CS_JointAxle_DOF3_Pin, 0x8021, &data, &safety) &&
          !ssc_bad_edges && spi3_hal_calls == 0, 1214);

    // The user's observed failure must still stop MANUAL, latch, and require a reset.
    reset(); warm(); command("S:MANUAL"); command("M:1,0,0,0,0,0");
    ssc_shift_reply = 1;
    SampleFastFeedback(); SuperviseMotionFeedback(mock_ms);
    check(motion_feedback_hold && !feedback_fault_latched &&
          !(latest_spi_mask & (1U << SENSOR_ELBOW)), 1215);
    mock_ms += FEEDBACK_SPI_MAX_AGE_MS + 1;
    SampleFastFeedback(); SuperviseMotionFeedback(mock_ms);
    check(feedback_fault_latched && system_state == STATE_ESTOP &&
          !position_control_active && nonzero_motor_writes == 0, 1216);
    ssc_shift_reply = 0; mock_ms += SENSOR_CONFIG_RETRY_MS; warm();
    check(feedback_fault_latched && !FeedbackReady(mock_ms), 1217);
    command("S:MODES"); warm();
    check(FeedbackReady(mock_ms) && !feedback_fault_latched && ssc_bad_edges == 0, 1218);
}

static void calibration_test_step(void) {
    mock_ms += 10;
    for (int i = 0; i < SENSOR_COUNT; ++i) feedback_samples[i].timestamp = mock_ms;
    joint_calibration.lease_ms = mock_ms; CalibrationControlTick(mock_ms);
}

static void joint_diagnostic_runtime(void) {
    reset(); warm(); command("S:MANUAL"); command("T:0");
    check(sizeof(JointDiagnosticPacket)==152 && JD_Crc32("123456789",9)==0xCBF43926U,1500);
    __io_putchar('X'); check(usb_blocking_writes==1,1501);
    const JointOffsets original=joint_calibration.active;
    command("D:START,91");
    check(JD_OwnsUart() && joint_diagnostic.active && !position_control_active &&
          nonzero_motor_writes==0 && !memcmp(&original,&joint_calibration.active,sizeof(original)),1502);
    __io_putchar('X'); check(usb_blocking_writes==1,1503);
    JD_Service(mock_ms);
    check(diagnostic_packet_tx.type==JD_CONFIG && diagnostic_packet_tx.run==91 &&
          diagnostic_packet_tx.data.config.values[0]==Kp_pos_L &&
          diagnostic_packet_tx.crc32==JD_Crc32(&diagnostic_packet_tx,148),1504);
    HAL_UART_TxCpltCallback(&huart2);
    check(!joint_diagnostic.busy && joint_diagnostic.head==joint_diagnostic.tail,1505);
    command("D:START,91"); // Retry may not duplicate/reset the configuration/run.
    check(joint_diagnostic.sequence==1,1506);
    command("D:MARK,92,9,4"); check(joint_diagnostic.cycle==0,1507);
    command("D:MARK,91,1,4"); command("P:10,20,5,0,0");
    slew_L_cal=-5; slew_R_cal=15; slew_3_cal=2; joint_diagnostic.control_dt_ms=12;
    mock_ms+=20; JD_Sample(mock_ms,0); JD_Service(mock_ms);
    test_diag_sample=diagnostic_packet_tx;
    JointDiagnosticSample *s=&diagnostic_packet_tx.data.sample;
    check(diagnostic_packet_tx.type==JD_SAMPLE && diagnostic_packet_tx.data.sample.command==1 &&
          s->target[0]==20 && s->target[1]==10 && s->target[2]==5 && s->cycle==1,1508);
    check(s->slew[0]==10 && s->slew[1]==5 && s->slew[2]==2 &&
          s->stage==JD_APPROACH && s->control_dt_ms==12,1509);
    check(fabsf(s->actual[0])<.02f && fabsf(s->actual[1])<.02f &&
          s->raw[1]==a1j && s->offsets[0]==OFFSET_L && (s->valid_mask&31)==31,1510);
    JointDiagnosticPacket held=joint_diagnostic.queue[joint_diagnostic.tail];
    // Fill the ring while UART is busy. Old/in-flight storage stays immutable.
    for (int i=0;i<4;++i) { mock_ms+=20; JD_Sample(mock_ms,0); }
    check(joint_diagnostic.dropped==2 &&
          !memcmp(&held,&joint_diagnostic.queue[joint_diagnostic.tail],sizeof(held)),1511);
    command("D:STOP,91"); JD_Service(mock_ms);
    check(!joint_diagnostic.active && joint_diagnostic.end_pending && JD_OwnsUart() &&
          position_control_active,1512);
    HAL_UART_TxCpltCallback(&huart2); JD_Service(mock_ms);
    check(!joint_diagnostic.end_pending,1513);
    for (int i=0;i<4;++i) { HAL_UART_TxCpltCallback(&huart2); JD_Service(mock_ms); }
    check(!JD_OwnsUart() && (diagnostic_packet_tx.flags&JD_END) && position_control_active,1514);
    __io_putchar('X'); check(usb_blocking_writes==2,1515);
    command("D:START,92"); mock_ms+=JD_LEASE_MS+1; JD_Service(mock_ms);
    check(!joint_diagnostic.active && position_control_active && joint_diagnostic.run==92,1516);
    // Logger is observational: expiry, transport pressure and Stop never move a motor.
    check(nonzero_motor_writes==0 && !memcmp(&original,&joint_calibration.active,sizeof(original)),1517);
}
static void calibration_runtime(void) {
    reset(); warm(); command("S:MANUAL"); command("T:0");
    command("J:71,1,BEGIN");
    check(joint_calibration.open && joint_calibration.phase == JC_READY, 1300);
    command("P:10,0,0,0,0"); command("M:1,1,1,1,1,1");
    check(!position_control_active && !move_dirs[0] && !move_dirs[2], 1301);
    command("J:71,2,SELECT,0,1"); command("J:71,3,MOVE");
    mock_ms += 10; SampleFastFeedback(); ServiceWristFeedback(mock_ms);
    CalibrationControlTick(mock_ms);
    check(joint_calibration.phase == JC_LEAD_IN && motor_tx_stats[0].requested > 0 &&
          motor_tx_stats[1].requested < 0 && motor_tx_stats[2].requested == 0, 1302);
    unsigned writes_before = nonzero_motor_writes;
    command("J:71,3,MOVE"); // retry must not restart the relative travel window
    check(fabsf(joint_calibration.start) < .02f && nonzero_motor_writes == writes_before, 1303);
    command("J:71,4,STOP");
    check(joint_calibration.phase == JC_STOPPED && motor_tx_stats[0].requested == 0 &&
          motor_tx_stats[1].requested == 0, 1304);
    // Actual joint-sensor captures, with nonzero pitch/roll/elbow in the old frame.
    spi_raw[1] = normalize_angle(-OFFSET_L - 5); // roll=-3, pitch=+2
    spi_raw[3] = normalize_angle(-OFFSET_R - 1);
    spi_raw[4] = normalize_angle(-OFFSET_BASE + 1);
    warm();
    for (unsigned axis = 0; axis < 3; ++axis) {
        char cmd[64]; snprintf(cmd, sizeof(cmd), "J:71,%u,SELECT,%u,1", 5 + axis * 2, axis); command(cmd);
        for (int i = 0; i < 100; ++i) calibration_test_step();
        snprintf(cmd, sizeof(cmd), "J:71,%u,CAPTURE", 6 + axis * 2); command(cmd);
        for (int i = 0; i < 100; ++i) calibration_test_step();
    }
    check(joint_calibration.phase == JC_REVIEW && joint_calibration.mask == 7, 1305);
    const JointOffsets defaults = {JC_DEFAULT_OFFSETS};
    check(fabsf(joint_calibration.candidate.offset[0] - (defaults.offset[0] + 5)) < .02f &&
          fabsf(joint_calibration.candidate.offset[1] - (defaults.offset[1] + 1)) < .02f, 1306);
    nonzero_motor_writes = 0;
    command("J:71,11,APPLY");
    check(joint_calibration.phase == JC_APPLIED && joint_calibration.dirty && !position_control_active &&
          nonzero_motor_writes == 0, 1307);
    for (int i = 0; i < 100; ++i) calibration_test_step();
    command("J:71,12,VERIFY"); command("J:71,13,SAVE"); command("J:71,14,COMMIT");
    check(joint_calibration.phase == JC_SAVING && calibration_commit, 1308);
    mock_flash_enabled = 1; mock_flash_writes = 0;
    huart6.gState = HAL_UART_STATE_READY;
    CalibrationService(); // Submit zero packets first; do not write Flash yet.
    check(mock_flash_writes == 0 && calibration_stop_flushed, 1309);
    for (int i = 0; i < 4; ++i) motor_tx_stats[i].completed = calibration_zero_done[i];
    CalibrationService();
    check(joint_calibration.phase == JC_SAVED && !joint_calibration.dirty &&
          calibration_generation == 1 && mock_flash_writes == CAL_STORE_WORDS, 1310);
    command("J:71,14,COMMIT"); CalibrationService();
    check(mock_flash_writes == CAL_STORE_WORDS && joint_calibration.phase == JC_SAVED, 1311);
    command("J:71,15,CANCEL");
    check(!joint_calibration.open && fabsf(OFFSET_L - (defaults.offset[0] + 5)) < .02f, 1312);
    CalibrationBootLoad();
    check(calibration_source == JC_STORED && calibration_generation == 1 &&
          fabsf(OFFSET_L - (defaults.offset[0] + 5)) < .02f, 1313);
    mock_flash_enabled = 0;
    reset(); warm(); command("S:MANUAL"); command("T:0"); command("J:72,1,BEGIN");
    command("J:72,2,MOVE"); mock_ms += JC_LEASE_MS + 1;
    SampleFastFeedback(); ServiceWristFeedback(mock_ms); CalibrationControlTick(mock_ms);
    check(joint_calibration.phase == JC_FAULT && joint_calibration.reason == JC_LEASE_EXPIRED &&
          motor_tx_stats[0].requested == 0 && motor_tx_stats[1].requested == 0, 1314);
    command("S:MODES"); check(!joint_calibration.open, 1315);
    reset(); warm(); calibration_source = JC_STORAGE_CORRUPT;
    command("S:AUTO"); check(system_state == STATE_MODES, 1316);
    command("S:MANUAL"); command("T:0"); command("J:73,1,BEGIN");
    check(joint_calibration.open, 1317);
    for (int i = 0; i < 100; ++i) calibration_test_step();
    command("J:73,2,IMPORT,-201.1,-24.8,-276.2,720,-95.4");
    command("J:73,3,APPLY");
    check(joint_calibration.phase == JC_REVIEW && !joint_calibration.dirty &&
          joint_calibration.reason == JC_BAD_VALUE && OFFSET_4 == -207.1f, 1319);
    command("S:ESTOP"); check(!joint_calibration.open && system_state == STATE_ESTOP, 1318);
}

void test_main(void) {
    joint_diagnostic_runtime();
    calibration_runtime();
    startup_and_commands();
    wrist_feedback();
    spi_validation_and_wrap();
    command_fifo();
    hot_and_late_sensor_power();
    transient_spi_during_manual();
    circular_dma_events();
    uart_fault_recovery();
    motor_transport_diagnostics();
    servo_status_readback();
    halfduplex_turnaround();
    elbow_clocked_transport();
    test_done();
}
