#pragma once
/* HAL adapter, included by main.c after its globals and function declarations. */
static uint32_t calibration_generation, calibration_last_request;
static unsigned calibration_source = JC_DEFAULTS;
static uint32_t calibration_reply_id, calibration_save_id, calibration_save_ms;
static uint32_t calibration_ping_reply_ms;
static uint32_t calibration_zero_done[4];
static uint32_t calibration_reply_session;
static int calibration_rejected_begin;
static unsigned calibration_reply, calibration_preview, calibration_last_phase;
static int calibration_commit, calibration_stop_flushed;
static char calibration_tx[160];
static unsigned calibration_history;
static uint32_t CalibrationBuild(void) {
    const char *text = __DATE__ " " __TIME__; uint32_t hash = 2166136261U;
    while (*text) { hash ^= (uint8_t)*text++; hash *= 16777619U; }
    return hash;
}

static uint32_t CalFlashRead(void *context, unsigned sector, uint32_t word) {
    (void)context;
    return *(const volatile uint32_t *)(0x08040000UL + sector * 0x20000UL + word * 4);
}
static void CalFlashInvalidate(void) {
    if (FLASH->ACR & FLASH_ACR_DCEN) {
        __HAL_FLASH_DATA_CACHE_DISABLE(); __HAL_FLASH_DATA_CACHE_RESET();
        __HAL_FLASH_DATA_CACHE_ENABLE();
    }
    __DSB(); __ISB();
}
static int CalFlashProgram(void *context, unsigned sector, uint32_t word, uint32_t value) {
    (void)context;
    if (sector > 1 || word >= 0x8000U) return 0;
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
        0x08040000UL + sector * 0x20000UL + word * 4, value);
    CalFlashInvalidate(); return status == HAL_OK;
}
static int CalFlashErase(void *context, unsigned sector) {
    (void)context;
    if (sector > 1) return 0;
    FLASH_EraseInitTypeDef erase = {0}; uint32_t error = 0;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = sector ? FLASH_SECTOR_7 : FLASH_SECTOR_6;
    erase.NbSectors = 1; erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* Nucleo 3.3 V */
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &error);
    CalFlashInvalidate(); return status == HAL_OK;
}
static const CalibrationStorage calibration_storage = {
    NULL, 0x8000U, CalFlashRead, CalFlashProgram, CalFlashErase
};
static JointCalibrationSample CalibrationSample(uint32_t now) {
    JointCalibrationSample s = {{a1j, a2j, a3j}, {0,0,0}, 0}; float q[6];
    GetFeedbackAngles(now, q);
    memcpy(s.q, q, sizeof(s.q)); s.valid = FeedbackReady(now);
    const float lo[3] = {LIMIT_PITCH_MIN, LIMIT_ROLL_MIN, LIMIT_DOF3_MIN};
    const float hi[3] = {LIMIT_PITCH_MAX, LIMIT_ROLL_MAX, LIMIT_DOF3_MAX};
    for (int i = 0; i < 3; ++i)
        s.valid &= isfinite(s.raw[i]) && isfinite(s.q[i]) &&
            s.q[i] >= lo[i] - FEEDBACK_LIMIT_MARGIN_DEG && s.q[i] <= hi[i] + FEEDBACK_LIMIT_MARGIN_DEG;
    return s;
}
static int CalibrationCandidateFits(uint32_t now) {
    if (!FeedbackReady(now)) return 0;
    const float *o = joint_calibration.candidate.offset;
    float left = Feedback_CalibrateSpiSector(a1j, o[0], LIMIT_ROLL_MIN-LIMIT_PITCH_MAX, LIMIT_ROLL_MAX-LIMIT_PITCH_MIN);
    float right = Feedback_CalibrateSpiSector(a2j, o[1], LIMIT_ROLL_MIN+LIMIT_PITCH_MIN, LIMIT_ROLL_MAX+LIMIT_PITCH_MAX);
    const float q[5] = {(right-left)/2, (right+left)/2,
        Feedback_CalibrateSpiSector(a3j,o[2],LIMIT_DOF3_MIN,LIMIT_DOF3_MAX),
        a4+o[3], ((float)pwm_pos_5-500)*180.0f/2000.0f+o[4]};
    const float lo[5] = {LIMIT_PITCH_MIN,LIMIT_ROLL_MIN,LIMIT_DOF3_MIN,LIMIT_DOF4_MIN,LIMIT_DOF5_MIN};
    const float hi[5] = {LIMIT_PITCH_MAX,LIMIT_ROLL_MAX,LIMIT_DOF3_MAX,LIMIT_DOF4_MAX,LIMIT_DOF5_MAX};
    for (int i=0;i<5;++i)
        if (!isfinite(q[i]) || q[i]<lo[i]-FEEDBACK_LIMIT_MARGIN_DEG || q[i]>hi[i]+FEEDBACK_LIMIT_MARGIN_DEG) return 0;
    return 1;
}
static void CalibrationResetControl(void) {
    position_control_active = 0; memset(move_dirs, 0, sizeof(move_dirs));
    integral_L = integral_R = integral_3 = 0;
    slew_L_cal = target_L_cal = CalibratedLeftGear();
    slew_R_cal = target_R_cal = CalibratedRightGear();
    slew_3_cal = target_3_cal = CalibratedElbow();
    prev_motor_angle_L = a1m; prev_motor_angle_R = a2m; prev_joint_angle_3 = a3j;
    StopArmOutputs();
}
static void CalibrationReply(uint32_t id, unsigned reason) {
    joint_calibration.reason = reason; calibration_reply_id = id; calibration_reply = 1;
    calibration_reply_session = joint_calibration.session; calibration_rejected_begin = 0;
}
static void CalibrationAbort(unsigned reason) {
    JC_Cancel(&joint_calibration); calibration_commit = calibration_stop_flushed = 0;
    if (reason != JC_OK) JC_Fault(&joint_calibration, reason);
    CalibrationResetControl(); calibration_preview = 1; CalibrationReply(0, reason);
}
static void CalibrationBootLoad(void) {
    const JointOffsets defaults = {JC_DEFAULT_OFFSETS};
    CalibrationRecord r = CalStore_Scan(&calibration_storage, 0);
    JC_Init(&joint_calibration, r.valid ? &r.offsets : &defaults);
    if (r.valid) joint_calibration.evidence = r.evidence;
    calibration_generation = r.generation;
    calibration_source = r.valid ? JC_STORED : (r.any_data ? JC_STORAGE_CORRUPT : JC_DEFAULTS);
    printf("[CAL] firmware build=%08lX %s %s schema=%08lX\r\n",
           (unsigned long)CalibrationBuild(), __DATE__, __TIME__, (unsigned long)CAL_STORE_SCHEMA);
    printf("[CAL] boot source=%u generation=%lu offsets=%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
           calibration_source, (unsigned long)calibration_generation,
           OFFSET_L, OFFSET_R, OFFSET_BASE, OFFSET_4, OFFSET_5);
}
static const JointCalibrationEvidence *CalibrationDisplayedEvidence(void) {
    return !joint_calibration.open || joint_calibration.phase == JC_REVIEW || joint_calibration.dirty ||
        joint_calibration.phase == JC_SAVED ? &joint_calibration.evidence : &joint_calibration.baseline_evidence;
}
static void CalibrationTx(void) {
    if (huart6.gState != HAL_UART_STATE_READY) return;
    int n = 0;
    if (calibration_reply) {
        n = snprintf(calibration_tx, sizeof(calibration_tx), "J:%lu,%lu,%u,%u,%u\n",
            (unsigned long)calibration_reply_session, (unsigned long)calibration_reply_id,
            calibration_rejected_begin ? JC_IDLE : joint_calibration.phase,
            calibration_rejected_begin ? 0 : joint_calibration.mask, joint_calibration.reason);
        if (HAL_UART_Transmit_IT(&huart6, (uint8_t *)calibration_tx, n) == HAL_OK) calibration_reply = 0;
    } else if (calibration_preview) {
        const JointOffsets *o = joint_calibration.phase == JC_REVIEW ?
            &joint_calibration.candidate : &joint_calibration.active;
        n = snprintf(calibration_tx, sizeof(calibration_tx), "O:%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%u\n",
            (unsigned long)joint_calibration.session, (unsigned long)calibration_generation,
            o->offset[0], o->offset[1], o->offset[2], o->offset[3], o->offset[4],
            joint_calibration.dirty ? JC_TEMPORARY : calibration_source);
        if (HAL_UART_Transmit_IT(&huart6, (uint8_t *)calibration_tx, n) == HAL_OK) {
            calibration_preview = 0; calibration_history = CalibrationDisplayedEvidence()->mask;
        }
    } else if (calibration_history) {
        unsigned axis = 0; while (!(calibration_history & (1U << axis))) ++axis;
        const JointCalibrationEvidence *e = CalibrationDisplayedEvidence();
        n = snprintf(calibration_tx, sizeof(calibration_tx), "H:%lu,%lu,%u,%.3f,%.3f,%.3f,%.3f,%lu\n",
            (unsigned long)joint_calibration.session, (unsigned long)calibration_generation, axis,
            e->raw[axis][0], e->raw[axis][1], e->raw[axis][2], e->reference[axis], (unsigned long)e->build);
        if (HAL_UART_Transmit_IT(&huart6, (uint8_t *)calibration_tx, n) == HAL_OK)
            calibration_history &= ~(1U << axis);
    }
}
static int CalibrationHandleCommand(char *line) {
    if (strncmp(line, "J:", 2)) return 0;
    unsigned long session, request; char verb[16]; int used = 0;
    if (sscanf(line, "J:%lu,%lu,%15[A-Z]%n", &session, &request, verb, &used) != 3) return 1;
    uint32_t now = HAL_GetTick(); JointCalibration *c = &joint_calibration;
    if (!strcmp(verb, "STATUS") || !strcmp(verb, "EXPORT")) {
        if (line[used]) return 1;
        calibration_preview = 1;
        if (!c->open && c->phase != JC_FAULT) CalibrationReply((uint32_t)request, calibration_source);
        else CalibrationReply((uint32_t)request, c->reason);
        return 1;
    }
    if (!strcmp(verb, "PING")) {
        if (!line[used] && (c->open || c->phase == JC_FAULT) && session == c->session) {
            // Retain/report the terminal fault if its first notification was
            // lost. A keepalive must never reopen or resume the closed session.
            if (c->open) c->lease_ms = now;
            if (!calibration_reply && (uint32_t)(now - calibration_ping_reply_ms) >= 250) {
                calibration_ping_reply_ms = now; CalibrationReply(0, c->reason);
            }
        }
        return 1;
    }
    /* Duplicated requests return the current result; never repeat a movement/save. */
    if (session == c->session && request && request <= calibration_last_request) {
        if (request == calibration_last_request) {
            CalibrationReply((uint32_t)request, c->reason); calibration_preview = 1;
        }
        return 1;
    }
    if (!strcmp(verb, "BEGIN")) {
        if (line[used]) return 1;
        if (c->open && (c->phase == JC_FAULT || c->phase == JC_SAVED)) CalibrationAbort(JC_OK);
        JointCalibrationSample s = CalibrationSample(now);
        if (!request || system_state != STATE_MANUAL || target_mode != 0 || feedback_fault_latched ||
            !JC_Begin(c, (uint32_t)session, now, &s)) {
            CalibrationReply((uint32_t)request, s.valid ? JC_BAD_STATE : JC_BAD_FEEDBACK);
            calibration_reply_session = (uint32_t)session; calibration_rejected_begin = 1;
            calibration_preview = 1; return 1;
        }
        CalibrationResetControl(); calibration_last_request = 0;
        calibration_commit = calibration_stop_flushed = 0; calibration_preview = 1;
    } else if (!c->open || session != c->session || !request) return 1;
    c->lease_ms = now;
    int ok = 1; unsigned reason = JC_BAD_STATE;
    if (!strcmp(verb, "BEGIN")) { /* already processed */ }
    else if (!strcmp(verb, "SELECT")) {
        unsigned axis; int direction, end = 0;
        ok = sscanf(line + used, ",%u,%d%n", &axis, &direction, &end) == 2 &&
            !line[used + end] && JC_Select(c, axis, direction);
    } else if (line[used] && strcmp(verb, "IMPORT")) { ok = 0; reason = JC_BAD_COMMAND; }
    else if (!strcmp(verb, "MOVE") || !strcmp(verb, "SWEEP")) {
        JointCalibrationSample s = CalibrationSample(now);
        ok = JC_Start(c, !strcmp(verb, "SWEEP"), now, &s);
    } else if (!strcmp(verb, "STOP")) { JC_Stop(c); CalibrationResetControl(); }
    else if (!strcmp(verb, "CAPTURE")) { ok = JC_Capture(c, now); reason = JC_NOT_STILL; }
    else if (!strcmp(verb, "APPLY")) {
        reason = JC_Still(c, now) ? JC_BAD_VALUE : JC_NOT_STILL;
        ok = CalibrationCandidateFits(now) && JC_Apply(c, now);
        if (ok) { CalibrationResetControl(); calibration_preview = 1; }
    } else if (!strcmp(verb, "VERIFY")) {
        reason = c->phase == JC_APPLIED ? JC_NOT_STILL : JC_BAD_STATE;
        ok = c->phase == JC_APPLIED && JC_Still(c, now);
        if (ok) c->phase = JC_VERIFIED;
    } else if (!strcmp(verb, "SAVE")) {
        reason = c->phase == JC_VERIFIED ? JC_NOT_STILL : JC_BAD_STATE;
        ok = c->phase == JC_VERIFIED && JC_Still(c, now);
        if (ok) {
            CalibrationResetControl(); c->phase = JC_SAVE_ARMED; calibration_save_ms = now;
        }
    } else if (!strcmp(verb, "COMMIT")) {
        ok = c->phase == JC_SAVE_ARMED && JC_Still(c, now) &&
            (uint32_t)(now - calibration_save_ms) < JC_SAVE_ARM_TIMEOUT_MS;
        if (ok) {
            calibration_commit = 1; calibration_stop_flushed = 0;
            calibration_save_id = (uint32_t)request; c->phase = JC_SAVING;
            CalibrationResetControl();
        } else if (c->phase == JC_SAVE_ARMED) {
            c->phase = JC_APPLIED; reason = JC_NOT_STILL;
        }
    } else if (!strcmp(verb, "CANCEL")) { CalibrationAbort(JC_OK); }
    else if (!strcmp(verb, "IMPORT")) {
        JointOffsets o; int end = 0;
        ok = (c->phase == JC_READY || c->phase == JC_STOPPED || c->phase == JC_REVIEW) &&
            sscanf(line + used, ",%f,%f,%f,%f,%f%n", &o.offset[0], &o.offset[1], &o.offset[2],
                &o.offset[3], &o.offset[4], &end) == 5 && !line[used + end] && JC_OffsetsValid(&o);
        if (ok) {
            c->candidate = o; c->mask = 7; c->phase = JC_REVIEW; calibration_preview = 1;
            memset(&c->evidence, 0, sizeof(c->evidence));
        }
    } else if (!strcmp(verb, "RESTORE")) {
        ok = c->phase == JC_READY || c->phase == JC_STOPPED || c->phase == JC_REVIEW;
        if (ok) {
            CalibrationRecord latest = CalStore_Scan(&calibration_storage, 0);
            CalibrationRecord previous = CalStore_Scan(&calibration_storage, latest.generation);
            ok = previous.valid;
            if (ok) {
                c->candidate = previous.offsets; c->mask = 7; c->phase = JC_REVIEW; calibration_preview = 1;
                memset(&c->evidence, 0, sizeof(c->evidence));
            }
        }
    } else { ok = 0; reason = JC_BAD_COMMAND; }
    calibration_last_request = (uint32_t)request;
    CalibrationReply((uint32_t)request, ok ? JC_OK : reason);
    return 1;
}
static void CalibrationControlTick(uint32_t now) {
    JointCalibration *c = &joint_calibration;
    JointCalibrationSample s = CalibrationSample(now);
    JC_Tick(c, now, &s);
    if (c->phase == JC_SAVE_ARMED && (uint32_t)(now - calibration_save_ms) > JC_SAVE_ARM_TIMEOUT_MS)
        JC_Fault(c, JC_TIME_LIMIT);
    if (c->phase == JC_FAULT && c->dirty) {
        c->active = c->baseline; c->dirty = 0; calibration_preview = 1;
        CalibrationResetControl();
    }
    if (c->phase == JC_FAULT) c->evidence = c->baseline_evidence;
    if (c->phase != calibration_last_phase) {
        calibration_last_phase = c->phase; CalibrationReply(0, c->reason);
        if (c->phase == JC_REVIEW) calibration_preview = 1;
    }
    if (!JC_Moving(c)) { StopArmOutputs(); return; }
    const float lo[3] = {LIMIT_PITCH_MIN, LIMIT_ROLL_MIN, LIMIT_DOF3_MIN};
    const float hi[3] = {LIMIT_PITCH_MAX, LIMIT_ROLL_MAX, LIMIT_DOF3_MAX};
    for (int i = 0; i < 3; ++i)
        if ((s.q[i] <= lo[i] && c->rates[i] < 0) || (s.q[i] >= hi[i] && c->rates[i] > 0)) {
            JC_Fault(c, JC_TRAVEL_LIMIT); StopArmOutputs(); return;
        }
    /* Differential velocity feed-forward plus measured-velocity correction.
       Existing motor polarities: +L/+R increase their gear angle; -J3 increases elbow. */
    const float desired[3] = {c->rates[1] - c->rates[0], c->rates[1] + c->rates[0], c->rates[2]};
    const float measured[3] = {c->velocity[1] - c->velocity[0], c->velocity[1] + c->velocity[0], c->velocity[2]};
    const float friction[3] = {u_fric_val_L, u_fric_val_R, u_fric_val_3};
    int16_t output[3];
    for (int i = 0; i < 3; ++i) {
        float v = desired[i];
        float u = fabsf(v) < .05f ? 0 : (v > 0 ? friction[i] : -friction[i]) +
            JC_VELOCITY_GAIN * (v - measured[i]);
        output[i] = (int16_t)JC_Clamp(u, -JC_MAX_OUTPUT, JC_MAX_OUTPUT);
    }
    Hiwonder_SetMotorSpeed_IT(&huart1, 254, output[0]);
    Hiwonder_SetMotorSpeed_IT(&huart3, 254, output[1]);
    Hiwonder_SetMotorSpeed_IT(&huart4, 254, -output[2]);
}
static int CalibrationUartsIdle(void) {
    return huart1.gState == HAL_UART_STATE_READY && huart3.gState == HAL_UART_STATE_READY &&
        huart4.gState == HAL_UART_STATE_READY && huart5.gState == HAL_UART_STATE_READY &&
        huart6.gState == HAL_UART_STATE_READY;
}
static void CalibrationService(void) {
    JointCalibration *c = &joint_calibration;
    if (calibration_commit) {
        uint32_t now = HAL_GetTick();
        if (c->phase != JC_SAVING || system_state != STATE_MANUAL ||
            command_queue.emergency || command_queue.overflow || feedback_fault_latched ||
            !CalibrationSample(now).valid || !JC_Still(c, now) ||
            (uint32_t)(now - calibration_save_ms) > JC_SAVE_TIMEOUT_MS) {
            calibration_commit = 0; JC_Fault(c, JC_BAD_STATE);
            CalibrationReply(calibration_save_id, JC_BAD_STATE);
        } else if (CalibrationUartsIdle()) {
            if (!calibration_stop_flushed) {
                for (int i = 0; i < 4; ++i) calibration_zero_done[i] = motor_tx_stats[i].completed + 1;
                StopArmOutputs(); calibration_stop_flushed = 1;
                for (int i = 0; i < 4; ++i)
                    if (motor_tx_stats[i].last_status != HAL_OK) calibration_stop_flushed = 0;
            } else {
                for (int i = 0; i < 4; ++i)
                    if ((int32_t)(motor_tx_stats[i].completed - calibration_zero_done[i]) < 0) {
                        calibration_stop_flushed = 0; return;
                    }
                /* All zero-speed UART packets completed. Portenta is quiescent.
                   Arm must support itself: Flash can stall ALL normal Nucleo code. */
                int ok = HAL_FLASH_Unlock() == HAL_OK;
                c->evidence.build = CalibrationBuild();
                if (ok) ok = CalStore_SaveWithEvidence(&calibration_storage, &c->active,
                    &c->evidence, &calibration_generation);
                HAL_FLASH_Lock(); calibration_commit = 0;
                c->lease_ms = HAL_GetTick();
                if (ok) {
                    c->baseline = c->active; c->dirty = 0; c->phase = JC_SAVED;
                    c->baseline_evidence = c->evidence;
                    calibration_source = JC_STORED;
                } else c->phase = JC_VERIFIED;
                /* Do not mistake pre-save sensor caches for fresh measurements. */
                for (int i = 0; i < SENSOR_COUNT; ++i) feedback_samples[i].successes = 0;
                latest_spi_mask = 0; c->have_previous = c->have_stable = 0;
                calibration_preview = 1;
                CalibrationReply(calibration_save_id, ok ? JC_OK : JC_STORAGE_ERROR);
            }
        }
    }
    CalibrationTx();
}
