#pragma once
/* Included once by main.c, after JointCalibrationRuntime.h. No motor commands. */
static struct {
    JointDiagnosticPacket queue[4];
    volatile unsigned head, tail, busy;
    uint32_t run, cycle, command, command_ms, sequence, dropped;
    uint32_t last_host_ms, last_ack_ms, last_sample_ms, control_dt_ms;
    uint16_t stage;
    unsigned active, end_pending, ack_pending, end_flags;
    char ack[64];
} joint_diagnostic;

static int JD_OwnsUart(void) {
    return joint_diagnostic.active || joint_diagnostic.end_pending ||
        joint_diagnostic.busy || joint_diagnostic.head != joint_diagnostic.tail;
}
static int JD_HasRoom(void) {
    return (joint_diagnostic.head+1U)%4U != joint_diagnostic.tail;
}
static JointDiagnosticPacket *JD_NewPacket(unsigned type, uint32_t now) {
    ++joint_diagnostic.sequence;
    if (!JD_HasRoom()) { ++joint_diagnostic.dropped; return NULL; }
    JointDiagnosticPacket *p = &joint_diagnostic.queue[joint_diagnostic.head];
    memset(p, 0, sizeof(*p)); p->sync = JD_SYNC; p->version = JD_VERSION;
    p->type = (uint8_t)type; p->size = sizeof(*p);
    p->sequence = joint_diagnostic.sequence; p->run = joint_diagnostic.run;
    p->timestamp_ms = now; return p;
}
static void JD_Publish(JointDiagnosticPacket *p) {
    p->crc32 = JD_Crc32(p, offsetof(JointDiagnosticPacket, crc32));
    __DMB(); joint_diagnostic.head = (joint_diagnostic.head+1U)%4U;
}
static void JD_Configuration(uint32_t now) {
    JointDiagnosticPacket *p = JD_NewPacket(JD_CONFIG, now);
    if (!p) return;
    JointDiagnosticConfig *c = &p->data.config;
    c->build = CalibrationBuild(); c->flash_schema = CAL_STORE_SCHEMA;
    c->calibration_source = calibration_source; c->calibration_generation = calibration_generation;
    const float values[23] = {Kp_pos_L,Kp_pos_R,Kp_pos_3,Ki_pos_L,Ki_pos_R,Ki_pos_3,
        Kd_pos_L,Kd_pos_R,Kd_pos_3,u_fric_val_L,u_fric_val_R,u_fric_val_3,
        Kg_pos_LR,Kg_pos_3,P_weight,deadband_deg,max_slew_rate_LR,max_slew_rate_3,
        OFFSET_L,OFFSET_R,OFFSET_BASE,OFFSET_4,OFFSET_5};
    memcpy(c->values, values, sizeof(values)); c->sample_interval_ms = JD_INTERVAL_MS;
    JD_Publish(p);
}
static void JD_Sample(uint32_t now, unsigned extra_flags) {
    if (!joint_diagnostic.active && !extra_flags) return;
    if (!extra_flags && (uint32_t)(now-joint_diagnostic.last_sample_ms) < JD_INTERVAL_MS) return;
    joint_diagnostic.last_sample_ms = now;
    JointDiagnosticPacket *p = JD_NewPacket(JD_SAMPLE, now);
    if (!p) return;
    p->flags = extra_flags | (position_control_active ? JD_POSITION_ACTIVE : 0) |
        (feedback_fault_latched ? JD_FEEDBACK_FAULT : 0) |
        (motion_feedback_hold ? JD_FEEDBACK_HOLD : 0) |
        (joint_calibration.open ? JD_CALIBRATION_OPEN : 0);
    JointDiagnosticSample *s = &p->data.sample;
    s->cycle = joint_diagnostic.cycle; s->command = joint_diagnostic.command;
    s->command_ms = joint_diagnostic.command_ms; s->control_dt_ms = joint_diagnostic.control_dt_ms;
    s->dropped = joint_diagnostic.dropped; s->calibration_generation = calibration_generation;
    s->target[0]=cmd_pitch; s->target[1]=cmd_roll; s->target[2]=cmd_base;
    s->slew[0]=(slew_R_cal-slew_L_cal)/2; s->slew[1]=(slew_R_cal+slew_L_cal)/2;
    s->slew[2]=slew_3_cal;
    float q[6]; GetFeedbackAngles(now,q); memcpy(s->actual,q,sizeof(s->actual));
    const float raw[5] = {a1m,a1j,a2m,a2j,a3j}; memcpy(s->raw,raw,sizeof(raw));
    const float offsets[3] = {OFFSET_L,OFFSET_R,OFFSET_BASE}; memcpy(s->offsets,offsets,sizeof(offsets));
    for (unsigned i=0; i<3; ++i) {
        s->requested[i] = motor_tx_stats[i].requested;
        s->submitted[i] = motor_tx_stats[i].submitted;
    }
    for (unsigned i=0; i<SENSOR_COUNT; ++i) {
        uint32_t age = now-feedback_samples[i].timestamp;
        s->age_ms[i] = !feedback_samples[i].successes || age>65535U ? 65535U : (uint16_t)age;
    }
    s->state = system_state; s->valid_mask = latest_spi_mask;
    if (wrist_address.locked && Feedback_Fresh(&feedback_samples[SENSOR_WRIST],now,
                                               FEEDBACK_WRIST_MAX_AGE_MS)) s->valid_mask |= 0x20;
    s->stage = joint_diagnostic.stage; JD_Publish(p);
}
static void JD_RequestStop(unsigned flags) {
    if (!joint_diagnostic.active) return;
    joint_diagnostic.active = 0; joint_diagnostic.end_pending = 1;
    joint_diagnostic.end_flags = flags;
}
static int JD_HandleCommand(const char *line) {
    if (strncmp(line,"D:",2)) return 0;
    unsigned long run,cycle; unsigned stage; int end=0; uint32_t now=HAL_GetTick();
    if (sscanf(line,"D:START,%lu%n",&run,&end)==1 && !line[end] && run) {
        if (joint_diagnostic.active && joint_diagnostic.run == run) {
            joint_diagnostic.last_host_ms=now; joint_diagnostic.ack_pending=1;
        } else if (!JD_OwnsUart() && !joint_calibration.open) {
            memset(&joint_diagnostic,0,sizeof(joint_diagnostic));
            joint_diagnostic.run=run; joint_diagnostic.active=1;
            joint_diagnostic.last_host_ms=now; joint_diagnostic.ack_pending=1;
            JD_Configuration(now);
        }
    } else if (sscanf(line,"D:MARK,%lu,%lu,%u%n",&run,&cycle,&stage,&end)==3 &&
               !line[end] && stage<=JD_FAULT && joint_diagnostic.active && run==joint_diagnostic.run) {
        joint_diagnostic.cycle=cycle; joint_diagnostic.stage=stage;
        joint_diagnostic.last_host_ms=now;
    } else if (sscanf(line,"D:PING,%lu%n",&run,&end)==1 && !line[end] && run==joint_diagnostic.run) {
        if (joint_diagnostic.active) joint_diagnostic.last_host_ms=now;
        if ((uint32_t)(now-joint_diagnostic.last_ack_ms)>=250) joint_diagnostic.ack_pending=1;
    } else if (sscanf(line,"D:STOP,%lu%n",&run,&end)==1 && !line[end] &&
               run==joint_diagnostic.run) {
        JD_RequestStop(0); joint_diagnostic.ack_pending=1;
    }
    return 1;
}
static void JD_Service(uint32_t now) {
    if (joint_diagnostic.active && (uint32_t)(now-joint_diagnostic.last_host_ms)>JD_LEASE_MS)
        JD_RequestStop(JD_LEASE_ENDED);
    if (joint_diagnostic.end_pending && JD_HasRoom()) {
        JD_Sample(now,JD_END|joint_diagnostic.end_flags); joint_diagnostic.end_pending=0;
    }
    if (!joint_diagnostic.busy && joint_diagnostic.head!=joint_diagnostic.tail &&
        huart2.gState==HAL_UART_STATE_READY) {
        joint_diagnostic.busy=1;
        if (HAL_UART_Transmit_IT(&huart2,(uint8_t *)&joint_diagnostic.queue[joint_diagnostic.tail],
                                 sizeof(JointDiagnosticPacket))!=HAL_OK) joint_diagnostic.busy=0;
    }
    if (joint_diagnostic.ack_pending && !joint_diagnostic.end_pending &&
        huart6.gState==HAL_UART_STATE_READY) {
        int n=snprintf(joint_diagnostic.ack,sizeof(joint_diagnostic.ack),"D:%lu,%u,%u,%u\n",
            (unsigned long)joint_diagnostic.run,joint_diagnostic.active,JD_INTERVAL_MS,
            (unsigned)sizeof(JointDiagnosticPacket));
        if (HAL_UART_Transmit_IT(&huart6,(uint8_t *)joint_diagnostic.ack,n)==HAL_OK) {
            joint_diagnostic.ack_pending=0; joint_diagnostic.last_ack_ms=now;
        }
    }
}
static void JD_TransmitComplete(void) {
    if (joint_diagnostic.busy) {
        joint_diagnostic.tail=(joint_diagnostic.tail+1U)%4U;
        __DMB(); joint_diagnostic.busy=0;
    }
}
