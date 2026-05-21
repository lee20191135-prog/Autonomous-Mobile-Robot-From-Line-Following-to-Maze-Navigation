unsigned int  t0_ms          = 0;
unsigned char t0_half        = 0;
unsigned char t0_3s_done     = 0;
unsigned char start_pressed  = 0;
unsigned char in_tunnel      = 0;
unsigned char tunnel_exited  = 0;
unsigned char maze_mode      = 0;
unsigned char finish_detected = 0;
unsigned char finish_done     = 0;

/* ================= SOFTWARE DELAYS ================= */

void delay_us_sw(unsigned int us) {
    unsigned int i;

    for (i = 0; i < us; i++) {
        asm nop;
        asm nop;
    }
}

void delay_ms_sw(unsigned int ms) {
    unsigned int i, j;

    for (i = 0; i < ms; i++) {
        for (j = 0; j < 333; j++) {
        }
    }
}

/* ================= PWM ================= */

void pwm_init(void) {
    T2CON   = 0x07;     // Timer2 ON, prescaler 1:16
    PR2     = 249;
    CCP1CON = 0x0C;     // CCP1 PWM mode
    CCP2CON = 0x0C;     // CCP2 PWM mode
    CCPR1L  = 0;
    CCPR2L  = 0;
}

void pwm_set(unsigned char left_duty, unsigned char right_duty) {
    CCPR1L = left_duty;
    CCPR2L = right_duty;
}

/* ================= MOTOR FUNCTIONS ================= */

void motors_forward(void) {
    PORTD = (PORTD & 0xF0) | 0x05;
}

void motors_stop(void) {
    PORTD = (PORTD & 0xF0) | 0x00;
    pwm_set(0, 0);
}

void motors_turn_left(void) {
    PORTD = (PORTD & 0xF0) | 0x05;
    pwm_set(50, 85);
}

void motors_turn_right(void) {
    PORTD = (PORTD & 0xF0) | 0x05;
    pwm_set(85, 50);
}

void motors_pivot_left(void) {
    PORTD = (PORTD & 0xF0) | 0x06;
    pwm_set(70, 70);
}

void motors_pivot_right(void) {
    PORTD = (PORTD & 0xF0) | 0x09;
    pwm_set(70, 70);
}

/* ================= ADC / LDR ================= */

unsigned int adc_read_ldr(void) {
    unsigned int result;

    ADCON0 = 0x41;          // AN0 selected, ADC ON, Fosc/8
    delay_us_sw(20);        // acquisition time
    ADCON0 |= 0x04;         // start conversion

    while (ADCON0 & 0x04) {
    }

    result = ((unsigned int)ADRESH << 8) | ADRESL;
    return result;
}

/* ================= ULTRASONIC TRIGGERS ================= */

void trig_front(void) {
    PORTD |= 0x40;          // RD6 HIGH
    delay_us_sw(10);
    PORTD &= ~0x40;         // RD6 LOW
}

void trig_left(void) {
    PORTD |= 0x20;          // RD5 HIGH
    delay_us_sw(10);
    PORTD &= ~0x20;         // RD5 LOW
}

void trig_right(void) {
    PORTD |= 0x80;          // RD7 HIGH
    delay_us_sw(10);
    PORTD &= ~0x80;         // RD7 LOW
}

/* ================= SERVO ================= */

void servo_raise_flag(void) {
    unsigned char k;

    for (k = 0; k < 120; k++) {
        PORTC |= 0x10;          // RC4 HIGH
        delay_us_sw(2000);      // 2 ms pulse
        PORTC &= ~0x10;         // RC4 LOW
        delay_ms_sw(18);        // total period around 20 ms
    }
}

/* ================= INTERRUPT ================= */

void interrupt(void) {
    if (INTCON & 0x02) {        // RB0 external interrupt flag
        start_pressed = 1;
        INTCON &= ~0x02;        // clear INTF
    }

    if (INTCON & 0x04) {        // Timer0 overflow flag
        INTCON &= ~0x04;        // clear T0IF
        TMR0 = 131;

        t0_half ^= 1;

        if (t0_half) {
            t0_ms++;

            if (t0_ms >= 3000) {
                t0_3s_done = 1;
            }
        }
    }
}

/* ================= ULTRASONIC MEASUREMENT ================= */

unsigned int measure_echo_polled(unsigned char pin_mask) {
    unsigned int timeout;
    unsigned int t;

    timeout = 0;

    while (!(PORTB & pin_mask)) {
        timeout++;

        if (timeout > 30000) {
            return 0;
        }
    }

    TMR1H = 0;
    TMR1L = 0;
    T1CON = 0x11;       // Timer1 ON, prescaler 1:2

    timeout = 0;

    while (PORTB & pin_mask) {
        timeout++;

        if (timeout > 30000) {
            T1CON = 0x10;
            return 0;
        }
    }

    T1CON = 0x10;       // Timer1 OFF
    t = ((unsigned int)TMR1H << 8) | TMR1L;

    return t / 58;      // distance in cm
}

unsigned int read_front_dist(void) {
    trig_front();
    delay_us_sw(15);
    return measure_echo_polled(0x02);      // RB1
}

unsigned int read_right_dist(void) {
    trig_right();
    delay_us_sw(15);
    return measure_echo_polled(0x04);      // RB2
}

unsigned int read_left_dist(void) {
    trig_left();
    delay_us_sw(15);
    return measure_echo_polled(0x08);      // RB3
}

/* ================= LINE FOLLOWING ================= */

void line_follow_step(void) {
    unsigned char ir_right;
    unsigned char ir_left;

    ir_right = (PORTB >> 4) & 0x01;     // RB4
    ir_left  = (PORTB >> 5) & 0x01;     // RB5

    if (!ir_left && !ir_right) {
        PORTD = (PORTD & 0xF0) | 0x05;
        pwm_set(85, 85);
    }
    else if (ir_right && !ir_left) {
        PORTD = (PORTD & 0xF0) | 0x04;
        pwm_set(60, 80);
    }
    else if (ir_left && !ir_right) {
        PORTD = (PORTD & 0xF0) | 0x01;
        pwm_set(80, 60);
    }
    else {
        PORTD = (PORTD & 0xF0) | 0x01;
        pwm_set(60, 80);
    }
}

/* ================= MAZE MODE ================= */

void maze_step(void) {
    unsigned int fd;
    unsigned int ld;

    fd = read_front_dist();
    ld = read_left_dist();

    PORTD = (PORTD & 0xF0) | 0x05;

    if (fd > 0 && fd < 25) {
        motors_pivot_right();
        delay_ms_sw(350);
        motors_stop();
        delay_ms_sw(100);
    }
    else if (ld == 0 || ld > 35) {
        motors_turn_left();
        delay_ms_sw(200);
    }
    else {
        pwm_set(85, 85);
    }
}

/* ================= MAIN ================= */

void main(void) {
    unsigned int ldr_val;
    unsigned char ir_right;
    unsigned char ir_left;
    unsigned char b;

    ADCON1 = 0x8E;      // AN0 analog, others digital, right justified

    TRISA = 0xFF;       // RA0 input for LDR
    TRISB = 0xFF;       // RB inputs
    TRISC = 0x00;       // PWM + servo outputs
    TRISD = 0x00;       // motors, buzzer, ultrasonic triggers
    TRISE = 0x00;

    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;

    OPTION_REG &= 0x7F;     // enable PORTB pull-ups

    pwm_init();

    T1CON = 0x10;           // Timer1 OFF, prescaler 1:2

    OPTION_REG &= ~0x40;    // external interrupt on falling edge RB0
    INTCON |= 0x10;         // enable RB0 external interrupt

    OPTION_REG &= 0b11000000;
    OPTION_REG |= 0b00000010;   // Timer0 prescaler 1:8

    TMR0 = 131;
    INTCON |= 0x20;         // enable Timer0 interrupt
    INTCON |= 0x80;         // enable global interrupt

    while (!start_pressed) {
        motors_stop();
    }

    t0_ms = 0;
    t0_half = 0;
    t0_3s_done = 0;

    while (!t0_3s_done) {
        motors_stop();
    }

    while (1) {
        if (tunnel_exited && !finish_done) {
            ir_right = (PORTB >> 4) & 0x01;
            ir_left  = (PORTB >> 5) & 0x01;

            if (ir_left && ir_right) {
                finish_detected = 1;
            }

            if (finish_detected) {
                motors_stop();
                delay_ms_sw(200);

                for (b = 0; b < 3; b++) {
                    PORTD |= 0x10;
                    delay_ms_sw(200);
                    PORTD &= ~0x10;
                    delay_ms_sw(200);
                }

                servo_raise_flag();
                finish_done = 1;

                while (1) {
                    motors_stop();
                }
            }
        }

        ldr_val = adc_read_ldr();

        if (ldr_val > 512) {
            PORTD |= 0x10;          // buzzer ON
            in_tunnel = 1;
        }
        else {
            PORTD &= ~0x10;         // buzzer OFF

            if (in_tunnel) {
                in_tunnel = 0;
                tunnel_exited = 1;
                maze_mode = 1;
            }
        }

        if (maze_mode) {
            maze_step();
            continue;
        }

        if (in_tunnel) {
            PORTD = (PORTD & 0xF0) | 0x05;
            pwm_set(75, 75);
            continue;
        }

        line_follow_step();
    }
}