/*
 * main.c — STM32L432KC bare-metal guitar tuner
 *
 * Audio pipeline:
 *   LM393 microphone module (AO pin) -> PB0 (ADC1 ch15)
 *   TIM6 at 16 kHz triggers ADC -> DMA1 ch1 -> 2048-sample circular buffer
 *   YIN pitch detection on each full buffer -> note + cents deviation
 *
 * Display (16x2 HD44780 via PCF8574 I2C backpack on PB6/PB7):
 *   Row 0:  <note>            <target>Hz
 *   Row 1:  <5-cell meter>   <measured>Hz
 *
 * Meter: middle cell always lit. In tune (<=5 cents) -> middle only + checkmark.
 *        Slightly off (<=20 cents) -> middle + 1 adjacent cell.
 *        Far off -> middle + 2 adjacent cells. Left = flat, right = sharp.
 */

#include "main.h"
#include "i2c/i2c.h"
#include "lcd/lcd.h"
#include "mic/mic.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/*  Register map                                                        */
/* ------------------------------------------------------------------ */

#define RCC_BASE        0x40021000UL
#define ADC1_BASE       0x50040000UL
#define ADC_COMMON_BASE 0x50040000UL
#define TIM6_BASE       0x40001000UL
#define DMA1_BASE       0x40020000UL

#define RCC_AHB1ENR   (*(volatile uint32_t *)(RCC_BASE + 0x48))
#define RCC_AHB2ENR   (*(volatile uint32_t *)(RCC_BASE + 0x4C))
#define RCC_APB1ENR1  (*(volatile uint32_t *)(RCC_BASE + 0x58))

#define GPIOB_BASE    0x48000400UL
#define GPIOB_MODER   (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_OTYPER  (*(volatile uint32_t *)(GPIOB_BASE + 0x04))
#define GPIOB_OSPEEDR (*(volatile uint32_t *)(GPIOB_BASE + 0x08))
#define GPIOB_PUPDR   (*(volatile uint32_t *)(GPIOB_BASE + 0x0C))
#define GPIOB_AFRL    (*(volatile uint32_t *)(GPIOB_BASE + 0x20))

#define ADC_CCR       (*(volatile uint32_t *)(ADC_COMMON_BASE + 0x308))
#define ADC1_ISR      (*(volatile uint32_t *)(ADC1_BASE + 0x00))
#define ADC1_CR       (*(volatile uint32_t *)(ADC1_BASE + 0x08))
#define ADC1_CFGR     (*(volatile uint32_t *)(ADC1_BASE + 0x0C))
#define ADC_SMPR2     (*(volatile uint32_t *)(ADC1_BASE + 0x18))
#define ADC_SQR1      (*(volatile uint32_t *)(ADC1_BASE + 0x30))
#define ADC1_DR       (*(volatile uint32_t *)(ADC1_BASE + 0x40))

#define TIM6_CR1      (*(volatile uint32_t *)(TIM6_BASE + 0x00))
#define TIM6_CR2      (*(volatile uint32_t *)(TIM6_BASE + 0x04))
#define TIM6_PSC      (*(volatile uint32_t *)(TIM6_BASE + 0x28))
#define TIM6_ARR      (*(volatile uint32_t *)(TIM6_BASE + 0x2C))

#define DMA1_IFCR     (*(volatile uint32_t *)(DMA1_BASE + 0x004))
#define DMA1_CCR1     (*(volatile uint32_t *)(DMA1_BASE + 0x008))
#define DMA1_CNDTR1   (*(volatile uint32_t *)(DMA1_BASE + 0x00C))
#define DMA1_CPAR1    (*(volatile uint32_t *)(DMA1_BASE + 0x010))
#define DMA1_CMAR1    (*(volatile uint32_t *)(DMA1_BASE + 0x014))

#define NVIC_ISER0    (*(volatile uint32_t *)0xE000E100UL)

/* ------------------------------------------------------------------ */
/*  Tuning constants                                                    */
/* ------------------------------------------------------------------ */

#define N_SAMPLES         2048
#define RELEASE_FRAMES    3       /* unpitched frames before display clears */

#define YIN_THRESHOLD     0.10f
#define TAU_MAX           205     /* lowest searchable period: ~78 Hz (low E) */
#define FS_HZ             16000.0f

#define IN_TUNE_CENTS     5.0f
#define SLIGHT_MAX_CENTS  20.0f

/* ------------------------------------------------------------------ */
/*  Custom LCD glyphs (CGRAM slots 1 and 2)                            */
/* ------------------------------------------------------------------ */

#define CH_TRACK  0x01u   /* thin baseline: unlit meter cell */
#define CH_TICK   0x02u   /* checkmark: in-tune indicator    */
#define CH_FILL   0xFFu   /* built-in solid block: lit meter cell */

static const uint8_t GLYPH_TRACK[8] = { 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00 };
static const uint8_t GLYPH_TICK[8]  = { 0x00, 0x01, 0x03, 0x16, 0x1C, 0x08, 0x00, 0x00 };

/* ------------------------------------------------------------------ */
/*  Note table (E1 = index 0, A4 = index 41, E6 = index 62)           */
/*  Equal-tempered frequency: f(i) = 440 * 2^((i-41)/12)             */
/* ------------------------------------------------------------------ */

static const char *note_names[] = {
    "E1",  "F1",  "F#1", "G1",  "G#1", "A1",  "A#1", "B1",
    "C2",  "C#2", "D2",  "D#2",
    "E2",  "F2",  "F#2", "G2",  "G#2", "A2",  "A#2", "B2",
    "C3",  "C#3", "D3",  "D#3",
    "E3",  "F3",  "F#3", "G3",  "G#3", "A3",  "A#3", "B3",
    "C4",  "C#4", "D4",  "D#4",
    "E4",  "F4",  "F#4", "G4",  "G#4", "A4",  "A#4", "B4",
    "C5",  "C#5", "D5",  "D#5",
    "E5",  "F5",  "F#5", "G5",  "G#5", "A5",  "A#5", "B5",
    "C6",  "C#6", "D6",  "D#6",
    "E6"
};
#define NUM_NOTES (int)(sizeof(note_names) / sizeof(note_names[0]))

/* ------------------------------------------------------------------ */
/*  Shared state                                                        */
/* ------------------------------------------------------------------ */

UART_HandleTypeDef huart2;

static volatile uint16_t adc_buf[N_SAMPLES];
static volatile uint8_t  adc_ready = 0;

void process_buffer(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* ------------------------------------------------------------------ */
/*  UART diagnostics                                                    */
/* ------------------------------------------------------------------ */

static void uart_log(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
}

static void uart_log_hex(const char *label, uint32_t val)
{
    char buf[48];
    const char *hex = "0123456789ABCDEF";
    char h[9];
    for (int i = 7; i >= 0; i--) { h[i] = hex[val & 0xF]; val >>= 4; }
    h[8] = '\0';
    int j = 0;
    for (int i = 0; label[i] && j < 38; i++) buf[j++] = label[i];
    buf[j++] = '0'; buf[j++] = 'x';
    for (int i = 0; i < 8; i++) buf[j++] = h[i];
    buf[j++] = '\r'; buf[j++] = '\n'; buf[j] = '\0';
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, j, 1000);
}

/* ------------------------------------------------------------------ */
/*  Display rendering                                                   */
/* ------------------------------------------------------------------ */

static float note_exact_hz(int idx)
{
    return 440.0f * exp2f((float)(idx - 41) / 12.0f);
}

/*
 * Right-justify "<hz>Hz" into a 6-character field starting at buf[start].
 * Leaves the field blank if hz <= 0.
 */
static void fmt_hz(uint8_t *buf, int start, int hz)
{
    for (int i = 0; i < 6; i++) buf[start + i] = ' ';
    if (hz <= 0) return;

    char s[8];
    int sn = 0;
    char digs[5];
    int dn = 0;
    int v = hz;
    while (v > 0 && dn < 5) { digs[dn++] = (char)('0' + (v % 10)); v /= 10; }
    for (int i = dn - 1; i >= 0; i--) s[sn++] = digs[i];
    s[sn++] = 'H';
    s[sn++] = 'z';

    int off = 6 - sn;
    if (off < 0) off = 0;
    for (int i = 0; i < sn && (off + i) < 6; i++)
        buf[start + off + i] = (uint8_t)s[i];
}

/*
 * Rebuild both display rows and push only the cells that changed.
 * Per-cell diffing avoids the flicker that a full lcd_clear() would cause.
 *
 * note_idx: index into note_names[], or -1 for "no signal"
 * have:     non-zero when a valid pitch was detected this frame
 * cents:    deviation from nearest note (positive = sharp)
 * cur_hz:   measured frequency in Hz (displayed on row 1)
 */
static void render_display(int note_idx, int have, float cents, int cur_hz)
{
    static uint8_t prev1[16] = {0};
    static uint8_t prev2[16] = {0};

    uint8_t r1[16], r2[16];
    for (int i = 0; i < 16; i++) { r1[i] = ' '; r2[i] = ' '; }

    /* Row 0: note name (left) + equal-tempered target Hz (right) */
    if (note_idx >= 0) {
        const char *nm = note_names[note_idx];
        for (int k = 0; nm[k] && k < 4; k++) r1[k] = (uint8_t)nm[k];
        fmt_hz(r1, 10, (int)(note_exact_hz(note_idx) + 0.5f));
    } else {
        r1[0] = '-'; r1[1] = '-';
    }

    /* Row 1: 5-cell meter (cols 0-4), checkmark (col 6), measured Hz (cols 10-15) */
    for (int i = 0; i < 5; i++) r2[i] = CH_TRACK;
    r2[2] = CH_FILL;   /* middle cell is always lit */

    if (have) {
        float ac = fabsf(cents);
        if (ac <= IN_TUNE_CENTS) {
            r2[6] = CH_TICK;
        } else {
            int sharp = (cents > 0.0f);
            if (sharp) r2[3] = CH_FILL; else r2[1] = CH_FILL;
            if (ac > SLIGHT_MAX_CENTS) {
                if (sharp) r2[4] = CH_FILL; else r2[0] = CH_FILL;
            }
        }
        fmt_hz(r2, 10, cur_hz);
    }

    for (int c = 0; c < 16; c++) {
        if (r1[c] != prev1[c]) {
            lcd_set_cursor(0, (uint8_t)c);
            lcd_print_char((char)r1[c]);
            prev1[c] = r1[c];
        }
    }
    for (int c = 0; c < 16; c++) {
        if (r2[c] != prev2[c]) {
            lcd_set_cursor(1, (uint8_t)c);
            lcd_print_char((char)r2[c]);
            prev2[c] = r2[c];
        }
    }
}

/* ------------------------------------------------------------------ */
/*  YIN pitch detection                                                 */
/* ------------------------------------------------------------------ */

/*
 * Runs on each completed DMA buffer. Steps:
 *   1. Difference function
 *   2. Cumulative mean normalized difference (CMND)
 *   3. First dip below YIN_THRESHOLD -> best period (tau)
 *   4. Parabolic refinement for sub-sample accuracy
 *   5. Note lookup + cents calculation + display update
 *
 * "No dip found" (best_tau == -1) is the silence gate: it is cleaner than
 * variance-based gating on the LM393 hardware, where amplitude overlaps
 * between silence and a ringing string make variance unreliable.
 */
void process_buffer(void)
{
    static float d[TAU_MAX + 2];
    static int   displayed_note = -1;
    static int   silence_count  = 0;

    /* DC removal */
    int64_t sum = 0;
    for (int i = 0; i < N_SAMPLES; i++) sum += adc_buf[i];
    int32_t mean = (int32_t)(sum / N_SAMPLES);

    /* Step 1: difference function */
    for (int tau = 1; tau <= TAU_MAX; tau++) {
        int64_t acc = 0;
        for (int i = 0; i < (N_SAMPLES - tau); i++) {
            int32_t diff = ((int32_t)adc_buf[i]       - mean)
                         - ((int32_t)adc_buf[i + tau] - mean);
            acc += (int64_t)diff * diff;
        }
        d[tau] = (float)acc;
    }

    /* Step 2: cumulative mean normalized difference */
    float run = 0.0f;
    d[0] = 1.0f;
    for (int tau = 1; tau <= TAU_MAX; tau++) {
        run   += d[tau];
        d[tau] = d[tau] * (float)tau / run;
    }

    /* Step 3: first dip below threshold; skip the central lobe (tau < 6) */
    int best_tau = -1;
    for (int tau = 6; tau < TAU_MAX; tau++) {
        if (d[tau] < YIN_THRESHOLD) {
            while (tau + 1 < TAU_MAX && d[tau + 1] < d[tau]) tau++;
            best_tau = tau;
            break;
        }
    }

    int   this_note = -1;
    int   cur_hz    = 0;
    float f_meas    = 0.0f;

    if (best_tau > 0) {
        /* Step 4: parabolic interpolation for fractional tau */
        float refined = (float)best_tau;
        if (best_tau > 1 && best_tau < TAU_MAX) {
            float y0 = d[best_tau - 1], y1 = d[best_tau], y2 = d[best_tau + 1];
            float denom = 2.0f * (y0 - 2.0f * y1 + y2);
            if (denom != 0.0f) refined += (y0 - y2) / denom;
        }
        f_meas = FS_HZ / refined;
        cur_hz = (int)(f_meas + 0.5f);

        /* Step 5: nearest note */
        int   bn = 0;
        float bd = fabsf(f_meas - note_exact_hz(0));
        for (int i = 1; i < NUM_NOTES; i++) {
            float dd = fabsf(f_meas - note_exact_hz(i));
            if (dd < bd) { bd = dd; bn = i; }
        }
        this_note = bn;
    }

    /*
     * Display hold logic:
     *   - Any pitched frame shows its note immediately with no lag.
     *   - A note change takes effect on the very next pitched frame.
     *   - Silence only clears the display after RELEASE_FRAMES consecutive
     *     unpitched frames, preventing single-dropout flicker.
     */
    if (best_tau > 0) {
        silence_count  = 0;
        displayed_note = this_note;
    } else {
        if (silence_count < RELEASE_FRAMES) silence_count++;
        if (silence_count >= RELEASE_FRAMES) displayed_note = -1;
    }

    int   target_note = (displayed_note >= 0) ? displayed_note : this_note;
    float cents = (target_note >= 0)
                  ? 1200.0f * log2f(f_meas / note_exact_hz(target_note))
                  : 0.0f;

    render_display(displayed_note, (best_tau > 0), cents, cur_hz);
}

void DMA1_Channel1_IRQHandler(void)
{
    DMA1_IFCR = (1u << 1);   /* clear transfer-complete flag (write-1-to-clear) */
    adc_ready = 1;
}

/* ------------------------------------------------------------------ */
/*  Entry point                                                         */
/* ------------------------------------------------------------------ */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* PB0 -> analog input for ADC1 channel 15 */
    RCC_AHB2ENR  |= (1u << 1);
    GPIOB_MODER   = (GPIOB_MODER & ~(0x3u << 0)) | (0x3u << 0);
    GPIOB_PUPDR  &= ~(0x3u << 0);

    /* ADC clock: CKMODE = HCLK/2 */
    RCC_AHB2ENR |= (1u << 13);
    ADC_CCR = (ADC_CCR & ~(0x3u << 16)) | (0x2u << 16);

    /* ADC voltage regulator: enable and wait >10 us */
    ADC1_CR = 0;
    ADC1_CR |= (1u << 28);
    for (volatile int i = 0; i < 640; i++);

    /* Calibration (single-ended) */
    ADC1_CR &= ~(1u << 30);
    ADC1_CR |=  (1u << 31);
    while (ADC1_CR & (1u << 31));

    /*
     * CFGR must be written before ADEN (RM0394 §16.4.2):
     *   EXTSEL = 0xD (TIM6_TRGO), EXTEN = 01 (rising edge), DMAEN + DMACFG (circular)
     */
    ADC1_CFGR = (0xDu << 6) | (0x1u << 10) | 0x3u;

    /* Enable ADC and wait for ADRDY */
    ADC1_CR |= 1u;
    while (!(ADC1_ISR & 1u));

    /* Channel 15 (PB0), SMP = 001 (2.5 ADC clock cycles) */
    ADC_SMPR2 = (0x1u << 15);
    ADC_SQR1  = (15u << 6);

    /* TIM6: 32 MHz / 2000 = 16 kHz, TRGO on update event */
    RCC_APB1ENR1 |= (1u << 4);
    TIM6_PSC = 0;
    TIM6_ARR = 1999;
    TIM6_CR2 = (TIM6_CR2 & ~(0x7u << 4)) | (0x2u << 4);
    TIM6_CR1 |= 1u;

    /* DMA1 channel 1: ADC_DR -> adc_buf, 16-bit, circular, TC interrupt */
    RCC_AHB1ENR |= 1u;
    DMA1_CPAR1  = (uint32_t)&ADC1_DR;
    DMA1_CMAR1  = (uint32_t)adc_buf;
    DMA1_CNDTR1 = N_SAMPLES;
    DMA1_CCR1   = (1u << 1)    /* TCIE  */
                | (1u << 5)    /* CIRC  */
                | (1u << 7)    /* MINC  */
                | (1u << 8)    /* PSIZE = 16-bit */
                | (1u << 10);  /* MSIZE = 16-bit */
    DMA1_CCR1  |= 1u;          /* EN */

    /* Start ADC conversions (after ADRDY is confirmed) */
    ADC1_CR |= (1u << 2);

    /* NVIC: DMA1_Channel1 = IRQ11, highest priority */
    *(volatile uint32_t *)0xE000E408UL = 0x00000000UL;
    NVIC_ISER0 = (1u << 11);

    MX_GPIO_Init();
    MX_USART2_UART_Init();

    /* I2C1 GPIO: PB6 = SCL, PB7 = SDA, AF4, open-drain */
    RCC_AHB2ENR |= (1u << 1);

    GPIOB_MODER &= ~((0x3u << (6*2)) | (0x3u << (7*2)));
    GPIOB_MODER |=  ((0x2u << (6*2)) | (0x2u << (7*2)));

    GPIOB_OTYPER |= (1u << 6) | (1u << 7);

    GPIOB_OSPEEDR &= ~((0x3u << (6*2)) | (0x3u << (7*2)));
    GPIOB_OSPEEDR |=  ((0x3u << (6*2)) | (0x3u << (7*2)));

    GPIOB_PUPDR &= ~((0x3u << (6*2)) | (0x3u << (7*2)));

    GPIOB_AFRL &= ~((0xFu << (6*4)) | (0xFu << (7*4)));
    GPIOB_AFRL |=  ((0x4u << (6*4)) | (0x4u << (7*4)));   /* AF4 = I2C1 */

    /* I2C1 peripheral: 100 kHz @ 32 MHz PCLK1 */
    RCC_APB1ENR1 |= (1u << 21);
    I2C1_CR1 &= ~(1u << 0);
    I2C1_TIMINGR = 0x10909CECu;
    I2C1_CR1 |=  (1u << 0);

    uart_log("\r\n=== TUNER BOOT ===\r\n");
    lcd_init();
    lcd_create_char(1, GLYPH_TRACK);
    lcd_create_char(2, GLYPH_TICK);
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Tuner ready");
    uart_log_hex("I2C1_ISR after init: ", I2C1_ISR);
    uart_log("=== BOOT DONE ===\r\n");

    while (1) {
        if (adc_ready) {
            adc_ready = 0;
            process_buffer();
        }
    }
}

/* ------------------------------------------------------------------ */
/*  HAL-generated boilerplate (clock, UART, GPIO)                      */
/* ------------------------------------------------------------------ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        Error_Handler();

    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState            = RCC_LSE_ON;
    RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM            = 1;
    RCC_OscInitStruct.PLL.PLLN            = 16;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
        Error_Handler();

    HAL_RCCEx_EnableMSIPLLMode();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance                    = USART2;
    huart2.Init.BaudRate               = 115200;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = LD3_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
