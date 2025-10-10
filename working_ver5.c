/**
 **************************************************************
 * @file            : main.c
 * @author          : You
 * @brief           : RGB PWM, Traffic Logic, Buzzer, UART, and Emergency Mode
 **************************************************************
 */
#define STM32F411xE
#include <stdint.h>
#include <stdio.h> // Required for sprintf
#include <math.h>
#include "stm32f4xx.h"

/* ===== ADC constants ===== */
#define VREF        3.3f
#define VCC         3.3f
#define ADC_MAXRES  4095.0f
/* NTC@PA0 (ADC1_IN0) */
#define RX_THERM    10000.0f
#define R0_NTC      10000.0f
#define T0_KELVIN   298.15f
#define BETA_NTC    3950.0f

#define RX_LDR      10000.0f
#define SLOPE       (-0.6875f)
#define OFFSET      5.1276f

#define FAN_ON_mC   27000   // 26.0°C (ปรับให้ต่ำลงเพื่อทดสอบ)
#define FAN_OFF_mC  25000   // 25.0°C

// --- Global Variables ---
volatile uint32_t THRESHOLD = 1333333u; // This value is calibrated by the potentiometer
#define MAX_BRIGHT 1000
volatile uint32_t car_count[2] = {0,0};
char stringOut[100];
volatile uint8_t is_emergency_mode = 0;
volatile uint32_t n = 0;
volatile uint32_t t = 0;
volatile int32_t g_temp_mC = 0; // อุณหภูมิ
volatile uint16_t lux = 0;

/* ===== เปิด FPU ===== */
static inline void fpu_enable(void){
    SCB->CPACR |= (0xFU << 20);
    __asm volatile("dsb");
    __asm volatile("isb");
}

static inline void EnableClocks(void)
{
    /* AHB1: GPIO */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN
                 |  RCC_AHB1ENR_GPIOBEN
                 |  RCC_AHB1ENR_GPIOCEN;

    /* APB1: USART2, TIM3 */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN
                 |  RCC_APB1ENR_TIM3EN;

    /* APB2: ADC1, TIM1, (SYSCFG เผื่อใช้ EXTI ภายหลัง) */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN
                 |  RCC_APB2ENR_TIM1EN
                 |  RCC_APB2ENR_SYSCFGEN;
}


/* ===== ADC: PA0 เฉพาะช่อง 0 ===== */
static inline void adc_init_pa0(void){
    GPIOA->MODER |= (3U<<(2*0));          /* PA0 analog */
    ADC1->CR2  |= ADC_CR2_ADON;
    ADC1->SMPR2 &= ~(7U<<(3*0));
    ADC1->SMPR2 |=  (6U<<(3*0));          /* sample time ยาวให้ค่านิ่ง */
    ADC1->SQR1 &= ~ADC_SQR1_L;            /* 1 conversion */
    ADC1->SQR3  = 0;                      /* ch0 */
}
static inline uint16_t adc_read_raw_ch0(void){
    ADC1->SR = 0;                          // clear flags
    ADC1->SQR3 = 0;                        // ch0
    ADC1->CR2 |= ADC_CR2_SWSTART;
    uint32_t to = 100000;
    while ((ADC1->SR & ADC_SR_EOC) == 0){
        if(--to == 0) return 0;            // กันค้าง
    }
    uint16_t d = (uint16_t)ADC1->DR;       // read DR เคลียร์ EOC
    (void)ADC1->SR;                         // sync read
    return d;
}

static inline float adc_read_voltage_ch0(void){
    return (adc_read_raw_ch0() * VREF) / ADC_MAXRES;
}

/* ===== คำนวณอุณหภูมิ (มิลลิองศาเซลเซียส) ===== */
static inline int32_t read_temperature_millic(void){
    float v = adc_read_voltage_ch0();
    if (v <= 0.0005f) v = 0.0005f;
    if (v >= (VCC - 0.0005f)) v = VCC - 0.0005f;

    /* วงจรแบ่งแรงดัน: Vout = VCC * Rntc / (Rntc + RX_THERM) -> Rntc = v*RX/(VCC - v) */
    float r_ntc = (v * RX_THERM) / (VCC - v);

    /* Beta equation */
    float invT  = (1.0f / T0_KELVIN) + (1.0f / BETA_NTC) * logf(r_ntc / R0_NTC);
    float tempC = (1.0f / invT) - 273.15f;

    return (int32_t)(tempC * 1000.0f);
}


void check_temp_for_fan(void) {
    static uint8_t fan_on = 0;
    g_temp_mC = read_temperature_millic();

    // เปิดเมื่อสูงกว่า 27°C
    if (!fan_on && g_temp_mC >= 27000) {
        GPIOB->BSRR = (1u << 7);      // เปิดพัดลม
        fan_on = 1;
    }
    // ปิดเมื่อเย็นต่ำกว่า 25°C
    else if (fan_on && g_temp_mC <= 26000) {
        GPIOB->BSRR = (1u << (7 + 16)); // ปิดพัดลม
        fan_on = 0;
    }
}


/* ===== ADC: only PA1 analog ===== */
static inline void adc_init_pa1(void){
    /* PA1 analog */
    GPIOA->MODER |= (3U<<(2*1));
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->SMPR2 &= ~(7U<<(3*1));
    ADC1->SMPR2 |=  (6U<<(3*1));
    ADC1->SQR1 &= ~ADC_SQR1_L;   /* length=1 */
    ADC1->SQR3  = 1;             /* ch1      */
}
static inline uint16_t adc_read_raw_ch1(void){
    ADC1->SQR3 = 1;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while ((ADC1->SR & ADC_SR_EOC) == 0) {
        /* wait */
    }
    return (uint16_t)(ADC1->DR & 0xFFFF);
}

static inline float adc_read_voltage_ch1(void){
    return ((float)adc_read_raw_ch1() * VREF) / ADC_MAXRES;
}

static inline uint16_t read_light_lux(void){
    float v = adc_read_voltage_ch1();
    if (v <= 0.0005f) v = 0.0005f;
    if (v >= (VREF - 0.0005f)) v = VREF - 0.0005f;

    float r_ldr = RX_LDR * v / (VREF - v);
    float lux   = powf(10.0f, (log10f(r_ldr) - OFFSET) / SLOPE);

    if (lux < 0.0f) {
        lux = 0.0f;
    } else if (lux > 65535.0f) {
        lux = 65535.0f;
    }
    return (uint16_t)(lux + 0.5f);
}


void check_light(void){
    lux = read_light_lux();
    if (lux < 100){
    	lux = 0;
    }
    else {
    	lux = 0; // turn off light
    }
}


// --- Forward Declarations ---
void check_emergency_button(void);


// --- Helper Functions ---
static inline uint16_t clamp_u16(uint16_t v, uint16_t hi) {
    return (v > hi) ? hi : v;
}

// * NEW DELAY FUNCTION FOR EMERGENCY MODE *
void responsive_threshold_delay(uint32_t duration_loops) {
    const uint32_t check_interval = 1024;

    for (volatile uint32_t i = 0; i < duration_loops; i++) {
        if ((i & (check_interval - 1)) == 0) {
            check_emergency_button();
            check_temp_for_fan();  // เพิ่มการตรวจสอบอุณหภูมิ
            if (is_emergency_mode == 0) {
                return;
            }
        }
    }
}

// Original delay function for countdowns
void delay_ms(uint32_t ms) {
    volatile uint32_t count_for_1ms = THRESHOLD / 1000;
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < count_for_1ms; j++) {}
        check_emergency_button();
        if (is_emergency_mode) return;
    }
}


// --- All other functions (USART, GPIO_Init, PWM_Init, etc.) ---
void USART2_Init(void) {
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER2_Pos) | (0b10 << GPIO_MODER_MODER3_Pos);
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
    GPIOA->AFR[0] |= (0b0111 << GPIO_AFRL_AFSEL2_Pos) | (0b0111 << GPIO_AFRL_AFSEL3_Pos);
    USART2->CR1 = 0; USART2->CR1 |= USART_CR1_UE; USART2->CR1 &= ~USART_CR1_M;
    USART2->CR2 &= ~USART_CR2_STOP; USART2->BRR = 139; USART2->CR1 |= USART_CR1_TE;
    USART2->CR1 |= USART_CR1_RE;
}
void vdg_UART_TxString(char strOut[]) {
    for (uint8_t idx = 0; strOut[idx] != '\0'; idx++) {
        while((USART2->SR & USART_SR_TXE) == 0);
        USART2->DR = strOut[idx];
    }
}
void log_car_counts(void) {
    sprintf(stringOut, "Lane 1 Cars: %lu, Lane 2 Cars: %lu\r\n", (unsigned long)car_count[0], (unsigned long)car_count[1]);
    vdg_UART_TxString(stringOut);
}
void log_emergency_on(void) {
    sprintf(stringOut, "EMERGENCY MODE ACTIVATED. Resetting car counts.\r\n");
    vdg_UART_TxString(stringOut);
}
void log_emergency_off(void) {
    sprintf(stringOut, "Emergency mode deactivated. Resuming normal operation.\r\n");
    vdg_UART_TxString(stringOut);
}
void log_remaining_cars(int lane_idx) {
    sprintf(stringOut, "Lane %d cycle finished. Remaining Lane 1 Cars: %lu, Lane 2 Cars: %lu\r\n", lane_idx + 1, (unsigned long)car_count[0], (unsigned long)car_count[1]);
    vdg_UART_TxString(stringOut);
}

void log_temp(void) {
    int32_t temp_mC = g_temp_mC;
    int32_t degrees = temp_mC / 1000;
    int32_t milli_part = abs(temp_mC % 1000);
    sprintf(stringOut, "Temp: %d.%03d C\r\n", degrees, milli_part);
    vdg_UART_TxString(stringOut);
}

void GPIO_Init(void) {
    GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER8_Pos) | (0b01 << GPIO_MODER_MODER9_Pos);
    GPIOB->MODER &= ~(GPIO_MODER_MODER10);
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODER10_Pos);
    GPIOC->MODER &= ~(GPIO_MODER_MODER7);
    GPIOC->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);
    GPIOA->MODER &= ~(GPIO_MODER_MODER10);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD10);
    GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPD10_Pos);
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD5);
    GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD3_Pos) | (0b01 << GPIO_PUPDR_PUPD5_Pos);
    GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER5_Pos) | (0b01 << GPIO_MODER_MODER6_Pos) | (0b01 << GPIO_MODER_MODER7_Pos);
    GPIOB->MODER &= ~(GPIO_MODER_MODER6);
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODER6_Pos);

    // ===== PB7 สำหรับควบคุมพัดลม - เพิ่มการตั้งค่าครบถ้วน =====
    // 1. ตั้งเป็น Output mode
    GPIOB->MODER &= ~(GPIO_MODER_MODER7);
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);

    // 2. ตั้ง Output Speed เป็น High Speed (50MHz)
    GPIOB->OSPEEDR &= ~(0b11 << (7 * 2));        // เคลียร์ 2 บิต
    GPIOB->OSPEEDR |= (0b10 << (7 * 2));         // 10 = High speed

    // 3. ตั้ง Output Type เป็น Push-Pull
    GPIOB->OTYPER &= ~(1 << 7);                  // 0 = Push-pull

    // 4. ตั้ง Pull-down เพื่อให้พัดลมปิดตอนเริ่มต้น
    GPIOB->PUPDR &= ~(0b11 << (7 * 2));          // เคลียร์
    GPIOB->PUPDR |= (0b10 << (7 * 2));           // 10 = Pull-down

    // 5. ตั้งค่าเริ่มต้นให้เป็น LOW (พัดลมปิด)
    GPIOB->BSRR = (1u << (7 + 16));              // Reset PB7 = LOW
}

void PWM_Init(void) {
    GPIOB->MODER &= ~((3U<<(15*2)) | (3U<<(14*2)) | (3U<<(13*2)) | (3U<<(1*2)));
    GPIOB->MODER |=  (2U<<(15*2)) | (2U<<(14*2)) | (2U<<(13*2)) | (2U<<(1*2));
    GPIOB->OSPEEDR &= ~((3U<<(15*2)) | (3U<<(14*2)) | (3U<<(13*2)) | (3U<<(1*2)));
    GPIOB->OSPEEDR |=  (2U<<(15*2)) | (2U<<(14*2)) | (2U<<(13*2)) | (2U<<(1*2));
    GPIOB->PUPDR   &= ~((3U<<(15*2)) | (3U<<(14*2)) | (3U<<(13*2)) | (3U<<(1*2)));
    GPIOB->AFR[1] &= ~((0xFU<<((13-8)*4)) | (0xFU<<((14-8)*4)) | (0xFU<<((15-8)*4)));
    GPIOB->AFR[1] |=  ((0x1U<<((13-8)*4)) | (0x1U<<((14-8)*4)) | (0x1U<<((15-8)*4)));
    GPIOB->AFR[0] &= ~(0xFU<<(1*4));
    GPIOB->AFR[0] |=  (0x2U<<(1*4));
    TIM1->CR1 = 0; TIM1->PSC = 84 - 1; TIM1->ARR = MAX_BRIGHT - 1;
    TIM1->CCMR1 &= ~(7U << 4); TIM1->CCMR1 |= (6U << 4); TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM1->CCMR1 &= ~(7U << 12); TIM1->CCMR1 |= (6U << 12); TIM1->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM1->CCMR2 &= ~(7U << 4); TIM1->CCMR2 |= (6U << 4); TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
    TIM1->CCER = 0; TIM1->CCER |= TIM_CCER_CC1NE; TIM1->CCER |= TIM_CCER_CC2NE; TIM1->CCER |= TIM_CCER_CC3NE;
    TIM1->BDTR = TIM_BDTR_MOE; TIM1->CR1 |= TIM_CR1_ARPE; TIM1->EGR  = TIM_EGR_UG; TIM1->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 = 0; TIM3->PSC = 84 - 1; TIM3->ARR = MAX_BRIGHT - 1;
    TIM3->CCMR2 &= ~(7U << 12); TIM3->CCMR2 |= (6U << 12); TIM3->CCMR2 |= TIM_CCMR2_OC4PE;
    TIM3->CCR4 = 0; TIM3->CCER = 0; TIM3->CCER |= TIM_CCER_CC4E;
    TIM3->CR1 |= TIM_CR1_ARPE; TIM3->EGR  = TIM_EGR_UG; TIM3->CR1 |= TIM_CR1_CEN;
}
static inline void Set_RGB(uint16_t R, uint16_t G, uint16_t B) {
    TIM1->CCR3 = clamp_u16(R, TIM1->ARR); TIM1->CCR2 = clamp_u16(G, TIM1->ARR); TIM3->CCR4 = clamp_u16(B, TIM3->ARR);
}
void ADC1_POT_Init(void) {
    GPIOA->MODER |= (3U << (2*4));
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->SMPR2 &= ~ADC_SMPR2_SMP4;
    ADC1->SMPR2 |=  (6U << ADC_SMPR2_SMP4_Pos);
    ADC1->SQR1 &= ~ADC_SQR1_L;
}

static inline uint16_t adc_read_raw_ch4(void){
    ADC1->SQR3 = 4;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while((ADC1->SR & ADC_SR_EOC)==0){}
    return (uint16_t)ADC1->DR;
}

void UpdateThresholdFromADC(void){
    uint32_t adc_value = adc_read_raw_ch4();
    const uint32_t MIN_US = 1500000u, MAX_US = 6000000u;
    THRESHOLD = MIN_US + (uint32_t)(((uint64_t)adc_value * (MAX_US - MIN_US))/4095u);
}


static inline void Buzzer_Off(void) { TIM1->CCR1 = 0; }
static inline void Buzzer_On(uint32_t freq) {
    uint32_t timer_clk = 1000000;
    TIM1->ARR = (timer_clk / freq) - 1; TIM1->CCR1 = TIM1->ARR / 2;
}
void Buzzer_Note(uint32_t freq, uint32_t ms) {
    if (freq == 0) { Buzzer_Off(); delay_ms(ms); return; }
    Buzzer_On(freq); delay_ms(ms); Buzzer_Off(); TIM1->ARR = MAX_BRIGHT - 1;
}

void Display_Number(char rx) {
    GPIOC->ODR &= ~(1 << 7);
    GPIOA->ODR &= ~(1 << 8);
    GPIOA->ODR &= ~(1 << 9);
    GPIOB->ODR &= ~(1 << 10);

    switch (rx) {
        case '0': break;
        case '1': GPIOC->ODR |= (1 << 7); break;
        case '2': GPIOA->ODR |= (1 << 8); break;
        case '3': GPIOA->ODR |= (1 << 8); GPIOC->ODR |= (1 << 7); break;
        case '4': GPIOB->ODR |= (1 << 10); break;
        case '5': GPIOB->ODR |= (1 << 10); GPIOC->ODR |= (1 << 7); break;
        case '6': GPIOA->ODR |= (1 << 8); GPIOB->ODR |= (1 << 10); break;
        case '7': GPIOA->ODR |= (1 << 8); GPIOB->ODR |= (1 << 10); GPIOC->ODR |= (1 << 7); break;
        case '8': GPIOA->ODR |= (1 << 9); break;
        case '9': GPIOA->ODR |= (1 << 9); GPIOC->ODR |= (1 << 7); break;
        default: break;
    }
}

void turn_off_all_lights_lane1(void){ GPIOA->ODR &= ~((1 << 6) | (1 << 7)); GPIOB->ODR &= ~(1 << 6); }
void set_green_light_lane1(void) { turn_off_all_lights_lane1(); GPIOB->ODR |= (1 << 6); }
void set_red_light_lane1(void) { turn_off_all_lights_lane1(); GPIOA->ODR |= (1 << 6); }
void set_yellow_light_lane1(void) { turn_off_all_lights_lane1(); GPIOA->ODR |= (1 << 7); }
static inline void LED_Red(void) { Set_RGB(MAX_BRIGHT, 0, 0); }
static inline void LED_Green(void) { Set_RGB(0, MAX_BRIGHT, 0); }
static inline void LED_Blue(void) { Set_RGB(0, 0, MAX_BRIGHT); }
static inline void LED_Yellow(void) { Set_RGB(MAX_BRIGHT, MAX_BRIGHT/2, 0); }
static inline void LED_Off(void) { Set_RGB(0,0,0); }


uint32_t countdown_display_lane1(uint32_t seconds, uint32_t car_count_lane) {
    if (seconds > 9) seconds = 9;
    uint32_t green_duration = (seconds > 2) ? (seconds - 2) : 0;
    for (int32_t remaining = seconds; remaining >= 0; remaining--) {
        if (is_emergency_mode) return car_count_lane;
        UpdateThresholdFromADC();
        check_temp_for_fan();
        check_light();
        Display_Number('0' + remaining);
        if (car_count_lane > 0) {
            if (remaining > 2) { set_green_light_lane1(); } else if (remaining == 2 ) { set_yellow_light_lane1(); } else { set_red_light_lane1(); }
        } else { set_red_light_lane1(); }
        LED_Red();
        delay_ms(1000);
    }
    if (car_count_lane > 0) return (car_count_lane > green_duration) ? (car_count_lane - green_duration) : 0;
    else return 0;
}
uint32_t countdown_display_lane2(uint32_t seconds, uint32_t car_count_lane) {
    if (seconds > 9) seconds = 9;
    uint32_t green_duration = (seconds > 2) ? (seconds - 2) : 0;
    for (int32_t remaining = seconds; remaining >= 0; remaining--) {
        if (is_emergency_mode) return car_count_lane;
        UpdateThresholdFromADC();
        check_temp_for_fan();
        check_light();
        Display_Number('0' + remaining);
        if (car_count_lane > 0) {
            if (remaining > 2) { LED_Green(); } else if (remaining == 2 ) { LED_Yellow(); } else { LED_Red(); }
        } else { LED_Red(); }
        set_red_light_lane1();
        delay_ms(1000);
    }
    if (car_count_lane > 0) return (car_count_lane > green_duration) ? (car_count_lane - green_duration) : 0;
    else return 0;
}

uint32_t time_check(uint32_t car_count) {
    uint32_t t;
    if (car_count == 0) {
        t = 0;
    } else {
        if (car_count < 10) {
            t = car_count;
        } else {
            t = 9;
        }
    }
    return t;
}

void wait_for_release_button(int lane_to_update) {
    uint8_t prev_PA10 = 1, prev_PB3 = 1;
    while ((GPIOB->IDR & GPIO_IDR_IDR_5) != 0) {
        check_emergency_button();
        if (is_emergency_mode) return;
        UpdateThresholdFromADC();
        check_temp_for_fan();
        check_light();
        uint8_t curr_PA10 = (GPIOA->IDR & GPIO_IDR_IDR_10) ? 1 : 0;
        uint8_t curr_PB3  = (GPIOB->IDR & GPIO_IDR_IDR_3) ? 1 : 0;
        if (prev_PA10 == 1 && curr_PA10 == 0) {
            car_count[lane_to_update] = car_count[lane_to_update] + 1;
            log_car_counts();
        }
        prev_PA10 = curr_PA10;
        if (prev_PB3 == 1 && curr_PB3 == 0) {
            if (car_count[lane_to_update] > 0) { car_count[lane_to_update] = car_count[lane_to_update] - 1; }
            log_car_counts();
        }
        prev_PB3 = curr_PB3;
    }
}
void check_emergency_button(void) {
    static uint8_t prev_PB4_state = 1;
    uint8_t curr_PB4_state = (GPIOB->IDR & GPIO_IDR_IDR_4) ? 1 : 0;
    if (prev_PB4_state == 1 && curr_PB4_state == 0) {
        is_emergency_mode = !is_emergency_mode;
        if (is_emergency_mode) {
            log_emergency_on();
            car_count[0] = 0;
            car_count[1] = 0;
            log_car_counts();
        } else {
            log_emergency_off();
            car_count[0] = 0;
            car_count[1] = 0;
            log_car_counts();
            Buzzer_Off();
            turn_off_all_lights_lane1();
            Set_RGB(0, 0, 0);
            Display_Number('0');
            TIM1->ARR = MAX_BRIGHT - 1;
            n = 0;
            t = 0;
        }
        for(volatile int i=0; i<500000; i++);
    }
    prev_PB4_state = curr_PB4_state;
}

int main(void) {
    EnableClocks();
    fpu_enable();
    USART2_Init();
    GPIO_Init();
    PWM_Init();
    ADC1_POT_Init();
    adc_init_pa0();
    adc_init_pa1();

    turn_off_all_lights_lane1();

    while (1) {
        check_emergency_button();

        if (is_emergency_mode) {
        	UpdateThresholdFromADC();
            check_temp_for_fan();
            check_light();
            Display_Number(' ');
            car_count[0] = 0;
            car_count[1] = 0;

            set_yellow_light_lane1();
            LED_Yellow();
            Buzzer_On(880);
            responsive_threshold_delay(THRESHOLD / 2);
            if (!is_emergency_mode) {
                Buzzer_Off();
                turn_off_all_lights_lane1();
                LED_Off();
                continue;
            }

            turn_off_all_lights_lane1();
            Set_RGB(0, 0, 0);
            Buzzer_Off();
            responsive_threshold_delay(THRESHOLD / 2);

        } else {
         	if (n%2 == 0){
         		GPIOA->ODR |= (1<<5);
             	turn_off_all_lights_lane1();
             	car_count[n%2] = countdown_display_lane1(t+2, car_count[n%2]);
             	if (is_emergency_mode) continue;
             	log_remaining_cars(0);
             	wait_for_release_button(1);
             	if (is_emergency_mode) continue;
             	t=time_check(car_count[(n+1)%2]);
             	n++;
         	}
         	else{
             	GPIOA->ODR &= ~(1 << 5);
         		car_count[n%2] = countdown_display_lane2(t+2, car_count[n%2]);
            	if (is_emergency_mode) continue;
            	log_remaining_cars(0);
            	wait_for_release_button(0);
            	if (is_emergency_mode) continue;
             	t=time_check(car_count[(n+1)%2]);
         		n++;
          	}
        }
    }
}
