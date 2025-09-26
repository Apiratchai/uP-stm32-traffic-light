#define STM32F411xE
#include <stdint.h>
#include "stm32f4xx.h"
#define THRESHOLD 1333333 // 1 seconds

// -------------------- USART --------------------
void USART2_Init(void) {
    // Enable clocks for A, B, C GPIO and USART2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN + RCC_AHB1ENR_GPIOBEN + RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    // PA2 = TX, PA3 = RX (AF7)
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER2_Pos) | (0b10 << GPIO_MODER_MODER3_Pos);
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
    GPIOA->AFR[0] |= (0b0111 << GPIO_AFRL_AFSEL2_Pos) | (0b0111 << GPIO_AFRL_AFSEL3_Pos);
    USART2->CR1 = 0;
    USART2->CR1 |= USART_CR1_UE; // enable USART
    USART2->CR1 &= ~USART_CR1_M; // 8-bit data
    USART2->CR2 &= ~USART_CR2_STOP; // 1 stop bit
    USART2->BRR = 139; // Baud = 115200 @16MHz
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; // enable Tx and Rx
}

char USART2_ReadChar(void) {
    while (!(USART2->SR & USART_SR_RXNE));
    return (char)(USART2->DR & 0xFF);
}

void GPIO_Init(void) {
    // 7-segment display outputs
    GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9); // PA8, PA9 for display
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER8_Pos) | (0b01 << GPIO_MODER_MODER9_Pos);
    GPIOB->MODER &= ~(GPIO_MODER_MODER10); // PB10 for display
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODER10_Pos);
    GPIOC->MODER &= ~(GPIO_MODER_MODER7); // PC7 for display
    GPIOC->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);
    
    // Button inputs (with internal pull-ups)
    GPIOA->MODER &= ~(GPIO_MODER_MODER10); // PA10 - car count increase
    GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPDR10_Pos); // pull-up
    
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5); // PB3, PB4, PB5
    GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPDR3_Pos) | (0b01 << GPIO_PUPDR_PUPDR5_Pos); // pull-up for PB3, PB5
    // PB4 emergency mode - no pull-up (external circuit handles)
    
    // LED outputs - Traffic Lights
    GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7); // PA5 blue, PA6 red, PA7 yellow
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER5_Pos) | (0b01 << GPIO_MODER_MODER6_Pos) | (0b01 << GPIO_MODER_MODER7_Pos);
    
    GPIOB->MODER &= ~(GPIO_MODER_MODER6); // PB6 green
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODER6_Pos);
}

void Display_Number(char rx) {
    // เคลียร์ก่อน (ปิดทุกขา)
    GPIOC->ODR &= ~(1 << 7); // PC7
    GPIOA->ODR &= ~(1 << 8); // PA8
    GPIOA->ODR &= ~(1 << 9); // PA9
    GPIOB->ODR &= ~(1 << 10); // PB10
    
    switch (rx) {
        case '0': break; // 0000 -> ทุกตัวดับ
        case '1': GPIOC->ODR |= (1 << 7); break; // 0001
        case '2': GPIOA->ODR |= (1 << 8); break; // 0010
        case '3': GPIOA->ODR |= (1 << 8); GPIOC->ODR |= (1 << 7); break; // 0011
        case '4': GPIOB->ODR |= (1 << 10); break; // 0100
        case '5': GPIOB->ODR |= (1 << 10); GPIOC->ODR |= (1 << 7); break; // 0101
        case '6': GPIOA->ODR |= (1 << 8); GPIOB->ODR |= (1 << 10); break; // 0110
        case '7': GPIOA->ODR |= (1 << 8); GPIOB->ODR |= (1 << 10); GPIOC->ODR |= (1 << 7); break; // 0111
        case '8': GPIOA->ODR |= (1 << 9); break; // 1000
        case '9': GPIOA->ODR |= (1 << 9); GPIOC->ODR |= (1 << 7); break; // 1001
        default: break;
    }
}
// -------------------- Traffic Light Functions --------------------
void turn_off_all_lights(void){
    GPIOA->ODR &= ~(1 << 5); // blue off
    GPIOA->ODR &= ~(1 << 6); // red off
    GPIOA->ODR &= ~(1 << 7); // yellow off
    GPIOB->ODR &= ~(1 << 6); // green off
}

void set_green_light(void) {
    turn_off_all_lights();
    GPIOB->ODR |= (1 << 6); // PB6 green - Lane 1 green
}

void set_red_light(void) {
    GPIOA->ODR |= (1 << 6); // PA6 red - Lane 1 red
}

void set_yellow_light(void) {
    GPIOA->ODR |= (1 << 7); // PA7 yellow - Lane 1 yellow
}

// -------------------- Countdown Display Function --------------------
uint32_t countdown_display(uint32_t seconds, uint8_t light_status, uint32_t car_count) {
    // ใช้ signed int เพื่อให้วนไปถึง 0 ได้
    for (int32_t remaining = seconds; remaining >= 0; remaining--) {
        // แสดงตัวเลข countdown
        Display_Number('0' + remaining);

        // เปิดไฟตามสถานะ
        if (light_status == 0) {
            set_green_light();
        } else if (light_status == 1) {
            set_yellow_light();
        } else if (light_status == 2) {
            set_red_light();
        }

        // ดีเลย์ประมาณ 1 วินาที
        for (volatile uint32_t i = 0; i < THRESHOLD; i++) {
            // busy-wait 1 วินาที
        }

        // ลด car_count เฉพาะตอนเป็นไฟเขียว
        if (light_status == 0 && car_count > 0) {
            car_count--;
        }
    }
    return car_count;
}

void wait_for_release_button(void) {
    while ( (GPIOB->IDR & (1 << 5)) != 0 ) {
        // วนอยู่ตรงนี้จนกว่าจะกดปุ่ม (ค่าในบิต PB5 = 0)
    }
}



// -------------------- Main Program --------------------
int main(void) {
    USART2_Init(); // cool terms
    GPIO_Init();   // gpios
    
    // Traffic light variables
    volatile uint32_t car_count[2] = {0,0};
    uint32_t n = 0;
    uint32_t t = 3; // default green time
    uint8_t release_car_pressed = 0;
    
    // Button state tracking (for edge detection)
    uint8_t prev_PA10 = 1; // car count increase
    uint8_t prev_PB3 = 1;  // car count decrease  
    uint8_t prev_PB5 = 1;  // release cars
    uint8_t prev_PB4 = 0;  // emergency mode
    
    turn_off_all_lights(); // start with all lights off
    
    while (1) {
        // Read current button states
        uint8_t a = (GPIOA->IDR & (1 << 10)) ? 1 : 0;
        uint8_t curr_PB3 = (GPIOB->IDR & (1 << 3)) ? 1 : 0;
        uint8_t curr_PB5 = (GPIOB->IDR & (1 << 5)) ? 1 : 0;
        uint8_t curr_PB4 = (GPIOB->IDR & (1 << 4)) ? 1 : 0;
        
        // --- Button Handling (Edge Detection) ---
        // PA10: Car Count Increase
        if (prev_PA10 == 1 && curr_PA10 == 0) {
            car_count[n%2] = (car_count[n%2] + 1); 
        }
        prev_PA10 = curr_PA10;
        
        // PB3: Car Count Decrease
        if (prev_PB3 == 1 && curr_PB3 == 0) {
            car_count[n%2] = (car_count[n%2] == 0) ? 0 : (car_count[n%2] - 1);
        }
        prev_PB3 = curr_PB3;
        
        // PB5: Release Cars
        if (prev_PB5 == 1 && curr_PB5 == 0) {
            release_car_pressed = 1;
        prev_PB5 = curr_PB5;
        
        // --- Main Traffic Light Logic ---
        // Single traffic light setup - only Lane 1 exists
        
        // GREEN PHASE - Lane 1 green (no cars waiting, show car count)
        car_count[n%2] = countdown_display(t, 0, car_count[n%2]);
        n++;
        
        // YELLOW PHASE
        turn_off_all_lights();
        car_count[n%2] = countdown_display(1, 1, car_count[n%2]);
        t=car_count[(n+1)%2]
        
        // RED PHASE - Lane 1 red (cars waiting, countdown display)
        turn_off_all_lights();
        car_count[n%2] = countdown_display(t, 2, car_count[n%2]);

        wait_for_release_button();
        turn_off_all_lights();
        
        // --- USART Debug Output ---
        if (USART2->SR & USART_SR_RXNE) {
            char rx = USART2_ReadChar();
            // You can add debug commands here if needed
            // For example: 'r' to reset, 'c' to show car count, etc.
        }
    }
}
}