# STM32 Traffic Light Controller with 7-Segment Display - Complete Prompt

## Hardware Pin Definitions
```
// Traffic Lights (Single Lane System)
GPIOA5  = Blue Light (unused in single lane setup)
GPIOA6  = Red Light (Lane 1)
GPIOA7  = Yellow Light (Lane 1)  
GPIOB6  = Green Light (Lane 1)

// 7-Segment Display
GPIOC7  = Display bit 0 (2^0)
GPIOA8  = Display bit 1 (2^1)
GPIOA9  = Display bit 3 (2^3)
GPIOB10 = Display bit 2 (2^2)

// Control Buttons (All Active LOW with pull-up resistors)
GPIOA10 = Car Count Increase Button
GPIOB3  = Car Count Decrease Button  
GPIOB5  = Release Cars Button
GPIOB4  = Emergency Mode Toggle (Active HIGH)

// Communication
PA2 = USART2 TX (AF7, 115200 baud)
PA3 = USART2 RX (AF7, 115200 baud)
```

## Global Variables & Constants
```c
#define THRESHOLD 1333333  // 1 second delay constant
volatile uint32_t car_count = 0;        // Cars waiting (0-9 max)
volatile uint32_t t = 3;                // Current green light duration
volatile uint8_t release_car_pressed = 0;   // Manual release flag

// Button edge detection variables
uint8_t prev_PA10 = 1, prev_PB3 = 1, prev_PB5 = 1, prev_PB4 = 0;
```

## Core Functions Required

### 1. Hardware Initialization
```c
void GPIO_Init(void) {
    // Configure 7-segment display outputs (PC7, PA8, PA9, PB10)
    // Configure traffic light outputs (PA6 red, PA7 yellow, PB6 green)
    // Configure button inputs with pull-up resistors (PA10, PB3, PB5)
    // Configure emergency input without pull-up (PB4)
}

void USART2_Init(void) {
    // Initialize USART2 on PA2/PA3 at 115200 baud, AF7
}
```

### 2. Display Control
```c
void Display_Number(char rx) {
    // Clear all display bits first
    // Set appropriate combination for digits 0-9:
    // '0': 0000, '1': 0001, '2': 0010, '3': 0011, '4': 0100
    // '5': 0101, '6': 0110, '7': 0111, '8': 1000, '9': 1001
}

void countdown_display(uint32_t seconds) {
    // Count down from 'seconds' to 0, displaying each number for 1 second
    // Check for emergency mode during each second
    // Clear display (show '0') when finished
}
```

### 3. Traffic Light Control
```c
void turn_off_all_lights(void);     // Clear PA6, PA7, PB6
void set_green_light(void);         // Turn on PB6, clear others
void set_red_light(void);           // Turn on PA6
void set_yellow_light(void);        // Turn on PA7
```

### 4. Delay Functions
```c
void delay_seconds(uint32_t seconds);   // Uses THRESHOLD constant
void delay_ms(uint32_t ms);            // Uses THRESHOLD/1000
// Both functions must check emergency mode during delay and exit early if activated
```

## Main Program Logic

### Initialization
```c
int main(void) {
    USART2_Init();
    GPIO_Init();
    
    car_count = 0;
    t = 3;
    turn_off_all_lights();
    Display_Number('0');
    
    // Main loop...
}
```

### Main Control Loop
```
while(1):
    // === BUTTON HANDLING (Edge Detection Pattern) ===
    Read current button states (curr_PA10, curr_PB3, curr_PB5, curr_PB4)
    
    // Car Count Increase (PA10 falling edge)
    if (prev_PA10 == 1 && curr_PA10 == 0):
        car_count = (car_count + 1) % 10      // Wrap at 9→0
    
    // Car Count Decrease (PB3 falling edge)  
    if (prev_PB3 == 1 && curr_PB3 == 0):
        car_count = (car_count == 0) ? 9 : (car_count - 1)   // Wrap at 0→9
    
    // Release Cars (PB5 falling edge)
    if (prev_PB5 == 1 && curr_PB5 == 0):
        release_car_pressed = 1
        t = 3                                 // Reset timing
    
    Update previous button states
    
    // === EMERGENCY MODE HANDLING ===
    if (PB4 is HIGH):
        turn_off_all_lights()
        while (PB4 remains HIGH):
            turn_off_all_lights()
            set_yellow_light()
            Display_Number('E')               // Emergency pattern
            delay_ms(500)
            turn_off_all_lights()  
            Display_Number('0')
            delay_ms(500)
        turn_off_all_lights()
        continue to start of main loop
    
    // === TRAFFIC LIGHT SEQUENCE ===
    
    // GREEN PHASE
    set_green_light()
    Display_Number('0' + car_count)           // Show current car count
    delay_seconds(t)
    if emergency_activated: continue
    
    // YELLOW PHASE  
    turn_off_all_lights()
    set_yellow_light()
    Display_Number('0')                       // Clear display
    delay_seconds(1)
    if emergency_activated: continue
    
    // RED PHASE
    turn_off_all_lights()
    set_red_light()
    if (car_count > 0):
        countdown_display(car_count)          // Count down from car_count to 0
    else:
        Display_Number('0')                   // Show 0 if no cars
    
    // === TIMING LOGIC ===
    if (car_count == 0):
        t = 1                                 // Quick cycle if no cars
    else:
        t = max(3, car_count)                 // Min 3 seconds, or 1 sec per car
        
        // Wait for manual release when cars are waiting
        release_car_pressed = 0
        while (!release_car_pressed && !emergency_mode):
            Check for PB5 button press
            delay_ms(10)                      // Prevent busy waiting
```

## Key Behavioral Requirements

### Display Behavior
- **Green Phase**: Shows number of cars waiting (0-9)
- **Yellow Phase**: Shows 0 (cleared)  
- **Red Phase**: Counts down from car_count to 0 (e.g., 5→4→3→2→1→0)
- **Emergency**: Alternates between 'E' pattern and 0 during yellow blink
- **Car Count Limits**: Wraps between 0-9 (9+1=0, 0-1=9)

### Traffic Light Timing
- **Default Green Time**: 3 seconds
- **Yellow Time**: 1 second (fixed)
- **Red Time**: Equal to car_count (1 second per car, countdown display)
- **No Cars Green**: 1 second (quick cycle)
- **Emergency Blink**: 500ms on/off yellow light

### Button Behavior
- **All buttons use edge detection** (prev/curr state comparison)
- **Car Count +/-**: Immediate effect, wraps at 0-9 bounds
- **Release Cars**: Sets flag, resets timing to default (t=3)
- **Emergency**: Overrides all operations, returns to normal when released

### Emergency Mode
- **Activation**: PB4 goes HIGH
- **Behavior**: Continuous yellow blinking at 1Hz, display shows 'E'/0 pattern
- **Deactivation**: PB4 goes LOW, returns to normal traffic sequence
- **Priority**: Overrides all other operations, checked during all delays

## Code Style Requirements
- Use direct register manipulation (ODR, IDR, MODER)
- Polling-based approach (no interrupts)
- Edge detection for all buttons using prev/curr pattern
- Volatile delay loops using THRESHOLD constant
- Check emergency mode during all delays
- Short variable names (curr_, prev_, t, n, etc.)
- Minimal comments in practical style

## Integration Notes
- Code must work with existing USART2_Init() and Display_Number() functions
- Maintains same GPIO configuration patterns as existing codebase
- Uses same delay methodology with THRESHOLD constant
- Follows established coding style and register access patterns