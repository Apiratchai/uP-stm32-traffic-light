an stm32 program written in C using the above defined GPIOs and TX, RX to….

define GPIOA5 = blue light
define GPIOA6 = red light
define GPIOA7 = yellow light
define GPIOB6 = green light

define GPIOA10 = button for car_count increase 
define GPIOB3 = button for car_count decrease
define GPIOB5 = release car (button to change lane)

define GPIOB4 = emergency mode (not used in this code)

define lane1, lane2
define car_count
define n = 0, t = 3
define car_count = 0

loop:
    current_lane = 1 + (n mod 2)
    other_lane = 1 + ((n+1) mod 2)
    
    // Green phase
    green light on current_lane
    red light on other_lane
    delay for t seconds
    
    // Yellow phase  
    yellow light on current_lane for 1 second
    
    // Check traffic for next cycle
    check car_count in other_lane
    if car_count = 0:
        t = 1  // Short cycle if no cars waiting
    else:
        t = max(3, car_count)  // Longer cycle based on traffic, minimum 3 seconds
    
    wait until GPIOB5 is high (button press)
    n = n + 1
end loop

interrupt on button press:
    if button on GPIOA10 pressed:
        car_count = car_count + 1  // Increment car count in other lane
    if button on GPIOB3 pressed:
        car_count = max(0, car_count - 1)  // Decrement car count, not below 0
    if button on GPIOB5 pressed:
        t = 3  // Reset time to default
end interrupt

interrupt on emergency mode (GPIOB4 high):
    turn on only yellow light on both lanes
    blink yellow lights every 0.5 seconds
    until GPIOB4 is low (emergency mode off)
end interrupt

the logic should also count from 9 t 0 using the
void Display_Number

it only counts for the lane that is currently red light
since there is only one set of traffic light, it will always be used for the lane1

car_count is logically maxed out at 0-9

