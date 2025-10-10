# uP-stm32-traffic-light
Implementation of a simple traffic light system using STM32 Nucleo.

to study how well would a generative AI can assist in low level microcontroller programming with provided datasheets

- vibe coded with Antropic's Cluade Sonnet 4
- there is  no implementation of RGB SMD yet


### Onboards
- [X] Buttons
- [X] LED_builtin
- [X] 7-Segments
- [X] Temp
- [X] Light
- [X] Potentiometer

### Add-ons
- [X] SMD RGB
- [X] Passive Buzzer
- [X] DC motor fan + relay
- [ ] External LED (police post light)

### Todolist
- [X] interrupt emergency mode
      - [X] with buzzer sound (PWM freq is possible)

- [X] Coolterms
  - [X] Show Temp, Light
  - [X] Show car_counts
- [X] Temp (low = do nothing, high = drive DC motor small fan)
- [ ] Light (low = LED_ON, high = LED_OFF)
- [X] Time adjust  (THRESHOLD adjust, interrupt maybe) with Potentiometer

