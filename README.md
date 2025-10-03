# uP-stm32-traffic-light
Implementation of a simple traffic light system using STM32 Nucleo.

to study how well would a generative AI can assist in low level microcontroller programming with provided datasheets

- vibe coded with Antropic's Cluade Sonnet 4
- there is  no implementation of RGB SMD yet


### Onboards
- [X] Buttons
- [X] LED_builtin
- [X] 7-Segments
- [ ] Temp
- [ ] Light
- [ ] Potentiometer

### Add-ons
- [X] SMD
- [ ] Buzzer
- [ ] DC motor fac

### Todolist
- [ ] interrupt emergency mode
      - [ ] with buzzer sound (PWM freq is possible)

- [ ] Coolterms
  - [ ] Show Temp, Light
  - [ ] Show car_counts
- [ ] Temp (low = do nothing, high = drive DC motor small fan)
- [ ] Light (low = LED_ON, high = LED_OFF)
- [ ] Time adjust  (THRESHOLD adjust, interrupt maybe) with Potentiometer

