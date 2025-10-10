# uP-stm32-traffic-light
Implementation of a simple traffic light system using STM32 Nucleo.
- ~~vibe coded with Antropic's Cluade Sonnet 4~~
- vibed coded with Antropic's Cluade Sonnet 4, Google Gemini Pro, OpenAI GPT-4
- with human supervisers and tons of datasheet provided
- ~~there is  no implementation of RGB SMD yet~~


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
  - [X] Show emergency status
- [X] Temp (low = do nothing, high = drive DC motor small fan)
- [ ] Light (low = LED_ON, high = LED_OFF)
- [X] Time adjust  (THRESHOLD adjust, interrupt maybe) with Potentiometer

