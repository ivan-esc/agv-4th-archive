
#include <stdint.h>
#include "stm32f051x8.h"
#include <stdlib.h>
#include <math.h>

volatile uint32_t t = 0;
volatile uint8_t rx_byte = 0;
volatile uint8_t button_state = 0;
volatile uint8_t emergency_on = 0;

volatile int16_t raw_x = 125;
volatile int16_t raw_y = 125;
volatile uint8_t menu_button = 0;
volatile uint8_t slider_x = 100;
volatile uint8_t rx_buffer[5];  // Buffer for 4-byte packet
volatile uint8_t rx_index = 0;  // Track current byte position

volatile int16_t x = 0;
volatile int16_t y = 0;
#define WHEEL_RADIUS      0.0508f   // 5.08 cm
#define WHEEL_DISTANCE    0.35f     // 35 cm
float V_req = 0;
float W_req = 0;
float w_l = 0, w_r = 0;
//avoids killing the hbridge
volatile uint8_t changed_direction = 0;
float last_w_l = 0, last_w_r = 0;
volatile int16_t elevator_duty = 0;
volatile int16_t prev_elevator_duty = 0; //avoids the kill
volatile uint8_t changed_dir_elevator = 0;

/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////SEGUIDOR DE LINEA  ////////////////////////////////////

int VELOCIDAD_BASE = 30;  // Base PWM value (~1 m/s)
float Kp = 15.0f;
float ki = 0.5f;
float kd = 8.0f;

int error_anterior = 0;
float error_integral = 0;
int duty_r = 0;
int duty_l = 0;
#define MAX_INTEGRAL 2000  // Anti-windup limit


///////////////////////////////////////|//////////////////////////////////////////////////////
///////////////////////////////ENCODERS y ultrasonicos  ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

volatile int32_t encoderPos1 = 0;
volatile int32_t encoderPos2 = 0;
volatile uint8_t direction1 = 0, direction2 = 0;
volatile int32_t revolutionsInt1 = 0, revolutionsInt2 = 0;
volatile float revolutionsFloat1 = 0.0f, revolutionsFloat2 = 0.0f;

volatile uint16_t echoStart[4] = {0};
volatile uint16_t echoEnd[4] = {0};
volatile float distance[4] = {0.0f};
uint8_t currentSensor = 0;

const uint8_t trigPins[4] = {0, 6, 8, 14};
const uint8_t echoPins[4] = {10, 7, 9, 13};
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// LUCES LED  ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

// Configuration - adjust based on your setup
#define LED_NUM          99//real 116, debug115  // Number of LEDs in your strip
#define BITS_PER_LED     24    // 24 bits per LED (G-R-B)
#define RESET_SLOTS      64    // 80us reset (64 * 1.25us)
#define PWM_TIMER_FREQ   8000000  // 8 MHz timer clock (adjust if your clock is different)

// Timing parameters (for 800kHz PWM)
#define PWM_PERIOD       (PWM_TIMER_FREQ / 800000) - 1  // 1.25us period (800kHz)
#define HIGH_PULSE       7     // 0.875us high (70% duty)
#define LOW_PULSE        3     // 0.375us high (30% duty)

// LED buffer (24 bits per LED + reset)
static uint16_t pwm_buffer[LED_NUM * BITS_PER_LED + RESET_SLOTS];
static uint8_t led_colors[LED_NUM][3];  // Stores colors in GRB format
static uint8_t last_menu_button = 0;
volatile uint8_t last_proximity_state = 0; // 0 = green, 1 = yellow, 2 = red

//volatile uint8_t dma_busy = 0; // Flag to track DMA state


// Function prototypes
void WS2812_Init(void);
void SetColor(uint8_t led, uint8_t red, uint8_t green, uint8_t blue);
void SendColors(void);
void DMA1_Channel4_5_IRQHandler(void);


//iniciar bluetooth y demas
void GPIOA_Init(void);
void USART2_Init(void);
void TIM14_Init(void);
void pwm_init(void);
void DifferentialDrive(void);
void autoline_init(void);
void autoline(void);
void Elevator(void);
void apply_wheels(int8_t w_r, int8_t w_l);

//funciones dedicadas al utlrasonico y los encoders
void start_hcsr04_trigger(uint8_t sensorIndex);
void update_encoder(uint8_t encoder);

void init_GPIO_EXTI_encoders(void);
void init_GPIO_EXTI_ultrasonic(void);

void emergency_stop(void);
void check_emergency_buttons(void);
void init_emergency_buttons(void);


////////////////////////////////////////////////////////////////////////////
///////////////////////////DECLARACIÓN DE MACROS /////////////////////////
////////////////////////////////////////////////////////////////////////////
#define right_LPWM(duty)  (TIM2->CCR1 = ((duty) * (TIM2->ARR)) / 100);  // PA0 (TIM2_CH1)
#define right_RPWM(duty)  (TIM2->CCR2 = ((duty) * (TIM2->ARR)) / 100);  // PA1 (TIM2_CH2)
#define left_RPWM(duty)   (TIM1->CCR3 = ((duty) * (TIM1->ARR)) / 100);  // PA10 (TIM1_CH3)
#define left_LPWM(duty)   (TIM1->CCR4 = ((duty) * (TIM1->ARR)) / 100);  // PA11 (TIM1_CH4)
#define elevator_LPWM(duty) (TIM1->CCR1 = ((duty) * (TIM1->ARR)) / 100);  // PA8 (TIM1_CH1)
#define elevator_RPWM(duty) (TIM1->CCR2 = ((duty) * (TIM1->ARR)) / 100);  // PA9 (TIM1_CH2)

#define MAX_SAMPLES 600  // 60s @ 50ms/sample

typedef struct {
    int8_t w_r; // Right wheel velocity (signed duty cycle)
    int8_t w_l; // Left wheel velocity
} PwmSample;

volatile PwmSample pwm_log[MAX_SAMPLES];
volatile uint16_t sample_index = 0;
volatile uint16_t playback_index = 0;
volatile uint8_t record_tick = 0;
volatile int8_t curr_w_r = 0;
volatile int8_t curr_w_l = 0;

int main(void)
{

 	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    NVIC->ISER[0] |= (1 << EXTI4_15_IRQn);

    GPIOA_Init();
    USART2_Init();
    WS2812_Init();
    TIM14_Init();
    pwm_init();
    autoline_init();
    init_GPIO_EXTI_encoders();
    init_GPIO_EXTI_ultrasonic();
    init_emergency_buttons();

    NVIC_SetPriority(USART2_IRQn, 0);
    TIM3->PSC = 8 - 1;
	TIM3->ARR = 0xFFFF;
	TIM3->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] |= (1 << TIM3_IRQn);

	    // Example: Set all LEDs
	for (uint8_t i = 0; i < LED_NUM; i++) {
		SetColor(i, 0, 0, 0);  // RGB
	}
    while (1){
    }
}


void TIM14_Init(void) { //10ms Timer Setup
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = 8 - 1;      // 8MHz/8 = 1MHz
    TIM14->ARR = 10000 - 1;  // 10ms period
    TIM14->DIER |= TIM_DIER_UIE;
    TIM14->CR1 |= TIM_CR1_CEN;
	NVIC_SetPriority(TIM14_IRQn, 1);  // Lower value = higher priority
    NVIC_EnableIRQ(TIM14_IRQn);
}
void TIM14_IRQHandler(void) { //10 ms timer
    if(TIM14->SR & TIM_SR_UIF) {
        TIM14->SR &= ~TIM_SR_UIF;  // Clear interrupt flag
        t += 10;
        check_emergency_buttons();
        if (menu_button == 1) {
            DifferentialDrive();
            Elevator();
        } else if (menu_button == 2) {
        	autoline();
        }
        record_tick++;
            if (record_tick >= 5) {  // 50ms
                record_tick = 0;

                if (menu_button == 4 && sample_index < MAX_SAMPLES) {
                	DifferentialDrive();
                	curr_w_r=w_r;
                	curr_w_l=w_l;
                    pwm_log[sample_index].w_r = curr_w_r;
                    pwm_log[sample_index].w_l = curr_w_l;
                    sample_index++;
                }

                else if (menu_button == 5 && playback_index < sample_index) {
                    PwmSample s = pwm_log[playback_index];
                    apply_wheels(s.w_r, s.w_l);
                    curr_w_r = s.w_r;
                    curr_w_l = s.w_l;
                    playback_index++;
                }

                else if (menu_button == 6 && playback_index > 0) {
                    playback_index--;
                    PwmSample s = pwm_log[playback_index];
                    apply_wheels(s.w_r, s.w_l);
                    curr_w_r = s.w_r;
                    curr_w_l = s.w_l;
                }

                else if (menu_button != 4 && menu_button != 5 && menu_button != 6) {
                    sample_index = 0;
                    playback_index = 0;
                }

                if ((menu_button == 5 && playback_index >= sample_index) ||
                    (menu_button == 6 && playback_index == 0)) {
                    apply_wheels(0, 0);  // Stop
                }
            }

        // Detect menu_button change
            if (menu_button != last_menu_button) {
                last_menu_button = menu_button;


				if (menu_button == 1) {
					for (uint8_t i = 0; i < LED_NUM; i++) {
						SetColor(i, 0, 0, 50); // Blue
					}
					SendColors();
				} else if (menu_button == 2) {
					for (uint8_t i = 0; i < LED_NUM; i++) {
						SetColor(i, 0, 50, 0); // Green
					}
					SendColors();
					last_proximity_state = 0; // Reset to green
				}
				else if (menu_button == 4) {
                    // Purple: R=50, B=50
                    for (uint8_t i = 0; i < LED_NUM; i++) {
                        SetColor(i, 50, 0, 50);
                    }
                    SendColors();

                    // Clear previous recording
                    sample_index = 0;
                    playback_index = 0;
                }
                else if (menu_button == 5) {
                    // Forward playback start
                    playback_index = 0;
                    for (uint8_t i = 0; i < LED_NUM; i++) {
                        SetColor(i, 0, 50, 0); // Green
                    }
                    SendColors();
                }
                else if (menu_button == 6) {
                    // Reverse playback start
                    if (sample_index > 0) {
                        playback_index = sample_index;  // <-- Set to END
                    }
                    for (uint8_t i = 0; i < LED_NUM; i++) {
                        SetColor(i, 0, 50, 0); // Green
                    }
                    SendColors();
                }
                else {
                    for (uint8_t i = 0; i < LED_NUM; i++) {
                        SetColor(i, 0, 0, 0); // Off
                    }
                    SendColors();
                }
            }

                // Update LED color only if proximity state changes
                if (menu_button == 2 || menu_button == 5 || menu_button == 6) {
                    uint8_t proximity_state = 0; // 0 = green, 1 = yellow, 2 = red

                    for (uint8_t i = 0; i < 4; i++) {
                        if (distance[i] >= 2.0f && distance[i] <= 26.0f) {
                            proximity_state = 2; // Red
                            right_LPWM(0);
                            right_RPWM(0);
                            left_LPWM(0);
                            left_RPWM(0);
                            break;
                        } else if (distance[i] >= 27.0f && distance[i] <= 100.0f && proximity_state < 1) {
                            proximity_state = 1; // Yellow
                            // Do not break, continue in case red is also found
                        }
                    }

                    if (proximity_state != last_proximity_state) {
                        last_proximity_state = proximity_state;

                        if (proximity_state == 2) {
                            for (uint8_t i = 0; i < LED_NUM; i++) {
                                SetColor(i, 50, 0, 0); // Red
                            }
                        } else if (proximity_state == 1) {
                            for (uint8_t i = 0; i < LED_NUM; i++) {
                                SetColor(i, 50, 50, 0); // Yellow
                            }
                        } else {
                            for (uint8_t i = 0; i < LED_NUM; i++) {
                                SetColor(i, 0, 50, 0); // Green
                            }
                        }
                        SendColors();
                    }
                }

        if (t >= 100) {
			start_hcsr04_trigger(currentSensor);
			currentSensor = (currentSensor + 1) % 4;
			t = 0;
         }

    }
}

void apply_wheels(int8_t w_r, int8_t w_l) {
    // Right motor
    if (w_r >= 0) {
        right_LPWM(w_r);
        right_RPWM(0);
    } else {
        right_LPWM(0);
        right_RPWM(-w_r);
    }

    // Left motor
    if (w_l >= 0) {
        left_LPWM(w_l);
        left_RPWM(0);
    } else {
        left_LPWM(0);
        left_RPWM(-w_l);
    }
}


void DifferentialDrive(void){
	//Logic calculation based on 250x250 map, where 125x125 is defined as center
	//Assume a +-20 deadzone for generous inputs

	 x = raw_x - 125; //Set initial coords to (0,0)
     y = 125- raw_y;
     if (abs(x) < 30){ x = 0;}
     if (abs(y) < 30){ y = 0;}
     //Normalize
     V_req = y/125.0f;
     W_req = x/125.0f;
     //Individual motor speeds (sistema de ecuaciones)
     w_l = ((V_req / WHEEL_RADIUS) - (WHEEL_DISTANCE * W_req / (2.0f * WHEEL_RADIUS))*2)*2;
     w_r = ((V_req / WHEEL_RADIUS) + (WHEEL_DISTANCE * W_req / (2.0f * WHEEL_RADIUS))*2)*2;

     // 3. Direction change detection (including zero-crossing)
	 if ((last_w_r * w_r < 0) || (last_w_l * w_l < 0)) {
		 right_RPWM(0);
		 right_LPWM(0);
		 left_RPWM(0);
		 left_LPWM(0);
		 changed_direction = 1;
	 }
	 else {
		 // 4. Apply PWM if no direction change
		 if (changed_direction == 0) {
			 if (w_r >= 0){
				 right_LPWM(w_r);
				 right_RPWM(0);
			 }
			 else if (w_r < 0){
				 right_RPWM(abs(w_r));  // Note: BTS7960 needs positive duty cycle
				 right_LPWM(0);
			 }
			 if (w_l >= 0){
				 left_LPWM(w_l);
				 left_RPWM(0);
			 }
			 else if (w_l < 0){
				 left_RPWM(abs(w_l));   // Note: Negate for LPWMright_RPWM(0);
				 left_LPWM(0);
			 }
		 }
	 }

	 // 5. Update last speeds and reset flag
	 last_w_r = w_r;
	 last_w_l = w_l;
	 changed_direction = 0;  // Reset after 10ms (next cycle)
}
void autoline_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  // Enable GPIOC clock

    // PC0–PC5 as input (6 sensors)
    GPIOC->MODER &= ~0x00003FFF;  // Clear mode bits for PC0-PC5
    GPIOC->PUPDR &= ~0x00003FFF;  // Clear pull-up/pull-down
    GPIOC->PUPDR |=  0x00002AAA;  // Pull-downs for stability (01 for each pin)

}

int leerError(void) {
    // Weighting for 6 sensors: [-3, -2, -1, 1, 2, 3]
    int pesos[6] = {-3, -2, -1, 1, 2, 3};
    int suma = 0, cuenta = 0;

    for (int i = 0; i < 6; ++i) {
        if (!(GPIOC->IDR & (1 << i))) {  // Sensor active (low)
            suma += pesos[i] * 100;
            cuenta++;
        }
    }

    if (cuenta == 0) {
        // If no sensors are active, return the previous error multiplied by 2
        // This helps the robot recover when it loses the line completely
        return error_anterior * 2;
    }
    return suma / cuenta;  // Weighted average
}

void autoline(void) {
    int error = leerError();

    // Calculate PID terms
    float p_term = Kp * error / 100.0f;

    // Update integral term with anti-windup
    error_integral += error;
    if (error_integral > MAX_INTEGRAL) error_integral = MAX_INTEGRAL;
    if (error_integral < -MAX_INTEGRAL) error_integral = -MAX_INTEGRAL;
    float i_term = ki * error_integral / 100.0f;

    int error_derivativo = error - error_anterior;
    float d_term = kd * error_derivativo / 100.0f;

    int ajuste = (int)(p_term + i_term + d_term);
    error_anterior = error;

    // Calculate motor duties
    duty_r = VELOCIDAD_BASE - ajuste;
    duty_l = VELOCIDAD_BASE + ajuste;

    // Apply non-linear mapping for better response at extremes
    if (duty_r > 100) duty_r = 100 + (duty_r - 100)/2;
    if (duty_r < 0) duty_r = duty_r/2;
    if (duty_l > 100) duty_l = 100 + (duty_l - 100)/2;
    if (duty_l < 0) duty_l = duty_l/2;

    // Final saturation
    duty_r = (duty_r > 150) ? 150 : ((duty_r < -50) ? -50 : duty_r);
    duty_l = (duty_l > 150) ? 150 : ((duty_l < -50) ? -50 : duty_l);

    // Apply PWM values
    if (duty_r > 0) {
        right_LPWM(duty_r/3);
        right_RPWM(0);
    } else {
        right_LPWM(0);
        right_RPWM(-duty_r/3);
    }

    if (duty_l > 0) {
        left_LPWM(duty_l/3);
        left_RPWM(0);
    } else {
        left_LPWM(0);
        left_RPWM(-duty_l/3);
    }
}


void Elevator(void){
	elevator_duty = slider_x - 100;
	if ((prev_elevator_duty * elevator_duty) < 0){
		elevator_LPWM(0);
		elevator_RPWM(0);
	}
	else{
		if(changed_dir_elevator == 0){
			if(elevator_duty >= 0){
				elevator_LPWM(elevator_duty);
				elevator_RPWM(0);
			}
			else if(elevator_duty < 0){
				elevator_RPWM(abs(elevator_duty));
				elevator_LPWM(0);
			}
		}
	}
	prev_elevator_duty = elevator_duty;
	changed_dir_elevator = 0;
}


void pwm_init(void) {
    // --- Enable clocks ---
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;     // GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;     // TIM2 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;     // TIM1 clock

    // --- Configure GPIOA pins PA0, PA1, PA8, PA9, PA10, PA11 to AF mode (AF2) ---
    GPIOA->MODER &= ~((0b11 << 0)  | (0b11 << 2)  | (0b11 << 16) |
                      (0b11 << 18) | (0b11 << 20) | (0b11 << 22));
    GPIOA->MODER |=  ((0b10 << 0)  | (0b10 << 2)  | (0b10 << 16) |
                      (0b10 << 18) | (0b10 << 20) | (0b10 << 22));

    // Push-pull output type
    GPIOA->OTYPER &= ~((1 << 0) | (1 << 1) | (1 << 8) | (1 << 9) | (1 << 10) | (1 << 11));

    // High speed for pins
    GPIOA->OSPEEDR |= ((0b11 << 0)  | (0b11 << 2)  | (0b11 << 16) |
                       (0b11 << 18) | (0b11 << 20) | (0b11 << 22));

    // Set alternate function AF2 (TIM2 and TIM1)
    GPIOA->AFR[0] &= ~((0xF << (0*4)) | (0xF << (1*4))); // Clear PA0, PA1
    GPIOA->AFR[0] |=  ((2 << (0*4)) | (2 << (1*4)));     // AF2

    GPIOA->AFR[1] &= ~((0xF << ((8-8)*4)) | (0xF << ((9-8)*4)) |
                       (0xF << ((10-8)*4)) | (0xF << ((11-8)*4)));
    GPIOA->AFR[1] |=  ((2 << ((8-8)*4)) | (2 << ((9-8)*4)) |
                       (2 << ((10-8)*4)) | (2 << ((11-8)*4))); // AF2

    // --- Configure TIM2 ---
    TIM2->PSC = 8 - 1;        // Prescaler for 1 MHz
    TIM2->ARR = 1000 - 1;     // 1 kHz PWM

    TIM2->CCMR1 &= ~((TIM_CCMR1_OC1M) | (TIM_CCMR1_OC2M));
    TIM2->CCMR1 |= ((6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos));

    TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E); // Enable CH1, CH2

    TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2

    // --- Configure TIM1 ---
    TIM1->PSC = 8 - 1;        // 1 MHz timer clock
    TIM1->ARR = 1000 - 1;     // 1 kHz PWM

    // Enable PWM mode 1 on CH1, CH2, CH3, CH4
    TIM1->CCMR1 &= ~((TIM_CCMR1_OC1M) | (TIM_CCMR1_OC2M));
    TIM1->CCMR1 |= ((6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos));

    TIM1->CCMR2 &= ~((TIM_CCMR2_OC3M) | (TIM_CCMR2_OC4M));
    TIM1->CCMR2 |= ((6 << TIM_CCMR2_OC3M_Pos) | (6 << TIM_CCMR2_OC4M_Pos));

    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E); // Enable all 4 channels

    TIM1->BDTR |= TIM_BDTR_MOE;  // Main output enable

    TIM1->CR1 |= TIM_CR1_CEN;    // Enable TIM1
}


void GPIOA_Init(void) //GPIOA para HC05
{
	// ENABLE A PINS...
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    // PA2 (TX) and PA3 (RX) to alternate function
    GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));   // Clear
    GPIOA->MODER |=  (2 << (2 * 2)) | (2 << (3 * 2));     // AF mode
    GPIOA->AFR[0] |= (1 << (4 * 2)) | (1 << (4 * 3));     // AF1 (USART2)
    GPIOA->PUPDR |= (1 << (2*2)) | (1 << (3*2)); // Pull-up


}
void USART2_Init(void) //Bluetooth Setup
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART2->BRR = 8000000 / 9600; // 8 MHz clock, 9600 baud → BRR = 833
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE; // Enable TX, RX, USART
    NVIC_EnableIRQ(USART2_IRQn); // Enable USART2 interrupt in NVIC
}
void USART2_IRQHandler(void) //Bluetooth Signals
{
    if (USART2->ISR & USART_ISR_RXNE)
    {
    	rx_buffer[rx_index++] = USART2->RDR;  // Store byte

		if (rx_index == 5) {  // Complete packet received
			// Process data (e.g., update globals)
			//uint8_t delim = rx_buffer[0];  // Delimiter (255)
			raw_x      = rx_buffer[1];  // Example: Joystick X
			raw_y      = rx_buffer[2];  // Joystick Y
			if(emergency_on == 0){
				menu_button     = rx_buffer[3];  // Button state
			}
			slider_x     = rx_buffer[4];  // Button state


			rx_index = 0;  // Reset for next packet
			// Optional: Echo back to MIT App for debugging
			//USART2->TDR = raw_x;  // Send first byte back
		}
    }
}

void init_GPIO_EXTI_encoders(void) {
    GPIOB->MODER &= ~((3 << (4 * 2)) | (3 << (5 * 2)) | (3 << (10 * 2)) | (3 << (11 * 2)));
    GPIOB->PUPDR &= ~((3 << (4 * 2)) | (3 << (5 * 2)) | (3 << (10 * 2)) | (3 << (11 * 2)));
    GPIOB->PUPDR |=  ((1 << (4 * 2)) | (1 << (5 * 2)) | (1 << (10 * 2)) | (1 << (11 * 2)));

    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB;
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PB;
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PB;

    EXTI->IMR |= (1 << 4) | (1 << 10);
    EXTI->RTSR |= (1 << 4) | (1 << 10);
}

void init_GPIO_EXTI_ultrasonic(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    for (int i = 0; i < 4; i++) {
        uint8_t trigPin = trigPins[i];
        uint8_t echoPin = echoPins[i];

        // Configure trigger pin as output
        GPIOB->MODER &= ~(3 << (trigPin * 2));
        GPIOB->MODER |=  (1 << (trigPin * 2));

        // Configure echo pin as input with pull-down
        GPIOB->MODER &= ~(3 << (echoPin * 2));
        GPIOB->PUPDR &= ~(3 << (echoPin * 2));
        GPIOB->PUPDR |=  (2 << (echoPin * 2));  // Pull-down

        // Connect EXTI line to GPIOB
        uint8_t exti_idx = echoPin >> 2;
        uint8_t exti_pos = (echoPin & 0x3) * 4;
        SYSCFG->EXTICR[exti_idx] &= ~(0xF << exti_pos);
        SYSCFG->EXTICR[exti_idx] |= (0x1 << exti_pos);  // GPIOB = 0x1

        // Configure EXTI line
        EXTI->IMR  |= (1 << echoPin);
        EXTI->RTSR |= (1 << echoPin);
        EXTI->FTSR |= (1 << echoPin);
    }

    // Enable EXTI4_15 interrupt
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void init_TIM3_for_trigger_timing(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 47;              // 1 MHz timer clock if 48MHz sysclk
    TIM3->ARR = 0xFFFF;
    TIM3->CCR1 = 0;
    TIM3->DIER |= TIM_DIER_CC1IE;
    TIM3->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM3_IRQn);
}

void start_hcsr04_trigger(uint8_t sensorIndex) {
    currentSensor = sensorIndex;
    uint8_t trigPin = trigPins[sensorIndex];
    GPIOB->BSRR = (1 << trigPin);              // Set high

    uint16_t start = TIM3->CNT;
    uint16_t target = start + 10;              // 10us pulse
    TIM3->CCR1 = target;
    TIM3->DIER |= TIM_DIER_CC1IE;
}

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_CC1IF) {
        TIM3->SR &= ~TIM_SR_CC1IF;
        uint8_t trigPin = trigPins[currentSensor];
        GPIOB->BRR = (1 << trigPin);           // Set low
        TIM3->DIER &= ~TIM_DIER_CC1IE;
    }
}

void EXTI4_15_IRQHandler(void) {
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t echoPin = echoPins[i];
        if (EXTI->PR & (1 << echoPin)) {
            EXTI->PR = (1 << echoPin);
            if (GPIOB->IDR & (1 << echoPin)) {
                // Rising edge
                echoStart[i] = TIM3->CNT;
            } else {
                // Falling edge
                if (echoStart[i] != 0) {
                    echoEnd[i] = TIM3->CNT;
                    uint16_t duration = (echoEnd[i] >= echoStart[i])
                                        ? (echoEnd[i] - echoStart[i])
                                        : (0xFFFF - echoStart[i] + echoEnd[i]);
                    float d = (duration * 0.0343f) / 2.0f;
                    if (d < 900.0f)  // Valid reading check
                        distance[i] = d;
                    echoStart[i] = 0;
                }
            }
        }
    }
}

void update_encoder(uint8_t encoder) {
    uint8_t A, B;
    if (encoder == 1) {
        A = (GPIOB->IDR >> 4) & 1;
        B = (GPIOB->IDR >> 5) & 1;
    } else {
        A = (GPIOB->IDR >> 10) & 1;
        B = (GPIOB->IDR >> 11) & 1;
    }

    if (B == 0) {
        if (encoder == 1) { encoderPos1++; direction1 = 1; }
        else { encoderPos2++; direction2 = 1; }
    } else {
        if (encoder == 1) { encoderPos1--; direction1 = 2; }
        else { encoderPos2--; direction2 = 2; }
    }
    if (encoder == 1) {
        revolutionsFloat1 = encoderPos1 / 500.0f;
        revolutionsInt1 = (int32_t)revolutionsFloat1;
    } else {
        revolutionsFloat2 = encoderPos2 / 500.0f;
        revolutionsInt2 = (int32_t)revolutionsFloat2;
    }
}


void init_emergency_buttons(void) {
    // Enable GPIOC clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Set PC6 and PC7 as inputs (MODER = 00)
    GPIOC->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)));

    // Optional: enable pull-downs if buttons are active-high
    GPIOC->PUPDR &= ~((3 << (6 * 2)) | (3 << (7 * 2))); // Clear
    GPIOC->PUPDR |=  ((2 << (6 * 2)) | (2 << (7 * 2))); // Pull-down (10)
}
void check_emergency_buttons(void) {
    // If PC6 is HIGH
    if (!(GPIOC->IDR & (1 << 6))) {
        //emergency_stop();
    }

    // If PC7 is HIGH
    if (!(GPIOC->IDR & (1 << 7))) {
        //emergency_stop();
    }
}
void emergency_stop(void) {
    menu_button = 0;
    emergency_on = 1;
	right_LPWM(0);
	right_RPWM(0);
	left_LPWM(0);
	left_RPWM(0);
	elevator_LPWM(0);
	elevator_RPWM(0);
	for (uint8_t i = 0; i < LED_NUM; i++) {
		  SetColor(i, 255, 0, 0); // Red
	}
    SendColors();
}


void WS2812_Init(void) {
    // 1. Enable clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;    // Enable GPIOB clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;  // Enable TIM15 clock
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;     // Enable DMA1 clock

    // 2. Configure PB15 as TIM15_CH2 alternate function
    GPIOB->MODER &= ~GPIO_MODER_MODER15_Msk;
    GPIOB->MODER |= (0x2 << GPIO_MODER_MODER15_Pos);  // Alternate function
    GPIOB->AFR[1] |= (0x1 << ((15 - 8) * 4));        // AF1 = TIM15_CH2

    // 3. Configure TIM15 for PWM
    TIM15->PSC = 0;                        // No prescaler
    TIM15->ARR = PWM_PERIOD;               // 1.25us period
    TIM15->CCR2 = 0;                       // Start with duty cycle 0
    TIM15->CCMR1 = (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE; // PWM mode 1, preload
    TIM15->CCER |= TIM_CCER_CC2E;          // Enable channel 2
    TIM15->BDTR |= TIM_BDTR_MOE;           // Main output enable
    TIM15->CR1 |= TIM_CR1_ARPE;            // Auto-reload preload
    TIM15->DIER |= TIM_DIER_UDE;           // Update DMA request

    // 4. Configure DMA for PWM data transfer
    DMA1_Channel5->CCR = 0;
    DMA1_Channel5->CPAR = (uint32_t)&TIM15->CCR2;  // PWM compare register
    DMA1_Channel5->CMAR = (uint32_t)pwm_buffer;    // Our data buffer
    DMA1_Channel5->CNDTR = sizeof(pwm_buffer)/sizeof(pwm_buffer[0]);
    DMA1_Channel5->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_0 |
                         DMA_CCR_MSIZE_0 | DMA_CCR_PL | DMA_CCR_TCIE;

    // 5. Enable DMA interrupt
    NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
    NVIC_SetPriority(DMA1_Channel4_5_IRQn, 2);
}

// Set color for a specific LED (RGB format)
void SetColor(uint8_t led, uint8_t red, uint8_t green, uint8_t blue) {
    if (led >= LED_NUM) return;
    led_colors[led][0] = green;  // WS2812 expects GRB order
    led_colors[led][1] = red;
    led_colors[led][2] = blue;
}

// Convert color data to PWM buffer
void PreparePWMBuffer(void) {
    uint16_t *p = pwm_buffer;

    // Convert each LED's color to PWM bits
    for (uint8_t i = 0; i < LED_NUM; i++) {
        uint8_t *color = led_colors[i];

        // Green (bit 0 to 7)
        for (int8_t bit = 7; bit >= 0; bit--) {
            *p++ = (color[0] & (1 << bit)) ? HIGH_PULSE : LOW_PULSE;
        }

        // Red (bit 8 to 15)
        for (int8_t bit = 7; bit >= 0; bit--) {
            *p++ = (color[1] & (1 << bit)) ? HIGH_PULSE : LOW_PULSE;
        }

        // Blue (bit 16 to 23)
        for (int8_t bit = 7; bit >= 0; bit--) {
            *p++ = (color[2] & (1 << bit)) ? HIGH_PULSE : LOW_PULSE;
        }
    }

    // Add reset code (all zeros)
    for (uint16_t i = 0; i < RESET_SLOTS; i++) {
        *p++ = 0;
    }
}

// Send colors to LED strip
void SendColors(void) {

    PreparePWMBuffer();

    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CMAR = (uint32_t)pwm_buffer;
    DMA1_Channel5->CNDTR = sizeof(pwm_buffer)/sizeof(pwm_buffer[0]);
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    TIM15->CR1 |= TIM_CR1_CEN;
}

// DMA interrupt handler
void DMA1_Channel4_5_IRQHandler(void) {
	if (DMA1->ISR & DMA_ISR_TCIF5) {
	        DMA1->IFCR |= DMA_IFCR_CTCIF5;
	        TIM15->CR1 &= ~TIM_CR1_CEN;
	        //dma_busy = 0; // Clear flag when done
	    }
	}
