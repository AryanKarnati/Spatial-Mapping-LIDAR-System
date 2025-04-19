#include <stdint.h>
#include "tm4c1294ncpdt.h" // Microcontroller-specific register definitions
#include "PLL.h" // Clock configuration
#include "SysTick.h"  // Timing delays via SysTick
#include "uart.h"  // UART communication
#include "VL53L1X_api.h"  // VL53L1X ToF sensor API
#include "VL53L1_platform_2dx4.h"  // Platform abstraction for your 2DX4 board
#include <stdio.h> // Needed for sprintf

void motor(int steps, int direction);
void perform_scan_rotation(void);
void wait_for_button_press(void);
void I2C_Init(void); // Setup for I2C peripheral
void PortJ_Init(void);  // Button input (PJ0)
void PortH_Init(void);  // Stepper motor output (PH0–3)
void PortN_Init(void);  // LED0 (status LED)
void PortF_Init(void);  // LED2, LED3
void PortM_Init(void);
void Stepper_Stop(void); // Resets motor output (all PH pins low)

volatile uint8_t motor_running = 0; // Toggles motor ON/OFF
volatile uint8_t motor_direction = 0;  // 0 = CW, 1 = CCW
volatile uint8_t default_step_angle = 1;  // 1 microstep
volatile int pos = 0; // Current step position
volatile int steps = 1;  // Steps per call


uint16_t dev = 0x29; // VL53L1X default I2C address


void I2C_Init(void) {
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0; // Enable I2C0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;   // Enable GPIOB
    while((SYSCTL_PRGPIO_R & 0x02) == 0) {};  // Wait until ready
    GPIO_PORTB_AFSEL_R |= 0x0C;  // Enable alternate function on PB2, PB3
    GPIO_PORTB_ODR_R |= 0x08; // Open drain for SDA (PB3)
    GPIO_PORTB_DEN_R |= 0x0C; // Digital enable for PB2, PB3
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~0x0000FF00) | 0x00002200; // I2C function
    I2C0_MCR_R = 0x10; // Master mode
    I2C0_MTPR_R = 0x15;  // Set clock speed
}

void PortJ_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0) {};
    GPIO_PORTJ_DIR_R &= ~0x03;
    GPIO_PORTJ_DEN_R |= 0x03;
    GPIO_PORTJ_PUR_R |= 0x03;
}

void PortH_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0) {};
    GPIO_PORTH_DIR_R |= 0x0F;
    GPIO_PORTH_DEN_R |= 0x0F;
}

void PortN_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0) {};
    GPIO_PORTN_DIR_R |= 0x03;
    GPIO_PORTN_DEN_R |= 0x03;
}

void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {};
    GPIO_PORTF_DIR_R |= 0x1F;
    GPIO_PORTF_DEN_R |= 0x1F;
}

void PortM_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0) {};
    GPIO_PORTM_DIR_R &= ~0x03;
    GPIO_PORTM_DEN_R |= 0x03;
    GPIO_PORTM_PUR_R |= 0x03;
}

void PortE_Init(void){
	 SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
   while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0) {}
   GPIO_PORTE_DIR_R |= 0x01;
   GPIO_PORTE_DEN_R |= 0x01;
}

void Stepper_Stop(void) {
    GPIO_PORTH_DATA_R = 0x00;
}

// PF4 - Measurement Status - LED 2
// PF0 - UART Tx - LED 3
// PN1 - Additional Status - LED 0


void LED3_Update(void) {
    GPIO_PORTF_DATA_R |= 0x01; // Turn on LED3 (PF0)
	  SysTick_Wait10ms(100000);  // Delay
    GPIO_PORTF_DATA_R &= ~0x01; // Turn off
}


void LED0_Update(void) {
    if (motor_direction == 0 && motor_running) {
        GPIO_PORTN_DATA_R |= 0x02; // Turn ON LED0
    } else {
        GPIO_PORTN_DATA_R &= ~0x02; // Turn OFF LED0
    }
}


void LED2_Update(int blink) {
    int blink_interval = blink; 

    if (pos % blink_interval == 0) {  
        GPIO_PORTF_DATA_R |= 0x10; // LED2 on (PF4)
		   	SysTick_Wait10ms(100000);
        GPIO_PORTF_DATA_R &= ~0x10;
				SysTick_Wait10ms(100000);
    }
}

void BusClockCheck(void) {
    while (1) {
        GPIO_PORTE_DATA_R ^= 0x01; // Toggle PE0
        SysTick_Wait10ms(1);       // 2 ms Period from output waveform
    }
}




void motor(int steps, int direction) {
    uint32_t delay = 2000; // Keep your original delay
		

    if (direction == 0) { // Clockwise
        for (int i = 0; i < steps; i++) {
            if (!motor_running) return; // Stop motor if toggled OFF
            
						if(default_step_angle == 1){
							LED2_Update(16);
						}else{
							LED2_Update(64);
						}
						pos += 1;
						GPIO_PORTH_DATA_R = 0b00000011;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000110;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001100;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001001;
            SysTick_Wait10ms(delay);
	
					
        }
    } else if (direction == 1) { // Counterclockwise
        for (int i = 0; i < steps; i++) {
            if (!motor_running) return; // Stop motor if toggled OFF
            
						
						pos -= 1; 
						GPIO_PORTH_DATA_R = 0b00001001;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001100;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000110;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000011;
            SysTick_Wait10ms(delay);
					
        }
    }
}

int main(void) {
		// Init all peripherals
    PLL_Init(); 
		SysTick_Init(); 
		UART_Init(); 
		I2C_Init();
    PortH_Init(); 
		PortJ_Init(); 
		PortN_Init(); 
		PortF_Init(); 
		PortM_Init();
		PortE_Init(); 
	
		// Optional: square wave test
		BusClockCheck();
		
		// Sensor Boot
    uint8_t sensorState = 0;
    VL53L1X_BootState(dev, &sensorState); // Wait for sensor boot-up
    VL53L1X_SensorInit(dev); // Load default config
    VL53L1X_StartRanging(dev);  // Begin continuous distance measurement

		// Runtime variables
    uint8_t last_button_state = 1; // Previous button state (for edge detection)
    uint8_t dataReady = 0; // Sensor flag: measurement ready
    uint16_t distance; // Measured distance (mm)
    int step_counter = 0;  // Total microsteps taken in current cycle
		float angle = 0;  // Current scanning angle (0°–360°)
		uint8_t scan_complete = 0; // Flag to pause motor when full sweep is done
		uint8_t scan_direction = 1; // 1 = clockwise, 0 = counterclockwise

    while (1) {
        uint8_t button_state = (GPIO_PORTJ_DATA_R & 0x01);
        if (last_button_state == 1 && button_state == 0) {
            motor_running ^= 1;	// Toggle motor on press
						scan_direction = 1;
						scan_complete = 0;
						step_counter = 0;
						angle = 0;
            SysTick_Wait10ms(100); // Debounce
        }
        last_button_state = button_state;

        if (motor_running) {
						if(!scan_complete){
							if(scan_direction){
								motor_direction = 0;
							}else{
								motor_direction = 1;
							}
							LED0_Update();
						if(step_counter == 0 && motor_direction == 0){
								LED3_Update();
						}
							
            motor(1, motor_direction); // Move 1 step
            step_counter++;

            if (scan_direction && (step_counter % 16 == 0)) { // Every 16 microsteps = 11.25°
                while (dataReady == 0) {
                    VL53L1X_CheckForDataReady(dev, &dataReady);
                    VL53L1_WaitMs(dev, 5);
                }

                VL53L1X_GetDistance(dev, &distance); // Read distance
                VL53L1X_ClearInterrupt(dev); // Reset interrupt flag

								
                angle = (step_counter /16) *11.25; // Calculate current angle
								
								sprintf(printf_buffer, "%u\n", distance); // Send distance over UART
                UART_printf(printf_buffer);
                SysTick_Wait10ms(1000); // Wait for MATLAB to catch up

                dataReady = 0;
								
            }
						
								if (step_counter >= 512) { // One full 360° rotation
                Stepper_Stop();
                SysTick_Wait10ms(200000); // 2 second pause

                step_counter = 0;
                angle = 0;
                scan_direction ^= 1; // Toggle scan direction

                // If returning, no need to reset motor_running
                // If finished returning, resume scan
            }
					}
        } else {
            Stepper_Stop();
        }


    }
}
