// Header file for nRF51822 peripheral set

typedef	unsigned int uint32_t;

// Reset and Clock Control (RCC)
#define RCC_BASE			((uint32_t) 0x40000000)
#define HFCLKSTART			(*(volatile unsigned long *)(RCC_BASE + 0x000)) // High Frequency Crystal start Register
#define HFCLKSTOP			(*(volatile unsigned long *)(RCC_BASE + 0x004)) // High Frequency Crystal stop
#define HFCLKSTAT			(*(volatile unsigned long *)(RCC_BASE + 0x40C)) // High Frequency Clock status


// GPIO Port Registers

#define GPIO_BASE			((uint32_t) 0x50000000)
#define PINCNF6				(*(volatile unsigned long *)(GPIO_BASE + 0x718)) // GPIO pin P0.06. Port configuration register.

#define OUTSET				(*(volatile unsigned long *)(GPIO_BASE + 0x508)) // Register to set GPIO pins
#define OUTCLR				(*(volatile unsigned long *)(GPIO_BASE + 0x50C)) // Register to clear GPIO pins


