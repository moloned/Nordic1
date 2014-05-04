// This code provides simplest "signs of life" for nRF51822
// Liam Goudge April 2014

#include "Headers/nRF51822.h"



int main(void)
{
	HFCLKSTART=0xffff;	// Start the external 16MHz crystal oscillator

	PINCNF6=0x1;		// Set P0.06 to regular output mode
	volatile unsigned int count=0;


	while (1)
	{
		for (count=0;count<10000;count++);
		OUTSET=0x40;	// Set P0.06 high
		for (count=0;count<10000;count++);
		OUTCLR=0x40;	// Set P0.06 low
	}

}






