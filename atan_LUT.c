
#include "stm32f4xx_conf.h"
#include "atan_LUT.h"

/* Angles go up to 45 degrees since atan(x) = 90 - atan(1/x) x e (0,45) */
/* Input is a number between 0 and 1 scaled by 100 (0 ... 100)*/
static const float atan_lookup[101] =
{0,1,1,2,3,3,4,4,5,5,6,7,7,8,8,9,9,10,10,11,12,12,13,
13,14,14,15,15,16,16,17,17,18,19,19,20,20,21,21,22,22,
23,23,24,24,24,25,25,26,26,27,27,28,28,29,29,29,30,30,
31,31,32,32,32,33,33,34,34,34,35,35,36,36,36,37,37,37,
38,38,38,39,39,40,40,40,41,41,41,42,42,42,42,43,43,43,
44,44,44,45,45,45};

/*
 * Arctan Lookup table Approximation
 * Computed from a minimal table of 101 entries
 * The remaining range is done via the modification:                 `
 * Arctan(x) = 90 - Arctan(1/x)	for X in [1, ...] and a result in [45, 90]
 * Calculation based on degrees and result in degrees
 * Missing : handle overflow big values
 * @param: X
 * @return: arctan(X)
 */
float atan_table(float x) {
	u16 temp;
	float val;
	if (x < 0) {							// if negative
		val = -(x*100);						// scale by 100, take absolute value
		if(val > 100) { 				 	// if x > 1.00
			if (val > 10000) {			 	// if too big prevent overflow
				val = 100;				 	// set a barrier
			}
			temp = (u16) (val);		 		// cast into u16 type
			temp = 10000/temp;			 	// 1/x
			val = atan_lookup[temp]-90;	 	// -(90 - arctan(1/x))
		} else {
			temp = (u16) (val);		 		// cast into u16 type
			val = -atan_lookup[temp];
		}
	} else {								// if positive
		val = x*100;						//scale by 100
		if(val > 100) { 				 	// if x > 1.00
			if (val > 10000) {			 	// if too big prevent overflow
				val = 100;				 	// set a barrier
			}
			temp = (u16) (val);				// cast into u16 type
			temp = 10000/temp;			 	// 1/x
			val = 90-atan_lookup[temp];  	// 90 - arctan(1/x)
		} else {
			temp = (u16) (val);				// cast into u16 type
			val = atan_lookup[temp];
		}
	}
	return val;
}

