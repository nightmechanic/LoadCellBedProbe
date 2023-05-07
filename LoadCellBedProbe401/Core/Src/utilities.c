#include "main.h"


void __attribute__((optimize("Ofast"))) my_memcpy(uint8_t * Target, uint8_t * Source, uint16_t Len){

	uint32_t * SourcePointer;
	uint32_t * TargetPointer;
	uint8_t * bSourceP;
	uint8_t * bTargetP;
	uint16_t index, offset;


	if (Len < 16){
		memcpy(Target, Source, Len);
		return;
	}

	if ( ((uintptr_t)Target & 0x3) != ((uintptr_t)Source & 0x3) ){
		memcpy(Target, Source, Len);
		return;
	}

	offset = (uintptr_t)Target & 0x3;

	while (offset > 0){
		*Target = *Source;
		Target++;
		Source++;
		offset--;
	}
	// we are 4 byte aligned now

	SourcePointer  = (uint32_t *) Source;
	TargetPointer = (uint32_t *) Target;

	index = 0;
	while(Len >= 4){
		TargetPointer[index] = SourcePointer[index];
		index++;
		Len -= 4;
	}

	bSourceP = (uint8_t * ) &SourcePointer[index];
	bTargetP = (uint8_t * ) &TargetPointer[index];
	index = 0;
	while(Len > 0){
		bTargetP[index] = bSourceP[index];
		index++;
		index++;
		Len--;
	}
}


int32_t int_pow(int32_t x, uint8_t y){
	int32_t value = 1;

	for (; y > 0; y-- ){
		value *= x;
	}

	return value;
}

float32_t frac_roundf(float32_t value, uint8_t frac_digits ){

	int32_t exp = int_pow(10, frac_digits);
	float32_t rem;

	value = value * exp;
	rem = value - ((int32_t)value);

	if (rem >= 0.5){
		value = value + 1;
	}

	return (value / exp);
}

uint8_t float_to_char(float value, char* buffer, uint8_t size, uint8_t base, uint8_t frac_digits){
  static const char charcodes[] = "0123456789abcdefghijklmnopqrstuvwxyz";
  static const char minus[]="-";
  static const char period[]=".";
  static const char space[]=" ";
  int32_t quot, digitval;
  uint8_t index = 0;
  uint8_t positive = 1;

  int32_t int_part;
  int32_t frac_part;

  if ( (base < 2) || (base > 36) )// check if we got a qualified base
  {
    *buffer = '\0';
    return 0;
  }

  if (value < 0) {
    positive = 0;
    value = -value;
  }

  value  = frac_roundf(value, frac_digits);

  int_part  = (int32_t) value;
  frac_part = (int32_t) ((value - int_part) * int_pow(10, frac_digits ));

  index = size - 1;

  if (frac_digits > 0){

    while (frac_digits > 0){
      quot = frac_part / base;
      digitval = frac_part - quot * base;
      frac_part = quot;
      if ((frac_part > 0) || (digitval > 0)) {
        buffer[index] =  charcodes[digitval];
       } else {
            buffer[index] = charcodes[0];
      }
      index--;
      frac_digits--;
    }
    buffer[index] = period[0];
    index--;
  }

  while (index >= 0){
    quot = int_part / base;
    digitval = int_part - quot * base; // get the value of the rightmost digit (avoid modulo, it would perform another slow division)
    int_part = quot;
    if ((int_part > 0) || (digitval > 0)) {
        buffer[index] =  charcodes[digitval];
    } else if (!positive) {
            buffer[index] = minus[0];
            positive = 1;
    }else{
            buffer[index] = space[0];;
        }

    index--;
  }

  return size;
}

uint8_t __attribute__((optimize("Ofast"))) float_to_char_opt(float value, char* buffer, uint8_t size, uint8_t base, uint8_t frac_digits){
	static const char charcodes[] = "0123456789abcdefghijklmnopqrstuvwxyz";
	static const char minus[]="-";
	static const char period[]=".";
	static const char space[]=" ";
	static const char Inf[]="Inf";
	int32_t lz_flag, digitval;
	uint8_t index = 0;
	uint8_t positive = 1;
	uint8_t int_digits = size;

	int32_t int_part;
	float32_t frac_part, int_partf;

	if ( (base < 2) || (base > 36) )// check if we got a qualified base
	  {
		*buffer = '\0';
		return 0;
	  }

	  if (value < 0) {
		positive = 0;
		value = -value;
	  }

	  value  = frac_roundf(value, frac_digits);

	  int_part  = (int) value;
	  frac_part = (value - int_part);

	  if (frac_digits > 0){

		int_digits = size - 1 - (frac_digits);
		index = int_digits;
		buffer[index] = period[0];
		index++;

		while (frac_digits > 0){
		  frac_part = frac_part * base;
		  digitval = (int)frac_part;
		  frac_part = frac_part - digitval ;

		  buffer[index] =  charcodes[digitval];

		  index++;
		  frac_digits--;
		}

	  }

	  if (positive){
		index = 0;
	  }else{
		int_digits--;
		index = 1;
	  }
	  int_partf = int_part/((float)int_pow(base, int_digits));

	  if (int_partf > 1){
		if (positive){
		  buffer[0] = Inf[0];
		  buffer[1] = Inf[1];
		  buffer[2] = Inf[2];
		  for (index = 3; index<size; index++){
			buffer[index] = space[0];
		  }
		  return size;
		}else {
		  buffer[0] = minus[0];
		  buffer[1] = Inf[0];
		  buffer[2] = Inf[1];
		  buffer[3] = Inf[2];
		  for (index = 4; index<size; index++){
			buffer[index] = space[0];
		  }
		  return size;
		}
	  }

	  lz_flag = 0;
	  while (int_digits > 0){
		int_partf = int_partf * base;
		digitval = (int)int_partf; // get the value of the rightmost digit (avoid modulo, it would perform another slow division)
		int_partf = int_partf - digitval;
		if (digitval > 0) {
		  buffer[index] =  charcodes[digitval];
		  if (!lz_flag){
			lz_flag = 1;
			if (!positive){
			  buffer[index-1] = minus[0];
			  positive = 1;
			}
		  }
		} else if (lz_flag) {
		  buffer[index] = charcodes[0];
		}else{
		  buffer[index] = space[0];
		}

		index++;
		int_digits--;
	  }

	  return size;
}
