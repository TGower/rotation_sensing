//
// ESP32-S3 minimal SIMD example
// Written by Larry Bank
// Copyright (c) 2024 BitBank Software, Inc.
//
// The purpose of this example is to show how to make use of ESP32-S3 SIMD instructions
// in your Arduino or ESP-IDF projects. The code is not comprehensive and just provides
// a starting point for someone wanting to learn how to use them. I wrote this because
// I couldn't find such an example and thought that people would appreciate saving some
// time with the research I did.
//

// The ADD instruction always saturates the results, so notice what happens to value 7
// in the output

extern "C" {
  int s3_add16x8(int16_t *pA, int16_t *pB, int16_t *pC);
}
// 128-bit (16-byte) loads and stores need to be 16-byte aligned
int16_t __attribute__((aligned (16))) u16_A[8] = {0x00, -0x100, 0x00, 0x1111, 0x00, 0x1234, 0x00, 0x7fff};
int16_t __attribute__((aligned (16))) u16_B[8] = {0x00, 0x3000, 0x00, 0x2222, 0x00, 0x4321, 0x00, 0x4000};
int16_t __attribute__((aligned (16))) u16_C[8] = {0};

void setup() {

  Serial.begin(115200);
  delay(3000); // wait for USB-CDC to start
  Serial.println("About to call Asm code");
  s3_add16x8(u16_A, u16_B, u16_C);
  Serial.println("Returned from Asm code");
  for (int i=0; i<8; i++) {
    Serial.printf("value %d = 0x%04x\n", i, u16_C[i]);
  }
} /* setup() */

void loop() {
} /* loop() */

The s3_simd.S file
//
// ESP32-S3 SIMD example
// Written by Larry Bank
// Copyright (c) 2024 BitBank Software, Inc.
//
#include "dsps_fft2r_platform.h"
#if (dsps_fft2r_sc16_aes3_enabled == 1)
	.text
	.align 4

// Simple signed 16-bit x 8 add
// registers with the args:     A2            A3            A4
// Call as int s3_add16x8(int16_t *pA, int16_t *pB, int16_t *pC);
	.global s3_add16x8
  .type   s3_add16x8,@function
s3_add16x8:
  entry   a1,16            # prepare windowed registers and reserve 16 bytes of stack
  ee.vld.128.ip	q0,a2,16   # load 8 "A" values into Q0 from A2, then add 16 to A2
  ee.vld.128.ip	q1,a3,16   # load 8 "B" values into Q1 from A3, then add 16 to A3
  ee.vadds.s16 q2,q0,q1    # C = A+B (with saturation)
  ee.vst.128.ip q2,a4,16   # store the 8 "C" values, then add 16 to A4
	movi.n	a2,0             # return value of 0
	retw.n                   # restore state (windowed registers) and return to caller

#endif // dsps_fft2r_sc16_aes3_enabled

