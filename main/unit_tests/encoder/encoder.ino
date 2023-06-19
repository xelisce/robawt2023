#include "hardware/pio.h"
#include "quadrature.pio.h"

#define QUADRATURE_A_PIN 0
#define QUADRATURE_B_PIN 1

PIO pio = pio0;
uint offset, sm;

void setup() {
  Serial.begin(9600);
  offset = pio_add_program(pio, &quadrature_program);
  sm = pio_claim_unused_sm(pio, true);
  quadrature_program_init(pio, sm, offset, QUADRATURE_A_PIN, QUADRATURE_B_PIN);
}

void loop() {
  pio_sm_exec_wait_blocking(pio, sm, pio_encode_in(pio_x, 32));
  uint encoder_value = pio_sm_get_blocking(pio, sm);
  Serial.println(encoder_value);
}
