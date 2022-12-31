#include <Arduino.h>
// if setup is active high, we need pulldown resistors on each input pin
#define ACTIVE_LOW 1

#define ADC_READY (ADCSRA & _BV(ADIF))

#define OUTPUT_CHANNEL 0

#define DEBOUNCE_MAX 2 

#define NOTE_ID(x) ((x + 36))
// #define NOTE_ID(x) (NOTE_ID_ARRAY[x])

/* IO map
PORTA
0-7: 22, 23, 24, 25, 26, 27, 28, 29
PORTB
0-7: 54, 53, 52, 51, 10, 11, 12, 13
PORTC
0-7: 37, 36, 35, 34, 33, 32, 31, 30
PORTK
0-7: 62-69, A8-A15
PORTL
0-7: 49-42
PORTH
0-1: 17-16
3-6: 6-9
PORTD
0-3: 21, 20, 19, 18
7: 38
PORTF
0-7: 54-61, A0-A7
PORTG
0-2: 41-39
5: 4
PORTJ
0-1: 15-14 (1 unused)

PORTE
0 and 1 are USART0, used for USB
3-5: 5, 2, 3. Use for input select

A, B, C, F, K, L are full ports.
*/

// sequence of ports used to scan keys
const volatile uint8_t* PORT_SEQUENCE[10] = {
    &PINA, // 8
    &PINB, // 16
    &PINC, // 24
    &PINK, // 32
    &PINL, // 40
    &PINH, // 46
    &PIND, // 51
    &PINF, // 56
    &PING, // 60
    &PINJ, // 61
};

// bits to pay attention to from each port
const uint8_t PORT_MASKS[10] = {
    0xFF,      // PORTA
    0xFF,      // PORTB
    0xFF,      // PORTC
    0xFF,      // PORTK
    0xFF,      // PORTL
    B01111011, // PORTH,
    B10001111, // PORTD
    B11111000, // PORTF, lowest 3 bits (A0-A2) used to read expression pedals
    B00100111, // PORTG
    B00000001, // PORTJ, one available pin masked off because unneeded
};

// inverted because we read active low
// const uint8_t PORT_MASKS[10] = {
//     0x00,      // PORTA
//     0x00,      // PORTB
//     0x00,      // PORTC
//     0x00,      // PORTK
//     0x00,      // PORTL
//     B10000100, // PORTH,
//     B01110000, // PORTD
//     B00000111, // PORTF, lowest 3 bits (A0-A2) used to read expression pedals
//     B11011000, // PORTG
//     B11111110, // PORTJ, one masked off because unneeded
// };

// MIDI note ids corresponding to each bit of each port
// 0 where a bit is not connected
// can reorder (except for zeros) if keys are connected differently
// the zero pattern could be constructed at boot time if that's easier
// const uint8_t NOTE_IDS[10][8] = {
//     36, 37, 38, 39, 40, 41, 42, 43, // PORTA
//     44, 45, 46, 47, 48, 49, 50, 51, // PORTB
//     52, 53, 54, 55, 56, 57, 58, 59, // PORTC
//     60, 61, 62, 63, 64, 65, 66, 67, // PORTK
//     68, 69, 70, 71, 72, 73, 74, 75, // PORTL
//     76, 76, 0,  78, 79, 89, 81, 0,  // PORTH 0-7
//     82, 83, 84, 85, 0,  0,  0,  86, // PORTD 0-7
//     0,  0,  0,  87, 88, 89, 90, 91, // PORTF 0-7
//     92, 93, 94, 0,  0,  95, 0,  0,  // PORTG 0-7
//     96, 0,  0,  0,  0,  0,  0,  0   // PORTJ 0-7
// };

// const uint8_t NOTE_ID_ARRAY[61] = {
//     36, 37, 38, 39, 40, 41, 42, 43, // PORTA
//     44, 45, 46, 47, 48, 49, 50, 51, // PORTB
//     52, 53, 54, 55, 56, 57, 58, 59, // PORTC
//     60, 61, 62, 63, 64, 65, 66, 67, // PORTK
//     68, 69, 70, 71, 72, 73, 74, 75, // PORTL
//     76, 76, 78, 79, 89, 81,         // PORTH 0-7
//     82, 83, 84, 85, 86,             // PORTD 0-7
//     87, 88, 89, 90, 91,             // PORTF 0-7
//     92, 93, 94, 95,                 // PORTG 0-7
//     96                              // PORTJ 0-7
// };

// MIDI continuous controllers: pitch wheel, breath, expression
// basically arbitrary choices
const uint8_t EXPR_CONTROLLERS[3] = {1, 2, 11};

// (packed) last read state for each key, repeated once for each keyboard
// always stored active high: a 1 means the corresponding key is pressed
// initialized to all zeros, representing no presses
uint8_t key_reads[3][10] = {};

// there are a limited number of pointer registers, so to avoid using an extra
// one, we'll interleave these two states, since we look at them at the same
// time
struct key_state {
  uint8_t debounce_count;
  bool on;
};
// uint8_t debounce_counts[61] = {};
// bool key_states[61] = {};

struct key_state key_states[3][61] = {};

// last read position of each expression pedal
uint8_t expr_state[3] = {}; // all zeros

// which expression pedal is currently being read, since ADC can only be
// connected to one at a time
uint8_t current_expr = 0;

/*

There are a few options for debouncing that will also address the possibility of
transients induced by other keys.

- keep the current key state and a count of the number of sequential reads
different from the key state; when the count hits a threshold, switch the key
state

- keep a running sum of +1 for each on and -1 for each off, saturating at 0 and
some threshold T. when the sum hits T turn on; when it hits 0 turn off.

- save the last T reads for each key and output on/off based on how many of
these were on or off.

I think I like option 2 the best, as it can have faster response than 1. Option
3 is probably hard to calibrate and also more resource-hungry.

*/

inline void start_adc(uint8_t pin) {
  ADMUX &= B11111000; // clear currently selected pin
  ADMUX |= pin;

  ADCSRA |= _BV(ADIF); // clear interrupt flag
  ADCSRA |= _BV(ADSC); // start next conversion
}

void setup() {
  // setup pullups
  PORTA |= 0xFF;
  PORTB |= 0xFF;
  PORTC |= 0xFF;
  PORTD |= 0xFF;
  // PORTF |= B11111000; // no pullups on ADC inputs
  PORTF |= 0xFF; // pullups on for testing
  PORTG |= 0xFF;
  PORTH |= 0xFF;
  PORTJ |= 0xFF;
  PORTK |= 0xFF;
  PORTL |= 0xFF;

  // if there is a pullup on all pins, we will always read 1 on disconnected
  // pins and hence never have a change to report

  // setup input ports
  DDRA &= 0x00;
  DDRB &= 0x00;
  DDRC &= 0x00;
  DDRD &= ~B10001111;
  DDRF &= B00000111;
  DDRG &= ~B00100011;
  DDRH &= ~B01111011;
  DDRJ &= ~B00000001;
  DDRK &= 0x00;
  DDRL &= 0x00;

  // keyboard select outputs
  DDRE |= B00111000;
  // set all keyboard select to high
  PORTE |= B00111000;

  Serial.begin(115200);

  // setup ADC
  PRR0 &= ~_BV(PRADC); // enable ADC in power reduction register

  ADMUX |= _BV(REFS0); // reference = VCC
  ADMUX |= _BV(ADLAR); // only need 8 bits of resolution, so most significant 8
                       // bits of output will be in ADCH
  // ADMUX |= current_expr; // it's zero now, but anyway

  // division factor of 32 for ADC clock, giving 500 kHz
  // ADC takes ~14 cycles to produce a read, so we get one every ~30 us.
  // This should be more than enough, and we could slow the clock down if needed
  ADCSRA |= _BV(ADPS0) | _BV(ADPS2);
  // enable ADC
  ADCSRA |= _BV(ADEN);

  // disable digital inputs on ADC pins
  DIDR0 |= _BV(ADC2D) | _BV(ADC1D) | _BV(ADC0D);

  // start conversion
  start_adc(current_expr);
}

void send_note_on(uint8_t note_id, uint8_t channel) {
  uint8_t command = 0x90 | channel;
  // Serial.print("on ");
  // Serial.println(note_id);
  Serial.write(command);
  Serial.write(note_id);
  Serial.write(0x40); // velocity, doesn't matter
}

void send_note_off(uint8_t note_id, uint8_t channel) {
  uint8_t command = 0x80 | channel;
  // Serial.print("off ");
  // Serial.println(note_id);
  Serial.write(command);
  Serial.write(note_id);
  Serial.write(0x00); // velocity, may as well be silent?
}

void send_expr(uint8_t value, uint8_t type) {
  // Serial.print("expr ");
  // Serial.println(value);
  Serial.write(0xB1); // continuous control command
  Serial.write(type);
  Serial.write(value);
}

inline void read_port(int channel, int port_idx) {
  uint8_t key_in = *(PORT_SEQUENCE[port_idx]);
#if ACTIVE_LOW
  key_in = ~key_in; // convert active low to active 1
#endif
  key_in &= PORT_MASKS[port_idx];
  key_reads[channel][port_idx] = key_in;
}

inline void update_key_state(uint8_t channel, uint8_t max_port_idx) {
  uint8_t key_idx = 0;
  struct key_state* current_key_ptr = key_states[channel];
  for (uint8_t port_idx = 0; port_idx < max_port_idx; port_idx++) {
    uint8_t key_read = key_reads[channel][port_idx];
    uint8_t port_mask = PORT_MASKS[port_idx];
    uint8_t current_bit_mask = 0x01;
    for (uint8_t current_bit = 0; current_bit < 8; current_bit++) {
      // uint8_t current_bit_mask = _BV(bit_idx);
      if (port_mask & current_bit_mask) { 
        // if this pin is actually a key
        uint8_t debounce_count = current_key_ptr->debounce_count;
        if (current_key_ptr->on) {           // key is currently on
          if (key_read & current_bit_mask) { // read on
            // saturating increment
            if (debounce_count < DEBOUNCE_MAX) {
              debounce_count++;
            }
            // key is already on so no message to send
          } else { // read off
            // if current state is on, debounce_count must be positive, so no need to check
            debounce_count--;
            if (debounce_count == 0) {
              current_key_ptr->on = false;
              send_note_off(NOTE_ID(key_idx), channel);
            }
          }

        } else {                             // key is currently off
          if (key_read & current_bit_mask) { // read key on
            // if current state is off, debounce_count < DEBOUNCE_MAX, so no need to check
            debounce_count++;
            if (debounce_count == DEBOUNCE_MAX) {
              current_key_ptr->on = true;
              send_note_on(NOTE_ID(key_idx), channel);
            }
          } else { // read key off
            if (debounce_count > 0) {
              debounce_count--;
            }
            // no message to send because key is already off
          }
        }
        current_key_ptr->debounce_count = debounce_count;
        key_idx++;
        current_key_ptr++;
      }
      current_bit_mask <<= 1;
    }
  }
}

void set_input_channel(int channel) {
  PORTE &= ~_BV(channel + 3);
}

void clear_input_channel() {
  PORTE |= B00111000;
}

void loop() {

  for (uint8_t channel = 0; channel < 3; channel++) {
    set_input_channel(channel);
    _delay_us(2);
    uint8_t max_port_idx = (channel == 2) ? 5 : 10;
    for (uint8_t port_idx = 0; port_idx < max_port_idx; port_idx++) {
      read_port(channel, port_idx);
    }
    update_key_state(channel, max_port_idx);

    if (ADC_READY) {
      uint8_t new_expr_state = ADCH;
      // small random excursions are common, so ignore changes of small
      // magnitude
      if ((new_expr_state > expr_state[current_expr] + 1) ||
          (new_expr_state < expr_state[current_expr] - 1)) {
        send_expr(new_expr_state, EXPR_CONTROLLERS[current_expr]);
        expr_state[current_expr] = new_expr_state;
      }

      // move to next pedal pin
      current_expr++;
      if (current_expr > 2) current_expr = 0;

      start_adc(current_expr);
    }
    clear_input_channel();
  }

  // should we delay slightly before starting again?
}
