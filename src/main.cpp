#include <Arduino.h>
// if setup is active high, we need pulldown resistors on each input pin
#define ACTIVE_LOW 1

#define ADC_READY (ADCSRA & _BV(ADIF))

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
const uint8_t NOTE_IDS[10][8] = {
    36, 37, 38, 39, 40, 41, 42, 43, // PORTA
    44, 45, 46, 47, 48, 49, 50, 51, // PORTB
    52, 53, 54, 55, 56, 57, 58, 59, // PORTC
    60, 61, 62, 63, 64, 65, 66, 67, // PORTK
    68, 69, 70, 71, 72, 73, 74, 75, // PORTL
    76, 76, 0,  78, 79, 89, 81, 0,  // PORTH 0-7
    82, 83, 84, 85, 0,  0,  0,  86, // PORTD 0-7
    0,  0,  0,  87, 88, 89, 90, 91, // PORTF 0-7
    92, 93, 94, 0,  0,  95, 0,  0,  // PORTG 0-7
    96, 0,  0,  0,  0,  0,  0,  0   // PORTJ 0-7
};

// MIDI continuous controllers: pitch wheel, breath, expression
// basically arbitrary choices
const uint8_t EXPR_CONTROLLERS[3] = {1, 2, 11};

// (packed) last read state for each key, repeated once for each keyboard
// always stored active high: a 1 means the corresponding key is pressed
// initialized to all zeros, representing no presses
uint8_t key_state[3][10] = {}; 

// last read position of each expression pedal
uint8_t expr_state[3] = {}; // all zeros

// which expression pedal is currently being read, since ADC can only be
// connected to one at a time
uint8_t current_expr = 0;

// unsigned long last_update[3][10][8] = {};

void setup() {
  // setup pullups
  PORTA |= 0xFF;
  PORTB |= 0xFF;
  PORTC |= 0xFF;
  PORTD |= 0xFF;
  // PORTF |= B11111000; // no pullups on ADC inputs
  PORTF |= 0xFF; //pullups on for testing
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

  // set up default unpressed state for all pins...active low
  // for (uint8_t channel = 0; channel < 3; channel++) {
  //   for (int port_idx = 0; port_idx < 10; port_idx++) {
  //     key_state[channel][port_idx] = 0xFF;
  //   }
  // }

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
  ADMUX |= current_expr; // it's zero now, but anyway

  // division factor of 32 for ADC clock, giving 500 kHz
  // ADC takes ~14 cycles to produce a read, so we get one every ~30 us.
  // This should be more than enough, and we could slow the clock down if needed
  ADCSRA |= _BV(ADPS0) | _BV(ADPS2);
  ADCSRA |= _BV(ADEN); // enable ADC

  // disable digital inputs on ADC pins
  DIDR0 |= _BV(ADC2D) | _BV(ADC1D) | _BV(ADC0D);

  ADCSRA |= _BV(ADSC); // start conversion
}

void send_note_on(uint8_t channel, int note_id) {
  uint8_t command = 0x90 | channel;
  Serial.print("on ");
  Serial.println(note_id);
  // Serial.write(command);
  // Serial.write(note_id);
  // Serial.write(0x40); // velocity, doesn't matter
}

void send_note_off(int channel, int note_id) {
  uint8_t command = 0x80 | channel;
  Serial.print("off ");
  Serial.println(note_id);
  // Serial.write(command);
  // Serial.write(note_id);
  // Serial.write(0x00); // velocity, may as well be silent?
}

void send_expr(uint8_t value, uint8_t type) {
  // Serial.print("expr ");
  // Serial.println(value);
  Serial.write(0xB1); // continuous control command
  Serial.write(type);
  Serial.write(value);
}

void read_port(int port_idx, uint8_t channel) {
  uint8_t key_in = *(PORT_SEQUENCE[port_idx]);
#if ACTIVE_LOW
  key_in = ~key_in; // convert active low to active 1
#endif
  key_in &= PORT_MASKS[port_idx];
  uint8_t changed_keys = (uint8_t)(key_in ^ key_state[channel][port_idx]);
  key_state[channel][port_idx] = key_in;

  // iterate through bits
  for (int bit_idx = 0; bit_idx < 8; bit_idx++) {
    if (changed_keys & _BV(bit_idx)) {
      // TODO: debounce?
      if (key_in & _BV(bit_idx)) {
        send_note_on(channel, NOTE_IDS[port_idx][bit_idx]);
      } else {
        send_note_off(channel, NOTE_IDS[port_idx][bit_idx]);
      }
    }
  }
}

inline void clear_channel_select() {
#if ACTIVE_LOW
  PORTE |= B00111000;
#else
  PORTE &= B11000111;
#endif
}

inline void select_channel(uint8_t channel) {
#if ACTIVE_LOW
  PORTE &= ~_BV(channel + 3);
#else
  PORTE |= _BV(channel + 3);
#endif
}

inline void restart_adc(uint8_t pin) {
  ADMUX &= B11111000;
  ADMUX |= pin;

  ADCSRA |= _BV(ADIF); // clear interrupt flag
  ADCSRA |= _BV(ADSC); // start next conversion
}

void loop() {
  // put your main code here, to run repeatedly:
  for (uint8_t channel = 0; channel < 1; channel++) {

    select_channel(channel);

    if (channel != 2) { // manuals
      for (int port_idx = 0; port_idx < 10; port_idx++) {
        read_port(port_idx, channel);
      }
    } else { // pedals
      for (int port_idx = 0; port_idx < 4; port_idx++) {
        read_port(port_idx, channel);
      }
    }

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
      current_expr = (current_expr + 1) % 3;

      restart_adc(current_expr);
    }

    // deactivate all channels
    // clear_channel_select();
    // _delay_us(2);
    // should we delay slightly before starting again?
  }
}
