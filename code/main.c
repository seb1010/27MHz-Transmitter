#include  <avr/io.h>
#include <util/delay_basic.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "project.h"


uint8_t output_buffer[MAX_PACKET_LEN];
uint8_t input_offset = 0;

#include "drivers/sht4x.c"
#include "coding_and_framing.c"

void mock_sig(uint16_t);
void prbs_8();
extern void phy_send_frame(uint8_t*);
void load_packet(uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t);
void adc_setup();
uint16_t read_adc(uint8_t);

extern void debug_uart_send_byte(uint8_t);

EMPTY_INTERRUPT(PCINT0_vect);

int main () {
  DDRB = (1 << DDB0) | (1 << DDB1) | (1 << DDB2);
  DDRA = (1 << UART_TX) | (1 << EN_POW);
  PORTA = (1 << EN_POW) | (1 << MISO); // enabling pullup on miso
  DDRA |= (1 << 5); // for debug uart

  //debug uart
  DDRA |= (1 << 5);
  PORTA |= (1 << 5);


  PCMSK0 = (1 << PCINT6);
  GIMSK = (1 << PCIE0);

  adc_setup();

  uint16_t temp_code;
  uint16_t rh_code;
  uint16_t battery_counts;
  uint16_t light_counts;
  uint16_t j = 0;

  uint8_t frame_buffer[FRAME_LENGTH];
  uint8_t* data_buffer = add_framing(frame_buffer);

  //about 20 counts per second
  uint16_t sleep_count = 20 * 60;

/*
uint8_t i;
while(1){
  i2c_init();
  sht4x_read_temp_and_rh(&temp_code, &rh_code, 0x44);


debug_uart_send_byte((uint8_t)(temp_code >> 8));
debug_uart_send_byte((uint8_t)temp_code);
debug_uart_send_byte(0xff);

for(i=0;i<10;i++)
  _delay_loop_2(0xffff);

}
*/
//mock_sig(100);
//prbs_8();  

  while(1){

    PORTA &= ~(1 << UART_TX); // output pin low
    PORTA &= ~(1 << EN_POW); // turn off RF chain

    sleep_for_awhile(sleep_count);

    PORTA |= (1 << EN_POW); // turn on RF chain

    i2c_init();
    _delay_loop_2(0x0400); // wait for powerup to happen
    sht4x_read_temp_and_rh(&temp_code, &rh_code, 0x44);

//_delay_loop_2(0xffff);
    light_counts = read_adc(PD_ADC_CH);
    battery_counts = read_adc(BAT_ADC_CH);

    load_packet(data_buffer, temp_code, rh_code,
        battery_counts, light_counts, j); 
    phy_send_frame(frame_buffer);

    j++;
  }
  return 0;
}


uint16_t read_adc(uint8_t channel){
  uint16_t adc_reading;
  ADCSRA |= (1 << ADEN); // enable ADC
  channel &= 0x07;
  ADMUX = ADMUX & 0xf8 | channel; // set correct channel

  ADCSRA |= (1 << ADSC) | (1 << ADIF); // clear flag and start converion 
  while(!(ADCSRA & (1 << ADIF))); // wait for flag to come high
  adc_reading = ADC;

  ADCSRA &= ~(1 << ADEN); // disable ADC

  return adc_reading;
}

void adc_setup(){
  ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // set prescaler
  DIDR0 = (1 << 7) | (1 << 3); // disable digital input on adc pins
}

void sleep_for_awhile(uint16_t sleep_count){
  uint16_t i = 0;
  MCUCR |= (1 << SM1);
  MCUCR &= ~(1 << SM0);
  sei();
  sleep_enable();
  while (i < sleep_count){
    sleep_cpu();
    i++;
  }
  sleep_disable();
  cli();
}

void load_packet(uint8_t* data_buffer,
                          uint16_t temp_code,
                          uint16_t rh_code,
                          uint16_t bat_code,
                          uint16_t pd_code,
                          uint16_t packet_number){
  uint8_t i;
  for(i=0;i<18; i++) // load lots of transistions for cdr
    data_buffer[i] = 0xaa;

  data_buffer[18] = 0x00; // device ID
  data_buffer[19] = 0x20;

  data_buffer[0] = (uint8_t)(packet_number >> 8);
  data_buffer[1] = (uint8_t)packet_number;
   

  data_buffer[4] = (uint8_t)(temp_code >> 8);
  data_buffer[5] = (uint8_t)temp_code;
  data_buffer[8] = (uint8_t)(rh_code >> 8);
  data_buffer[9] = (uint8_t)rh_code;

  data_buffer[10] = (uint8_t)(bat_code >> 8);
  data_buffer[11] = (uint8_t)bat_code;
  data_buffer[12] = (uint8_t)(pd_code >> 8);
  data_buffer[13] = (uint8_t)pd_code;

  encode_data(data_buffer);
}

void send_debug_frame(uint8_t* frame_buffer){
// clocked out frame that without proper timing
  uint8_t i;
  for(i=0;i<FRAME_LENGTH;i++){
    send_debug_byte(frame_buffer[i]);
  }
}

void send_debug_byte(uint8_t data){
  uint8_t i;

  DDRA |= (1 << 5);

  for(i=0;i<8;i++){
    if(!(data & 0x80))
      PORTA |= (1 << UART_TX);
    else
      PORTA &= ~(1 << UART_TX);

    PORTA |= (1 << 5);
    _delay_loop_2(0x0c);
    PORTA &= ~(1 << 5);
    _delay_loop_2(0x0c);
    data <<= 1;
  }
}

void mock_sig(uint16_t delay){
  uint16_t i;
  while(1){
    for(i=0; i<delay; i++)
      _delay_loop_1(0x01);
    PORTA ^= (1 << UART_TX);
  }
}

void prbs_8(){
  uint8_t poly, port, count, reg = 0x59;

  poly = (1 << 4) | (1 << 3) | (1 << 2) | (1 << 0);
  port = PORTA;
  asm volatile(
    "main_loop: \n\t"
      "lsl %[reg] \n\t"
      "brcc no_eor \n\t"
        "eor %[reg], %[poly] \n\t"
      "no_eor: \n\t"      

    "cbr %[port], (1 << %[tx_pin]) | (1 << 5) \n\t" // delete (1 << 5)
    "sbrc %[reg], 0 \n\t"
      "sbr %[port], (1 << %[tx_pin]) \n\t"
    "out %[port_addr], %[port] \n\t"

    "ldi %[cnt], 50 \n\t"
    "stupid_loop: \n\t"
    "dec %[cnt] \n\t"
    "brne stupid_loop \n\t"

    "rjmp main_loop \n\t"
  :
  : [poly] "r" (poly),
    [port] "a" (port),
    [reg] "a" (reg),
    [cnt] "r" (count),
    [tx_pin] "M" (UART_TX),
    [port_addr] "M" (_SFR_IO_ADDR(PORTA))
  );

}
