#include "avr/io.h"
#include "project.h"
#include "utils/uart/uart_10k.S"

.global phy_send_frame
phy_send_frame:
  push r16
  in r16, _SFR_IO_ADDR(SREG)
  push r16
  push r17
  push r18
  push r19
  push r20
  push r26
  push r27

  mov r26, r24 // frame pointer low byte
  mov r27, r25 // frame pointer high byte

  ; send clock sync
  ld r16, x
  in r17, _SFR_IO_ADDR(PORTA)

  ldi r18, 50
  sync_loop:
    nop
    nop
    rcall byte_out_loop
    dec r18
    brne sync_loop

  ldi r18, FRAME_LENGTH
  frame_loop:
    ld r16, x+
    rcall byte_out_loop

    dec r18
    brne frame_loop
    
  pop r27
  pop r26
  pop r20
  pop r19
  pop r18
  pop r17
  pop r16
  out _SFR_IO_ADDR(SREG), r16
  pop r16
  ret


byte_out_loop: // careful lots of register reuse
  push r16
  push r18
  
  ldi r18, 8
  bit_out_loop:
    cpi r18, 8
    breq skip_extra_delay
    ldi r19, 7
    mini_delay_loop:
      dec r19
      brne mini_delay_loop
    skip_extra_delay:

    sbr r17, (1 << UART_TX)
    sbrs r16, 7
    cbr r17, (1 << UART_TX)
    out _SFR_IO_ADDR(PORTA), r17
    lsl r16

//sbi _SFR_IO_ADDR(PINA), 5
//sbi _SFR_IO_ADDR(DDRA), 5
//sbi _SFR_IO_ADDR(PINA), 5

    ldi r19, 43
    main_delay_loop:
      dec r19
      brne main_delay_loop

  dec r18
  brne bit_out_loop

  pop r18
  pop r16
  ret
