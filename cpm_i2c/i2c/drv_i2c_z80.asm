; driver for the Phillips I2C bus controller PCF8584 for 8085 CPU
; uses no IRQs, all code control is done via status register
                .module PCF8584_DRIVER
                ;.area   SMALLC_GENERATED  (REL,CON,CSEG)
                ;.list   (err, loc, bin, eqt, cyc, lin, src, lst, md)
                ;.nlist  (pag)
                ;.globl  pcf8584_init, reset_i2c_bus
                ;.globl  send_buf, recv_buf, send_buf_addr, udelay

PCF8584_S0      .equ    0x58                ;data register
PCF8584_S1      .equ    PCF8584_S0+1        ;control/status register
OWN_ADDR        .equ    0x55                ;effective own I2C address is shifted to left
                                            ;it will become AAH
CLCK_443        .equ    0x10                ;system clock is 4.43 MHz; SCL = 90 kHz
CLCK_600        .equ    0x14                ;system clock is 6 MHz; SCL = 90 kHz
TIMEOUT         .equ	0                   ;timeout (256 tries)
                ; control bits
PIN             .equ	0x80                ;pending interrupt (is also status bit)
ESO             .equ	0x40                ;enable serial output
ES1             .equ    0x20                ;controls selection of other registers
ES2             .equ    0x10                ;controls selection of other registers
ENI             .equ    0x08                ;enable external interrupt output
STA_            .equ    0x04                ;generate start condition (STA is reserved 8080 instruction code)
STO             .equ    0x02                ;generate stop condition
ACK             .equ    0x01                ;generate acknowledge
                ; status bits
STS             .equ    0x20                ;stop detected (when in slave mode)
BER             .equ    0x10                ;bus error
LRB             .equ    0x08                ;last received bit
AAS             .equ    0x04                ;addressed as slave bit
LAB             .equ    0x02                ;lost arbitration bit
BB              .equ    0x01                ;bus busy bit
                ;
SEND_I2C_START 	.equ    PIN | ESO | STA_
SEND_I2C_STOP   .equ    PIN | ESO | STO
CLEAR_I2C_BUS   .equ    PIN | ESO | ACK
NEGATIVE_ACK    .equ    ESO                 ;ACK bit is hereby set to 0, that causes negative ack
                                            ;which halts further transmision
S2              .equ    PIN + ES1
RESET           .equ    PIN
                ;
OK              .equ    0
ERR_WAIT_PIN    .equ    1
ERR_NO_ACK      .equ    2
ERR_BUS_BUSY    .equ    3
                ;
                ; wait aprox. 80us (42t fixed + 24t x LOOP)
                ; 5MHz CPU 400 cycles (act.402)
                ; 8MHz CPU 640 cycles
udelay:         push bc                      ;[12]
                ld  b,15                    ;[10]  const CRYSTAL/2
dela1:          dec bc                       ;[6]
                ld  a,b                     ;[4]
                or	a,c                       ;[4]
                jp nz, dela1                   ;[10]
                pop bc                       ;[10]
                ret                         ;[10]
pcf8584_init:   ; initialize controller
                ld  a,RESET                 ; will also choose register S0_OWN i.e. next byte will be
                out (PCF8584_S1), A              ; loaded into reg S0^ (own address reg); serial interface off.
                ld  a,OWN_ADDR              ; loads byte 55H into reg S0^ effective own address becomes AAH.
                out (PCF8584_S0), A              ; pcf8584 shifts this value left one bit
                ld  a,S2                    ; loads byte A0H into reg S1, i.e. next byte will
                out (PCF8584_S1), A              ; be loaded into the clock control reg S2.
                ld  a,CLCK_443              ; loads byte 10H into reg S2;system clock is 4.43 MHz; SCL = 90 kHz
                out (PCF8584_S0), A
                ld  a,CLEAR_I2C_BUS         ; loads byte C1H into reg S1; reg enable serial interface
                out (PCF8584_S1), A              ; next write or read operation will be to/from data transfer reg S0
                call udelay
                ret
reset_i2c_bus:  ; reset I2C bus
                ld  a,SEND_I2C_STOP         ; stop C2
                out (PCF8584_S1), A              ; send to S1
                call udelay                 ; wait
                ld  a,CLEAR_I2C_BUS         ; clear C1
                out (PCF8584_S1), A              ; send to S1
                ret
wait_for_bus:   ; wait for bus, set CY on timeout
                push bc                      ; backup BC
                ld  b,TIMEOUT               ; timeout constant
wfbl1:          in A, (PCF8584_S1)               ; read status
                and a, BB                      ; zero indicates that the bus is busy
                jp	z, wfbl2                    ; bit=0, bus still busy
                pop bc                       ; restore BC
                ret                         ; bit=1, I2C bus access now possible
wfbl2:          dec bc                       ; timeout reached?
                jp nz, wfbl3                   ; no
                scf                         ; yes, set CY
                pop bc                       ; restore BC
                ld  h,ERR_BUS_BUSY          ; error code
                ret                         ; return
wfbl3:          call udelay                 ; wait
                jp  wfbl1                   ; try again
wait_for_pin:   ; wait for PIN, set CY on timeout
                push bc                      ; backup BC
                ld  b,TIMEOUT               ; timeout constant
wfpl1:          in A, (PCF8584_S1)               ; read status
                and a, PIN                     ; bit=1, transmission is in progress
                jp nz, wfpl2                   ;
                pop bc                       ; bit=0, restore BC, return
                ret                         ; PIN cleared, transmission completed
wfpl2:          dec bc                       ; timeout reached?
                jp nz, wfpl3                   ; no
                scf                         ; yes, set CY
                pop bc                       ; restore BC
                ld  h,ERR_WAIT_PIN          ; error
                ret                         ; return
wfpl3:          call udelay                 ; wait
                jp  wfpl1                   ; try again
xmit_raddr:     ; transmit address in read mode, A address
                or	A, 0x01                    ; set last address bit in READ mode
                jp  xmit_start              ; begin transmission
xmit_waddr:     ; transmit address in write mode, A address
                and a, 0xFE                    ; clear last address bit in WRITE mode
                ;jp  xmit_start             ; begin transmission - fall through
xmit_start:     ; common routine, begin address transmission
                out (PCF8584_S0), A              ; send(set) address
                ld  a,SEND_I2C_START+ACK    ; begin communication
                out (PCF8584_S1), A              ; the transmission of address from S0 is now started
                call wait_for_pin           ; wait for the i2c send to finish
                ret	c                          ; error, abort
                ;jp  check_ack              ; check slave acknowledge - fall through
check_ack:      ; check if peer acknowledged
                in A, (PCF8584_S1)               ; read status
                and a, LRB                     ; check LRB
                ret	z                          ; LRB=0, return
                scf                         ; peer did not acknowledge, set CY
                ld  h,ERR_NO_ACK            ; set error code
                ret                         ; return
xmit_byte:      ; transmit one byte
                out (PCF8584_S0), A              ; send byte
                call wait_for_pin           ; wait for the i2c send to finish
                ret c                          ; error, abort
                jp  check_ack               ; check slave acknowledge
recv_byte:      ; receive one byte
                call wait_for_pin           ; wait for the i2c read to finish
                ret c                          ; error abort
                in A, (PCF8584_S0)               ; read byte
                ld (de), A                      ; store byte to dest.buffer
                scf                         ;set flacg carry
				ccf							; clear CY
                ;cmc                         ;
                ret
recv_buf:       ; receive buffer, A-I2C address, BC-length, DE-destination
                ; receives data from device, writes it to the memory buffer
                ld  h,a                     ; back up I2C address
                call wait_for_bus           ; return when bus not available
                ret c                          ; bus busy error
                ld  a,h                     ; restore address
                call xmit_raddr             ; send I2C address
                jp	c, rcerr                    ; error
                in A, (PCF8584_S0)               ; dummy read, begins transfer of first value from bus to S0
                                            ; therefore this very first value must be discarded
rcb1:           dec bc                       ; counter
                ld  a,b                     ; check BC
                or  a,c                       ; both zero?
                jp	z, rcdone                   ; last byte
                call recv_byte              ; receive one byte
                call	c, check_ack               ; did master (this controller) acknowledge?
                jp	c, rcerr                    ; error occured
                inc de                       ; increment destination pointer
                jp  rcb1                    ; next byte 
rcdone:         ld  a,NEGATIVE_ACK          ; last byte (master receiver mode)
                out (PCF8584_S1), A              ; send negative acknowledge
                call recv_byte              ; receive last byte
                jp	c, rcerr                    ; error occured
                jp  done                    ; release bus
send_buf:       ; send buffer, A-I2C address, C-length, HL-source
                ; sends buffer content to the device (pure data)
                ld  d,a                     ; back up I2C address
                call wait_for_bus           ; return when bus not available
                ret c                          ; bus busy error
seb1:           ld  a,d                     ; restore address
                call xmit_waddr             ; send address
                jp	c, rcerr                    ; error
seb2:           ld  a,(HL)                     ; get byte
                call xmit_byte              ; transmit it
                jp	c, rcerr                    ; error occured
                dec bc                       ; counter
                jp	z,  done                    ; last byte?
                inc hl                       ; increment source pointer
                jp  seb2                    ; next byte
send_buf_addr:  ; send buffer with address, A-I2C address, C-length, HL-source
                ; DE-dest. device internal address, sends device's internal dest. 
                ; address in the first two bytes, followed by buffer content
                ; e.g. as needed by EEPROMs, this convinience method should
                ; actually be in EEPROMs driver file but it has dependences here
                push de                      ; back up DE
                ld  d,a                     ; back up I2C address
                call wait_for_bus           ; return when bus not available
                ret c                          ; bus busy error
seba1:          ld  a,d                     ; restore address
                call xmit_waddr             ; send device I2C address
                pop de                       ; restore DE
                jp	c, rcerr                    ; error occured
                ld  a,d                     ; higher byte of dest.address
                call xmit_byte              ; transmit it
                ld  a,e                     ; lower byte of dest.address
                call xmit_byte              ; transmit it
                jp  seb2                    ; continue with the data
rcerr:          ld  a,SEND_I2C_STOP         ; error
                out (PCF8584_S1), A              ; send stop
                scf                         ; error flag
                ret                         ; abort
done:           ld  a,SEND_I2C_STOP + ACK   ; done, stop
                out (PCF8584_S1), A              ; send ack
                ld  h,OK                    ; all OK
                ret
                ;
                .endmodule
