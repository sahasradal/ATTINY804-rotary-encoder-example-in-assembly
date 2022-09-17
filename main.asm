;
; 804rotary.asm
; works 2 steps per detent =,+,clk , data without lsr r16 in display loop
; works 1 step/detent with lsr r16 (count divide by2) inside display loop
; Created: 06/09/2022 19:00:53
; Author : Manama
;


.def	save_SREG	= r14
.equ data_command1 = 0b00001001		; data control nibble ,led on P3, EN 0 on P2, R/W 0 (write) in P1 , RS 1 (0 instruction, 1 data) = 1001  =0x09
.equ data_command2 = 0b00001101		; data control nibble , 1101  = 0x0D   - EN goes hi=1
.equ data_command3 = 0b11111011		; data control nibble , 1001  = 0x09   - EN goes low=0
.equ inst_command1 = 0b00001000		; instruction control nibble ,  led on en-lo,Rw-0,rs =0   = 1000   = 0x08
.equ inst_command2 = 0b00001100		; instruction control nibble ,  led on,EN hi , rs/RW 0    = 1100   = 0x0C
.equ inst_command3 = 0b11111011		; instruction control nibble  , led on, EN lo ,rs/rw 0    = 1000   = 0x08
.EQU SLAVE_ADDRESSW = 0X4E			; OLED = 0X78 ,1602 =4E
.equ SLAVE_ADDRESSR = 0x4F			; 1602 adress + read command
.equ fclk = 10000000
.DEF  SLAVE_REG = R17
.DEF  TEMP = R16
.def data = r3
.def HundredCounter = r25
.def TenthCounter = r24		;	
.def OneCounter   = r23		;
.def ZEROREG      = r22		;



.macro micros					; macro for delay in us
ldi temp,@0
rcall delayTx1uS
.endm

.macro millis					; macro for delay in ms
ldi YL,low(@0)
ldi YH,high(@0)
rcall delayYx1mS
.endm

.macro pos						; macro for sending cursor position to LCD 
ldi r16,@1
mov r6,r16
ldi r16,@0
rcall posi
.endm

.macro arrayread				; macro for sending a string to LCD
ldi ZL,low(2*@0)
ldi ZH,high(2*@0)
rcall array
.endm



.dseg
BUFFER: .byte 20
PAD: .BYTE 1
PAD1: .byte 1
new: .byte 1
old: .byte 1
count0: .byte 1
count1: .byte 1
count2: .byte 1
count3: .byte 1





.cseg
.ORG 0X00
	rjmp main
.org 0x03
	rjmp PORTA_ISR


	

PORTA_ISR:
	lds r14,CPU_SREG	; save status register
	push r16			; save r16
	push r17			; save r17
	clr r27				; clear r27 for 0 and carry propagation
	lds r16,PORTA_IN	; copy portA pin status to r16
	andi r16,0b00000110 ; bit 1 & 2 is to be read PA1 & PA2
	lsr r16				; right shift 1 step to convert red values in the range of 3,2,1 or 0 (00,01,11,10,00)
	sts new,r16			; store a copy of the new position of ENCODER to SRAM register named "new"
	lds r17,old			; copy old encoder position /previous position before interrupt
	eor r16,r17			; exclusiveOR old & new encoder values.the changed signal will read 1 , 0 is unchanged (11eor10 = 01 1st bit changed)
	cpi r16,1			; signal a changed , bit 0 is considered as A signal
	breq A				; branch to label A if r16 = 1 (01)
	cpi r16,2			; signal b changed , bit 1 is considered as B signal
	breq B				; branch to label B if r16 = 2 (10)
	lds r16,PORTA_INTFLAGS	; copy interrupt flag
	sts PORTA_INTFLAGS,r16	; write back interrupt flag to clear pin change interrupt flags
	pop r17				; restore r17
	pop r16				; restore r16
	sts CPU_SREG,r14	; restore status register
	reti				; any other value than compared above (1,2) exit interrupt as value is invalid 

A:						; this block of code executes if signal A changed level
	lds r16,new			; copy new encoder value to r16 from SRAM
	lds r17,old			; copy old encoder value to r17 from SRAM
	mov r18,r16			; create another copy of new value in R18
	andi r18,1			; and r18 with 1 to isolate bit 0 , this checks whether A is high(1) or low(0)
	brne Ahigh			; if r18 signal A (bit0) is high branch to Ahigh label
	mov r18,r17			; if r18 signal A (bit0) is low copy old encoder value to r18 , this is to check old B relative to new A
	andi r18,2			; and r18 with 0b10(2) to isolate bit1
	brne jccw			; if r18 bit1 is set branch to label jccw (IF A is HIGH & B is HIGH count++[cw] else count--[ccw])
	rjmp cw				; if r18 bit1 is cleared (0) jump to label cw (IF A is LOW & B is LOW count++[cw] else count--[ccw])

Ahigh:					; reach here if signal A changed and is high
	mov r18,r17			; move another copy of old encoder value in r18
	andi r18,2			; isolate bit1 (signal B,static), next instruction checks whether B is high or low relative to A
	brne jcw			; rule (IF A is HIGH & B is HIGH count++[cw] else count--[ccw])
	rjmp ccw			; rule (IF A is HIGH & B is HIGH count++[cw] else count--[ccw])

B:						; reach here if signal B changed level and caused interrupt on PA2
	lds r16,new			; copy new encoder value to r16 from SRAM
	lds r17,old			; copy old encoder value to r17 from SRAM
	mov r18,r16			; create another copy of new value in R18
	andi r18,2			; and r18 with 2 to isolate bit 1 , this checks whether B is high(1) or low(0)
	brne Bhigh			; if r18 signal B (bit1) is high branch to Bhigh label
	mov r18,r17			; if r18 signal B (bit1) is low copy old encoder value to r18 , this is to check old A relative to new B
	andi r18,1			; and r18 with 0b01(1) to isolate bit0
	brne jcw			; if r18 bit0 is set branch to label jcw (IF B is LOW & A is HIGH count++[cw] else count--[ccw])
	rjmp ccw			; if r18 bit0 is cleared (0) jump to label ccw (IF B is LOW & A is HIGH count++[cw] else count--[ccw])

Bhigh:					; reach here if B changed level and is high
	mov r18,r17			; move another copy of old encoder value in r18 
	andi r18,1			; isolate bit0 (signal A,static), next instruction checks whether A is high or low relative to B
	brne jccw			; rule (IF B is HIGH & A is LOW count++[cw] else count--[ccw])
	rjmp cw				; rule (IF B is HIGH & A is LOW count++[cw] else count--[ccw])

jcw:
	rjmp cw
jccw:
	rjmp ccw
cw:						; reach here if either B or A changed and is in clockwise direction
	lds r16,count0		; copy to r16 current count value (0-255)
	subi r16,-1			; add 1 to current count value
	sts count0,r16		; store the new count value back in SRAM register count0
	lds r16,new			; copy new encoder value of A&B signals
	sts old,r16			; store new encoder value in old for reference during next interrupt
	lds r16,PORTA_INTFLAGS	; copy interrupt flag
	sts PORTA_INTFLAGS,r16	; write back interrupt flag to clear pin change interrupt flags
	pop r17				; restore r17
	pop r16				; restore r16
	sts CPU_SREG,r14	; restore status register
	reti				; return from interrupt

ccw:					; reach here if either B or A changed and is in anticlockwise direction
	lds r16,count0		; copy to r16 current count value (0-255)
	subi r16,1			; subtract 1 from current count value
	sts count0,r16		; store the new count value back in SRAM register count0
	lds r16,new			; copy new encoder value of A&B signals
	sts old,r16			; store new encoder value in old for reference during next interrupt
	lds r16,PORTA_INTFLAGS	; copy interrupt flag
	sts PORTA_INTFLAGS,r16	; write back interrupt flag to clear pin change interrupt flags
	pop r17				; restore r17
	pop r16				; restore r16
	sts CPU_SREG,r14	; restore status register
	reti				; return from interrupt
	








	


	


main:
	rcall PROT_WRITE					; changes clock speed to 10mhz
	rcall TWI_INIT						; initialize TWI
	LDI SLAVE_REG,SLAVE_ADDRESSW		; LCD write address is transmitted
	rcall TWI_START						; TWI start routine
	rcall LCD_INIT						; LCD initialization routine
	pos 0 , 0							; cursor on the first line left most position
    arrayread message					; displays message "hello"
	millis 1000							; delay 1000 ms
	rcall TWI_STOP						; stop TWI transmission to LCD

setup:
	clr r27
	sts count0,r27
	sts count1,r27
	sts count2,r27
	sts count3,r27
	ldi r16,0b00001001				; pull up on and interrupt on rising &  falling edge PA1
	sts PORTA_PIN1CTRL,r16
	sts PORTA_PIN2CTRL,r16			; pull up on and interrupt on rising &  falling edge PA2
	lds r16,PORTA_IN				; copy porta_IN to get the current values of PA1 & PA2 , this is the initial state of the rotary encoder
	andi r16, 0b00000110			; discard all other bits ,we are interested in PA1 and PA3
	lsr r16							; logical shift to right to bring the read values in the range of 0,1,2,3 as out put by a rotary encoder
	sts OLD,r16						; store in sram register OLD as this value will be used as the initial position of the rotary encoder to calculate valid new positions
	sei								; enable interrupts globally
	rcall TWI_INIT					; initialize TWI
	LDI SLAVE_REG,SLAVE_ADDRESSW	; LCD write address is transmitted
	rcall TWI_START					; TWI start routine



loop:
	rcall clear_screen				; clear LCD screen
	lds r16,count0					; copy value in count0 to r16
	lsr r16							; this step divides count by 2 as chinese encoder have 2 counts per detent , this makes 1 step/detent
	lds r17,count1					; copy value in count1 to r17
	rcall ASCII_CONVERT				; call subroutine to convert binary numbers to ASCII values to be printed on LCD
	pos 0, 1						; cursor positioned at first line and 1 step to rhs
	mov r17,HundredCounter			
	rcall data_write
	mov r17,TenthCounter
	rcall data_write
	mov r17,OneCounter
	rcall data_write
	ldi r17,' '
	rcall data_write
	millis 10
	rjmp	loop

message:
.db "Hello! ",0 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;PROTECTED WRITE for processor speed
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PROT_WRITE:
		ldi r16,0Xd8
		out CPU_CCP,r16
		ldi r16,0x01						; clk prescaler of 2, 20Mhz/2 = 10Mhz
		STS CLKCTRL_MCLKCTRLB,R16
		RET
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C ROUTINES
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


TWI_INIT:
		ldi r16,80
		sts TWI0_MBAUD,R16
		LDI R16,0b00000011			;SMEN,ENABLE
		STS TWI0_MCTRLA,R16
		LDI R16,0b00001000			;FLUSH ADDR & DATA REGISTERS
		STS TWI0_MCTRLB,R16
		LDI R16,0b00000001			;FORCE IDLE
		STS TWI0_MSTATUS,R16
		ret
		


TWI_START:
		MOV R16,SLAVE_REG			;SLAVE_REG IS R17, READ OR WRITE ADDRESS SHOULD BE LOADED HERE PRIOR TO CALL
		STS TWI0_MADDR,R16
		RCALL WAIT_WIF
		rcall CHECKACK				;checks whether slave has acked
		ret

TWI_WRITE:
		MOV R16,SLAVE_REG
		STS TWI0_MDATA,R16
		RCALL WAIT_WIF
		rcall CHECKACK
		micros 10
		ret

CHECKACK:
		push r19
		clr r19
	ACK:
		inc r19
		cpi r19,30
		breq NOACK
		LDS R16,TWI0_MSTATUS
		SBRC R16,4
		rjmp ACK
	NOACK:	pop r19
		RET

/*
TWI_READ:
		ldi SLAVE_REG,DS1307WAD
		rcall TWI_START
		ldi SLAVE_REG,0x00			; send instruction/READ_ADDRESS to THE SLAVE FROM WHICH DATA IS READ ,first register of DS1307 is 0x00
		rcall TWI_WRITE
		ldi r16,0x00				;loading 0 in ACKACT bit enables master to send ack after reading data register
		sts TWI0_MCTRLB,r16
		ldi r16,DS1307RAD			; repeated start ;  I2C slave address + read bit (1) SHOULD BE LOADED HERE FOR READING DATA FROM SLAVE READ_ADDRESS GIVEN ABOVE
		STS TWI0_MADDR,R16
		rcall WAIT_RIF

		ldi r16,read_data_len		;load r16 with number of bytes to be read
		cpi r16,0x02				;is num of bytes less than or greater than 2
		brlo BYYTE					;if less than 2 branch to 1BYTE as NACK+STOP will be loaded prior to read
		dec r16						; decreace one count from the total count to get loop value,NACK should be sent before the last byte read
		mov r5,r16					; move the count -1 value to counter r5
loop_read:
		LDS R16,TWI0_MDATA			;MDATA REGISTER IS COPIED TO R16,DATA IS RECIVED INTO MDATA FROM SLAVE
		ST X+,R16					;DATA IN R16 IS STORED IN SRAM BUFFER FOR LATER USE. 
		RCALL WAIT_RIF				;wait for read flag
		dec r5						;decrease counter after each read
		brne loop_read				;go throug loop till {count - 1} is finished
BYYTE: 
		LDI R16,0b00000111			;CLEAR ACKACT BIT BEFORE READING LAST BYTE AND ISSUE A STOP = NACK+STOP
		STS TWI0_MCTRLB,R16
		LDS R16,TWI0_MDATA			;MDATA REGISTER IS COPIED TO R16,THIS THE LAST DATA IS RECEIVED  FROM SLAVE
		ST X+ ,R16					;DATA IN R16 IS STORED IN SRAM BUFFER FOR LATER USE. 
		RET
*/

TWI_STOP:
		LDI R16,0b00000011          ;STOP
		STS TWI0_MCTRLB,R16
		RET


WAIT_WIF:
		LDS R16,TWI0_MSTATUS
		SBRS R16,6					;CHECK WIF IS SET,IF SET SKIP NEXT INSTRUCTION (write interrupt flag)
		RJMP WAIT_WIF
		RET


WAIT_RIF:
		LDS R16,TWI0_MSTATUS
		SBRS R16,7					;CHECK RIF IS SET,IF SET SKIP NEXT INSTRUCTION (read interrupt flag)
		RJMP WAIT_RIF
		RET


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;LCD ROUTINES for sending commands and data
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
COMMAND_WRITE:
		STS PAD,R17					;copy SLAVE_REG to SRAM address PAD for processing
		ANDI R17,0XF0				;preserve upper nibble of SLAVE_REG by anding with 0xF0 ,lower 4 becomes 0
		ORI R17,inst_command1		;add instruction_command1 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		rcall twentyms
		ORI R17,inst_command2		;add instruction_command2 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		rcall twentyms
		ANDI R17,inst_command3		;add instruction_command3 to lower nibble of r17 by OR ing it
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		rcall twentyms		
		LDS R17,PAD					;copy back data from PAD to r17 for processing the remaining lower nibble
		SWAP R17					;swap the nibbles in R17 so thlower nibble will occupy the upper area of reg
		ANDI R17,0XF0				;preserve upper nibble of SLAVE_REG by anding with 0xF0
		ORI R17,inst_command1		;add instruction_command1 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		rcall twentyms
		ORI R17,inst_command2		;add instruction_command2 to lower nibble of r17 by OR ing it
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		rcall twentyms
		ANDI R17,inst_command3		;add instruction_command3 to lower nibble of r17 by OR ing it
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		rcall twentyms
		RET	


DATA_WRITE:
		STS PAD,R17					;copy SLAVE_REG to SRAM address PAD for processing
		ANDI R17,0XF0				;preserve upper nibble of SLAVE_REG by anding with 0xF0,lower 4 becomes 0
		ORI R17,data_command1		;add data_command1 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		rcall twentyms
		ORI R17,data_command2		;add data_command2 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		rcall twentyms
		ANDI R17,data_command3		;add data_command3 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		rcall twentyms
		LDS R17,PAD					;copy back data from PAD to r17 for processing the remaining lower nibble
		SWAP R17					;swap the nibbles in R17 so thlower nibble will occupy the upper area of reg
		ANDI R17,0XF0				;preserve upper nibble of SLAVE_REG by anding with 0xF0
		ORI R17,data_command1		;add data_command1 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		rcall twentyms
		ORI R17,data_command2		;add data_command2 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		rcall twentyms
		ANDI R17,data_command3		;add data_command3 to lower nibble of r17 by OR ing it 
		RCALL TWI_WRITE				;call TWI_WRITE routine to transmit command data to LCD
		rcall twentyms
		RET	

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Hitachi LCD 1602 initialisation procedure
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

LCD_INIT:
		rcall fiftyms				; call 50ms prewritten code with 50ms macro to reduce code size due to repetition of macro
		LDI R17,0b00111100
		RCALL TWI_WRITE
		rcall twentyms				; call 20ms prewritten code with 20ms macro to reduce code size due to repetition of macro
		LDI R17,0B00111000
		RCALL TWI_WRITE
		rcall fiftyms				; call 50ms prewritten code with 50ms macro to reduce code size due to repetition of macro
 		

		
		LDI R17,0b00111100
		RCALL TWI_WRITE
		rcall twentyms				; call 20ms prewritten code with 20ms macro to reduce code size due to repetition of macro
		LDI R17,0B00111000
		RCALL TWI_WRITE
		rcall fiftyms				; call 50ms prewritten code with 50ms macro to reduce code size due to repetition of macro
 		

		
		LDI R17,0b00111100
		RCALL TWI_WRITE
		rcall twentyms				; call 20ms prewritten code with 20ms macro to reduce code size due to repetition of macro
		LDI R17,0B00111000
		RCALL TWI_WRITE
		rcall fiftyms				; call 50ms prewritten code with 50ms macro to reduce code size due to repetition of macro
 		

		
		LDI R17,0b00101100
		RCALL TWI_WRITE
		rcall twentyms				; call 20ms prewritten code with 20ms macro to reduce code size due to repetition of macro
		LDI r17,0b00101000
		RCALL TWI_WRITE
		rcall fiftyms				; call 50ms prewritten code with 50ms macro to reduce code size due to repetition of macro
 		

		
		LDI R17,0b00101000
		RCALL COMMAND_WRITE
		rcall twentyms				; call 20ms prewritten code with 20ms macro to reduce code size due to repetition of macro

		
		LDI R17,0b00001100
		RCALL COMMAND_WRITE
		rcall twentyms				; call 20ms prewritten code with 20ms macro to reduce code size due to repetition of macro

		
		LDI R17,0b00000110
		RCALL COMMAND_WRITE
		rcall twentyms				; call 20ms prewritten code with 20ms macro to reduce code size due to repetition of macro

		
		LDI R17,0b00000001
		RCALL COMMAND_WRITE
		rcall twentyms				; call 20ms prewritten code with 20ms macro to reduce code size due to repetition of macro
		RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;the below routines are for recurring delays used in the program
;using macros will increase code size so created these subroutines to be called
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

fiftyms:
millis 50
ret

twentyms:
micros 20
ret

twotoums:                           
millis 2000
ret	

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; below routines used part of macro pos and macro arrayread for cursor position and reading array messages
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

posi:
		cpi r16,0x00					;check the first parameter of macro is 0 or a higher number
		breq line1						;if zero go to label first line
		ldi r16,0xc0					; if not zero the input is for 2nd line. load 0xC0 in r16 which is address of 2nd line first position DDRAM
		add r6,r16						; add with horizontal postion on 2nd line to get the correct DDRAM address
		mov SLAVE_REG,r6				; copy new LCD DDRAM address to SLAVE_REG
		rcall COMMAND_WRITE				; call TWI transmit command
		ret
	line1:
		ldi r16,0x80					; if Y = 0 which means 1st line of LCD load address of 1st line 1st position DDRAM =0x80
		add r6,r16						; add 0x80 to X position saved in r6 to get the start postion on 1st line
		mov SLAVE_REG,r6				; copy new LCD DDRAM address to SLAVE_REG
		rcall COMMAND_WRITE				; call TWI transmit command
		ret


array:
		lpm SLAVE_REG,Z+				;load from program memory to r17 data pointed by Z pointer
		cpi SLAVE_REG,0					; check for null terminator in the string
		breq exit						; if zero go to exit
		rcall DATA_WRITE				; transmit copied data to LCD via TWI
		rjmp array						; jump back to array until null
	exit: 	ret


NO1line:                            ; subroutine created for cursor to pos 0,0 as using macro will increase code size
pos 0, 0
ret



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; ============================== Time Delay Subroutines =====================
; Name:     delayYx1mS
; Purpose:  provide a delay of (YH:YL) x 1 mS
; Entry:    (YH:YL) = delay data
; Exit:     no parameters
; Notes:    the 16-bit register provides for a delay of up to 65.535 Seconds
;           requires delay1mS

delayYx1mS:
    rcall    delay1mS                        ; delay for 1 mS
    sbiw    YH:YL, 1                        ; update the the delay counter
    brne    delayYx1mS                      ; counter is not zero

; arrive here when delay counter is zero (total delay period is finished)
    ret
; ---------------------------------------------------------------------------
; Name:     delayTx1mS
; Purpose:  provide a delay of (temp) x 1 mS
; Entry:    (temp) = delay data
; Exit:     no parameters
; Notes:    the 8-bit register provides for a delay of up to 255 mS
;           requires delay1mS

delayTx1mS:
    rcall    delay1mS                        ; delay for 1 mS
    dec     temp                            ; update the delay counter
    brne    delayTx1mS                      ; counter is not zero

; arrive here when delay counter is zero (total delay period is finished)
    ret

; ---------------------------------------------------------------------------
; Name:     delay1mS
; Purpose:  provide a delay of 1 mS
; Entry:    no parameters
; Exit:     no parameters
; Notes:    chews up fclk/1000 clock cycles (including the 'call')

delay1mS:
    push    YL                              ; [2] preserve registers
    push    YH                              ; [2]
    ldi     YL, low(((fclk/1000)-18)/4)     ; [1] delay counter              (((fclk/1000)-18)/4)
    ldi     YH, high(((fclk/1000)-18)/4)    ; [1]                            (((fclk/1000)-18)/4)

delay1mS_01:
    sbiw    YH:YL, 1                        ; [2] update the the delay counter
    brne    delay1mS_01                     ; [2] delay counter is not zero

; arrive here when delay counter is zero
    pop     YH                              ; [2] restore registers
    pop     YL                              ; [2]
    ret                                     ; [4]

; ---------------------------------------------------------------------------
; Name:     delayTx1uS
; Purpose:  provide a delay of (temp) x 1 uS with a 16 MHz clock frequency
; Entry:    (temp) = delay data
; Exit:     no parameters
; Notes:    the 8-bit register provides for a delay of up to 255 uS
;           requires delay1uS

delayTx1uS:
    rcall    delay10uS                        ; delay for 1 uS
    dec     temp                            ; decrement the delay counter
    brne    delayTx1uS                      ; counter is not zero

; arrive here when delay counter is zero (total delay period is finished)
    ret

; ---------------------------------------------------------------------------
; Name:     delay10uS
; Purpose:  provide a delay of 1 uS with a 16 MHz clock frequency ;MODIFIED TO PROVIDE 10us with 1200000cs chip by Sajeev
; Entry:    no parameters
; Exit:     no parameters
; Notes:    add another push/pop for 20 MHz clock frequency

delay10uS:
    ;push    temp                            ; [2] these instructions do nothing except consume clock cycles
    ;pop     temp                            ; [2]
    ;push    temp                            ; [2]
    ;pop     temp                            ; [2]
    ;ret                                     ; [4]
     nop
     nop
     nop
     ret

; ============================== End of Time Delay Subroutines ==============

clear_screen:		; routine to clear LCD screen and reposition cursor to 0,0
LDI R17,0b00000001
RCALL COMMAND_WRITE	; call routine to transmit LCD commands
micros 20
RET


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;* Integer to ASCII converter subrountine here ,r17:r16 is the input registers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ASCII_CONVERT:
	ldi		HundredCounter,0	; set counters values to zero
	ldi		TenthCounter,0		;	
	ldi		OneCounter,0		;
	ldi		ZEROREG,48			; ASCII = Number + 48; In ASCII table, we can see that '0' is 48 (0x30); therefore, need to add 48 to the single digits
								; which convert integer value to
								; ASCII value
IntToASCII:
	LDI R18,HIGH(100)			;16bit comparison
	CLC							;clear carry
	CPI R16,100					;16bit comparison
	CPC R17,R18					;16bit comparison
	
	brge	DivideBy100			; jump if (intVal>=100) 
	LDI R18,HIGH(10)			;16bit comparison
	CLC							;clear carry
	CPI R16,10					;16bit comparison
	CPC R17,R18					;16bit comparison
	
	brge	DivideBy10				; jump if (intVal>=10)
	mov		OneCounter,r16
	add		HundredCounter,ZEROREG	; here, we got there single digits
	add		TenthCounter,ZEROREG	; from that, we convert them to
	add		OneCounter,ZEROREG		; ASCII values !!!
	ret

DivideBy100:
	
	CLR		R19
	CLC
	SUBI	R16,100
	SBC		R17,R19
	inc		HundredCounter
	rjmp	IntToASCII

DivideBy10:
	
	CLR R19
	CLC
	SUBI R16,10
	SBC R17,R19
	inc		TenthCounter
	rjmp	IntToASCII



QEM: .db 0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0
;QEM: .db 0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0

/*			y axis 0123 is old values and x axis 0123 is new values in a 2 dimensional matrix. the above QEM is a 1 dimensional form of the below
	The values are 0 where the old and new values are same means no change. if old was 0 or 00 and new was 1 or 01 the addition value to the count is -1 that
	means 1 is deducted for that stepp. If the intersection of old and new leads to X that means it invalid signal due to switch bounce or code slow to catch up
	with signal input. If we substitute X with 0 in the matrix no action is taken on the invalid signal and the step is ignored. The QEM bove is just all 4 rows
	written one after another in a straight line, all x is replaced with 0. First 4 values is for old value 0 , next 4 values if old value was 1 (0b01), next 4 values
	if old value was 2(0b10), next 4 values if old value was 3 (0b11). correct array value is found by( OLD * 4 +new). where new = signalB * 2 + signalA


  | 0		1		 2		 3
-------------------------------
0 | 0		-1		 1		 x
-------------------------------
1 |	1		0		x		-1
-------------------------------
2 |	-1		x		0		 1
-------------------------------
3 |	x		1		-1		 0

*/
