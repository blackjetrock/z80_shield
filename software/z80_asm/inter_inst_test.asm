	;; Test code that runs betwen each instruction
	
        SECTION SHADOW
	ORG     0A000H
	
SHADOW_A:     DEFB    1
SHADOW_B:     DEFB    1
SHADOW_AD:    DEFB    1

        SECTION AD
	
	ORG  0B000H
ADS0:	       DEFB    2
ADS1:	       DEFB    2

        SECTION CODE
	ORG 0

START:	LD  SP, 9000H

	;; LCD DB4-7
	LD   C, IO_ADDR_PIO1_AC
	LD   D, IO_ADDR_PIO1_AD
	LD   H, 0FH
	LD   L, 00H
	CALL PIOINIT

;RS and E
	LD   C, IO_ADDR_PIO1_BC
	LD   D, IO_ADDR_PIO1_BD
	LD   H, 0FCH
	LD   L, 00H
	CALL PIOINIT

		;A/D	
	LD   C, IO_ADDR_PIO0_AC
	LD   D, IO_ADDR_PIO0_AD
	LD   H, 0F8H
	LD   L, AD_CS
	CALL PIOINIT

	LD   A, AD_CS
	LD   (SHADOW_AD), A
	
        ;JP  AD

	LD   A, 0
	LD   (SHADOW_A), A
	LD   A, 0
	LD   (SHADOW_B), A

	CALL RS_LOW
	CALL E_LOW

; Initialise LCD
      	CALL DELAY		;
	
	LD   D, 03H
	CALL SDATA4
     	CALL DELAY

	LD   D, 03H
	CALL SDATA4
     	CALL DELAY

       LD   D, 03H
	CALL SDATA4
     	CALL DELAY

       	LD   D, 02H
	CALL SDATA4
     	CALL DELAY

	LD   D, 0EH
	CALL SDATA8
     	CALL DELAY
	
        LD   D, 06H
	CALL SDATA8
     	CALL DELAY

	LD   D, 01H
	CALL SDATA8
     	CALL DELAY

	       ld  d, 'Z'
	     call DDATA8
	          	CALL DELAY
	     
	       ld  d, '8'
	     call DDATA8
	          	CALL DELAY
	     
	       ld  d, '0'
	     call DDATA8
	          	CALL DELAY
	     
	       ld  d, ' '
	     call DDATA8
	          	CALL DELAY
	       ld  d, 'S'
	     call DDATA8
	          	CALL DELAY
	       ld  d, 'h'
	     call DDATA8
	          	CALL DELAY
	       ld  d, 'i'
	     call DDATA8
	          	CALL DELAY
	       ld  d, 'e'
	     call DDATA8
	          	CALL DELAY
	       ld  d, 'l'
	     call DDATA8
	          	CALL DELAY
	       ld  d, 'd'
	     call DDATA8
	          	CALL DELAY
	     
	     ;	       LD  HL, BAN
	     ;CALL DSTR

AD:	       ld d, 2
	       call     SDATA8
	
		CALL	 ADSAMPLE
		LD       (ADS0), BC

	       LD A, 'A'
	       ADD    C
	       LD D, A
	 call DDATA8
	JR     AD
	
	
LOOP:	JR   LOOP

     DSTR: LD A,(HL)
	     CP    0
	     JR     NZ, CONT

	     RET
	     CONT:  LD A,(HL)
	     LD D, A
	     CALL  DDATA8
	     INC HL
	     JR DSTR

; A quick test to see if we can get samples from the A/D
;

ADSAMPLE:

; Set up ready for transfer
	CALL CLK_HIGH
	
; Enable chip	
	CALL CS_LOW

; There now needs to be seven clock pulses and low data as
; a preamble

	CALL CLK_LOW
        CALL CLK_HIGH
	CALL CLK_LOW
        CALL CLK_HIGH
	CALL CLK_LOW
        CALL CLK_HIGH
	CALL CLK_LOW
        CALL CLK_HIGH
	CALL CLK_LOW
        CALL CLK_HIGH
	CALL CLK_LOW
        CALL CLK_HIGH
	CALL CLK_LOW
        CALL CLK_HIGH

        CALL CLK_LOW
	
; Now set up start bit
	CALL DIN_HIGH     	; START bit

;And clock it in

        CALL CLK_HIGH

; Single ended/ Differential bit
	CALL CLK_LOW
	CALL DIN_HIGH           ;SGL
	CALL CLK_HIGH

	;; Channel 0
	CALL CLK_LOW
	CALL DIN_LOW
	CALL CLK_HIGH
	
	CALL CLK_LOW
	CALL DIN_LOW
	CALL CLK_HIGH
	
	CALL CLK_LOW
	CALL DIN_LOW
	CALL CLK_HIGH

	;;  two clocks for sample
	CALL CLK_LOW
	CALL DIN_LOW
	CALL CLK_HIGH
	
	CALL CLK_LOW
	CALL DIN_LOW
	CALL CLK_HIGH

	;; Clock out 10 bits of data
	LD	D, 10
	LD      BC, 0    	;result

GETLOOP:
        CALL  CLK_HIGH
	CALL GET_DATABIT

	;; Shift up
	SLA     C
	RL      B
	AND     1
	OR      C
	LD      C, A
	CALL   CLK_LOW

	DEC D
	JR   NZ, GETLOOP

	CALL CS_HIGH

	RET
	     
DELAY:  ;RET
	LD   H,02H   
          ; 
LOOPH:    LD   L,0FFH   
LOOPL:    DEC   L   
          JR   NZ,LOOPL   
          DEC   H   
          JR   NZ,LOOPH   
          RET      

	;;  Init PIO port
	;;  C: PIO control address
	;;  D: PIO data address
	;;  L: Data to set up
	;;  H: IO pattern
	
PIOINIT:
	;; Mode 3
	LD A, 0CFH
	OUT (C), A
	;;  IO patterne
	LD A, H
	OUT (C), A
	
	;; Write initial data
	LD A, L
	LD C, D
	OUT (C), A
	RET

RS_LOW:PUSH   BC
      PUSH    DE
	LD    C, IO_ADDR_PIO1_BD
	LD    HL,SHADOW_B
	LD    A, (HL)
	RES   0, A
	LD    (HL), A
	OUT   (C), A
	POP   DE
	POP   BC
	
	RET

RS_HIGH:PUSH  BC
      PUSH    DE
        LD    C, IO_ADDR_PIO1_BD
	LD    HL,SHADOW_B
	LD    A, (HL)
	SET   0, A
	LD    (HL), A
	OUT   (C), A
	POP   DE	
	POP   BC
	RET

E_LOW:  PUSH  BC
        PUSH    DE
	LD    C, IO_ADDR_PIO1_BD
	LD    HL,SHADOW_B
	LD    A, (HL)
	RES   1, A
	LD    (HL), A
	OUT   (C), A
	POP   DE
	POP   BC
	RET

E_HIGH: PUSH BC
      PUSH    DE
	LD    C, IO_ADDR_PIO1_BD
	LD    HL,SHADOW_B
	LD    A, (HL)
	SET   1, A
	LD    (HL), A
	OUT   (C), A
	POP   DE	
	POP   BC
	RET
	
	;;  C: PIO data address
	;;  D:Data required (8 bits)
SDATA8: 
	LD    C, IO_ADDR_PIO1_AD
	CALL  RS_LOW
	CALL  E_HIGH

	LD    A, D
	

	LD    C, IO_ADDR_PIO1_AD	
	OUT   (C), A
	CALL  E_LOW

	CALL  E_HIGH
	LD    A, D
	SLA   A
	SLA   A
	SLA   A
	SLA   A

	OUT   (C), A
	CALL  E_LOW

	CALL  RS_HIGH
	
	RET

	  ;;  C: PIO data address
	;;  D:Data required (8 bits)
DDATA8: 
	LD    C, IO_ADDR_PIO1_AD
	CALL  RS_HIGH
	CALL  E_HIGH

	LD    A, D
	

	LD    C, IO_ADDR_PIO1_AD	
	OUT   (C), A
	CALL  E_LOW

	CALL  E_HIGH
	LD    A, D
	SLA   A
	SLA   A
	SLA   A
	SLA   A

	OUT   (C), A
	CALL  E_LOW

	CALL  RS_HIGH
	
	RET
	
; Send 4 bit data
SDATA4: CALL  RS_LOW
	CALL  E_HIGH
	LD    A, D
	SLA   A
	SLA   A
	SLA   A
	SLA   A
	LD    C, IO_ADDR_PIO1_AD
	OUT   (C), A
	CALL  E_LOW
	CALL  RS_HIGH
	RET

	;; --------------------------------------------------------------------------------
	;;
	;;  A/D subroutines
	;;
	;;

CS_LOW: PUSH    BC
        PUSH    DE
	LD    C, IO_ADDR_PIO0_AD
	LD    HL,SHADOW_AD
	LD    A, (HL)
	RES   0, A
	LD    (HL), A
	OUT   (C), A
	POP   DE
	POP   BC
	
	RET

CS_HIGH: PUSH    BC
        PUSH    DE
	LD    C, IO_ADDR_PIO0_AD
	LD    HL,SHADOW_AD
	LD    A, (HL)
	SET   0, A
	LD    (HL), A
	OUT   (C), A
	POP   DE
	POP   BC
	
	RET

CLK_LOW: PUSH    BC
        PUSH    DE
	LD    C, IO_ADDR_PIO0_AD
	LD    HL,SHADOW_AD
	LD    A, (HL)
	RES   1, A
	LD    (HL), A
	OUT   (C), A
	POP   DE
	POP   BC
	
	RET

CLK_HIGH: PUSH    BC
        PUSH    DE
	LD    C, IO_ADDR_PIO0_AD
	LD    HL,SHADOW_AD
	LD    A, (HL)
	SET   1, A
	LD    (HL), A
	OUT   (C), A
	POP   DE
	POP   BC
	
	RET

DIN_LOW: PUSH    BC
        PUSH    DE
	LD    C, IO_ADDR_PIO0_AD
	LD    HL,SHADOW_AD
	LD    A, (HL)
	RES   2, A
	LD    (HL), A
	OUT   (C), A
	POP   DE
	POP   BC
	
	RET

DIN_HIGH: PUSH    BC
        PUSH    DE
	LD    C, IO_ADDR_PIO0_AD
	LD    HL,SHADOW_AD
	LD    A, (HL)
	SET   2, A
	LD    (HL), A
	OUT   (C), A
	POP   DE
	POP   BC
	
	RET

GET_DATABIT: PUSH    BC
        PUSH    DE
	LD    C, IO_ADDR_PIO0_AD
	IN     A, (C)
	AND    08H
	SRL    A
	SRL    A
	SRL    A	
	
	POP   DE
	POP   BC
	
	RET
