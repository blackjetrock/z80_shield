;;;  The inter instruction code used t dump registers
	;; For this to work correctly, the get_instruction_length
	;; function has to return the correct instruction length for
	;; an opcode, (length includes opcode).
start:	
	push af
	push hl
	push bc
	push de
	pop  de
	pop  bc
	pop  hl
	pop  af
	jr start
