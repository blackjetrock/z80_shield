;;;  The inter instruction code used t dump registers
	;; For this to work correctly, the get_instruction_length
	;; function has to return the correct instruction length for
	;; an opcode, (length includes opcode).
start:	
;;;  Standard register set
	;;  This is a bit inefficient, but a smaller, faster
	;; version can be created later if needed.
	push af
	push hl
	push bc
	push de
	push ix
	push iy
	pop  iy
	pop  ix
	pop  de
	pop  bc
	pop  hl
	pop  af

	;;  Alternate register set
	exx
	push af
	push hl
	push bc
	push de
	pop  de
	pop  bc
	pop  hl
	pop  af
	exx

	;;  restore PC
	jr start
