;;;  The inter instruction code used t dump registers
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
