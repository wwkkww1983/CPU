xori $1, 0x8070
lui $5, 0x8040
loop:
bgtz $4, loop2
sb $1, 0x0000($5)
lw $1, 0x0002($5)
movn $3, $2, $0
movn $4, $2, $2
beq $3, $0, loop
nop
loop2: