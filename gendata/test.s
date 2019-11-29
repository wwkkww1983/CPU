lui $1, 0x8010
lui $2, 0x8040
loop:
sw $2, 0x0000($1)
lw $3, 0x0000($1)
beq $4, $0, loop