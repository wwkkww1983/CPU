lui $1, 0x8040
lui $4, 0x8041
xori $3, $0, 1
lw $2, 0x0000($1)
sw $1, 0x0000($1)
sb $3, 0x0000($1)
lb $6, 0x0000($1)
lw $6, 0x0000
lui $5, 0x0001
nop