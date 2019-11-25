ori $1, $0 ,0x1
loop:
bgtz $4, loop2
sw $1, 0x0000($0)
lw $2, 0x0000($0)
movn $3, $2, $0
movn $4, $2, $2
beq $3, $0, loop
nop
loop2: