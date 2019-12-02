ori $v0, $0, 0
ori $v1, $0, 1
ori $a0, $0, 0
ori $a1, $0, 1
ori $14, $0, 58
loop:
addiu $14, $14, 0xffff
addu $a3, $a1, $v1
addu $a2, $a0, $v0
ori $t1, $0, 32
ori $12, $0, 0
cmploop:
addiu $t1, $t1, 0xffff
srl $10, $a3, $t1
srl $11, $a1, $t1
beq $10, $11, judge
nop
andi $13, $11, 1
bne $13, $0, change
nop
j end
nop
judge:
bne $t1, $0, cmploop
nop
j end
nop
change:
ori $12, $0, 1
end:
addu $a2, $a2, $12
ori $v0, $a0, 0
ori $v1, $a1, 0
ori $a0, $a2, 0
ori $a1, $a3, 0
bgtz $14, loop
nop
lui $15, 0x8040
sw $a1, 0($15)
sw $a0, 4($15)
jr $ra
nop


