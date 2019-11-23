SRC_DIR := thinpad_top.srcs/sources_1/new
SIM_DIR := thinpad_top.srcs/sim_1/new
SRCS     := $(wildcard ${SRC_DIR}/*.sv)
SIM_SRCS := $(wildcard ${SIM_DIR}/*.v) $(wildcard ${SIM_DIR}/*.sv)

thinpad_top.vcd: thinpad_top.vvp 
	vvp thinpad_top.vvp

thinpad_top.vvp: ${SRCS} ${SIM_SRCS} 
	iverilog -g2012 -I ${SRC_DIR} -I ${SRC_DIR}/include -I ${SIM_DIR} -I ${SIM_DIR}/include -o thinpad_top.vvp ${SRCS} ${SIM_SRCS}

wave: thinpad_top.vcd 
	gtkwave thinpad_top.vcd signal_list.gtkw --rcvar 'disable_mouseover 0' --rcvar 'left_justify_sigs 1'
	
clean: 
	rm -f thinpad_top.vvp thinpad_top.vcd