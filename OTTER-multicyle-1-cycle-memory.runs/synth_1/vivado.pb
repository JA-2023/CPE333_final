
?
Command: %s
1870*	planAhead2?
?read_checkpoint -auto_incremental -incremental C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/utils_1/imports/synth_1/OTTER_Wrapper.dcp2default:defaultZ12-2866h px? 
?
;Read reference checkpoint from %s for incremental synthesis3154*	planAhead2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/utils_1/imports/synth_1/OTTER_Wrapper.dcp2default:defaultZ12-5825h px? 
T
-Please ensure there are no constraint changes3725*	planAheadZ12-7989h px? 
z
Command: %s
53*	vivadotcl2I
5synth_design -top OTTER_Wrapper -part xc7a35tcpg236-12default:defaultZ4-113h px? 
:
Starting synth_design
149*	vivadotclZ4-321h px? 
?
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2
	Synthesis2default:default2
xc7a35t2default:defaultZ17-347h px? 
?
0Got license for feature '%s' and/or device '%s'
310*common2
	Synthesis2default:default2
xc7a35t2default:defaultZ17-349h px? 
V
Loading part %s157*device2#
xc7a35tcpg236-12default:defaultZ21-403h px? 
?
HMultithreading enabled for synth_design using a maximum of %s processes.4828*oasys2
22default:defaultZ8-7079h px? 
a
?Launching helper process for spawning children vivado processes4827*oasysZ8-7078h px? 
`
#Helper process launched with PID %s4824*oasys2
164562default:defaultZ8-7075h px? 
?
%s*synth2?
wStarting RTL Elaboration : Time (s): cpu = 00:00:06 ; elapsed = 00:00:06 . Memory (MB): peak = 1261.523 ; gain = 0.000
2default:defaulth px? 
?
synthesizing module '%s'%s4497*oasys2!
OTTER_Wrapper2default:default2
 2default:default2j
TC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER_Wrapper.sv2default:default2
232default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys2
	OTTER_MCU2default:default2
 2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
622default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys2
Mult4to12default:default2
 2default:default2e
OC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/Mult4to1.sv2default:default2
522default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Mult4to12default:default2
 2default:default2
12default:default2
12default:default2e
OC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/Mult4to1.sv2default:default2
522default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
	ProgCount2default:default2
 2default:default2f
PC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/ProgCount.sv2default:default2
232default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	ProgCount2default:default2
 2default:default2
22default:default2
12default:default2f
PC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/ProgCount.sv2default:default2
232default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2

HazardUnit2default:default2
 2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/new/HazardUnit.sv2default:default2
232default:default8@Z8-6157h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2"
IF_DE_ctrl_reg2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/new/HazardUnit.sv2default:default2
482default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2 
memread1_reg2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/new/HazardUnit.sv2default:default2
512default:default8@Z8-6014h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

HazardUnit2default:default2
 2default:default2
32default:default2
12default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/new/HazardUnit.sv2default:default2
232default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2&
OTTER_registerFile2default:default2
 2default:default2i
SC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/registerFile.sv2default:default2
232default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2&
OTTER_registerFile2default:default2
 2default:default2
42default:default2
12default:default2i
SC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/registerFile.sv2default:default2
232default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2$
OTTER_CU_Decoder2default:default2
 2default:default2g
QC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/CU_Decoder.sv2default:default2
232default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2$
OTTER_CU_Decoder2default:default2
 2default:default2
52default:default2
12default:default2g
QC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/CU_Decoder.sv2default:default2
232default:default8@Z8-6155h px? 
?
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
intTaken2default:default2$
OTTER_CU_Decoder2default:default2
decoder2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1542default:default8@Z8-7071h px? 
?
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2
decoder2default:default2$
OTTER_CU_Decoder2default:default2
112default:default2
102default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1542default:default8@Z8-7023h px? 
?
synthesizing module '%s'%s4497*oasys2
Mult2to12default:default2
 2default:default2e
OC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/Mult4to1.sv2default:default2
652default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Mult2to12default:default2
 2default:default2
62default:default2
12default:default2e
OC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/Mult4to1.sv2default:default2
652default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2!
BranchCondGen2default:default2
 2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/new/BranchCondGen.sv2default:default2
232default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2!
BranchCondGen2default:default2
 2default:default2
72default:default2
12default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/new/BranchCondGen.sv2default:default2
232default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
	OTTER_ALU2default:default2
 2default:default2k
UC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/ArithLogicUnit.sv2default:default2
222default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	OTTER_ALU2default:default2
 2default:default2
82default:default2
12default:default2k
UC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/ArithLogicUnit.sv2default:default2
222default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2#
Forwarding_unit2default:default2
 2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/new/Forwarding_unit.sv2default:default2
232default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2#
Forwarding_unit2default:default2
 2default:default2
92default:default2
12default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/new/Forwarding_unit.sv2default:default2
232default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2"
OTTER_mem_byte2default:default2
 2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/bram_dualport_pipeline.sv2default:default2
972default:default8@Z8-6157h px? 
?
,$readmem data file '%s' is read successfully3426*oasys2$
otter_memory.mem2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/bram_dualport_pipeline.sv2default:default2
1362default:default8@Z8-3876h px? 
?
-case statement is not full and has no default155*oasys2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/bram_dualport_pipeline.sv2default:default2
2102default:default8@Z8-155h px? 
?
-case statement is not full and has no default155*oasys2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/bram_dualport_pipeline.sv2default:default2
1962default:default8@Z8-155h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2
j_reg2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/bram_dualport_pipeline.sv2default:default2
1542default:default8@Z8-6014h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2"
OTTER_mem_byte2default:default2
 2default:default2
102default:default2
12default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/bram_dualport_pipeline.sv2default:default2
972default:default8@Z8-6155h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2)
if_de_reg_reg[opcode]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1222default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2)
de_ex_reg_reg[opcode]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1462default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2*
de_ex_reg_reg[rd_used]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1462default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2*
de_ex_reg_reg[REG_opA]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1462default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2*
de_ex_reg_reg[REG_opB]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1462default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2.
de_ex_reg_reg[REG_S_immed]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1462default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2.
de_ex_reg_reg[REG_U_immed]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1462default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2-
de_ex_reg_reg[REG_aluRes]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1462default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2/
de_ex_reg_reg[REG_PCsource]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1462default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2/
de_ex_reg_reg[REG_mem2_out]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1462default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2,
ex_mem_reg_reg[rs1_addr]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
2382default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2,
ex_mem_reg_reg[rs2_addr]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
2382default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2*
mem_wb_reg_reg[REG_IR]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1512default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2,
mem_wb_reg_reg[rs1_addr]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1512default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2,
mem_wb_reg_reg[rs2_addr]2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
1512default:default8@Z8-6014h px? 
?
RFound unconnected internal register '%s' and it is trimmed from '%s' to '%s' bits.3455*oasys2*
ex_mem_reg_reg[REG_IR]2default:default2
322default:default2
152default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
2382default:default8@Z8-3936h px? 
?
0Net %s in module/entity %s does not have driver.3422*oasys2
csr_reg2default:default2
	OTTER_MCU2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
3202default:default8@Z8-3848h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	OTTER_MCU2default:default2
 2default:default2
112default:default2
12default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/sources_1/imports/Downloads/otter_mcu_pipeline_template_v2.sv2default:default2
622default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2

SevSegDisp2default:default2
 2default:default2s
]C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/SevSegDisp.sv2default:default2
232default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys2
BCD2default:default2
 2default:default2l
VC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/BCD.sv2default:default2
232default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
BCD2default:default2
 2default:default2
122default:default2
12default:default2l
VC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/BCD.sv2default:default2
232default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2!
CathodeDriver2default:default2
 2default:default2v
`C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/CathodeDriver.sv2default:default2
192default:default8@Z8-6157h px? 
?
default block is never used226*oasys2v
`C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/CathodeDriver.sv2default:default2
412default:default8@Z8-226h px? 
?
default block is never used226*oasys2v
`C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/CathodeDriver.sv2default:default2
442default:default8@Z8-226h px? 
?
default block is never used226*oasys2v
`C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/CathodeDriver.sv2default:default2
662default:default8@Z8-226h px? 
?
default block is never used226*oasys2v
`C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/CathodeDriver.sv2default:default2
882default:default8@Z8-226h px? 
?
default block is never used226*oasys2v
`C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/CathodeDriver.sv2default:default2
1102default:default8@Z8-226h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2!
CathodeDriver2default:default2
 2default:default2
132default:default2
12default:default2v
`C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/CathodeDriver.sv2default:default2
192default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

SevSegDisp2default:default2
 2default:default2
142default:default2
12default:default2s
]C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/SevSegDisp.sv2default:default2
232default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2%
debounce_one_shot2default:default2
 2default:default2z
dC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/debounce_one_shot.sv2default:default2
252default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2%
debounce_one_shot2default:default2
 2default:default2
152default:default2
12default:default2z
dC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/debounce_one_shot.sv2default:default2
252default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2"
KeyboardDriver2default:default2
 2default:default2w
aC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/KeyboardDriver.sv2default:default2
292default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys2
keyboard2default:default2
 2default:default2w
aC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/KeyboardDriver.sv2default:default2
912default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys2
ps2_rx2default:default2
 2default:default2w
aC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/KeyboardDriver.sv2default:default2
2322default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
ps2_rx2default:default2
 2default:default2
162default:default2
12default:default2w
aC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/KeyboardDriver.sv2default:default2
2322default:default8@Z8-6155h px? 
?
-case statement is not full and has no default155*oasys2w
aC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/KeyboardDriver.sv2default:default2
1482default:default8@Z8-155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
keyboard2default:default2
 2default:default2
172default:default2
12default:default2w
aC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/KeyboardDriver.sv2default:default2
912default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2"
KeyboardDriver2default:default2
 2default:default2
182default:default2
12default:default2w
aC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/peripherals/KeyboardDriver.sv2default:default2
292default:default8@Z8-6155h px? 
?
-case statement is not full and has no default155*oasys2j
TC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER_Wrapper.sv2default:default2
1232default:default8@Z8-155h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2 
r_vga_we_reg2default:default2j
TC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER_Wrapper.sv2default:default2
1202default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2"
uart_start_reg2default:default2j
TC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER_Wrapper.sv2default:default2
1212default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2 
r_vga_wa_reg2default:default2j
TC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER_Wrapper.sv2default:default2
1262default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2 
r_vga_wd_reg2default:default2j
TC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER_Wrapper.sv2default:default2
1272default:default8@Z8-6014h px? 
?
+Unused sequential element %s was removed. 
4326*oasys2!
uart_data_reg2default:default2j
TC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER_Wrapper.sv2default:default2
1312default:default8@Z8-6014h px? 
?
0Net %s in module/entity %s does not have driver.3422*oasys2
VGA_RGB2default:default2!
OTTER_Wrapper2default:default2j
TC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER_Wrapper.sv2default:default2
332default:default8@Z8-3848h px? 
?
0Net %s in module/entity %s does not have driver.3422*oasys2
VGA_HS2default:default2!
OTTER_Wrapper2default:default2j
TC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER_Wrapper.sv2default:default2
342default:default8@Z8-3848h px? 
?
0Net %s in module/entity %s does not have driver.3422*oasys2
VGA_VS2default:default2!
OTTER_Wrapper2default:default2j
TC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER_Wrapper.sv2default:default2
352default:default8@Z8-3848h px? 
?
0Net %s in module/entity %s does not have driver.3422*oasys2
Tx2default:default2!
OTTER_Wrapper2default:default2j
TC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER_Wrapper.sv2default:default2
362default:default8@Z8-3848h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2!
OTTER_Wrapper2default:default2
 2default:default2
192default:default2
12default:default2j
TC:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER_Wrapper.sv2default:default2
232default:default8@Z8-6155h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
CU_FUNC7[6]2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
CU_FUNC7[4]2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
CU_FUNC7[3]2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
CU_FUNC7[2]2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
CU_FUNC7[1]2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
CU_FUNC7[0]2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
intTaken2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
DE_EX_rt[4]2default:default2

HazardUnit2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
DE_EX_rt[3]2default:default2

HazardUnit2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
DE_EX_rt[2]2default:default2

HazardUnit2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
DE_EX_rt[1]2default:default2

HazardUnit2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
DE_EX_rt[0]2default:default2

HazardUnit2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
INTR2default:default2
	OTTER_MCU2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[7]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[6]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[5]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[4]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[3]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[2]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[1]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[0]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
VGA_HS2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
VGA_VS2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
Tx2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2 
SWITCHES[15]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2 
SWITCHES[14]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2 
SWITCHES[13]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2 
SWITCHES[12]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2 
SWITCHES[11]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2 
SWITCHES[10]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[9]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[8]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[7]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[6]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[5]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[4]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[3]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[2]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[1]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[0]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
%s*synth2?
wFinished RTL Elaboration : Time (s): cpu = 00:00:07 ; elapsed = 00:00:08 . Memory (MB): peak = 1261.523 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
M
%s
*synth25
!Start Handling Custom Attributes
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Handling Custom Attributes : Time (s): cpu = 00:00:08 ; elapsed = 00:00:09 . Memory (MB): peak = 1261.523 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished RTL Optimization Phase 1 : Time (s): cpu = 00:00:08 ; elapsed = 00:00:09 . Memory (MB): peak = 1261.523 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2 
00:00:00.0392default:default2
1261.5232default:default2
0.0002default:defaultZ17-268h px? 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px? 
>

Processing XDC Constraints
244*projectZ1-262h px? 
=
Initializing timing engine
348*projectZ1-569h px? 
?
Parsing XDC File [%s]
179*designutils2r
\C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/Basys3_constraints-1.xdc2default:default8Z20-179h px? 
?
Finished Parsing XDC File [%s]
178*designutils2r
\C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/Basys3_constraints-1.xdc2default:default8Z20-178h px? 
?
?Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2p
\C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/Basys3_constraints-1.xdc2default:default23
.Xil/OTTER_Wrapper_propImpl.xdc2default:defaultZ1-236h px? 
H
&Completed Processing XDC Constraints

245*projectZ1-263h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2 
00:00:00.0012default:default2
1261.5232default:default2
0.0002default:defaultZ17-268h px? 
~
!Unisim Transformation Summary:
%s111*project29
%No Unisim elements were transformed.
2default:defaultZ1-111h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common24
 Constraint Validation Runtime : 2default:default2
00:00:002default:default2 
00:00:00.0122default:default2
1261.5232default:default2
0.0002default:defaultZ17-268h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
}Finished Constraint Validation : Time (s): cpu = 00:00:17 ; elapsed = 00:00:18 . Memory (MB): peak = 1261.523 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
V
%s
*synth2>
*Start Loading Part and Timing Information
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
J
%s
*synth22
Loading part: xc7a35tcpg236-1
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Loading Part and Timing Information : Time (s): cpu = 00:00:17 ; elapsed = 00:00:18 . Memory (MB): peak = 1261.523 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
Z
%s
*synth2B
.Start Applying 'set_property' XDC Constraints
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished applying 'set_property' XDC Constraints : Time (s): cpu = 00:00:17 ; elapsed = 00:00:18 . Memory (MB): peak = 1261.523 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
I
%s
*synth21
Start Preparing Guide Design
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
?The reference checkpoint %s is not suitable for use with incremental synthesis for this design. Please regenerate the checkpoint for this design with -incremental_synth switch in the same Vivado session that synth_design has been run. Synthesis will continue with the default flow4740*oasys2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.srcs/utils_1/imports/synth_1/OTTER_Wrapper.dcp2default:defaultZ8-6895h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
zFinished Doing Graph Differ : Time (s): cpu = 00:00:17 ; elapsed = 00:00:18 . Memory (MB): peak = 1261.523 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
~Finished Preparing Guide Design : Time (s): cpu = 00:00:17 ; elapsed = 00:00:18 . Memory (MB): peak = 1261.523 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys2
PS_reg2default:default2%
debounce_one_shot2default:defaultZ8-802h px? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys2!
state_reg_reg2default:default2
keyboard2default:defaultZ8-802h px? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys2 
StateReg_reg2default:default2"
KeyboardDriver2default:defaultZ8-802h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default20
"OTTER_mem_byte:/memory_reg"2default:defaultZ8-7030h px? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2t
`                   State |                     New Encoding |                Previous Encoding 
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2s
_                 ST_init |                              000 | 00000000000000000000000000000000
2default:defaulth p
x
? 
?
%s
*synth2s
_              ST_BTN_low |                              001 | 00000000000000000000000000000001
2default:defaulth p
x
? 
?
%s
*synth2s
_      ST_BTN_low_to_high |                              010 | 00000000000000000000000000000010
2default:defaulth p
x
? 
?
%s
*synth2s
_             ST_BTN_high |                              011 | 00000000000000000000000000000011
2default:defaulth p
x
? 
?
%s
*synth2s
_      ST_BTN_high_to_low |                              100 | 00000000000000000000000000000100
2default:defaulth p
x
? 
?
%s
*synth2s
_             ST_one_shot |                              101 | 00000000000000000000000000000101
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
PS_reg2default:default2

sequential2default:default2%
debounce_one_shot2default:defaultZ8-3354h px? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2t
`                   State |                     New Encoding |                Previous Encoding 
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2s
_            ST_lowercase |                           000001 |                              000
2default:defaulth p
x
? 
?
%s
*synth2s
_                ST_shift |                           000010 |                              010
2default:defaulth p
x
? 
?
%s
*synth2s
_   ST_ignore_shift_break |                           000100 |                              011
2default:defaulth p
x
? 
?
%s
*synth2s
_             ST_capslock |                           001000 |                              100
2default:defaulth p
x
? 
?
%s
*synth2s
_    ST_ignore_caps_break |                           010000 |                              101
2default:defaulth p
x
? 
?
%s
*synth2s
_         ST_ignore_break |                           100000 |                              001
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2!
state_reg_reg2default:default2
one-hot2default:default2
keyboard2default:defaultZ8-3354h px? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2t
`                   State |                     New Encoding |                Previous Encoding 
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2s
_                 ST_WAIT |                                0 | 00000000000000000000000000000000
2default:defaulth p
x
? 
?
%s
*synth2s
_               ST_INTRPT |                                1 | 00000000000000000000000000000001
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2 
StateReg_reg2default:default2

sequential2default:default2"
KeyboardDriver2default:defaultZ8-3354h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished RTL Optimization Phase 2 : Time (s): cpu = 00:00:19 ; elapsed = 00:00:19 . Memory (MB): peak = 1261.523 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
U
%s
*synth2=
)

Incremental Synthesis Report Summary:

2default:defaulth p
x
? 
N
%s
*synth26
"1. Incremental synthesis run: no

2default:defaulth p
x
? 
a
%s
*synth2I
5   Reason for not running incremental synthesis : 


2default:defaulth p
x
? 
?
?Flow is switching to default flow due to incremental criteria not met. If you would like to alter this behaviour and have the flow terminate instead, please set the following parameter config_implementation {autoIncr.Synth.RejectBehavior Terminate}4868*oasysZ8-7130h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
L
%s
*synth24
 Start RTL Component Statistics 
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
:
%s
*synth2"
+---Adders : 
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   32 Bit       Adders := 6     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   3 Input   32 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   20 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    8 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    5 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    4 Bit       Adders := 36    
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    3 Bit       Adders := 8     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    2 Bit       Adders := 2     
2default:defaulth p
x
? 
8
%s
*synth2 
+---XORs : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     32 Bit         XORs := 1     
2default:defaulth p
x
? 
=
%s
*synth2%
+---Registers : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               32 Bit    Registers := 19    
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               16 Bit    Registers := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               15 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               11 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                8 Bit    Registers := 5     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                5 Bit    Registers := 5     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                4 Bit    Registers := 3     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                3 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                2 Bit    Registers := 7     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                1 Bit    Registers := 18    
2default:defaulth p
x
? 
?
%s
*synth2'
+---Multipliers : 
2default:defaulth p
x
? 
X
%s
*synth2@
,	              32x32  Multipliers := 1     
2default:defaulth p
x
? 
8
%s
*synth2 
+---RAMs : 
2default:defaulth p
x
? 
l
%s
*synth2T
@	             512K Bit	(16384 X 32 bit)          RAMs := 1     
2default:defaulth p
x
? 
i
%s
*synth2Q
=	             1024 Bit	(32 X 32 bit)          RAMs := 1     
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input   32 Bit        Muxes := 11    
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   32 Bit        Muxes := 8     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   9 Input   32 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   7 Input   32 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   6 Input   32 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   16 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    8 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    8 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	  19 Input    8 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   6 Input    6 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    6 Bit        Muxes := 4     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   3 Input    6 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    5 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    5 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   5 Input    5 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    4 Bit        Muxes := 32    
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    4 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   5 Input    3 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    3 Bit        Muxes := 12    
2default:defaulth p
x
? 
X
%s
*synth2@
,	   6 Input    3 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    2 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   6 Input    2 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   5 Input    2 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    2 Bit        Muxes := 3     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   3 Input    2 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	  11 Input    1 Bit        Muxes := 3     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   7 Input    1 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    1 Bit        Muxes := 28    
2default:defaulth p
x
? 
X
%s
*synth2@
,	   9 Input    1 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   3 Input    1 Bit        Muxes := 5     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   6 Input    1 Bit        Muxes := 8     
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
O
%s
*synth27
#Finished RTL Component Statistics 
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
H
%s
*synth20
Start Part Resource Summary
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2j
VPart Resources:
DSPs: 90 (col length:60)
BRAMs: 100 (col length: RAMB18 60 RAMB36 30)
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Finished Part Resource Summary
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
W
%s
*synth2?
+Start Cross Boundary and Area Optimization
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
H
&Parallel synthesis criteria is not met4829*oasysZ8-7080h px? 
h
%s
*synth2P
<DSP Report: Generating DSP ALUOut0, operation Mode is: A*B.
2default:defaulth p
x
? 
g
%s
*synth2O
;DSP Report: operator ALUOut0 is absorbed into DSP ALUOut0.
2default:defaulth p
x
? 
g
%s
*synth2O
;DSP Report: operator ALUOut0 is absorbed into DSP ALUOut0.
2default:defaulth p
x
? 
s
%s
*synth2[
GDSP Report: Generating DSP ALUOut0, operation Mode is: (PCIN>>17)+A*B.
2default:defaulth p
x
? 
g
%s
*synth2O
;DSP Report: operator ALUOut0 is absorbed into DSP ALUOut0.
2default:defaulth p
x
? 
g
%s
*synth2O
;DSP Report: operator ALUOut0 is absorbed into DSP ALUOut0.
2default:defaulth p
x
? 
h
%s
*synth2P
<DSP Report: Generating DSP ALUOut0, operation Mode is: A*B.
2default:defaulth p
x
? 
g
%s
*synth2O
;DSP Report: operator ALUOut0 is absorbed into DSP ALUOut0.
2default:defaulth p
x
? 
g
%s
*synth2O
;DSP Report: operator ALUOut0 is absorbed into DSP ALUOut0.
2default:defaulth p
x
? 
s
%s
*synth2[
GDSP Report: Generating DSP ALUOut0, operation Mode is: (PCIN>>17)+A*B.
2default:defaulth p
x
? 
g
%s
*synth2O
;DSP Report: operator ALUOut0 is absorbed into DSP ALUOut0.
2default:defaulth p
x
? 
g
%s
*synth2O
;DSP Report: operator ALUOut0 is absorbed into DSP ALUOut0.
2default:defaulth p
x
? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
CU_FUNC7[6]2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
CU_FUNC7[4]2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
CU_FUNC7[3]2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
CU_FUNC7[2]2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
CU_FUNC7[1]2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
CU_FUNC7[0]2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
intTaken2default:default2$
OTTER_CU_Decoder2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
INTR2default:default2
	OTTER_MCU2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[7]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[6]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[5]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[4]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[3]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[2]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[1]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2

VGA_RGB[0]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
VGA_HS2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
VGA_VS2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
Tx2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2 
SWITCHES[15]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2 
SWITCHES[14]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2 
SWITCHES[13]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2 
SWITCHES[12]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2 
SWITCHES[11]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2 
SWITCHES[10]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[9]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[8]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[7]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[6]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[5]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[4]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[3]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[2]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[1]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
9Port %s in module %s is either unconnected or has no load4866*oasys2
SWITCHES[0]2default:default2!
OTTER_Wrapper2default:defaultZ8-7129h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2default:default2
12default:default2
322default:default2-
"MCU/BYTE_MEM/memory_reg"2default:defaultZ8-7030h px? 
?
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2/
DB/FSM_sequential_PS_reg[2]2default:default2!
OTTER_Wrapper2default:defaultZ8-3332h px? 
?
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2/
DB/FSM_sequential_PS_reg[1]2default:default2!
OTTER_Wrapper2default:defaultZ8-3332h px? 
?
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2/
DB/FSM_sequential_PS_reg[0]2default:default2!
OTTER_Wrapper2default:defaultZ8-3332h px? 
?
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2;
'KEYBD/keybd/FSM_onehot_state_reg_reg[5]2default:default2!
OTTER_Wrapper2default:defaultZ8-3332h px? 
?
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2;
'KEYBD/keybd/FSM_onehot_state_reg_reg[4]2default:default2!
OTTER_Wrapper2default:defaultZ8-3332h px? 
?
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2;
'KEYBD/keybd/FSM_onehot_state_reg_reg[3]2default:default2!
OTTER_Wrapper2default:defaultZ8-3332h px? 
?
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2;
'KEYBD/keybd/FSM_onehot_state_reg_reg[2]2default:default2!
OTTER_Wrapper2default:defaultZ8-3332h px? 
?
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2;
'KEYBD/keybd/FSM_onehot_state_reg_reg[1]2default:default2!
OTTER_Wrapper2default:defaultZ8-3332h px? 
?
ESequential element (%s) is unused and will be removed from module %s.3332*oasys2;
'KEYBD/keybd/FSM_onehot_state_reg_reg[0]2default:default2!
OTTER_Wrapper2default:defaultZ8-3332h px? 
?
ESequential element (%s) is unused and will be removed from module %s.3332*oasys25
!KEYBD/FSM_sequential_StateReg_reg2default:default2!
OTTER_Wrapper2default:defaultZ8-3332h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Cross Boundary and Area Optimization : Time (s): cpu = 00:00:32 ; elapsed = 00:00:32 . Memory (MB): peak = 1261.523 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?---------------------------------------------------------------------------------
Start ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth px? 
~
%s*synth2f
R---------------------------------------------------------------------------------
2default:defaulth px? 
d
%s*synth2L
8
Block RAM: Preliminary Mapping Report (see note below)
2default:defaulth px? 
?
%s*synth2?
?+-------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
2default:defaulth px? 
?
%s*synth2?
?|Module Name  | RTL Object | PORT A (Depth x Width) | W | R | PORT B (Depth x Width) | W | R | Ports driving FF | RAMB18 | RAMB36 | 
2default:defaulth px? 
?
%s*synth2?
?+-------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
2default:defaulth px? 
?
%s*synth2?
?|MCU/BYTE_MEM | memory_reg | 16 K x 32(READ_FIRST)  | W | R | 16 K x 32(WRITE_FIRST) |   | R | Port A and B     | 0      | 16     | 
2default:defaulth px? 
?
%s*synth2?
?+-------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+

2default:defaulth px? 
?
%s*synth2?
?Note: The table above is a preliminary report that shows the Block RAMs at the current stage of the synthesis flow. Some Block RAMs may be reimplemented as non Block RAM primitives later in the synthesis flow. Multiple instantiated Block RAMs are reported only once. 
2default:defaulth px? 
j
%s*synth2R
>
Distributed RAM: Preliminary Mapping Report (see note below)
2default:defaulth px? 
{
%s*synth2c
O+-------------+------------+-----------+----------------------+--------------+
2default:defaulth px? 
|
%s*synth2d
P|Module Name  | RTL Object | Inference | Size (Depth x Width) | Primitives   | 
2default:defaulth px? 
{
%s*synth2c
O+-------------+------------+-----------+----------------------+--------------+
2default:defaulth px? 
|
%s*synth2d
P|MCU/Register | RF_reg     | Implied   | 32 x 32              | RAM32M x 12  | 
2default:defaulth px? 
|
%s*synth2d
P+-------------+------------+-----------+----------------------+--------------+

2default:defaulth px? 
?
%s*synth2?
?Note: The table above is a preliminary report that shows the Distributed RAMs at the current stage of the synthesis flow. Some Distributed RAMs may be reimplemented as non Distributed RAM primitives later in the synthesis flow. Multiple instantiated RAMs are reported only once.
2default:defaulth px? 
?
%s*synth2p
\
DSP: Preliminary Mapping Report (see note below. The ' indicates corresponding REG is set)
2default:defaulth px? 
?
%s*synth2?
+------------+----------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+
2default:defaulth px? 
?
%s*synth2?
?|Module Name | DSP Mapping    | A Size | B Size | C Size | D Size | P Size | AREG | BREG | CREG | DREG | ADREG | MREG | PREG | 
2default:defaulth px? 
?
%s*synth2?
+------------+----------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+
2default:defaulth px? 
?
%s*synth2?
?|OTTER_ALU   | A*B            | 18     | 16     | -      | -      | 48     | 0    | 0    | -    | -    | -     | 0    | 0    | 
2default:defaulth px? 
?
%s*synth2?
?|OTTER_ALU   | (PCIN>>17)+A*B | 16     | 16     | -      | -      | 48     | 0    | 0    | -    | -    | -     | 0    | 0    | 
2default:defaulth px? 
?
%s*synth2?
?|OTTER_ALU   | A*B            | 18     | 18     | -      | -      | 48     | 0    | 0    | -    | -    | -     | 0    | 0    | 
2default:defaulth px? 
?
%s*synth2?
?|OTTER_ALU   | (PCIN>>17)+A*B | 18     | 16     | -      | -      | 48     | 0    | 0    | -    | -    | -     | 0    | 0    | 
2default:defaulth px? 
?
%s*synth2?
?+------------+----------------+--------+--------+--------+--------+--------+------+------+------+------+-------+------+------+

2default:defaulth px? 
?
%s*synth2?
?Note: The table above is a preliminary report that shows the DSPs inferred at the current stage of the synthesis flow. Some DSP may be reimplemented as non DSP primitives later in the synthesis flow. Multiple instantiated DSPs are reported only once.
2default:defaulth px? 
?
%s*synth2?
?---------------------------------------------------------------------------------
Finished ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth px? 
~
%s*synth2f
R---------------------------------------------------------------------------------
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
R
%s
*synth2:
&Start Applying XDC Timing Constraints
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Applying XDC Timing Constraints : Time (s): cpu = 00:00:43 ; elapsed = 00:00:44 . Memory (MB): peak = 1261.523 ; gain = 0.000
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
F
%s
*synth2.
Start Timing Optimization
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
|Finished Timing Optimization : Time (s): cpu = 00:00:44 ; elapsed = 00:00:45 . Memory (MB): peak = 1277.359 ; gain = 15.836
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2?
?---------------------------------------------------------------------------------
Start ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
M
%s
*synth25
!
Block RAM: Final Mapping Report
2default:defaulth p
x
? 
?
%s
*synth2?
?+-------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
2default:defaulth p
x
? 
?
%s
*synth2?
?|Module Name  | RTL Object | PORT A (Depth x Width) | W | R | PORT B (Depth x Width) | W | R | Ports driving FF | RAMB18 | RAMB36 | 
2default:defaulth p
x
? 
?
%s
*synth2?
?+-------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
2default:defaulth p
x
? 
?
%s
*synth2?
?|MCU/BYTE_MEM | memory_reg | 16 K x 32(READ_FIRST)  | W | R | 16 K x 32(WRITE_FIRST) |   | R | Port A and B     | 0      | 16     | 
2default:defaulth p
x
? 
?
%s
*synth2?
?+-------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+

2default:defaulth p
x
? 
S
%s
*synth2;
'
Distributed RAM: Final Mapping Report
2default:defaulth p
x
? 
{
%s
*synth2c
O+-------------+------------+-----------+----------------------+--------------+
2default:defaulth p
x
? 
|
%s
*synth2d
P|Module Name  | RTL Object | Inference | Size (Depth x Width) | Primitives   | 
2default:defaulth p
x
? 
{
%s
*synth2c
O+-------------+------------+-----------+----------------------+--------------+
2default:defaulth p
x
? 
|
%s
*synth2d
P|MCU/Register | RF_reg     | Implied   | 32 x 32              | RAM32M x 12  | 
2default:defaulth p
x
? 
|
%s
*synth2d
P+-------------+------------+-----------+----------------------+--------------+

2default:defaulth p
x
? 
?
%s
*synth2?
?---------------------------------------------------------------------------------
Finished ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-
Start Technology Mapping
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys22
MCU/BYTE_MEM/memory_reg_bram_82default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys22
MCU/BYTE_MEM/memory_reg_bram_82default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys22
MCU/BYTE_MEM/memory_reg_bram_92default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys22
MCU/BYTE_MEM/memory_reg_bram_92default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_102default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_102default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_112default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_112default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_122default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_122default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_132default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_132default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_142default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_142default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_152default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_152default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_162default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_162default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_172default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_172default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_182default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_182default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_192default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_192default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_202default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_202default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_212default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_212default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_222default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_222default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_232default:default2
Block2default:defaultZ8-7052h px? 
?
?The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys23
MCU/BYTE_MEM/memory_reg_bram_232default:default2
Block2default:defaultZ8-7052h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
{Finished Technology Mapping : Time (s): cpu = 00:00:45 ; elapsed = 00:00:46 . Memory (MB): peak = 1348.168 ; gain = 86.645
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2'
Start IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
Q
%s
*synth29
%Start Flattening Before IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
T
%s
*synth2<
(Finished Flattening Before IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
H
%s
*synth20
Start Final Netlist Cleanup
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Finished Final Netlist Cleanup
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
vFinished IO Insertion : Time (s): cpu = 00:00:51 ; elapsed = 00:00:52 . Memory (MB): peak = 1362.918 ; gain = 101.395
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
O
%s
*synth27
#Start Renaming Generated Instances
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Instances : Time (s): cpu = 00:00:51 ; elapsed = 00:00:52 . Memory (MB): peak = 1362.918 ; gain = 101.395
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
L
%s
*synth24
 Start Rebuilding User Hierarchy
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Rebuilding User Hierarchy : Time (s): cpu = 00:00:51 ; elapsed = 00:00:52 . Memory (MB): peak = 1362.918 ; gain = 101.395
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Start Renaming Generated Ports
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Ports : Time (s): cpu = 00:00:51 ; elapsed = 00:00:52 . Memory (MB): peak = 1362.918 ; gain = 101.395
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
M
%s
*synth25
!Start Handling Custom Attributes
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Handling Custom Attributes : Time (s): cpu = 00:00:51 ; elapsed = 00:00:52 . Memory (MB): peak = 1362.918 ; gain = 101.395
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
J
%s
*synth22
Start Renaming Generated Nets
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Nets : Time (s): cpu = 00:00:51 ; elapsed = 00:00:52 . Memory (MB): peak = 1362.918 ; gain = 101.395
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Start Writing Synthesis Report
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
A
%s
*synth2)

Report BlackBoxes: 
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
J
%s
*synth22
| |BlackBox name |Instances |
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
A
%s*synth2)

Report Cell Usage: 
2default:defaulth px? 
F
%s*synth2.
+------+---------+------+
2default:defaulth px? 
F
%s*synth2.
|      |Cell     |Count |
2default:defaulth px? 
F
%s*synth2.
+------+---------+------+
2default:defaulth px? 
F
%s*synth2.
|1     |BUFG     |     2|
2default:defaulth px? 
F
%s*synth2.
|2     |CARRY4   |    89|
2default:defaulth px? 
F
%s*synth2.
|3     |DSP48E1  |     3|
2default:defaulth px? 
F
%s*synth2.
|4     |LUT1     |     7|
2default:defaulth px? 
F
%s*synth2.
|5     |LUT2     |   275|
2default:defaulth px? 
F
%s*synth2.
|6     |LUT3     |   179|
2default:defaulth px? 
F
%s*synth2.
|7     |LUT4     |   236|
2default:defaulth px? 
F
%s*synth2.
|8     |LUT5     |   263|
2default:defaulth px? 
F
%s*synth2.
|9     |LUT6     |   901|
2default:defaulth px? 
F
%s*synth2.
|10    |MUXF7    |   141|
2default:defaulth px? 
F
%s*synth2.
|11    |MUXF8    |    64|
2default:defaulth px? 
F
%s*synth2.
|12    |RAM32M   |    10|
2default:defaulth px? 
F
%s*synth2.
|13    |RAM32X1D |     4|
2default:defaulth px? 
F
%s*synth2.
|14    |RAMB36E1 |    16|
2default:defaulth px? 
F
%s*synth2.
|16    |FDRE     |   519|
2default:defaulth px? 
F
%s*synth2.
|17    |FDSE     |     4|
2default:defaulth px? 
F
%s*synth2.
|18    |IBUF     |     2|
2default:defaulth px? 
F
%s*synth2.
|19    |OBUF     |    28|
2default:defaulth px? 
F
%s*synth2.
|20    |OBUFT    |    11|
2default:defaulth px? 
F
%s*synth2.
+------+---------+------+
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Writing Synthesis Report : Time (s): cpu = 00:00:51 ; elapsed = 00:00:52 . Memory (MB): peak = 1362.918 ; gain = 101.395
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
s
%s
*synth2[
GSynthesis finished with 0 errors, 1 critical warnings and 46 warnings.
2default:defaulth p
x
? 
?
%s
*synth2?
Synthesis Optimization Runtime : Time (s): cpu = 00:00:40 ; elapsed = 00:00:49 . Memory (MB): peak = 1362.918 ; gain = 101.395
2default:defaulth p
x
? 
?
%s
*synth2?
?Synthesis Optimization Complete : Time (s): cpu = 00:00:51 ; elapsed = 00:00:52 . Memory (MB): peak = 1362.918 ; gain = 101.395
2default:defaulth p
x
? 
B
 Translating synthesized netlist
350*projectZ1-571h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2 
00:00:00.0542default:default2
1365.1522default:default2
0.0002default:defaultZ17-268h px? 
g
-Analyzing %s Unisim elements for replacement
17*netlist2
3272default:defaultZ29-17h px? 
j
2Unisim Transformation completed in %s CPU seconds
28*netlist2
02default:defaultZ29-28h px? 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px? 
v
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
12default:default2
142default:defaultZ31-138h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
1366.7502default:default2
0.0002default:defaultZ17-268h px? 
?
!Unisim Transformation Summary:
%s111*project2?
?  A total of 14 instances were transformed.
  RAM32M => RAM32M (inverted pins: WCLK) (RAMD32(x6), RAMS32(x2)): 10 instances
  RAM32X1D => RAM32X1D (inverted pins: WCLK) (RAMD32(x2)): 4 instances
2default:defaultZ1-111h px? 
g
$Synth Design complete, checksum: %s
562*	vivadotcl2
ab5cf47f2default:defaultZ4-1430h px? 
U
Releasing license: %s
83*common2
	Synthesis2default:defaultZ17-83h px? 
?
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
1362default:default2
1172default:default2
12default:default2
02default:defaultZ4-41h px? 
^
%s completed successfully
29*	vivadotcl2 
synth_design2default:defaultZ4-42h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2"
synth_design: 2default:default2
00:00:562default:default2
00:00:582default:default2
1366.7502default:default2
105.2272default:defaultZ17-268h px? 
u
%s6*runtcl2Y
ESynthesis results are not added to the cache due to CRITICAL_WARNING
2default:defaulth px? 
?
 The %s '%s' has been generated.
621*common2

checkpoint2default:default2?
?C:/Users/joshu/Desktop/School/CPE333/OTTER-multicyle-1-cycle-memory/OTTER-multicyle-1-cycle-memory.runs/synth_1/OTTER_Wrapper.dcp2default:defaultZ17-1381h px? 
?
%s4*runtcl2?
pExecuting : report_utilization -file OTTER_Wrapper_utilization_synth.rpt -pb OTTER_Wrapper_utilization_synth.pb
2default:defaulth px? 
?
Exiting %s at %s...
206*common2
Vivado2default:default2,
Sat May 21 23:22:40 20222default:defaultZ17-206h px? 


End Record