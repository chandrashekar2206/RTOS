
   .def setPSP
   .def get_PSP
   .def PUSH_r4_11
   .def POP_r4_11
   .def PUSH_xPSR_R0
   .def get_SV_val
   .def getR0
   .def getR1
   .def send_LR


.thumb

.text

setPSP:
		MRS R1, CONTROL
		ORR R1 , #2
		MSR CONTROL , R1
		MSR PSP, R0
		BX LR
get_PSP:
       MRS R0,PSP
       BX  LR


PUSH_r4_11:
  	   MRS R0,PSP
       SUB R0, #4
       STR R4, [R0]
       SUB R0, #4
       STR R5, [R0]
       SUB R0, #4
       STR R6, [R0]
       SUB R0, #4
       STR R7, [R0]
       SUB R0, #4
       STR R8, [R0]
       SUB R0, #4
       STR R9, [R0]
       SUB R0, #4
       STR R10, [R0]
       SUB R0, #4
       STR R11, [R0]
       MSR PSP,R0
       BX  LR

POP_r4_11:
 	   MSR PSP,R0
       LDR R11, [R0]
       ADD R0, #4
       LDR R10, [R0]
       ADD R0, #4
       LDR R9, [R0]
       ADD R0, #4
       LDR R8, [R0]
       ADD R0, #4
       LDR R7, [R0]
       ADD R0, #4
       LDR R6, [R0]
       ADD R0, #4
       LDR R5, [R0]
       ADD R0, #4
       LDR R4, [R0]
       ADD R0, #4
       MSR PSP,R0
       BX  LR


PUSH_xPSR_R0:

      MOV R3,R0
      MRS R0,PSP
      MOV R1,#20
      MOV R2, #0x61000000
      SUB R0, #4
      STR R2, [R0];//xPSR
      SUB R0, #4
      STR R3, [R0];//PC
      SUB R0, #4
      STR R1, [R0];//lr
      SUB R0, #4
      STR R1, [R0];//r12
      SUB R0, #4
      STR R1, [R0];//r3
      SUB R0, #4
      STR R1, [R0];r2
      SUB R0, #4
      STR R1, [R0];//r1
      SUB R0, #4
      STR R1, [R0] ;   //r0
      MSR PSP,R0
      BX LR

send_LR:
      MOV LR,#0xFF000000
      ORR LR,LR,#0x00FF0000
      ORR LR,LR,#0x0000FF00
      ORR LR,LR,#0x000000FD
      BX  LR


get_SV_val:
      MRS R0, PSP
      ADD R0, #24
      LDR R1, [R0]
      SUB R1, #2
      LDR R0, [R1]
      BX  LR


getR0:
     MRS R0, PSP
     LDR R0,[R0]
     BX  LR

getR1:
     MRS R0, PSP
     ADD R0,#4
     LDR R0,[R0]
     BX  LR

