			TTL Exercise12 The Game
;****************************************************************
;The game lights up different comfigurations on the LED on the board
;the conbinations are none, both, red or green
;there are 10 rounds each round decreasing in time
;Name:  Jordan Jock Alejandro Vasquez
;Date:  11/30/2017
;Class:  CMPE-250
;Section:  4, Thursday, 11:00AM
;---------------------------------------------------------------
;Keil Template for KL46
;R. W. Melton
;September 25, 2017
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;12:UART0 IRQ mask
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;12:UART0 IRQ pending status
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;12:UART0 IRQ mask
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  0x1F
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  0xC0

;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << UART0_PRI_POS)
;--UART0
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK

;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at 24 MHz count rate
;0.01 s * 24,000,000 Hz = 240,000
;TSV = 240,000 - 1
PIT_LDVAL_10ms  EQU  239999
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRLn:  PIT timer control register n
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
; Management record structure field displacements 
IN_PTR    	EQU    	0 
OUT_PTR   	EQU    	4 
BUF_STRT  	EQU    	8 
BUF_PAST  	EQU    	12 
BUF_SIZE  	EQU    	16 
NUM_ENQD  	EQU    	17 

; Queue structure sizes 
Q_BUF_SZ  	EQU    	80        ;Room for 80 characters 
Q_REC_SZ  	EQU    	18        ;Management record size
	
CR			EQU		0x0D
LF			EQU		0x0A
	
MAX_STRING	EQU		79
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
Reset_Handler  PROC  {},{}
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL46 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
			BL		Init_UART0_ISR
			;initalizing the stop by putting the RunStopWatch to Zero
			LDR		R0,=RunStopWatch
			LDRB	R1,[R0,#0]
			MOVS	R1,#0
			STRB	R1,[R0,#0]
			;need to initalize the Count variable to zero
			LDR		R0,=Count
			LDR		R1,[R0,#0]
			MOVS	R1,#0
			STR		R1,[R0,#0]
			;initalize the PIT_IRQ
			BL		Init_PIT_IRQ
			;show the instructions
			LDR		R0,=Instruction
			BL		PutStringSB
			BL		NextLine
			LDR		R0,=Rule1
			BL		PutStringSB
			BL		NextLine
			LDR		R0,=Rule2
			BL		PutStringSB
			BL		NextLine
			LDR		R0,=Rule3
			BL		PutStringSB
			BL		NextLine
			LDR		R0,=KeyPress
			BL		PutStringSB
			BL		NextLine
			;need to wait for TxQueue to empty
			LDR		R2,=TxRecord
			LDRB	R1,[R2,#NUM_ENQD]
			MOVS	R0,#0
			
EmptyQueueLoop	CPSIE	I
				LDRB	R1,[R2,#NUM_ENQD]
				CMP		R1,#0
				BNE		EmptyQueueLoop
			;the TxQueue is empty now
        CPSID		I
			;check for Key Pressed
        LDR		R0,=RxRecord
			;num enqueueed -> R1
NoKey		LDRB	R1,[R0,#NUM_ENQD]
			CMP		R1,#0
			;no key pressed if nothing equeued want to keep checking 
			CPSIE	I
			BEQ		NoKey
			;dequeue the character from Rxqueue
			MOVS	R1,R0
			BL		Dequeue
			;elsewise a key was pressed and game can start
			BL		NextLine
;			LDR		R0,=NewRound
			BL		PutStringSB
			BL		NextLine
			
			;Want to generate a random number 
			
			;Then display LED and start timer
			
			LDR		R0,=RxRecord
			;Loop through comparing the time and checking for correct imput
			;num enqueueed -> R1
			LDRB	R1,[R0,#NUM_ENQD]
			CMP		R1,#0
			;no key pressed if nothing equeued want to keep checking 
			
      CPSIE	I
			BEQ		NoKey
			
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
;>>>>> begin subroutine code <<<<<
Init_UART0_ISR		PROC	{R0-R14},{}
;Initalyzes the KL64 for polled serial I/O
;initializes with the format 8 bits no parity and 1 stop @ 9600 baud
;Registers LR, PC, PSR are changed 
				PUSH{R0-R2,LR}				;push registers going to use inorder to give values back at end
			;Initalyze TxQueue
				LDR		R0,=TxQueue
				LDR		R1,=TxRecord
				LDR		R2,=Q_BUF_SZ
				BL		InitQueue
			;Initalyze RxQueue
				LDR		R0,=RxQueue
				LDR		R1,=RxRecord
				LDR		R2,=Q_BUF_SZ
				BL		InitQueue
			;Select MCGPLLCLK / 2 as UART0 clock source 
				LDR		R0,=SIM_SOPT2
				LDR		R1,=SIM_SOPT2_UART0SRC_MASK
				LDR		R2,[R0,#0]
				BICS	R2,R2,R1
				LDR		R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
				ORRS	R2,R2,R1
				STR  	R2,[R0,#0]
			;Enable external connection for UART0
				LDR		R0,=SIM_SOPT5
				LDR		R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
				LDR		R2,[R0,#0]
				BICS	R2,R2,R1
				STR 	R2,[R0,#0]
			;Enable clock for UART0 module
				LDR		R0,=SIM_SCGC4
				LDR		R1,=SIM_SCGC4_UART0_MASK
				LDR		R2,[R0,#0]
				ORRS	R2,R2,R1
				STR		R2,[R0,#0]
			;Enable clock for Port A module
				LDR		R0,=SIM_SCGC5
				LDR		R1,=SIM_SCGC5_PORTA_MASK
				LDR		R2,[R0,#0]
				ORRS	R2,R2,R1
				STR		R2,[R0,#0]
			;Connect PORT A pin 1 (PTA1) to UART0 Rx (J1 pin 02)
				LDR		R0,=PORTA_PCR1
				LDR		R1,=PORT_PCR_SET_PTA1_UART0_RX
				STR		R1,[R0,#0]
			;Connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 pin 04)
				LDR		R0,=PORTA_PCR2
				LDR		R1,=PORT_PCR_SET_PTA2_UART0_TX
				STR		R1,[R0,#0]
			;Disable UART0 receiver and transmitter
				LDR		R0,=UART0_BASE
				MOVS	R1,#UART0_C2_TI_RI
				LDRB	R2,[R0,#UART0_C2_OFFSET]
				BICS	R2,R2,R1
				STRB	R2,[R0,#UART0_C2_OFFSET]
            ;set UART0 IRQ Priority
                LDR     R0,=UART0_IPR
                LDR     R2,=NVIC_IPR_UART0_PRI_3
                LDR     R3,[R0,#0]
                ORRS    R3,R3,R2
                STR     R3,[R0,#0]
            ;clear any pending UART0 interrupts
                LDR     R0,=NVIC_ICPR
                LDR     R1,=NVIC_ICPR_UART0_MASK
                STR     R1,[R0,#0]
           ;unmask UART0 interrupt
                LDR     R0,=NVIC_ISER
                LDR     R1,=NVIC_ISER_UART0_MASK
                STR     R1,[R0,#0]
				
			;Set UART0 for 9600 baud, 8N1 Protocol
				LDR		R0,=UART0_BASE
				MOVS	R1,#UART0_BDH_9600
				STRB	R1,[R0,#UART0_BDH_OFFSET]
				MOVS	R1,#UART0_BDL_9600
				STRB	R1,[R0,#UART0_BDL_OFFSET]
				MOVS	R1,#UART0_C1_8N1
				STRB	R1,[R0,#UART0_C1_OFFSET]
				MOVS	R1,#UART0_C3_NO_TXINV
				STRB	R1,[R0,#UART0_C3_OFFSET]
				MOVS	R1,#UART0_C4_NO_MATCH_OSR_16
				STRB	R1,[R0,#UART0_C4_OFFSET]
				MOVS	R1,#UART0_C5_NO_DMA_SSR_SYNC
				STRB	R1,[R0,#UART0_C5_OFFSET]
				MOVS	R1,#UART0_S1_CLEAR_FLAGS
				STRB	R1,[R0,#UART0_S1_OFFSET]
				MOVS	R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
				STRB	R1,[R0,#UART0_S2_OFFSET]
			;Enable UART0 reciever and transmitter
				MOVS	R1,#UART0_C2_T_RI
				STRB	R1,[R0,#UART0_C2_OFFSET]
				POP{R0-R2,PC}
				ENDP
				LTORG
;-----------------------------------------------------------------------------------------------
UART0_ISR       PROC        {R0-R14},{}
;sets up the interrupt 
;
;
                CPSID       I
                PUSH{LR}
                ;interrupt source found in UART0_S1
                LDR         R0,=UART0_BASE
                ;if Tx interupt is enabled 
				;load the UART0_C2
				LDRB		R1,[R0,#UART0_C2_OFFSET]
				;R1 has UART0_C2
                MOVS			R2,#UART0_C2_TIE_MASK
				;want to add it with R1 to get value of TIE
				ANDS		R1,R1,R2
				CMP			R1,#0
				BEQ			SecondIf
				;if not equal TIE was set and Tx is enabled
				;need to look at TDRE in UART0_S1, its at bit 7
				LDRB		R1,[R0,#UART0_S1_OFFSET]
				MOVS		R2,#UART0_S1_TDRE_MASK
				ANDS		R1,R1,R2
				CMP			R1,#0
				BEQ			SecondIf
				;TDRE is set
				;dequeue and see if success
				LDR			R1,=TxRecord
				BL			Dequeue
				BCS			FailedDequeue
				;success
				;write cahracter that is in R0 to UART0 Transmit data reg
				LDR			R1,=UART0_BASE
				STRB		R0,[R1,#UART0_D_OFFSET]
				B			SecondIf
FailedDequeue	;disables Txinterrupt
				LDR			R0,=UART0_BASE
				LDR			R1,=UART0_C2_T_RI
				STRB		R1,[R0,#UART0_C2_OFFSET]
				B			SecondIf
SecondIf		LDR			R0,=UART0_BASE
				;need to check the RDRF that is in UART0_S1
				LDRB		R1,[R0,#UART0_S1_OFFSET]
				MOVS		R2,#UART0_S1_RDRF_MASK
				ANDS		R1,R1,R2
				CMP			R1,#0
				BEQ			Disabled
				;RDRF is enabled
				;read char in UART0_D
				LDR			R0,=UART0_BASE
				LDRB		R0,[R0,#UART0_D_OFFSET]
				;BL			GetChar
				;char is in R0
				LDR			R1,=RxRecord
				BL			Enqueue
				;if Rxqueue is full character is lost
				;all done
Disabled
				POP{PC}
				CPSIE		I
				ENDP
                
;-----------------------------------------------------------------------------------------------

Init_PIT_IRQ				PROC	{R0-R14},{}
;initalizes the PIT to generate an interrupt
;every 0.01s from PIT channel 0
			PUSH{R0-R3}
		;Enable clock for PIT module
			LDR		R0,=SIM_SCGC6
			LDR		R1,=SIM_SCGC6_PIT_MASK
			LDR		R2,[R0,#0]
			ORRS	R2,R2,R1
			STR		R2,[R0,#0]
		;Disable PIT timer 0
			LDR		R0,=PIT_CH0_BASE
			LDR		R1,=PIT_TCTRL_TEN_MASK
			LDR		R2,[R0,#PIT_TCTRL_OFFSET]
			BICS	R2,R2,R1
			STR		R2,[R0,#PIT_TCTRL_OFFSET]
		;Set PIT interrupt priority set to zero
			LDR		R0,=PIT_IPR
			LDR		R1,=NVIC_IPR_PIT_MASK
			;LDR	R2,=NVIC_IPR_PIT_PRI_0
			LDR		R3,[R0,#0]
			BICS	R3,R3,R1
			;ORRS	R3,R3,R2
			STR		R3,[R0,#0]
		;Clear any pending PIT interrupts
			LDR		R0,=NVIC_ICPR
			LDR		R1,=NVIC_ICPR_PIT_MASK
			STR		R1,[R0,#0]
		;Unmask PIT interrupts
			LDR		R0,=NVIC_ISER
			LDR		R1,=NVIC_ISER_PIT_MASK
			STR		R1,[R0,#0]
		;Enable PIT module
			LDR		R0,=PIT_BASE
			LDR		R1,=PIT_MCR_EN_FRZ
			STR		R1,[R0,#PIT_MCR_OFFSET]
		;Set PIT timer 0 period for 0.01 s
			LDR		R0,=PIT_CH0_BASE
			LDR		R1,=PIT_LDVAL_10ms
			STR		R1,[R0,#PIT_LDVAL_OFFSET]
		;Enable PIT timer 0 interrupt
			LDR		R1,=PIT_TCTRL_CH_IE
			STR		R1,[R0,#PIT_TCTRL_OFFSET]
			POP{R0-R3}
			BX		LR
			ENDP
;-----------------------------------------------------------------------------------------------			
PIT_ISR			PROC    {R0-R14},{}
;interrupt for the PIT
;interrupt vector 38
;uses RunStopWatch to determine if time shall be incremented
;Word Variable count used to measure the time

			CPSID	I
			PUSH{R0-R2}
			LDR		R0,=RunStopWatch
			LDRB	R1,[R0,#0]
			;compare to zero, zero means don't increment
			CMP		R1,#0
			BEQ		NoIncrement
			;if not zero going to increment
			LDR		R1,=Count
			LDR		R0,[R1,#0]
			ADDS	R0,R0,#1
			STR		R0,[R1,#0]
NoIncrement	
			;do nothing to Count
			;clear interrupts
			LDR		R1,=PIT_TFLG0
			LDR		R2,=PIT_TFLG_TIF_MASK
			STR		R2,[R1,#0]
			POP{R0-R2}
			CPSIE	I
			BX		LR
			ENDP
;-----------------------------------------------------------------------------------------------
InitQueue			PROC		{R0-R14},{}
;initializes the queue record structure
;and the queue buffer
;Inputs:
;R1: Start Address for record structure
;R0: Start Address for queue buffer
;R2: size of queue

				PUSH{R3}
				STR		R0,[R1,#IN_PTR]				;initializes the start pointer
				STR		R0,[R1,#OUT_PTR]			;initializes the end pointer
				STR		R0,[R1,#BUF_STRT]			;initializes the start address for the queue
				ADDS	R3,R2,R0					;finds the ending address
				STR		R3,[R1,#BUF_PAST]			;initializes the end address for the queue
				STRB	R2,[R1,#BUF_SIZE]			;initializes the buffer size
				MOVS	R3,#0						;Moves zero into R3 used for size
				STRB	R3,[R1,#NUM_ENQD]			;number enqueued == zero
				POP{R3}
				BX		LR
				ENDP
;-----------------------------------------------------------------------------------------------
Dequeue			PROC		{R0-R14},{}
;Attempts to get a char from the queue
;if queue not empty gets a single char 
;Inputs: R1 Record stucture
;Returns: R0 the dequeued char
;C flag if cleared successful dequeue; set not successful
				PUSH{R1-R3}
				;check to see if queue is empty
				LDRB	R2,[R1,#NUM_ENQD]			;finds the amount of enqueued char
				CMP		R2,#0						;compares the num enqueued and zero
				BLE		EMPTY						;R2 <= 0 
				LDR		R3,[R1,#OUT_PTR]			;gives R3 the outpointer which tells what address to dequeue
				LDRB	R0,[R3,#0]					;gives the value at R3 to R0
                ;update the num enqueued
				SUBS	R2,R2,#1					;R2 --
				STRB	R2,[R1,#NUM_ENQD]			;Stores the decreased num back to record
				;update the outpointer
				ADDS	R3,R3,#1					;Adds one word value to the out pointer
				LDR		R4,[R1,#BUF_PAST]			;load the ending address into R4
				CMP		R3,R4						;check if they are same
				BNE		Store						;if not equal done 
				;if equal then out pointer needs to go to start
				LDR		R3,[R1,#BUF_STRT]			;R3 get the start address
Store			STR		R3,[R1,#OUT_PTR]			;out pointer gets the start
				;need to show success and clear C flag
				MRS		R1,APSR
				MOVS	R2,#0x20
				LSLS	R2,R2,#24
				BICS	R1,R1,R2
				MSR		APSR,R1
				;Cleared the c flag
				B		Done
EMPTY			
				;need to set the C flag to show failure
				MRS		R1,APSR
				MOVS	R2,#0x20
				LSLS	R2,R2,#24
				ORRS	R1,R1,R2
				MSR		APSR,R1
				;set the c flag
				B		Done
Done			POP{R1-R3}
				BX		LR
				ENDP
;-----------------------------------------------------------------------------------------------
Enqueue			PROC		{R0-R14},{}
;Attempts to put a char in the queue
;if queue not full enqueues a single char 
;inputs: R0 the char to enqueue
;R1 queue record address
;retrun C flag 
;cleared == success set == failure
				PUSH{R0-R5}
				;check if queue is full
				LDRB	R2,[R1,#NUM_ENQD]			;R2 gets the number of enqueued
				LDRB	R3,[R1,#BUF_SIZE]			;Gets the buffer size into R3
				CMP		R2,R3						;compares the num enqueue to size
				BGE		Full						;if R2 >= R3 queue is full
				;If queue not full
				LDR		R4,[R1,#IN_PTR]				;R2 gets the in pointer
				STRB	R0,[R4,#0]					;stores the char at the inpointer
				;increment the number enqueued
				ADDS	R2,R2,#1					;Number enqueued ++
				STRB	R2,[R1,#NUM_ENQD]
				;increment the inpointer
				ADDS	R4,R4,#1					;increases the pointer by one
				LDR		R5,[R1,#BUF_PAST]			;R5 gets end of buffer
				;check to see if in pointer is less than R5
				CMP		R4,R5
				BLT		AllGood
				;R4 is equal or Greater
				LDR		R4,[R1,#BUF_STRT]			;R4 gets the buffer start
AllGood			STR		R4,[R1,#IN_PTR]				;In pointer changed to R4
				;need to show success and clear C flag
				MRS		R1,APSR
				MOVS	R2,#0x20
				LSLS	R2,R2,#24
				BICS	R1,R1,R2
				MSR		APSR,R1
				;Cleared the c flag
				B		DoneQueue
Full			
				;need to set the C flag to show failure
				MRS		R1,APSR
				MOVS	R2,#0x20
				LSLS	R2,R2,#24
				ORRS	R1,R1,R2
				MSR		APSR,R1
				;set the c flag
				B		DoneQueue
DoneQueue		
				POP{R0-R5}
				BX		LR
				ENDP
;-----------------------------------------------------------------------------------------------
PutNumHex		PROC		{R0-R14},{}
;prints to the terminal screen the text hex representation
;unsigned word value 
;inputs: R0 the unsigned word value
;Going to use a counter to count the amount of time gone through loop
;Uses PutNumU
				;word value has four byte values,8 four bit values
				PUSH{R0-R3,LR};push original values used at the very end
				MOVS		R1,#0x0F			;Creates a mask
				MOVS		R3,#8				;Initalize the counter
Loop		CMP			R3,#0				;Compare to see if done
				BLE			EndLoop
				SUBS		R3,R3,#1			;Counter --
				PUSH{R0};push original value
				ANDS		R0,R0,R1			;Get the four bit value
				CMP			R0,#9				;See if less than or equal
				BLE			Number
				;If greater than 9 it is letter
				SUBS		R0,R0,#10			;find the difference 
				ADDS		R0,R0,#65			;Add Uppercase ascii A
				;Clear the original value into another register
				POP{R2}
				PUSH{R0}						;Push onto stack for later print
				B LoopPrePare
Number	ADDS		R0,R0,#0x030			;Add 0x030 to get to ascii numbers
				;Clear the original value into another register
				POP{R2}
				PUSH{R0}
				B LoopPrePare
LoopPrePare		MOVS		R0,R2				;Move origianl value back into R0
				LSRS		R0,R0,#4			;Want to shift right four bits
				B			Loop

EndLoop			MOVS		R3,#8				;Initalize counter again
PrintLoop1		CMP			R3,#0				;Compare to see if done
				BLE			ENDSUB
				POP{R0}							;pop first char from stack
				BL			PutChar				;Print that char
				SUBS		R3,R3,#1			;Counter --
				B			PrintLoop1

ENDSUB			POP{R0-R3,PC}
				ENDP
;-----------------------------------------------------------------------------------------------
PutNumUB		PROC		{R0-R14},{}
;prints to the terminal the text decimal representation
;unsigned byte value
;inputs: R0 the unsigned byte value
;Uses: PutNumU
				PUSH{R0-R1,LR}
				;create a mask to only get the byte value in R0
				LDR		R1,=0x000000FF			;the mask for the byte value
				ANDS	R0,R0,R1				;The byte value goes into R0
				BL		PutNumU
				POP{R0-R1,PC}
				ENDP
;-----------------------------------------------------------------------------------------------
PutNumU			PROC	{R0-R14},{}
;displays the unsigned word value
;Successive Division with 10 as divisor
;pushs result onto a stack and then pops the results and prints
;going to display the quotient
;Uses PutChar, DIVU	

				PUSH{R0-R4,LR}				;push to preserve registers
				MOVS	R2,#0				;initalize the counter
				MOVS	R1,R0				;moves the hex number of length to R1
				MOVS	R0,#10				;gives the divisor to R0
Loop1			BL		DIVU				;R1/10
				;R0 Quotient R1 Remainder
				PUSH{R1}					;push the remainder onto stack
				ADDS	R2,R2,#1			;Add 1 to the counter
				CMP		R0,#0				;see if quotient is zero
				BEQ		PRINT				;if true all done
				MOVS	R1,R0				;move the quotient to r1 for next division
				MOVS	R0,#10				;move 10 back to R0 for division
				B		Loop1
				
PRINT		SUBS	R2,R2,#1			;remove from counter to see how many numbers are on stack
				POP{R0}						;take 1 num off stack
				ADDS	R0,R0,#0x30			;change number to ascii
				BL		PutChar				;print num
				CMP		R2,#0				;compare counter to see if done
				BEQ		EndLoop1				
				B		PRINT
				
														
EndLoop1		POP{R1}
				POP{R0-R3}
				POP{PC}
				ENDP

;-----------------------------------------------------------------------------------------------
DIVU			PROC 	{R2-R14},{}
;Copmutes the division of R1 the dividend 
;R0 being the divisor
;Returning the quotient in R0
;and the remainder in R1
;R1 / R0 = R0 remainder R1

				PUSH 	{R2}
				CMP		R0,#0			;compare divisor with number 0
				BEQ		DivZero			;if divisor is equal to zero
				MOVS	R2,#0
				CMP		R0,#1			;Compare R0 and 1
				BEQ		OneDem			;if R0 == 1
DIVULoop
				CMP		R0,R1			;compare r0 to r1
				BHI		Greater			; R0 > R1
				SUBS	R1,R1,R0		;R1 - R0
				ADDS	R2,R2,#1		;R2 + 1, Division has happened once
				B		  DIVULoop		;Back to loop

Greater	
				MOVS	R0,R2			;MOVE into R0 for quotient
				B		  End1
			
OneDem
				MOVS	R0,R1			;Becuse the denominator is 1 the numerator is as many times as it goes into it R0 <- R1
				MOVS	R1,#0			;no remainder
				B		  End1
				
End1		
				PUSH{R0}				;push R0 onto stack to prepare for clearing of C flag
				PUSH{R1}				;push R1 for same reason
				;Clearing C flag
				MRS		R0,APSR
				MOVS	R1,#0x20
				LSLS	R1,R1,#24
				BICS	R0,R0,R1
				MSR		APSR,R0
				;Done Clearing
				POP{R1}					;Pop all values back off into correct places
				POP{R0}
				POP{R2}
				BX		LR
				ENDP
				
DivZero		
				POP{R2}					;pop R2 before setting 
				PUSH{R0}				;Push R0 and R1 onto the stack before setting C flag
				PUSH{R1}
				;Setting C flag
				MRS		R0,APSR
				MOVS	R1,#0x20
				LSLS	R1,R1,#24
				ORRS	R0,R0,R1
				MSR		APSR,R0
				;DONE setting C flag
				POP{R0}					;pop R1, R0 back off
				POP{R1}
				BX		LR
				ENDP
;-----------------------------------------------------------------------------------------------
PutChar		PROC	{R0-R14},{}
;Puts a char onto the terminal for user to view
;enqueues char to TxQueue
				PUSH{R0-R2,LR}
				LDR			R1,=TxRecord
RepeatPut		CPSID		I
				BL			Enqueue
				CPSIE		I
				BCS			RepeatPut
				;Enable TxInterrupt
				LDR			R0,=UART0_BASE
				MOVS		R1,#UART0_C2_TI_RI
				STRB		R1,[R0,#UART0_C2_OFFSET]
				POP{R0-R2,PC}
				ENDP
;-----------------------------------------------------------------------------------------------
GetChar			PROC	{R1-R14},{}
;Reads a single character from the terminal key board into R0
;uses interupt 
;dequeue from the Rxqueue
;return char in  R0
				PUSH{R1-R4,LR}
				LDR		R1,=RxRecord
RepeatGet
				CPSID	I
				BL		Dequeue
				CPSIE	I
				BCS		RepeatGet
				POP{R1-R4,PC}
				ENDP
;-----------------------------------------------------------------------------------------------
PutStringSB				PROC	{R0-R14},{}
;Displays a null-terminated string from memory
;starting from R0 points to address
;Input R0 Pointer to source string
;Changes	APSR
;Uses	PutChar	
						PUSH{R0-R2,LR}
						
						MOVS	R2,R0				;Moves address to R2 
Loop2					LDRB	R0,[R2,#0]			;loads the byte value of R2
  						ADDS	R2,#1				;Moves R2 address up  one byte 
						CMP		R0,#0x00			;If null terminator comes up
						BEQ		Zero				
						BL		PutChar				;Takes value of R1 and displays it
						B		Loop2				;Back to loop
Zero					
						POP{R0-R2}
						POP{PC}
						ENDP
;-----------------------------------------------------------------------------------------------
NextLine				PROC	{R0-R14},{}
;moves the cursor to the next line at the begining
;Nothing is going to be altered with exception of ASPR 
;Uses PutChar
						PUSH{R0,LR}
						MOVS	R0,#CR				;puts carriage return in R0
						BL		PutChar
						MOVS	R0,#LF				;Puts line feed in R0
						BL		PutChar
						POP{R0,PC}
						ENDP
;-----------------------------------------------------------------------------------------------
GetStringSB				PROC	{R0-R14},{}
;reads a string from the terminal keyboard 
;until "Enter" is pressed
;Stores the string in memory stating at the address
;of where R0 points
;Inputs: R0: pointer to desination string
;Changes: APSR
;Uses: GetChar 

						PUSH{R0-R4,LR}
						MOVS	R3,#MAX_STRING		;puts max strign value in R3
						MOVS	R2,#0				;R2 sets to zero
						MOVS	R4,R0				;Moves the address pointer to R4 due to R0 used for getchar
Begin					PUSH{R4}
						BL 		GetChar				;R0 <- char typed
						POP{R4}
						CMP		R0,#CR				;EQU CR == 0x0D compares char with CR
						BEQ		CarReturn			;if char == CR then breaks
						
						ADDS	R2,#1				;R2 going to see how many chars are typed
						CMP		R2,R3				;compares number of times and max string-1
						BHS		Begin				;Don't want to store anymore chars if string is full
						;Else wise want to store the char in string
						PUSH{R4}
						BL		PutChar
						POP{R4}
						STRB 	R0,[R4,#0]			;stores byte(char) into R4 with no offset
						ADDS	R4,#1				;R4 address moves up 1 byte
						B		Begin
CarReturn				MOVS	R0,#0x00				;R0 gets null terminator
						STRB	R0,[R4,#0]
						POP{R0-R4}
						POP{PC}
						ENDP
;-----------------------------------------------------------------------------------------------
Clear_StopWatch			PROC	{R0-R14},{}
;clears the variable RunStopWatch by setting it to zero
						PUSH{R0-R1}
						LDR		R0,=RunStopWatch
						LDRB	R1,[R0,#0]
						MOVS	R1,#0
						STRB	R1,[R0,#0]
						POP{R0-R1}
						BX		LR
						ENDP
;-----------------------------------------------------------------------------------------------
Start_Watch				PROC	{R0-R14},{}
;clears the stopwatch count variable and sets the runstopwatch to 1
						PUSH{R0-R1}
					;clear the stopwatch count variable and set RunStopWatch to 1
						LDR		R0,=Count
						LDR		R1,[R0,#0]
						MOVS	R1,#0
						STR		R1,[R0,#0]
						LDR		R0,=RunStopWatch
						LDRB	R1,[R0,#0]
						MOVS	R1,#1
						STRB	R1,[R0,#0]
						POP{R0-R1}
						BX		LR
						ENDP
;-----------------------------------------------------------------------------------------------
Start_LED   PROC {R0-R14},{}
            PUSH {R0-R1}
            
            ;Enable port E
            LDR  R0,=SIM_SCGC5
            LDR  R0,=SIM_SCGC5_PORTE_MASK
            LDR  R1,[R0,#0]       ;current SIM_SCGC5 value
            ORRS R1,R1,R0         ;only PORTE bit set
            STR  R1,[R0,#0]       ;update SIM_SCGC5
            
            ;Enable port D
            LDR  R0,=SIM_SCGC5
            LDR  R0,=SIM_SCGC5_PORTD_MASK
            LDR  R1,[R0,#0]       ;current SIM_SCGC5 value
            ORRS R1,R1,R0         ;only PORTD bit set
            STR  R1,[R0,#0]       ;update SIM_SCGC5

;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
            IMPORT  HardFault_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    HardFault_Handler  ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendableSrvReq (pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 xfer complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 xfer complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 xfer complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 xfer complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:command complete; read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:I2C1
            DCD    Dummy_Handler      ;26:SPI0 (all IRQ sources)
            DCD    Dummy_Handler      ;27:SPI1 (all IRQ sources)
            DCD    UART0_ISR	      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:UART1 (status; error)
            DCD    Dummy_Handler      ;30:UART2 (status; error)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:TPM2
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    PIT_ISR		      ;38:PIT (all IRQ sources)
            DCD    Dummy_Handler      ;39:I2S0
            DCD    Dummy_Handler      ;40:USB0
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:Segment LCD
            DCD    Dummy_Handler      ;46:PORTA pin detect
            DCD    Dummy_Handler      ;47:PORTC and PORTD pin detect
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
Instruction		DCB		"Welcome to the Game! The rules of the game follow.\0",0
Rule1			    DCB		"There are 10 rounds to the game decreasing in time each round.\0",0
Rule2			    DCB		"You are going to try to guess the LED light and will get points for guessign correctly.\0",0
Rule3 			  DCB		"There are three choices for LED N(no LED), B(Both LED), R(Red LED),and G(Green LED)\0",0
KeyPress		  DCB		"Press any key to get started\0",0
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
RxQueue		    SPACE		Q_BUF_SZ
TxQueue		    SPACE		Q_BUF_SZ

RxRecord	    SPACE		Q_REC_SZ
    ALIGN
TxRecord	    SPACE		Q_REC_SZ
    ALIGN
Count			    SPACE		4
RunStopWatch	SPACE		1
    ALIGN
StringBuf		  SPACE	  MAX_STRING
    ALIGN
After         SPACE   2
Before        SPACE   2


  
;>>>>>   end variables here <<<<<
            ALIGN
            END
