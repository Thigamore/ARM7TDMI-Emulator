package arm7tdmiemu

import (
	"github.com/Thigamore/arm7tdmiEmu/bits"
	dt "github.com/Thigamore/arm7tdmiEmu/dataStruct"
)

// An ARM7tdmi processor

type CPU struct {
	//The bus
	bus *BUS
	//Important registers
	//13 - SP
	//14 - Link Register
	//15 - PC
	regs    [16]*uint32 //Current registers in use
	regsUsr [16]uint32  //Registers for user mode
	regsFiq [16]uint32  //Registers for Fast interrupt mode
	regsSvc [16]uint32  //Supervisor mode
	regsAbt [16]uint32  //Abort mode
	regsIrq [16]uint32  //Interrupt mode
	regsUnd [16]uint32  //Undefined mode
	cpsr    uint32      //Current Program Status Register
	spsrFiq uint32      //Old CPSR for different modes
	spsrIrq uint32
	spsrSvc uint32
	spsrAbt uint32
	spsrUnd uint32

	//Signals
	signals []*bool

	//Stores the previous mode
	prevMode int
}

const (
	//Important Registers
	SP = 13 //Stack Pointer
	LR = 14 //Link Register
	PC = 15 //Program Counter

	//Masks for bits in CPSR/SPSR
	//FLAGS
	SIGN_F     uint32 = 0b10000000000000000000000000000000
	ZERO_F     uint32 = 0b01000000000000000000000000000000
	CARRY_F    uint32 = 0b00100000000000000000000000000000
	OVERFLOW_F uint32 = 0b00010000000000000000000000000000
	//Disables
	IRQ_DIS uint32 = 0b00000000000000000000000010000000
	FIQ_DIS uint32 = 0b00000000000000000000000001000000
	//Misc
	STATE uint32 = 0b00000000000000000000000000100000
	MODE  uint32 = 0b00000000000000000000000000011111

	//Operation numbers
	ADC = byte(iota - 11) //Add with carry
	ADD
	AND
	B    //Branch
	BIC  //Bit Clear
	BL   //Branch With Link
	BX   //Branch and Exchange
	CDP  //Coprocessor Data Processing
	CMN  //Compare Negative
	CMP  //Compare
	EOR  //XOR
	LDC  //Load coprocessor from memory
	LDM  //Load multiple registers
	LDR  //Load register from memory
	MCR  //Move CPU register to coprocessor register
	MLA  //Multiply Accumulate
	MLAL //Multiply Accumulate Long
	MOV  //Move register or constant
	MRC  //Move from coprocessor register to CPU register
	MRS  //Move PSR status to register
	MSR  //Move register to PSR status
	MUL  //Multiply
	MULL //Multiply Long
	MVN  //Move negative number
	ORR  //OR
	RSB  //Reverse subtract
	RSC  //Reverse subtract with carry
	SBC  //Subtract with Carry
	STC  //Store coprocessor register to memory
	STM  //Store multiple
	STR  //Store register to memory
	SUB  //Subtract
	SWI  //Software interrupt
	SWP  //Swap register with memory
	TEQ  //Test bitwise equality
	TST  //Test bits

	//Signals
	BIGEND = iota - 47

	//Modes
	USR = iota - 48
	FIQ_M
	SVC
	ABT
	IRQ_M
	UND

	//Exceptions
	BL_E = iota - 54
	RESET
	DABT
	FIQ_E
	IRQ_E
	PABT
	UDEF
	SWI_E

	//State
	ARM   = 0
	THUMB = 1
)

//* ------------------------ Creation -----------------------
//Initializes a new CPU
func InitCpu() *CPU {
	//Initialize the signals
	signals := make([]*bool, 1)
	cpu := &CPU{
		signals: signals,
	}
	bus := &BUS{
		signals: signals,
	}
	for i := 0; i < len(cpu.regs); i++ {
		cpu.regs[0] = &cpu.regsUsr[0]
	}
	cpu.bus = bus
	cpu.prevMode = USR
	return cpu
}

//* ------------------------- Mode Switch -------------------
func (cpu *CPU) modeSwitch(mode int) {
	if cpu.prevMode == mode {
		return
	}
	switch mode {
	case USR:
		if cpu.prevMode == FIQ_M {
			for i := 8; i < 15; i++ {
				cpu.regs[i] = &cpu.regsUsr[i]
			}
		} else {
			for i := 13; i < 15; i++ {
				cpu.regs[i] = &cpu.regsUsr[i]
			}
		}
	case FIQ_M:
		for i := 8; i < 15; i++ {
			cpu.regs[i] = &cpu.regsFiq[i]
		}
		cpu.spsrFiq = cpu.cpsr
	case SVC:
		if cpu.prevMode == FIQ_M {
			for i := 8; i < 15; i++ {
				cpu.regs[i] = &cpu.regsSvc[i]
			}
		} else {
			for i := 13; i < 15; i++ {
				cpu.regs[i] = &cpu.regsSvc[i]
			}
		}
		cpu.spsrSvc = cpu.cpsr
	case ABT:
		if cpu.prevMode == FIQ_M {
			for i := 8; i < 15; i++ {
				cpu.regs[i] = &cpu.regsAbt[i]
			}
		} else {
			for i := 13; i < 15; i++ {
				cpu.regs[i] = &cpu.regsAbt[i]
			}
		}
		cpu.spsrAbt = cpu.cpsr
	case IRQ_M:
		if cpu.prevMode == FIQ_M {
			for i := 8; i < 15; i++ {
				cpu.regs[i] = &cpu.regsIrq[i]
			}
		} else {
			for i := 13; i < 15; i++ {
				cpu.regs[i] = &cpu.regsIrq[i]
			}
		}
		cpu.spsrIrq = cpu.cpsr
	case UND:
		if cpu.prevMode == FIQ_M {
			for i := 8; i < 15; i++ {
				cpu.regs[i] = &cpu.regsUnd[i]
			}
		} else {
			for i := 13; i < 15; i++ {
				cpu.regs[i] = &cpu.regsUnd[i]
			}
		}
		cpu.spsrUnd = cpu.cpsr
	}
	cpu.prevMode = mode
}

//* -----------------------  Exception --------------------------
func (cpu *CPU) Exception(exception int) {
	pc := *cpu.regs[PC]
	switch exception {
	case DABT:
		if cpu.state() == ARM {
			pc += 8
		} else {
			pc += 8
		}
		cpu.modeSwitch(ABT)
		cpu.disableIRQ()
	case BL_E:
		if cpu.state() == ARM {
			pc += 4
		} else {
			pc += 2
		}
	case SWI_E:
		if cpu.state() == ARM {
			pc += 4
		} else {
			pc += 2
		}
		cpu.modeSwitch(SVC)
		cpu.disableIRQ()

	case PABT:
		if cpu.state() == ARM {
			pc += 4
		} else {
			pc += 4
		}
		cpu.modeSwitch(ABT)
		cpu.disableIRQ()
	case FIQ_E:
		if cpu.state() == ARM {
			pc += 4
		} else {
			pc += 4
		}
		cpu.modeSwitch(FIQ_M)
		cpu.disableIRQ()
		cpu.disableFIQ()
	case IRQ_E:
		if cpu.state() == ARM {
			pc += 4
		} else {
			pc += 4
		}
		cpu.modeSwitch(IRQ_M)
		cpu.disableIRQ()
	case UDEF:
		if cpu.state() == ARM {
			pc += 4
		} else {
			pc += 2
		}
		cpu.modeSwitch(UND)
		cpu.disableIRQ()
	}
	(*cpu.regs[LR]) = pc
	(*cpu.regs[PC]) = cpu.getExceptionVector(exception)
	cpu.toArm()
}

//Gets the Exception Vector address
func (cpu *CPU) getExceptionVector(exception int) uint32 {
	switch exception {
	case RESET:
		return cpu.bus.Read(0x0)
	case UDEF:
		return cpu.bus.Read(0x4)
	case SWI_E:
		return cpu.bus.Read(0x8)
	case PABT:
		return cpu.bus.Read(0xC)
	case DABT:
		return cpu.bus.Read(0x10)
	case IRQ_E:
		return cpu.bus.Read(0x18)
	case FIQ_E:
		return cpu.bus.Read(0x1C)
	}
}

//* -------------------------- Cycle -----------------------
//TODO add waitstates etc

//Calculates the total number of cycles
//Set as one cycle of time due to memory not needing more time to be access
//aka everything is already done sequentially
func (cpu *CPU) CalcCycle(S int, N int, I int, C int) int {
	total := 0
	total += S
	total += N
	total += I
	return total
}

//* -------------------------- OP Tree -----------------------
//A tree which gives which operation should be done
//Left = 0
//Right = 1
var opTree dt.OpTree

func createOpTree() {
}

//* --------------------------- Flags -----------------------
//Functions that handle flag + arm/thumb state

//Checks whether the sign flag is set
func (cpu *CPU) signIsSet() bool {
	return (cpu.cpsr & SIGN_F) > 0
}

//Checks whether the zero flag is set
func (cpu *CPU) zeroIsSet() bool {
	return (cpu.cpsr & ZERO_F) > 0
}

//Checks whether the carry flag is set
func (cpu *CPU) carryIsSet() bool {
	return (cpu.cpsr & CARRY_F) > 0
}

//Checks whether the overflow flag is set
func (cpu *CPU) overflowIsSet() bool {
	return (cpu.cpsr & OVERFLOW_F) > 0
}

//Sets the sign flag
func (cpu *CPU) setSign() {
	cpu.cpsr |= SIGN_F
}

//Sets the zero flag
func (cpu *CPU) setZero() {
	cpu.cpsr |= ZERO_F
}

//Sets the carry flag
func (cpu *CPU) setCarry() {
	cpu.cpsr |= CARRY_F
}

//Sets the overflow flag
func (cpu *CPU) setOverflow() {
	cpu.cpsr |= OVERFLOW_F
}

//Clears the sign flag
func (cpu *CPU) clearSign() {
	cpu.cpsr &= (^SIGN_F)
}

//Clears the zero flag
func (cpu *CPU) clearZero() {
	cpu.cpsr &= (^ZERO_F)
}

//Clears the carry flag
func (cpu *CPU) clearCarry() {
	cpu.cpsr &= (^CARRY_F)
}

//Clears the overflow flag
func (cpu *CPU) clearOverflow() {
	cpu.cpsr &= (^OVERFLOW_F)
}

//Returns the current state, arm or thumb
func (cpu *CPU) state() int {
	return int(bits.GetBit(cpu.cpsr, 26))
}

//Changes mode to ARM
func (cpu *CPU) toArm() {
	cpu.cpsr &= (^STATE)
}

//Changes mode to THUMB
func (cpu *CPU) toThumb() {
	cpu.cpsr |= STATE
}

//Disables IRQ
func (cpu *CPU) disableIRQ() {
	cpu.cpsr |= IRQ_DIS
}

//Disables FIQ
func (cpu *CPU) disableFIQ() {
	cpu.cpsr |= FIQ_DIS
}

//Enables IRQ
func (cpu *CPU) enableIRQ() {
	cpu.cpsr &= (^IRQ_DIS)
}

//Enables FIQ
func (cpu *CPU) enableFIQ() {
	cpu.cpsr &= (^FIQ_DIS)
}

// * -------------------- Conditions --------------------------
//All conditions are based on their code

//Checks if equal
func (cpu *CPU) eq() bool {
	return cpu.zeroIsSet()
}

//Checks if not equal
func (cpu *CPU) ne() bool {
	return !cpu.zeroIsSet()
}

//Checks if unsigned higher or same
//Carry set
//cs/hs
func (cpu *CPU) hs() bool {
	return cpu.carryIsSet()
}

//Checks if unsigned lower
//Carry cleared
//cc/lo
func (cpu *CPU) lo() bool {
	return !cpu.carryIsSet()
}

//Checks if negative
func (cpu *CPU) mi() bool {
	return cpu.signIsSet()
}

//Checks if positive
func (cpu *CPU) pl() bool {
	return !cpu.signIsSet()
}

//Checks if signed signed overflow
//V Set
func (cpu *CPU) vs() bool {
	return cpu.overflowIsSet()
}

//Checks if no signed overflow
//V Cleared
func (cpu *CPU) vc() bool {
	return !cpu.overflowIsSet()
}

//Checks if unsigned higher
func (cpu *CPU) hi() bool {
	return (cpu.carryIsSet()) && (!cpu.zeroIsSet())
}

//Checks if unsigned lower or same
func (cpu *CPU) ls() bool {
	return (!cpu.carryIsSet()) || (cpu.zeroIsSet())
}

//Checks if signed greater or equal
func (cpu *CPU) ge() bool {
	return cpu.signIsSet() == cpu.overflowIsSet()
}

//Checks if signed less than
func (cpu *CPU) lt() bool {
	return cpu.signIsSet() != cpu.overflowIsSet()
}

//Checks if signed greater than
func (cpu *CPU) gt() bool {
	return (!cpu.zeroIsSet()) && (cpu.signIsSet() == cpu.overflowIsSet())
}

//Checks if signed less than or equal
func (cpu *CPU) le() bool {
	return (cpu.zeroIsSet()) || (cpu.signIsSet() != cpu.overflowIsSet())
}

//Always executed
func (cpu *CPU) al() bool {
	return true
}

//Never executed
func (cpu *CPU) nv() bool {
	return false
}

// * --------------------- Machine Cycle -----------------
//Runs the next instruction and returns the machine cycles it takes
func (cpu *CPU) executeNext() int {
	//Fetch the next operation
	var op uint32
	//TODO

	//Check the condition
	cond := false
	switch op >> 28 {
	case 0b0000:
		cond = cpu.eq()
	case 0b0001:
		cond = cpu.ne()
	case 0b0010:
		cond = cpu.hs()
	case 0b0011:
		cond = cpu.lo()
	case 0b0100:
		cond = cpu.mi()
	case 0b0101:
		cond = cpu.pl()
	case 0b0110:
		cond = cpu.vs()
	case 0b0111:
		cond = cpu.vc()
	case 0b1000:
		cond = cpu.hi()
	case 0b1001:
		cond = cpu.ls()
	case 0b1010:
		cond = cpu.ge()
	case 0b1011:
		cond = cpu.lt()
	case 0b1100:
		cond = cpu.gt()
	case 0b1101:
		cond = cpu.le()
	case 0b1110:
		cond = cpu.al()
	case 0b1111:
		cond = cpu.nv()
	}

	if cond {
		switch getOpNum(op) {
		case ADC:
			cpu.ADC(op)
		case ADD:
			cpu.ADD(op)
		case AND:
			cpu.AND(op)
		case B:
			cpu.B(op)
		case BIC:
			cpu.BIC(op)
		case BL:
			cpu.BL(op)
		case BX:
			cpu.BX(op)
		case CDP:
			cpu.CDP(op)
		case CMN:
			cpu.CMN(op)
		case CMP:
			cpu.CMP(op)
		case EOR:
			cpu.EOR(op)
		case LDC:
			cpu.LDC(op)
		case LDM:
			cpu.LDM(op)
		case LDR:
			cpu.LDR(op)
		case MCR:
			cpu.MCR(op)
		case MLA:
			cpu.MLA(op)
		case MLAL:
			cpu.MLAL(op)
		case MOV:
			cpu.MOV(op)
		case MRC:
			cpu.MRC(op)
		case MRS:
			cpu.MRS(op)
		case MSR:
			cpu.MSR(op)
		case MUL:
			cpu.MUL(op)
		case MULL:
			cpu.MULL(op)
		case MVN:
			cpu.MVN(op)
		case ORR:
			cpu.ORR(op)
		case RSB:
			cpu.RSB(op)
		case RSC:
			cpu.RSC(op)
		case SBC:
			cpu.SBC(op)
		case STC:
			cpu.STC(op)
		case STM:
			cpu.STM(op)
		case STR:
			cpu.STR(op)
		case SUB:
			cpu.SUB(op)
		case SWI:
			cpu.SWI(op)
		case SWP:
			cpu.SWP(op)
		case TEQ:
			cpu.TEQ(op)
		case TST:
			cpu.TST(op)
		}
	}
	return 1
}

//Takes an opcode and gives the operation number
func getOpNum(op uint32) byte {
	currentBit := 4
	curNode := opTree.Root
	for child := curNode; child != nil; {
		curNode = child
		child = curNode.Children[bits.GetBit(op, currentBit)]
	}
	return curNode.Val
}

//* ------------------------ Operation Functions ------------------------
//Functions for all operations as stated in the const

//Supplemental operation functions for ease

//Returns the spsr register to write to
func (cpu *CPU) getModeSpsr() *uint32 {
	switch bits.GetBits(cpu.cpsr, 27, 31) {
	case 0b10001:
		return &cpu.spsrFiq
	case 0b10010:
		return &cpu.spsrIrq
	case 0b10011:
		return &cpu.spsrSvc
	case 0b10111:
		return &cpu.spsrAbt
	case 0b11011:
		return &cpu.spsrUnd
	}
	return nil
}

//Returns shift, r15 op byte delay(due to prefetching), reg2, and carry
func (cpu *CPU) shift(op uint32) (uint32, uint32, uint32, bool) {
	toShift := (*cpu.regs[bits.GetBits(op, 28, 31)])
	shiftType := bits.GetBits(op, 25, 26)
	var reg, delay uint32

	var shiftBy byte

	if bits.GetBit(op, 27) == 1 {
		//12
		reg = bits.GetBits(op, 20, 23)
		shiftBy = byte(bits.GetBits((*cpu.regs[reg]), 24, 31))
		delay = 12
	} else {
		//8
		shiftBy = byte(bits.GetBits(op, 20, 24))
		delay = 8
	}

	if shiftBy == 0 {
		return toShift, delay, reg, cpu.carryIsSet()
	}

	switch shiftType {
	case 0b0:
		if shiftBy == 32 {
			return 0, delay, reg, bits.GetBit(toShift, 31) == 1
		} else if shiftBy > 32 {
			return 0, delay, reg, false
		}
		//logical left
		return toShift << shiftBy, delay, reg, bits.GetBit(toShift, int(shiftBy-1)) == 1
	case 0b01:
		if shiftBy == 32 {
			return 0, delay, reg, bits.GetBit(toShift, 0) == 1
		} else if shiftBy > 32 {
			return 0, delay, reg, false
		}
		//logical right
		return toShift >> shiftBy, delay, reg, bits.GetBit(toShift, int(32-shiftBy)) == 1
	case 0b10:
		//arithmetic right
		out := toShift >> shiftBy
		replace := bits.GetBit(toShift, 0)
		if shiftBy > 32 {
			shiftBy = 32
		}
		if replace == 1 {
			for i := byte(0); i < shiftBy; i++ {
				out |= 0b10000000000000000000000000000000 >> i
			}
		}
		return out, delay, reg, bits.GetBit(toShift, int(32-shiftBy)) == 1
	case 0b11:
		//rotate right
		if shiftBy == 0 {
			//Rotate Right Extended
			if cpu.carryIsSet() {
				return (toShift >> 1) | 1<<31, delay, reg, bits.GetBit(toShift, 31) == 1
			}
		} else {
			if shiftBy == 32 {
				return toShift, delay, reg, bits.GetBit(toShift, 0) == 1
			}
			if shiftBy > 32 {
				shiftBy = (shiftBy % 32)
				if shiftBy == 0 {
					shiftBy = 32
				}
			}
			return bits.RotateRight(toShift, int(shiftBy)), delay, reg, bits.GetBit(toShift, int(32-shiftBy)) == 1
		}
	}
	return 0, delay, reg, false
}

//does the rotate instead of shift operation for data processing instructions
func (cpu *CPU) rotate(op uint32) (uint32, bool) {
	rotateBy := bits.GetBits(op, 20, 23)
	val := bits.GetBits(op, 24, 31)
	return bits.RotateRight(val, int(rotateBy*2)), bits.GetBit(val, int(32-(rotateBy*2))) == 1
}

//Returns op1, op2, and carry
func (cpu *CPU) getDataProcessingOperands(op uint32, cycles *[]int) (uint32, uint32, bool) {
	reg1 := bits.GetBits(op, 12, 15)

	var op1, op2, reg2, delay uint32
	var carry bool
	switch bits.GetBit(op, 6) {
	case 0:
		op2, delay, reg2, carry = cpu.shift(op)
		(*cycles)[2] += 1
	case 1:
		op2, carry = cpu.rotate(op)
		delay = 8
	}
	if reg1 == 15 {
		op1, _ = bits.Add((*cpu.regs[reg1]), delay)
	} else if reg2 == 15 {
		op2, _ = bits.Add(op2, delay)
	}
	return op1, op2, carry
}

//Gets the operands for a multiply instruction and the accumulate
//op1,op2,accumulate
func (cpu *CPU) getMultiplyOperands(op uint32) (uint32, uint32, uint32) {
	return (*cpu.regs[bits.GetBits(op, 28, 31)]), (*cpu.regs[bits.GetBits(op, 20, 23)]), (*cpu.regs[bits.GetBits(op, 16, 19)])
}

func (cpu *CPU) dataProcessFlagSet(result uint32, carry bool) {
	//Set negative flag
	if bits.GetBit(result, 0) == 1 {
		cpu.setSign()
	} else {
		cpu.clearSign()
	}
	//Set the zero flag
	if result == 0 {
		cpu.setZero()
	} else {
		cpu.clearZero()
	}
	//Set the carry flag
	if carry {
		cpu.setCarry()
	} else {
		cpu.clearCarry()
	}

}

func (cpu *CPU) setMultiplyFlag(res uint32) {
	if res == 0 {
		cpu.setZero()
	} else {
		cpu.clearZero()
	}
	if bits.GetBit(res, 0) == 1 {
		cpu.setSign()
	} else {
		cpu.clearSign()
	}
}

func (cpu *CPU) setMultiplyLongFlag(hi uint32, lo uint32) {
	if hi == 0 && lo == 0 {
		cpu.setZero()
	} else {
		cpu.clearZero()
	}
	if bits.GetBit(hi, 0) == 1 {
		cpu.setSign()
	} else {
		cpu.clearSign()
	}
}

func (cpu *CPU) ADC(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, _ := cpu.getDataProcessingOperands(op, &cycles)
	var res uint32
	var over bool
	if cpu.carryIsSet() {
		res, _ = bits.AddCarry(op2, op1, 1)
	} else {
		res, _ = bits.Add(op2, op1)
	}
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		(*cpu.regs[reg]) = res

		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, over)
			if over {
				cpu.setOverflow()
			} else {
				cpu.clearOverflow()
			}
		}
	}
	//TODO work on cycles
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) ADD(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, _ := cpu.getDataProcessingOperands(op, &cycles)
	res, over := bits.Add(op1, op2)
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, over)
			if over {
				cpu.setOverflow()
			} else {
				cpu.clearOverflow()
			}
		}
	}
	//TODO work on cycles
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) AND(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, carry := cpu.getDataProcessingOperands(op, &cycles)
	res := op1 & op2
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, carry)
		}
	}
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) B(op uint32) int {
	offset := bits.SignExtend(bits.GetBits(op, 8, 31) << 2)
	(*cpu.regs[PC]), _ = bits.Add((*cpu.regs[PC]), offset)
	(*cpu.regs[PC]) += 8 //Two bytes ahead in normal arm7tdmi
	//TODO add Cycles
	return cpu.CalcCycle(2, 1, 0, 0)
}

func (cpu *CPU) BIC(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, carry := cpu.getDataProcessingOperands(op, &cycles)
	res := op1 & (^op2)
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, carry)
		}
	}
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) BL(op uint32) int {
	offset := bits.SignExtend(bits.GetBits(op, 8, 31) << 2)
	(*cpu.regs[LR]) = (*cpu.regs[PC]) + 4
	(*cpu.regs[PC]), _ = bits.Add((*cpu.regs[PC]), offset)
	(*cpu.regs[PC]) += 12 //Two bytes ahead in normal arm7tdmi + next instruction
	//TODO add Cycles
	return cpu.CalcCycle(2, 1, 0, 0)
}

func (cpu *CPU) BX(op uint32) int {
	regNum := bits.GetBits(op, 28, 32)
	cpu.regs[PC] = cpu.regs[regNum]
	if bits.GetBit((*cpu.regs[regNum]), 31) == 1 {
		cpu.cpsr |= 0b00000000000000000000000000100000
	} else {
		cpu.cpsr &= 0b11111111111111111111111111011111
	}
	//TODO add cycles
	return cpu.CalcCycle(2, 1, 0, 0)
}

func (cpu *CPU) CDP(op uint32) int {

}

func (cpu *CPU) CMN(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, _ := cpu.getDataProcessingOperands(op, &cycles)
	res, over := bits.Add(op1, op2)

	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, over)
			if over {
				cpu.setOverflow()
			} else {
				cpu.clearOverflow()
			}
		}
	}
	//TODO work on cycles
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) CMP(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, _ := cpu.getDataProcessingOperands(op, &cycles)
	res, bor := bits.Sub(op1, op2)
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, bor)
			if bor {
				cpu.setOverflow()
			} else {
				cpu.clearOverflow()
			}
		}
	}
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) EOR(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, carry := cpu.getDataProcessingOperands(op, &cycles)
	res := op1 ^ op2
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, carry)
		}
	}
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) LDC(op uint32) int {

}

func (cpu *CPU) LDM(op uint32) int {

}

//TODO
func (cpu *CPU) LDR(op uint32) int {
	var offset uint32
	var proc func(uint32, uint32) (uint32, bool)
	var postIndex bool
	dstReg := bits.GetBits(op, 16, 19)
	baseReg := (*cpu.regs[bits.GetBits(op, 12, 15)])
	if bits.GetBit(op, 6) == 0 {
		offset = bits.GetBits(op, 20, 31)
	} else {
		offset, _, _, _ = cpu.shift(op)
	}
	if bits.GetBit(op, 8) == 0 {
		proc = bits.Sub
	} else {
		proc = bits.Add
	}
	if bits.GetBit(op, 7) == 0 {
		baseReg, _ = proc(baseReg, offset)
	} else {
		postIndex = true
	}

	//0 = word
	//1 = byte
	if bits.GetBit(op, 9) == 0 {
		(*cpu.regs[dstReg]) = cpu.bus.Read(baseReg)
	} else {
		(*cpu.regs[dstReg]) = uint32(cpu.bus.ReadByte(baseReg))
	}
	//Write Back / Post-index
	if bits.GetBit(op, 10) == 1 || postIndex {
		(*cpu.regs[baseReg]), _ = proc(baseReg, offset)
	}
	if dstReg == 15 {
		return cpu.CalcCycle(2, 2, 1, 0)
	}
	return cpu.CalcCycle(1, 1, 1, 0)
}

func (cpu *CPU) MCR(op uint32) int {

}

func (cpu *CPU) MLA(op uint32) int {
	op1, op2, acc := cpu.getMultiplyOperands(op)
	_, res := bits.Mul(op1, op2)
	res, _ = bits.Add(res, acc)
	if bits.GetBit(op, 11) == 1 {
		cpu.setMultiplyFlag(res)
	}
	cycles := 0
	top24 := bits.GetBits(op, 0, 24)
	top16 := bits.GetBits(op, 0, 16)
	top8 := bits.GetBits(op, 0, 8)
	if top8 == 0 {
		if top16 == 0 {
			if top24 == 0 {
				cycles = 1
			} else {
				cycles = 2
			}
		} else {
			cycles = 3
		}
	} else if top8 == 0b11111111 {
		if top16 == 0b1111111111111111 {
			if top24 == 0b111111111111111111111111 {
				cycles = 1
			} else {
				cycles = 2
			}
		} else {
			cycles = 3
		}
	} else {
		cycles = 4
	}
	//TODO add cycles
	return cpu.CalcCycle(1, 0, cycles+1, 0)
}

func (cpu *CPU) MLAL(op uint32) int {
	var hi, lo uint32
	var carry bool
	cycles := 0
	regHi := bits.GetBits(op, 12, 15)
	regLo := bits.GetBits(op, 16, 19)
	if bits.GetBit(op, 9) == 0 {
		hi, lo = bits.Mul((*cpu.regs[bits.GetBits(op, 28, 31)]), (*cpu.regs[bits.GetBits(op, 20, 23)]))
		lo, carry = bits.Add(lo, (*cpu.regs[regLo]))
		if carry {
			hi, _ = bits.AddCarry(hi, (*cpu.regs[regHi]), 1)
		} else {
			hi, _ = bits.Add(hi, (*cpu.regs[regHi]))
		}

	} else {
		hi, lo = bits.MulSign((*cpu.regs[bits.GetBits(op, 28, 31)]), (*cpu.regs[bits.GetBits(op, 20, 23)]))
		lo, carry = bits.Add(lo, (*cpu.regs[regLo]))
		if carry {
			hi, _ = bits.AddCarry(hi, (*cpu.regs[regHi]), 1)
		} else {
			hi, _ = bits.Add(hi, (*cpu.regs[regHi]))
		}
		top24 := bits.GetBits(op, 0, 24)
		top16 := bits.GetBits(op, 0, 16)
		top8 := bits.GetBits(op, 0, 8)
		if top8 == 0 {
			if top16 == 0 {
				if top24 == 0 {
					cycles = 1
				} else {
					cycles = 2
				}
			} else {
				cycles = 3
			}
		} else {
			cycles = 4
		}
	}
	(*cpu.regs[bits.GetBits(op, 12, 15)]) = hi
	(*cpu.regs[regLo]) = lo
	if bits.GetBit(op, 11) == 1 {
		cpu.setMultiplyLongFlag(hi, lo)
	}
	return cpu.CalcCycle(1, 0, cycles+2, 0)
}

func (cpu *CPU) MOV(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	_, op2, carry := cpu.getDataProcessingOperands(op, &cycles)
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		(*cpu.regs[reg]) = op2
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		(*cpu.regs[reg]) = op2
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(op2, carry)
		}
	}
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) MRC(op uint32) int {

}

//transfer PSR contents to register
func (cpu *CPU) MRS(op uint32) int {
	if bits.GetBit(op, 9) == 0 {
		(*cpu.regs[bits.GetBits(op, 16, 19)]) = cpu.cpsr
	} else {
		(*cpu.regs[bits.GetBits(op, 16, 19)]) = *cpu.getModeSpsr()
	}
	return cpu.CalcCycle(1, 0, 0, 0)
}

//transfer register contents to PSR
func (cpu *CPU) MSR(op uint32) int {
	//checks whether bits only or not
	if bits.GetBit(op, 15) == 0 {
		if bits.GetBit(op, 9) == 0 {
			cpu.cpsr = (*cpu.regs[bits.GetBits(op, 28, 31)])
		} else {
			spsr := cpu.getModeSpsr()
			(*spsr) = (*cpu.regs[bits.GetBits(op, 28, 31)])
		}
	} else {
		var src uint32
		if bits.GetBit(op, 6) == 0 {
			src = (*cpu.regs[bits.GetBits(op, 28, 31)])
		} else {
			src, _ = cpu.rotate(op)
		}
		if bits.GetBit(src, 0) == 1 {
			cpu.setSign()
		} else {
			cpu.clearSign()
		}
		if bits.GetBit(src, 1) == 1 {
			cpu.setZero()
		} else {
			cpu.clearZero()
		}
		if bits.GetBit(src, 2) == 1 {
			cpu.setCarry()
		} else {
			cpu.clearCarry()
		}
		if bits.GetBit(src, 3) == 1 {
			cpu.setOverflow()
		} else {
			cpu.clearOverflow()
		}
	}
	return cpu.CalcCycle(1, 0, 0, 0)
}

func (cpu *CPU) MUL(op uint32) int {
	op1, op2, _ := cpu.getMultiplyOperands(op)
	_, res := bits.Mul(op1, op2)
	if bits.GetBit(op, 11) == 1 {
		cpu.setMultiplyFlag(res)
	}
	cycles := 0
	top24 := bits.GetBits(op, 0, 24)
	top16 := bits.GetBits(op, 0, 16)
	top8 := bits.GetBits(op, 0, 8)
	if top8 == 0 {
		if top16 == 0 {
			if top24 == 0 {
				cycles = 1
			} else {
				cycles = 2
			}
		} else {
			cycles = 3
		}
	} else if top8 == 0b11111111 {
		if top16 == 0b1111111111111111 {
			if top24 == 0b111111111111111111111111 {
				cycles = 1
			} else {
				cycles = 2
			}
		} else {
			cycles = 3
		}
	} else {
		cycles = 4
	}
	//TODO add cycles
	return cpu.CalcCycle(1, 0, cycles, 0)
}

func (cpu *CPU) MULL(op uint32) int {
	var hi, lo uint32
	cycles := 0
	top24 := bits.GetBits(op, 0, 24)
	top16 := bits.GetBits(op, 0, 16)
	top8 := bits.GetBits(op, 0, 8)
	if bits.GetBit(op, 9) == 0 {
		hi, lo = bits.Mul((*cpu.regs[bits.GetBits(op, 28, 31)]), (*cpu.regs[bits.GetBits(op, 20, 23)]))
		if top8 == 0 {
			if top16 == 0 {
				if top24 == 0 {
					cycles = 1
				} else {
					cycles = 2
				}
			} else {
				cycles = 3
			}
		} else {
			cycles = 4
		}
	} else {
		hi, lo = bits.MulSign((*cpu.regs[bits.GetBits(op, 28, 31)]), (*cpu.regs[bits.GetBits(op, 20, 23)]))
		if top8 == 0 {
			if top16 == 0 {
				if top24 == 0 {
					cycles = 1
				} else {
					cycles = 2
				}
			} else {
				cycles = 3
			}
		} else if top8 == 0b11111111 {
			if top16 == 0b1111111111111111 {
				if top24 == 0b111111111111111111111111 {
					cycles = 1
				} else {
					cycles = 2
				}
			} else {
				cycles = 3
			}
		} else {
			cycles = 4
		}

	}
	(*cpu.regs[bits.GetBits(op, 12, 15)]) = hi
	(*cpu.regs[bits.GetBits(op, 16, 19)]) = lo
	if bits.GetBit(op, 11) == 1 {
		cpu.setMultiplyLongFlag(hi, lo)
	}
	return cpu.CalcCycle(1, 0, cycles+1, 0)
}

func (cpu *CPU) MVN(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	_, op2, carry := cpu.getDataProcessingOperands(op, &cycles)
	res := ^op2
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, carry)
		}
	}
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) ORR(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, carry := cpu.getDataProcessingOperands(op, &cycles)
	res := op1 | op2
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, carry)
		}
	}
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) RSB(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, _ := cpu.getDataProcessingOperands(op, &cycles)
	res, bor := bits.Sub(op2, op1)
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, bor)
			if bor {
				cpu.setOverflow()
			} else {
				cpu.clearOverflow()
			}
		}
	}
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) RSC(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, _ := cpu.getDataProcessingOperands(op, &cycles)
	var res uint32
	var bor bool
	if !cpu.carryIsSet() {
		res, bor = bits.SubCarry(op2, op1, 1)
	} else {
		res, bor = bits.Sub(op2, op1)
	}
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, bor)
			if bor {
				cpu.setOverflow()
			} else {
				cpu.clearOverflow()
			}
		}
	}
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) SBC(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, _ := cpu.getDataProcessingOperands(op, &cycles)
	var res uint32
	var bor bool
	if !cpu.carryIsSet() {
		res, bor = bits.SubCarry(op1, op2, 1)
	} else {
		res, bor = bits.Sub(op1, op2)
	}
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, bor)
			if bor {
				cpu.setOverflow()
			} else {
				cpu.clearOverflow()
			}
		}
	}
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) STC(op uint32) int {

}

func (cpu *CPU) STM(op uint32) int {

}

func (cpu *CPU) STR(op uint32) int {

}

func (cpu *CPU) SUB(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, _ := cpu.getDataProcessingOperands(op, &cycles)
	res, bor := bits.Sub(op1, op2)
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		(*cpu.regs[reg]) = res
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, bor)
			if bor {
				cpu.setOverflow()
			} else {
				cpu.clearOverflow()
			}
		}
	}
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) SWI(op uint32) int {

}

func (cpu *CPU) SWP(op uint32) int {

}

func (cpu *CPU) TEQ(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, carry := cpu.getDataProcessingOperands(op, &cycles)
	res := op1 ^ op2
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, carry)
		}
	}
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}

func (cpu *CPU) TST(op uint32) int {
	cycles := []int{1, 0, 0, 0}
	op1, op2, carry := cpu.getDataProcessingOperands(op, &cycles)
	res := op1 & op2
	reg := bits.GetBits(op, 16, 19)
	if reg == 15 {
		if bits.GetBit(op, 11) == 1 {
			cpu.cpsr = *cpu.getModeSpsr()
		}
		cycles[0] += 1
		cycles[1] += 1
	} else {
		if bits.GetBit(op, 11) == 1 {
			cpu.dataProcessFlagSet(res, carry)
		}
	}
	return cpu.CalcCycle(cycles[0], cycles[1], cycles[2], cycles[3])
}
