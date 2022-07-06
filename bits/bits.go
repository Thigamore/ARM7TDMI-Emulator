package bits

import "math/bits"

//From 0 on MSB to 31 on LSB
//Gets a bit
func GetBit(num uint32, bitNum int) uint32 {
	return ((num << bitNum) >> bitNum) >> (31 - bitNum)
}

//From 0 on MSB to 31 on LSB
//Gets a series of bits
func GetBits(num uint32, firstBit int, lastBit int) uint32 {
	return ((num << firstBit) >> firstBit) >> (31 - lastBit)
}

//Rotates right
func RotateRight(num uint32, rotateBy int) uint32 {
	return (num >> rotateBy) | (num << (32 - rotateBy))
}

//Checks to see if the uint32 is negative through 2's Complement
func IsNegative(num uint32) bool {
	return num >= 0b10000000000000000000000000000000
}

//Adds 2 two's complement uint32 return true if overflow
func Add(num1 uint32, num2 uint32) (uint32, bool) {
	sum, carry := bits.Add32(num1, num2, 0)
	return sum, carry == 1
}

//Adds 2 two's complement uint32 return true if overflow with carry
func AddCarry(num1 uint32, num2 uint32, carry uint32) (uint32, bool) {
	sum, carry := bits.Add32(num1, num2, carry)
	return sum, carry == 1
}

//Add two uint32 numbers
func Sub(num1 uint32, num2 uint32) (uint32, bool) {
	diff, bor := bits.Sub32(num1, num2, 0)
	return diff, bor == 1
}

//Add two uint32 numbers with carry
func SubCarry(num1 uint32, num2 uint32, carry uint32) (uint32, bool) {
	diff, bor := bits.Sub32(num1, num2, carry)
	return diff, bor == 1
}

//Multiples two uint32
//hi, lo
func Mul(num1 uint32, num2 uint32) (uint32, uint32) {
	hi, lo := bits.Mul32(num1, num2)
	return hi, lo
}

//Multiplies signed numbers
//hi, lo
func MulSign(num1 uint32, num2 uint32) (uint32, uint32) {
	hi, lo := bits.Mul32(num1, num2)
	if GetBit(num1, 0) == 1 {
		if GetBit(num1, 0) == 1 {
			return hi, lo
		} else {
			return TwoComplement(hi, lo)
		}
	}
	if GetBit(num1, 0) == 1 {
		return TwoComplement(hi, lo)
	}
	return hi, lo
}

//Converts a 64 uint, made up of two uint32, to two's complement
func TwoComplement(hi uint32, lo uint32) (uint32, uint32) {
	num := (uint64(hi) << 32) | uint64(lo)
	return uint32(num >> 32), uint32(num << 32 >> 32)
}

//Sign extends a 26bit uint32
//For Branch instruction
func SignExtend(num uint32) uint32 {
	if GetBit(num, 6) == 1 {
		return (uint32(0b111111) << 26) | num
	} else {
		return (uint32(0b000000) << 26) | num
	}
}
