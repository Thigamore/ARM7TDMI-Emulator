package arm7tdmiemu

type BUS struct {
	signals []*bool
}

func (bus *BUS) Write(addr uint32, toWrite uint32) {

}

func (bus *BUS) WriteByte(addr uint32, toWrite byte) {

}

func (bus *BUS) Read(addr uint32) uint32 {

}

func (bus *BUS) ReadByte(addr uint32) byte {

}
