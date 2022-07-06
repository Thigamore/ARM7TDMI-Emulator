package datastruct

//A stack
type stackNode[T any] struct {
	val   T
	below *stackNode[T]
}

type stack[T any] struct {
	top *stackNode[T]
}

func CreateStack[T any]() stack[T] {
	return stack[T]{}
}

func (stack stack[T]) Push(val T) {
	node := &stackNode[T]{
		val:   val,
		below: stack.top,
	}
	stack.top = node
}

func (stack stack[T]) Pop() T {
	val := stack.top.val
	stack.top = stack.top.below
	return val
}
