package datastruct

//Binary tree for op tree

type TreeNode struct {
	Op       int
	Pos      int
	Children []*TreeNode
}

type OpTree struct {
	Root *TreeNode
}
