# CS_205_Project_1

New implementation ahcieves about 4x speed up for Depth=24 solution, in naive implementation, the node stores all the previous states up to now, it is time consuming to store and compare. In this new implementation, each node only stores the current state and the most recent action, and it also connects to its parent. When you want to know the whole chain up to now, you can traverse the tree up to the root.
