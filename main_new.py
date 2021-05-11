import numpy as np
import heapq
import copy 

GOAL_STATE = [1,2,3,4,5,6,7,8,0]

def misplaced_tile_dist(current_state):
    # Flatten the current state to a vector size of 9
    # Substract the goal state from the current state
    dist_array = current_state.flatten() - GOAL_STATE
    # In the result, the position where not equal to zero
    # means this tile is misplaced 
    misplaced_num = np.where(dist_array != 0)[0].shape[0]

    # If the position of 0 (place holder) is not correct, we minus 1
    # Because we don't want to count the place holder's misplacement
    if current_state[2,2] != 0:
        misplaced_num -= 1

    return misplaced_num

def manhattan_dist(current_state):

    # Similar to the misplaced_tile_dist, we fist calculate the dist_array
    dist_array = current_state.flatten() - GOAL_STATE

    # Decode the dist_array to dist_x and dist_y, for instance, now 6
    # is in position 2, 6-2 = 4, floor(4/3) = 1, mod(4,3) = 1, then we 
    # know 6 should be in the next column and next row, the total distance
    # is 1+1 = 2
    dist_x = np.floor(abs(dist_array)/3)
    dist_y = np.remainder(abs(dist_array),3)
    dist_sum = dist_x + dist_y

    # Find the position of 0 and set the distance sum to zero, since we 
    # don't want to count 0 
    index_0 = np.where(current_state == 0) 
    dist_sum[index_0[0][0] *3 + index_0[1][0]] = 0
    
    return int(np.sum(dist_sum))
    
def find_next_state(current_state):

    next_state_list = []
    # First, find the position of the zero (place holder)
    index = np.where(current_state == 0)
    ind_x = index[0][0]
    ind_y = index[1][0]
    # Move up the number below zero
    if ind_x != 2:
        next_state = np.copy(current_state)
        next_state[ind_x, ind_y] = next_state[ind_x+1, ind_y]
        next_state[ind_x+1, ind_y] = 0
        next_state_list.append((0,next_state))

    # Move down the number above zero
    if ind_x !=0 :
        next_state = np.copy(current_state)
        next_state[ind_x, ind_y] = next_state[ind_x-1, ind_y]
        next_state[ind_x-1, ind_y] = 0
        next_state_list.append((1,next_state))
    
    # Move left the number on the right side of zero
    if ind_y != 2:
        next_state = np.copy(current_state)
        next_state[ind_x, ind_y] = next_state[ind_x, ind_y+1]
        next_state[ind_x, ind_y+1] = 0
        next_state_list.append((2,next_state))
    # Move right the number on the left side of zero
    if ind_y != 0:
        next_state = np.copy(current_state)
        next_state[ind_x, ind_y] = next_state[ind_x, ind_y-1]
        next_state[ind_x, ind_y-1] = 0
        next_state_list.append((3,next_state))
    
    return next_state_list

def expand_node(node):

    node_list = []
    # Take the most recent state from the current node
    curret_state = node.state
    dist_metric = node.dist_metric
    # Find valid moves from the current state 
    next_state_list = find_next_state(curret_state)
    
    for next_state_tuple in next_state_list:
        next_action = next_state_tuple[0]
        next_state = next_state_tuple[1]
        
        new_node = Tree(next_state,dist_metric)        
        new_node.connect_parent(node, next_action)
        node_list.append(new_node)
    
    return node_list

def hash_state(current_state):
    # As we all know, another key issue is how to remove redundant states
    # My solution is hash the current state to a value, each state will 
    # have a unique value
    hash_value = current_state.flatten() * np.array([1e8,1e7,1e6,1e5,1e4,1e3,1e2,10,1])
    hash_value = int(np.sum(hash_value))
    return hash_value

class Tree:
    def __init__(self, state, dist_metric):
        # Tree information
        self.parent = None

        # Action, State, value information
        self.dist_metric = dist_metric
        self.action = None
        self.state = state
        self.g_n = 0
        self.h_n = 0
        self.update_hn(state)

    def connect_parent(self, node_parent, action):
        self.action = action
        self.g_n = node_parent.g_n + 1
        self.parent = node_parent

    def update_hn(self, next_state):
        # Calculate the hueristic function based on the node type
        # 0 for uniform cost search
        if self.dist_metric == 0:
            self.h_n = 0
        # 1 for A* with misplaced tile heuristic
        elif self.dist_metric == 1:
            self.h_n = misplaced_tile_dist(next_state)
            # raise NotImplementedError
        # 2 for A* with Manhattan Distance heuristic
        elif self.dist_metric == 2:
            self.h_n = manhattan_dist(next_state)
        else:
            print("Please not hack into the program")
            raise NotImplementedError

    def __lt__(self, other):
        return self.g_n+self.h_n < other.g_n + other.h_n
    
# init_state = np.array([1,2,3,4,0,5,6,7,8]).reshape([3,3])
# goal_state = np.array(GOAL_STATE).reshape([3,3])

# print(misplaced_tile_dist(init_state))

def general_search(init_state, method):
    assert method in {0,1,2}
    init_node = Tree(init_state,method)
    node_heap = []
    explored_states = set()
    heapq.heappush(node_heap, init_node)
    explored_states.add(hash_state(init_state))
    i = 0
    while(1):
        if len(node_heap) == 0:
            print("FAILURE!")
            return -1
        min_node = heapq.heappop(node_heap)
        if misplaced_tile_dist(min_node.state) == 0:
            print(i)
            print("FOUND!")
            action_list = []
            while min_node.parent != None:
                action_list.append(min_node.action)
                min_node = min_node.parent
            action_list.reverse()
            return action_list

        node_list = expand_node(min_node)
        for node in node_list:
            node_hash = hash_state(node.state)
            if node_hash not in explored_states:
                explored_states.add(node_hash)
                heapq.heappush(node_heap, node)

        # print("{} + {} = {}".format(min_node.g_n, min_node.h_n,
        # min_node.g_n + min_node.h_n))
        i += 1
    
if __name__ == "__main__":
    # init_state = np.array([1,3,6,5,0,2,4,8,7]).reshape([3,3])
    # init_state = np.array([0,7,2,4,6,1,3,5,8]).reshape([3,3]) #24
    # init_state = np.array([7,1,2,4,8,5,6,3,0]).reshape([3,3]) # 20
    init_state = np.array([1,6,7,5,0,3,4,8,2]).reshape([3,3]) # 16
    # init_state = np.array([1,3,6,5,0,7,4,8,2]).reshape([3,3]) # 12
    # init_state = np.array([1,3,6,5,0,2,4,7,8]).reshape([3,3]) # 8
    # init_state = np.array([1,2,3,5,0,6,4,7,8]).reshape([3,3]) # 4
    # init_state = np.array([1,2,3,4,5,6,0,7,8]).reshape([3,3]) # 2

    # print(hash_state(init_state))
    action_res = general_search(init_state,2)
    print(action_res)
    # init_state = np.array([1,3,6,5,0,2,4,8,7]).reshape([3,3])
    
    # print(misplaced_tile_dist(init_state))
    # print(manhattan_dist(init_state))
