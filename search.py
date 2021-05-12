import heapq
import timeit
import numpy as np

from utils import expand_node, hash_state, Tree
from utils import misplaced_tile_dist

def general_search(init_state, method, verbose = False):
    assert method in {0,1,2}
    
    # The queue for nodes is implemented by queue
    node_heap = []
    # Track the explored states to avoid repeated states
    explored_states = set()
    # Push the initial node to the queue 
    init_node = Tree(init_state,method)
    heapq.heappush(node_heap, init_node)
    explored_states.add(hash_state(init_state))
    # Save the number of expanded nodes and maximum length of queue
    i = 0
    max_queue_len = 0
    while(1):
        if len(node_heap) > max_queue_len: max_queue_len = len(node_heap)
        
        if len(node_heap) == 0:
            print("FAILURE!")
            return -1
        # Pop up the node with mimimum cost
        min_node = heapq.heappop(node_heap)
        # Print out details about this node
        if verbose: print("g(n) = {} , h(n) = {}, f(n) = {}".format(min_node.g_n, min_node.h_n, min_node.g_n + min_node.h_n))

        # Judge if all numbers are in the correct position
        if misplaced_tile_dist(min_node.state) == 0:
            print("We expanded {} nodes, the maximum length of queue is {}".format(i, max_queue_len))
            print("SOLUTION FOUND!")
            # Track all the actions and history states taken so far 
            action_list = []
            state_list = []
            while min_node.parent != None:
                action_list.append(min_node.action)
                state_list.append(min_node.state)
                min_node = min_node.parent
            state_list.append(min_node.state)
            # This visit is from goal to initial state, we need to reverse it
            action_list.reverse()
            state_list.reverse()
            return action_list, state_list

        # Find all the possible next movements based on current position
        node_list = expand_node(min_node)
        for node in node_list:
            # For each move, we check if it is already visited before
            node_hash = hash_state(node.state)
            if node_hash not in explored_states:
                # If this is a new state, add the node to queue and update
                # explored states
                explored_states.add(node_hash)
                heapq.heappush(node_heap, node)
        i += 1

def demo():      
    # init_state = np.array([1,3,6,5,0,2,4,8,7]).reshape([3,3])
    
    start = timeit.default_timer()

    # init_state = np.array([0,7,2,4,6,1,3,5,8]).reshape([3,3]) # 24
    # init_state = np.array([7,1,2,4,8,5,6,3,0]).reshape([3,3]) # 20
    init_state = np.array([1,6,7,5,0,3,4,8,2]).reshape([3,3]) # 16
    # init_state = np.array([1,3,6,5,0,7,4,8,2]).reshape([3,3]) # 12
    # init_state = np.array([1,3,6,5,0,2,4,7,8]).reshape([3,3]) # 8
    # init_state = np.array([1,2,3,5,0,6,4,7,8]).reshape([3,3]) # 4
    # init_state = np.array([1,2,3,4,5,6,0,7,8]).reshape([3,3]) # 2
    # init_state = np.array([1,2,3,4,5,6,7,8,0]).reshape([3,3]) # 0
    # print(hash_state(init_state))

    # action_res, state_res  = general_search(init_state,1,verbose=True)
    print("The initial state for 8 puzzle is")
    print(init_state)
    
    action_list = ["up", "down", "left","right"]
    
    print("\n------  Start Uniform Cost Search  ------")
    action_res, state_res  = general_search(init_state,0)
    action_print = [action_list[a] for a in action_res]
    print("You should do the following")
    print(action_print)
    

    print("\n------  Start A* with Misplaced Tile Heuristic  ------")
    action_res, state_res  = general_search(init_state,1)
    action_print = [action_list[a] for a in action_res]
    print("You should do the following")
    print(action_print)

    print("\n------  Start A* with Mandattan Distance Heuristic  ------")
    action_res, state_res  = general_search(init_state,2)
    action_print = [action_list[a] for a in action_res]
    print("You should do the following")
    print(action_print)


if __name__ == "__main__":
    demo()