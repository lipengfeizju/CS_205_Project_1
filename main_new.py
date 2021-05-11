import numpy as np
import heapq
import copy 
import timeit
from utils import expand_node, hash_state, Tree

def general_search(init_state, method):
    assert method in {0,1,2}
    init_node = Tree(init_state,method)
    node_heap = []
    explored_states = set()
    heapq.heappush(node_heap, init_node)
    explored_states.add(hash_state(init_state))
    i = 0
    max_queue_len = 0
    while(1):

        if len(node_heap) > max_queue_len: max_queue_len = len(node_heap)
        
        if len(node_heap) == 0:
            print("FAILURE!")
            return -1
        min_node = heapq.heappop(node_heap)
        print("g(n) = {} , h(n) = {}, f(n) = {}".format(min_node.g_n, min_node.h_n, min_node.g_n + min_node.h_n))
        if min_node.h_n == 0:
            print(i)
            print("FOUND!")
            action_list = []
            state_list = []
            while min_node.parent != None:
                action_list.append(min_node.action)
                state_list.append(min_node.state)
                min_node = min_node.parent
            state_list.append(min_node.state)
            action_list.reverse()
            state_list.reverse()
            return action_list, state_list

        node_list = expand_node(min_node)
        for node in node_list:
            node_hash = hash_state(node.state)
            if node_hash not in explored_states:
                explored_states.add(node_hash)
                heapq.heappush(node_heap, node)

        i += 1
    
if __name__ == "__main__":
    # init_state = np.array([1,3,6,5,0,2,4,8,7]).reshape([3,3])
    
    start = timeit.default_timer()

    # init_state = np.array([1,6,7,5,0,3,4,8,2]).reshape([3,3]) # 34

    # init_state = np.array([0,7,2,4,6,1,3,5,8]).reshape([3,3]) #24
    # init_state = np.array([7,1,2,4,8,5,6,3,0]).reshape([3,3]) # 20
    init_state = np.array([1,6,7,5,0,3,4,8,2]).reshape([3,3]) # 16
    # init_state = np.array([1,3,6,5,0,7,4,8,2]).reshape([3,3]) # 12
    # init_state = np.array([1,3,6,5,0,2,4,7,8]).reshape([3,3]) # 8
    # init_state = np.array([1,2,3,5,0,6,4,7,8]).reshape([3,3]) # 4
    # init_state = np.array([1,2,3,4,5,6,0,7,8]).reshape([3,3]) # 2

    # print(hash_state(init_state))
    action_res, state_res  = general_search(init_state,2)
    print(action_res)

    stop = timeit.default_timer()
    print('Time: ', stop - start) 

    # init_state = np.array([1,3,6,5,0,2,4,8,7]).reshape([3,3])
    
    # print(misplaced_tile_dist(init_state))
    # print(manhattan_dist(init_state))
