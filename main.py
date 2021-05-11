import numpy as np
from search import general_search

def check_puzzle_array(puzzle_array):
    if puzzle_array.shape[0] != 9:
        print("Please input exactly 9 numbers, your input contains only {} numbers".format(puzzle_array.shape[0]))
        return -1
    if any(puzzle_array>8):
        print("Please don't input any number larger than 8")
        return -1
    if any(puzzle_array<0):
        print("Please don't input any number smaller than 0")
        return -1
    if len(set(puzzle_array)) <9:
        print("Your input contains repeated numbers")
        return -1
    return 1

if __name__ == "__main__":

    print("Welcome! This program can be used to find the solution for 8 puzzles.Please read the instruction below carefully to save more time trying around")
    print("First step: please enter your 8 Puzzle initial state row by row, sperate the numbers by space, input zero as the place holder")

    for i in range(5):
        print("Example input:  1 2 3 4 5 6 0 7 8 ")
        print("INPUT YOUR 8 PUZZLES")
        x = input().split(' ')
        try:
            puzzle_array = np.array(x).astype(int)
        except ValueError:
            print("The input is not recognized, please only input numbers. \n(The error may be caused by typing in characters (e.g. ,) by accident)")
        else:
            if check_puzzle_array(puzzle_array) < 0:
                continue
            
            puzzle_array = puzzle_array.reshape(3,3)
            print("The initial state for 8 puzzle you just typed in is:")
            print(puzzle_array)

            print("Select algorithm now, please type a integer, 0 or 1 or 2 \n(0) Uniform Cost Research \n(1) A* with Misplaced Tile heuristic \n(2) A* with the Manhattan Distance heuristic.")
            x = input()
            try:
                method_i = int(x)
            except ValueError:
                print("Please only input 0 or 1 or 2")
            print("-------  Start Searching  -------")
            action_res, state_res  = general_search(puzzle_array, method_i)

            print("-------- Finshed Search   ------")
            action_list = ["up", "down", "left","right"]
            action_print = [action_list[a] for a in action_res]
            print("You should do the following")
            print(action_print)
            print("In toal you will need {} steps".format(len(action_print)))
            exit()

    print("Reach the maximum attempts, please restart the program")

