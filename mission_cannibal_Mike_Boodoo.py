import heapq

# Michael Boodoo
# Artificial Intelligence Assign 01

# DFS and A* algorithms acting on Node class who's state is represented as a dictionary
# DFS's fringe/frontier is a list and A* is a min-priority queue, using the heapq module
# Since dictionaries aren't hashable, the closed/reached set contains string representation of state, str(Node.state)
# Node class has global Node class counter that is cleared after DFS or A* are run
# Main work happens in the Node.perform_action method that takes an action from the action tuple and ensures it is valid
# New node is only generated if resulting new state is valid and satifies constraints of the problem

# Output and discussion at end

class Node:

    node_count = 0

    def __init__(self, state:dict, parent, prev_action, depth:int, total_cost:int):
        self.state = state
        self.parent:Node = parent
        self.prev_action:tuple = prev_action
        self.depth = depth
        Node.node_count += 1

    def A_star_cost(self, depth): # f(n) used in A*
        # need total number on left for both side heuristics
        num_left = self.state[ML] + self.state[CL]
        if self.state["position"] == left and num_left < 3:
            return 1 + depth
        elif self.state["position"] == left and num_left > 2:
            return ((num_left-2)*2)+1+depth
        elif self.state["position"] == right:
            return (2*num_left) + depth

    def print_path(self, flag:bool):
        num_miss:int
        num_cann:int
        direction = ""
        
        while self.parent is not None:
            for miss, cann in self.prev_action.items():
                num_miss, num_cann = miss, cann
            if (self.state["position"] == left):
                direction = right
            else:
                direction = left
            if flag != True:
                print("DFS SOLUTION NODE -", str(self.state), "at depth: ",self.depth, "by bringing", num_miss, "miss and", num_cann, " cann from the", direction)
            else: 
                print("A* SOLUTION NODE -", str(self.state), "at depth: ",self.depth, "by bringing", num_miss, "miss and", num_cann, " cann from the", direction, "f(n) =", self.total_cost)
            self = self.parent
        print("INITIAL NODE -", str(self.state), "at depth: ",self.depth)

    def perform_action(self, action:dict, fringe:list, reached:set):

        num_miss:int
        num_cann:int
        for miss, cann in action.items():
            num_miss, num_cann = miss, cann

        # check if action is actually possible (enough miss or cann on that side)
        # for initial state, {3,3,L,0,0} gave me no missed actions as expected
        if self.state["position"] == left: # all actions will be applied to left -> right
            if num_miss > self.state[ML]: 
                #print("Action not performed, not enough miss to move, num_miss: ", num_miss)
                return # ensure there are enough miss and cann to transfer
            if num_cann > self.state[CL]: 
                #print("Action not performed, not enough cann to move, num_cann: ", num_cann)
                return

            # now ensure that the state is legal (cann don't outnumber miss)
            next_miss_left, next_miss_right = self.state[ML] - num_miss, self.state[MR] + num_miss
            next_cann_left, next_cann_right = self.state[CL] - num_cann, self.state[CR] + num_cann

            if next_cann_left > next_miss_left and next_miss_left > 0:
                #print("Illegal move detected, cannibals:", next_cann_left, " outnumber missionaries:", next_miss_left, "on left!")
                #print("The attemped move was sending", num_cann, "cannibals and", num_miss, "missionaries to the right")
                return
            if next_cann_right > next_miss_right and next_miss_right > 0:
                #print("Illegal move detected, cannibals:", next_cann_right, " outnumber missionaries:", next_miss_right, "on right!")
                #print("The attemped move was sending", num_cann, "cannibals and", num_miss, " missionaries to the right")
                return

            # legal nodes can now be added to frontier/fringe
            # create the state of the new node
            new_state:dict = {ML:next_miss_left, CL:next_cann_left, "position":right, MR:next_miss_right, CR:next_cann_right}
            state_string = str(new_state)
            if state_string not in reached:
                new_node:Node = Node(new_state, self, action, self.depth+1, None)
                new_node.total_cost = new_node.A_star_cost(self.depth)
                print("New unique node created with state:", new_state, "with depth:", new_node.depth)
                fringe.append(new_node)
                reached.add(state_string)
                #print("state string added to reached with value:", state_string)

        if self.state["position"] == right: # all actions will be applied to right -> left
            if num_miss > self.state[MR]: 
                #print("Action not performed, not enough miss to move, num_miss: ", num_miss)
                return # ensure there are enough miss and cann to transfer
            if num_cann > self.state[CR]: 
                #print("Action not performed, not enough cann to move, num_cann: ", num_cann)
                return

            # now ensure that the state is legal (cann don't outnumber miss)
            next_miss_left, next_miss_right = self.state[ML] + num_miss, self.state[MR] - num_miss
            next_cann_left, next_cann_right = self.state[CL] + num_cann, self.state[CR] - num_cann

            if next_cann_left > next_miss_left and next_miss_left > 0:
                #print("Illegal move detected, cannibals:", next_cann_left, " outnumber missionaries:", next_miss_left, "on left!")
                #print("The attemped move was sending", num_cann, "cannibals and", num_miss, "missionaries to the right")
                return
            if next_cann_right > next_miss_right and next_miss_right > 0:
                #print("Illegal move detected, cannibals:", next_cann_right, " outnumber missionaries:", next_miss_right, "on right!")
                #print("The attemped move was sending", num_cann, "cannibals and", num_miss, " missionaries to the right")
                return

            # legal nodes can now be added to frontier/fringe
            # create the state of the new node
            new_state:dict = {ML:next_miss_left, CL:next_cann_left, "position":left, MR:next_miss_right, CR:next_cann_right}
            state_string = str(new_state)
            if state_string not in reached:
                new_node:Node = Node(new_state, self, action, self.depth+1, None)
                print("New unique node created with state:", new_state, "with depth:", new_node.depth)
                fringe.append(new_node)
                reached.add(state_string)
                #print("state string added to reached with value:", state_string)
    
            
    # A_star action
    def perform_action_a_star(self, action:dict, priority_queue, reached:set):

        num_miss:int
        num_cann:int
        for miss, cann in action.items():
            num_miss, num_cann = miss, cann

        # check if action is actually possible (enough miss or cann on that side)
        # for initial state, {3,3,L,0,0} gave me no missed actions as expected
        if self.state["position"] == left: # all actions will be applied to left -> right
            if num_miss > self.state[ML]: 
                #print("Action not performed, not enough miss to move, num_miss: ", num_miss)
                return # ensure there are enough miss and cann to transfer
            if num_cann > self.state[CL]: 
                #print("Action not performed, not enough cann to move, num_cann: ", num_cann)
                return

            # now ensure that the state is legal (cann don't outnumber miss)
            next_miss_left, next_miss_right = self.state[ML] - num_miss, self.state[MR] + num_miss
            next_cann_left, next_cann_right = self.state[CL] - num_cann, self.state[CR] + num_cann

            if next_cann_left > next_miss_left and next_miss_left > 0:
                #print("Illegal move detected, cannibals:", next_cann_left, " outnumber missionaries:", next_miss_left, "on left!")
                #print("The attemped move was sending", num_cann, "cannibals and", num_miss, "missionaries to the right")
                return
            if next_cann_right > next_miss_right and next_miss_right > 0:
                #print("Illegal move detected, cannibals:", next_cann_right, " outnumber missionaries:", next_miss_right, "on right!")
                #print("The attemped move was sending", num_cann, "cannibals and", num_miss, " missionaries to the right")
                return

            # legal nodes can now be added to frontier/fringe
            # create the state of the new node
            new_state:dict = {ML:next_miss_left, CL:next_cann_left, "position":right, MR:next_miss_right, CR:next_cann_right}
            state_string = str(new_state)
            if state_string not in reached:
                new_node:Node = Node(new_state, self, action, self.depth+1, None)
                new_node.total_cost = new_node.A_star_cost(new_node.depth)
                print("New unique node created with state:", new_state, "with depth:", new_node.depth, "and A* cost:", new_node.total_cost)
                #fringe.append(new_node)
                heapq.heappush(priority_queue, (new_node.total_cost, state_string, new_node))
                reached.add(state_string)
                #print("state string added to reached with value:", state_string)

        if self.state["position"] == right: # all actions will be applied to right -> left
            if num_miss > self.state[MR]: 
                #print("Action not performed, not enough miss to move, num_miss: ", num_miss)
                return # ensure there are enough miss and cann to transfer
            if num_cann > self.state[CR]: 
                #print("Action not performed, not enough cann to move, num_cann: ", num_cann)
                return

            # now ensure that the state is legal (cann don't outnumber miss)
            next_miss_left, next_miss_right = self.state[ML] + num_miss, self.state[MR] - num_miss
            next_cann_left, next_cann_right = self.state[CL] + num_cann, self.state[CR] - num_cann

            if next_cann_left > next_miss_left and next_miss_left > 0:
                #print("Illegal move detected, cannibals:", next_cann_left, " outnumber missionaries:", next_miss_left, "on left!")
                #print("The attemped move was sending", num_cann, "cannibals and", num_miss, "missionaries to the right")
                return
            if next_cann_right > next_miss_right and next_miss_right > 0:
                #print("Illegal move detected, cannibals:", next_cann_right, " outnumber missionaries:", next_miss_right, "on right!")
                #print("The attemped move was sending", num_cann, "cannibals and", num_miss, " missionaries to the right")
                return

            # legal nodes can now be added to frontier/fringe
            # create the state of the new node
            new_state:dict = {ML:next_miss_left, CL:next_cann_left, "position":left, MR:next_miss_right, CR:next_cann_right}
            state_string = str(new_state)
            if state_string not in reached:
                new_node:Node = Node(new_state, self, action, self.depth+1, None)
                new_node.total_cost = new_node.A_star_cost(new_node.depth)
                print("New unique node created with state:", new_state, "with depth:", new_node.depth, "and A* cost:", new_node.total_cost)
                #fringe.append(new_node)
                heapq.heappush(priority_queue, (new_node.total_cost, state_string, new_node))
                reached.add(state_string)
                #print("state string added to reached with value:", state_string)
        


if __name__ == "__main__":

    ML, MR = "miss_left", "miss_right"
    CL, CR = "cann_left", "cann_right"
    left, right = "left", "right"
    frontier:list = []
    actions:tuple = ({2:0}, {0:2}, {1:1}, {1:0}, {0:1}) # tuple of {miss:cann} entries. key of tuple entry is miss, value of tuple entry is cann
    initial_state = {ML:3, CL:3, "position":left, MR:0, CR:0}
    goal_state = {ML:0, CL:0, "position":right, MR:3, CR:3}
    reached:set = set()
    
    Node.node_count = 0
    inital_node = Node(initial_state, None, None, 0, None)

    for action in actions: # each item is a miss:cann dictionary
        a_star_flag = False
        inital_node.perform_action(action, frontier, reached)
        found_solution = False
        while frontier and (found_solution != True):
            current_node:Node = frontier.pop()
            if current_node.state == goal_state:
                print("VALID STATE REACHED at depth:", current_node.depth)
                found_solution = True
                current_node.print_path(a_star_flag)
                print("Number of nodes created:", Node.node_count)
                break
            else:
                for action_2 in actions:
                    current_node.perform_action(action_2, frontier, reached)
    print("End reached...")
    reached.clear()

    # A* version of above which is same except for min_priority_queue operations
    Node.node_count = 0
    reached_a_star:set = set()
    priority_queue = []
    for action in actions: # each item is a miss:cann dictionary
        a_star_flag = True
        inital_node.perform_action_a_star(action, priority_queue, reached_a_star)
        found_solution = False
        while priority_queue and (found_solution != True):
            popped_node_tuple = heapq.heappop(priority_queue)
            current_node:Node = popped_node_tuple[2]
            if current_node.state == goal_state:
                print("VALID STATE REACHED at depth:", current_node.depth)
                found_solution = True
                current_node.print_path(a_star_flag)
                print("Number of nodes created:", Node.node_count)
                break
            else:
                for action_2 in actions:
                    current_node.perform_action_a_star(action_2, priority_queue, reached_a_star)
    print("End reached...")
    reached_a_star.clear()

# Code output and analysis below:

# Without using a reached set, DFS never stopped and generated nodes at depth > 40,000 until manually stopping program
# Using a reached set, DFS found a solution at depth 11 while generating 15 legal nodes, output below:

# Solution nodes output in reverse order (Goal_state -> ... -> Initial_state) along with action previously applied to get to that state
 
"""
VALID STATE REACHED at depth: 11
DFS SOLUTION NODE - {'miss_left': 0, 'cann_left': 0, 'position': 'right', 'miss_right': 3, 'cann_right': 3} at depth:  11 by bringing 0 miss and 2  cann from the left
DFS SOLUTION NODE - {'miss_left': 0, 'cann_left': 2, 'position': 'left', 'miss_right': 3, 'cann_right': 1} at depth:  10 by bringing 0 miss and 1  cann from the right
DFS SOLUTION NODE - {'miss_left': 0, 'cann_left': 1, 'position': 'right', 'miss_right': 3, 'cann_right': 2} at depth:  9 by bringing 0 miss and 2  cann from the left
DFS SOLUTION NODE - {'miss_left': 0, 'cann_left': 3, 'position': 'left', 'miss_right': 3, 'cann_right': 0} at depth:  8 by bringing 0 miss and 1  cann from the right
DFS SOLUTION NODE - {'miss_left': 0, 'cann_left': 2, 'position': 'right', 'miss_right': 3, 'cann_right': 1} at depth:  7 by bringing 2 miss and 0  cann from the left
DFS SOLUTION NODE - {'miss_left': 2, 'cann_left': 2, 'position': 'left', 'miss_right': 1, 'cann_right': 1} at depth:  6 by bringing 1 miss and 1  cann from the right
DFS SOLUTION NODE - {'miss_left': 1, 'cann_left': 1, 'position': 'right', 'miss_right': 2, 'cann_right': 2} at depth:  5 by bringing 2 miss and 0  cann from the left
DFS SOLUTION NODE - {'miss_left': 3, 'cann_left': 1, 'position': 'left', 'miss_right': 0, 'cann_right': 2} at depth:  4 by bringing 0 miss and 1  cann from the right
DFS SOLUTION NODE - {'miss_left': 3, 'cann_left': 0, 'position': 'right', 'miss_right': 0, 'cann_right': 3} at depth:  3 by bringing 0 miss and 2  cann from the left
DFS SOLUTION NODE - {'miss_left': 3, 'cann_left': 2, 'position': 'left', 'miss_right': 0, 'cann_right': 1} at depth:  2 by bringing 0 miss and 1  cann from the right
DFS SOLUTION NODE - {'miss_left': 3, 'cann_left': 1, 'position': 'right', 'miss_right': 0, 'cann_right': 2} at depth:  1 by bringing 0 miss and 2  cann from the left
INITIAL NODE - {'miss_left': 3, 'cann_left': 3, 'position': 'left', 'miss_right': 0, 'cann_right': 0} at depth:  0
"""



# Without using a reached set, A* found a solution at depth 11 while generating 23 legal nodes, output below:
"""
VALID STATE REACHED at depth: 11
A* SOLUTION NODE - {'miss_left': 0, 'cann_left': 0, 'position': 'right', 'miss_right': 3, 'cann_right': 3} at depth:  11 by bringing 0 miss and 2  cann from the left f(n) = 11
A* SOLUTION NODE - {'miss_left': 0, 'cann_left': 2, 'position': 'left', 'miss_right': 3, 'cann_right': 1} at depth:  10 by bringing 0 miss and 1  cann from the right f(n) = 11
A* SOLUTION NODE - {'miss_left': 0, 'cann_left': 1, 'position': 'right', 'miss_right': 3, 'cann_right': 2} at depth:  9 by bringing 0 miss and 2  cann from the left f(n) = 11
A* SOLUTION NODE - {'miss_left': 0, 'cann_left': 3, 'position': 'left', 'miss_right': 3, 'cann_right': 0} at depth:  8 by bringing 0 miss and 1  cann from the right f(n) = 11
A* SOLUTION NODE - {'miss_left': 0, 'cann_left': 2, 'position': 'right', 'miss_right': 3, 'cann_right': 1} at depth:  7 by bringing 2 miss and 0  cann from the left f(n) = 11
A* SOLUTION NODE - {'miss_left': 2, 'cann_left': 2, 'position': 'left', 'miss_right': 1, 'cann_right': 1} at depth:  6 by bringing 1 miss and 1  cann from the right f(n) = 11
A* SOLUTION NODE - {'miss_left': 1, 'cann_left': 1, 'position': 'right', 'miss_right': 2, 'cann_right': 2} at depth:  5 by bringing 2 miss and 0  cann from the left f(n) = 9
A* SOLUTION NODE - {'miss_left': 3, 'cann_left': 1, 'position': 'left', 'miss_right': 0, 'cann_right': 2} at depth:  4 by bringing 0 miss and 1  cann from the right f(n) = 9
A* SOLUTION NODE - {'miss_left': 3, 'cann_left': 0, 'position': 'right', 'miss_right': 0, 'cann_right': 3} at depth:  3 by bringing 0 miss and 2  cann from the left f(n) = 9
A* SOLUTION NODE - {'miss_left': 3, 'cann_left': 2, 'position': 'left', 'miss_right': 0, 'cann_right': 1} at depth:  2 by bringing 0 miss and 1  cann from the right f(n) = 9
A* SOLUTION NODE - {'miss_left': 3, 'cann_left': 1, 'position': 'right', 'miss_right': 0, 'cann_right': 2} at depth:  1 by bringing 0 miss and 2  cann from the left f(n) = 9
INITIAL NODE - {'miss_left': 3, 'cann_left': 3, 'position': 'left', 'miss_right': 0, 'cann_right': 0} at depth:  0
Number of nodes created: 23
-------------------------------------------------------------------------------------------------------------------
"""

# Using a reached set to avoid cycles, A* found a solution at depth 11 while generating 14 nodes, output below:
"""
VALID STATE REACHED at depth: 11
A* SOLUTION NODE - {'miss_left': 0, 'cann_left': 0, 'position': 'right', 'miss_right': 3, 'cann_right': 3} at depth:  11 by bringing 0 miss and 2  cann from the left f(n) = 11
A* SOLUTION NODE - {'miss_left': 0, 'cann_left': 2, 'position': 'left', 'miss_right': 3, 'cann_right': 1} at depth:  10 by bringing 0 miss and 1  cann from the right f(n) = 11
A* SOLUTION NODE - {'miss_left': 0, 'cann_left': 1, 'position': 'right', 'miss_right': 3, 'cann_right': 2} at depth:  9 by bringing 0 miss and 2  cann from the left f(n) = 11
A* SOLUTION NODE - {'miss_left': 0, 'cann_left': 3, 'position': 'left', 'miss_right': 3, 'cann_right': 0} at depth:  8 by bringing 0 miss and 1  cann from the right f(n) = 11
A* SOLUTION NODE - {'miss_left': 0, 'cann_left': 2, 'position': 'right', 'miss_right': 3, 'cann_right': 1} at depth:  7 by bringing 2 miss and 0  cann from the left f(n) = 11
A* SOLUTION NODE - {'miss_left': 2, 'cann_left': 2, 'position': 'left', 'miss_right': 1, 'cann_right': 1} at depth:  6 by bringing 1 miss and 1  cann from the right f(n) = 11
A* SOLUTION NODE - {'miss_left': 1, 'cann_left': 1, 'position': 'right', 'miss_right': 2, 'cann_right': 2} at depth:  5 by bringing 2 miss and 0  cann from the left f(n) = 9
A* SOLUTION NODE - {'miss_left': 3, 'cann_left': 1, 'position': 'left', 'miss_right': 0, 'cann_right': 2} at depth:  4 by bringing 0 miss and 1  cann from the right f(n) = 9
A* SOLUTION NODE - {'miss_left': 3, 'cann_left': 0, 'position': 'right', 'miss_right': 0, 'cann_right': 3} at depth:  3 by bringing 0 miss and 2  cann from the left f(n) = 9
A* SOLUTION NODE - {'miss_left': 3, 'cann_left': 2, 'position': 'left', 'miss_right': 0, 'cann_right': 1} at depth:  2 by bringing 0 miss and 1  cann from the right f(n) = 9
A* SOLUTION NODE - {'miss_left': 3, 'cann_left': 1, 'position': 'right', 'miss_right': 0, 'cann_right': 2} at depth:  1 by bringing 0 miss and 2  cann from the left f(n) = 9
INITIAL NODE - {'miss_left': 3, 'cann_left': 3, 'position': 'left', 'miss_right': 0, 'cann_right': 0} at depth:  0
Number of nodes created: 14
"""
# DFS found solution at same depth as A*, I believe this is due to the constraints of the problem severely reducing the amount of nodes being made. 
# The branching factor ends up being less than 5 many times because of the constraints
# Should test with bigger initial state with more miss and cann to see if algorithms diverge

# Unlike DFS, A* found the same solution with and without a closed/reached set where DFS only worked with a closed list since
# due to being trapped in a cycle. 
# If the heuristic function is admissible and consistent this would then suggest depth 11 is the optimal solution depth.