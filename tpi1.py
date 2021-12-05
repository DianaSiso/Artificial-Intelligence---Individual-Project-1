#DIANA ELISABETE SISO OLIVEIRA
#NMEC 98607, LICENCIATURA EM ENGENHARIA INFORMÁTICA

from tree_search import *
from cidades import *

class MyNode(SearchNode):
    def __init__(self,state,parent,heuristic,cost,eval):
        super().__init__(state,parent)
        self.children = []
        self.cost = cost
        self.eval = eval
        self.heuristic = heuristic
        

class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',seed=0): 
        super().__init__(problem,strategy,seed)
        root = MyNode(problem.initial, None, self.problem.domain.heuristic(problem.initial, problem.goal), 0, self.problem.domain.heuristic(problem.initial, problem.goal))
        self.all_nodes = [root]
        self.open_nodes = [0]
        self.strategy = strategy
        self.solution = None
        self.non_terminals = 0
        self.curr_pseudo_rand_number = seed
        self.used_shortcuts = []

    def astar_add_to_open(self,lnewnodes):
        self.open_nodes += lnewnodes
        self.open_nodes.sort(key = lambda y: self.all_nodes[y].heuristic + self.all_nodes[y].cost)

    def propagate_eval_upwards(self,node):
        if (node.parent != None):
            #if node.parent == None, it's root
            mini = None
            for n in node.children:
                if mini is None or mini > (n.cost + n.heuristic):
                    #node.eval is always the same eval as the child with the lowest eval
                    mini = n.cost + n.heuristic
            self.all_nodes[node.parent].eval = mini        
            self.propagate_eval_upwards(self.all_nodes[node.parent])

    def search2(self,atmostonce=False):
        self.states = [] #ex5
        while self.open_nodes != []:

            nodeID = self.open_nodes.pop(0)
            node = self.all_nodes[nodeID]
            if self.problem.goal_test(node.state):
                self.solution = node
                self.terminals = len(self.open_nodes)+1
                return self.get_path(node)

            lnewnodes = []
            self.non_terminals += 1

            for a in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state,a)
                heuristic = self.problem.domain.heuristic(newstate, self.problem.goal)
                cost = node.cost + self.problem.domain.cost(node.state, a) 

                if newstate not in self.get_path(node):  
                    if (atmostonce) and newstate not in self.states: #ex5
                        self.states.append(newstate)
                        
                        newnode = MyNode(newstate, nodeID, heuristic, cost, cost + heuristic)   
                        self.all_nodes.append(newnode)
                        lnewnodes.append(len(self.all_nodes)-1)
                        node.children.append(newnode)
                        node.eval = self.propagate_eval_upwards(node)

                    if atmostonce == False:    
                        newnode = MyNode(newstate, nodeID, heuristic, cost, cost + heuristic)   
                        self.all_nodes.append(newnode)
                        lnewnodes.append(len(self.all_nodes)-1)
                        node.children.append(newnode)
                        node.eval = self.propagate_eval_upwards(node)
            
            self.add_to_open(lnewnodes)
            
        return None

    def repeated_random_depth(self,numattempts=3,atmostonce=False): 

        self.solution_tree=MyTree(self.problem, 'rand_depth', 0)
        #for now, seed = 0 is our best solution
        best_result = self.solution_tree.search2()

        for i in range(1,numattempts):  
            #we start in 1 because we already did the tree for the value 0
            self.temp=MyTree(self.problem, 'rand_depth', i)
            result=self.temp.search2()
            if self.temp.solution.cost<self.solution_tree.solution.cost:
                best_result = result
                self.solution_tree=self.temp

        return best_result

    def make_shortcuts(self):
        var_counter = 0
        #to access the indexes
        self._states_ = []
        #array with all states already checked

        self.next_check = self.get_path(self.solution)[0]
        self._new_path = [self.next_check]

        for s in self.get_path(self.solution):
            self._states_.append(s)
            self.next_state = s
            #to know which states we have already visited

            if (s == self.next_check):
                if (s != self.get_path(self.solution)[-1]):
                    self.next_state = self.get_path(self.solution)[var_counter + 1]
                
                self.next_check = self.next_state
                self.exist_shortcut = False
                
                for c in self.problem.domain.actions(s):
                    for x in self.get_path(self.solution):
                        if (x not in self._states_ and x != self.next_state and x in c):
                            #x not in self._states_ -> we still need check
                            #x != self.next_state -> if x == next_state we don't need check, it isn't a shortcut
                            
                            self.temp = c[1]
                            #to know the next state to check

                            self.used_shortcuts.append(c)
                            self.exist_shortcut = True
                            break
                
                if self.exist_shortcut:
                    self.next_check = self.temp
               
                if (self.next_check not in self._new_path):
                    self._new_path.append(self.next_check)
                    
            var_counter += 1
        return self._new_path


class MyCities(Cidades):

    def maximum_tree_size(self,depth):   # assuming there is no loop prevention
        
        neighbors = 0  
        #the average number os neighbors per state
        result = 0
        #the maximum size of a tree with that depth
        cities = len(self.coordinates)
        #total of cities

        for city in self.coordinates:  
            for connection in self.connections:
                if city in connection:
                    neighbors += 1
        for i in range(0, depth+1):
            result += (neighbors / cities) ** i

        return result


