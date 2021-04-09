from DataStructures import Graph
from utils import euclidean_distance
from random import shuffle
import math
import time

class Problem(object):

    """The abstract class for a formal problem. You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal. Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        raise NotImplementedError

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state).
        This is actually the Transition Model."""
        raise NotImplementedError

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        if isinstance(self.goal, list):
            return state in self.goal
        else:
            return state == self.goal

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self, state):
        """For optimization problems, each state has a value.  Hill-climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError

class PathProblem(Problem):

    """The problem of searching a path from one node to another."""

    def __init__(self, initial, goal, graph, graph_locations=None):
        Problem.__init__(self, initial, goal)
        if graph_locations is not None:
            self.locations = graph_locations
        if not isinstance(graph, Graph):
            raise TypeError
            
        self.graph = graph

    def actions(self, A):
        """The actions at a graph node are just its neighbors."""
        return list(self.graph.get(A).keys())

    def result(self, state, action):
        """The result of going to a neighbor is just that neighbor."""
        return action

    def path_cost(self, cost_so_far, A, action, B):
        return cost_so_far + (self.graph.get(A, B) or math.inf)

    def find_min_edge(self):
        """Find minimum value of edges."""
        m = math.inf
        for d in self.graph.dict.values():
            local_min = min(d.values())
            m = min(m, local_min)

        return m

    def h(self, node):
        """The heuristic h(n) function: here it's used the straight distance."""
        locs = self.locations
        if locs:
            return int(euclidean_distance(locs[node.state], locs[self.goal]))
        else:
            return math.inf

class QueensProblem(Problem):

    """
    Classical n-queens problem on the chessboard.
    The state is represented by a vector of numbers.
    The component r at index c, represents the position
    of a queen at row r of column c in a chessboard with
    the (0,0) at the top left corner.
    """

    def __init__(self, n):
        """Initializes an empty board."""
        initial_state = tuple([-1 for i in range(n)])
        Problem.__init__(self, initial_state)
        self.n = n

    def result(self, state, action):
        """
        The result state is simply obtained by adding the coords
        of the new queen to the state.
        action is an (x,y) pair, where x is the row and y the column.
        """
        new_state = list(state)
        new_state[action[1]] = action[0]
        return tuple(new_state)

    def actions(self, state):
        """Applicable actions from a given set of queens are all the locations
        not attackable by the queens already in the state."""
        for i in range(self.n):
            # If the column isn't empty then skip it
            if state[i] != -1:
                continue

            forbidden_locations = []
            for j in range(self.n):
                if i != j and state[j] != -1:
                    forbidden_locations.append(state[j])
                    if state[j] - abs(i-j) > 0:
                        forbidden_locations.append(state[j] - abs(i-j))
                    if state[j] + abs(i-j) < self.n:
                        forbidden_locations.append(state[j] + abs(i-j))

            return [(x,i) for x in range(self.n) if x not in forbidden_locations]

    def goal_test(self, state):
        """The state is a goal state if no queen attacks the other."""
        for i in range(self.n):
            # If a column is empty then we have less than n queens on the board
            if state[i] == -1:
                return False

            forbidden_locations = []
            for j in range(self.n):
                if i != j and state[j] != -1:
                    forbidden_locations.append(state[j])
                    if state[j] - abs(i-j) > 0:
                        forbidden_locations.append(state[j] - abs(i-j))
                    if state[j] + abs(i-j) < self.n:
                        forbidden_locations.append(state[j] + abs(i-j))

            if state[i] in forbidden_locations:
                return False

        return True

class SlidingBlockPuzzleProblem(Problem):

    """Classical SlidingBlock Puzzle problem of size nxn.
    It is known to be NP-Complete."""

    def __init__(self, n=3, initial=None):
        """If no initial state is provided a random solvable one will be generated."""
        self.n = n if n>2 else 3
        gen = [i for i in range(0, self.n**2)]
        #gen.append(0)
        goal = tuple(gen)
        
        shuffle(gen)
        while not self.isSolvable(gen):
            shuffle(gen)

        gen = initial if initial is not None and self.isSolvable(initial) else tuple(gen)

        Problem.__init__(self, gen, goal)

    def result(self, state, action):
        """Simply get position of character '0' and switch it with
        the other character in the appropriate position."""
        new_state = list(state)

        if action=='left':
            blank = state.index(0)
            old = state[blank-1]
            new_state[blank-1] = 0
            new_state[blank] = old
        elif action=='right':
            blank = state.index(0)
            old = state[blank+1]
            new_state[blank+1] = 0
            new_state[blank] = old
        elif action=='up':
            blank = state.index(0)
            old = state[blank-self.n]
            new_state[blank-self.n] = 0
            new_state[blank] = old
        else:
            blank = state.index(0)
            old = state[blank+self.n]
            new_state[blank+self.n] = 0
            new_state[blank] = old

        return tuple(new_state)

    def actions(self, state):
        """Check for perimetral positions of the matrix."""
        good_actions = []
        blank = state.index(0)

        # Perimetral rows
        if blank//self.n != self.n-1:
            good_actions.append('down')
        if blank//self.n != 0:
            good_actions.append('up')

        # Perimetral cols
        if blank%self.n != self.n-1:
            good_actions.append('right')
        if blank%self.n != 0:
            good_actions.append('left')

        return good_actions

    def isSolvable(self, state):
        """If the number of inversions is even then the
        puzzle is solvable."""
        inversions = 0
        for i in range(len(state)):
            for j in range(i, len(state)):
                if state[i]>state[j] and state[j]!=0:
                    inversions += 1

        return inversions%2==0

    def h_misplaced(self, node):
        """Number of misplaced tiles heuristic."""
        should = self.goal
        actually = node.state
        diff = 0
        for i in range(self.n**2):
            if should[i]!=actually[i]:
                diff += 1

        return diff

    def h_manhattan(self, node):
        """Sum of manhattan distances between each tile
        and the place where it should be."""
        dist = 0
        for i in range(self.n**2):
            # where it should be
            x = node.state[i]%self.n
            y = node.state[i]//self.n
            # where it is
            x0 = i%self.n
            y0 = i//self.n
            # manhattan d
            dist += abs(x-x0) + abs(y-y0)
        return dist

    def h_euclidean(self, node):
        dist = 0
        for i in range(self.n**2):
            # where it should be
            x = node.state[i]%self.n
            y = node.state[i]//self.n
            # where it is
            x0 = i%self.n
            y0 = i//self.n
            # manhattan d
            dist += math.sqrt((x-x0)**2 + (y-y0)**2)
        return dist

    def printable(self, state):
        """Shows the grid from the state string"""
        out = ''
        for i in range(self.n):
            out += '| '
            for j in range(self.n):
                out += str(state[j+i*self.n])+(' ' if j!=self.n-1 else ' |\n')
        return out

    def show(self, solution):
        """Shows the solution."""
        current = self.initial
        for i in range(len(solution)):
            print(self.printable(current))
            current = self.result(current, solution[i])
            time.sleep(.3)

class NavigationProblem(Problem):

    def __init__(self, initial, goal, obstacles):
        self.obstacles = obstacles
        Problem.__init__(self, initial, goal)

    def actions(self, state):
        (x,y) = state
        reachable_points = []
        i=1
        while i:
            base = [x-i,y-i]

            for r in range(1+2*i):
                row = []
                for c in range(0, 1+2*i, (1 if (r==0 or r==2*i) else 2*i)):
                    current = (base[0]+c, base[1]+r)
                    if not self.intersects(state, current):
                        row.append(current)
                reachable_points += row

            if reachable_points:
                return reachable_points
            i+=1

    def result(self, state, action):
        return action

    def computeIntersection(self, A, B, P0, P1):
        """Returns (x,y) of intersection"""
        (xa, ya) = A
        (xb, yb) = B
        (x0, y0) = P0
        (x1, y1) = P1
        #print("Checking intersection between {},{} and {},{}".format(A,B,P0,P1))

        # Speed test
        if A==P0 or A==P1:
            return A
        if B==P0 or B==P1:
            return B

        # Vertical line
        if xa==xb and x0!=x1:
            r = (y1-y0)/(x1-x0)
            return (xa, r*(xa-x0)+y0)
        elif xa==xb and x0==x1:
            if xa==x0:
                return 0
            else:
                return None
        elif x0==x1 and xa!=xb:
            incr_ratio = (ya-yb)/(xa-xb)
            return (x0, incr_ratio*(x0-xb)+yb)

        incr_ratio = (ya-yb)/(xa-xb)
        r = (y1-y0)/(x1-x0)

        # Check for parallelism
        if abs(incr_ratio-r)==0:
            return None

        x = (r*x0-incr_ratio*xb+yb-y0)/(r-incr_ratio)
        y = r*(x-x0)+y0
        return (x,y)

    def intersects(self, A, B):
        """Returns true if line A-B intersects any of the obstacles"""
        (xa, ya) = A
        (xb, yb) = B
        for i in range(len(self.obstacles)):
            # Verticies
            P0 = (x0, y0) = self.obstacles[i][0]
            P1 = (x1, y1) = self.obstacles[i][1]
            P2 = (x2, y2) = self.obstacles[i][2]
            # First segment
            test = self.computeIntersection(A,B,P0,P1)
            if test==0:#Coincident vertical lines
                if (y1>=min(ya,yb) and y1<=max(ya,yb)) or (y0>=min(ya,yb) and y0<=max(ya,yb)):
                    return True
            elif test is not None:
                (xi, yi) = test
                if xi>=min(x0, x1) and xi<=max(x0, x1) and yi>=min(y0, y1) and yi<=max(y0, y1):
                    if xi>=min(xa, xb) and xi<=max(xa, xb) and yi>=min(ya, yb) and yi<=max(ya, yb):
                        return True
            # Second segment
            test = self.computeIntersection(A,B,P0,P2)
            if test==0:#Coincident vertical lines
                if (y2>=min(ya,yb) and y2<=max(ya,yb)) or (y0>=min(ya,yb) and y0<=max(ya,yb)):
                    return True
            elif test is not None:
                (xi, yi) = test
                if xi>=min(x0, x2) and xi<=max(x0, x2) and yi>=min(y0, y2) and yi<=max(y0, y2):
                    if xi>=min(xa, xb) and xi<=max(xa, xb) and yi>=min(ya, yb) and yi<=max(ya, yb):
                        return True
            # Third segment
            test = self.computeIntersection(A,B,P1,P2)
            if test==0:#Coincident vertical lines
                if (y1>=min(ya,yb) and y1<=max(ya,yb)) or (y2>=min(ya,yb) and y2<=max(ya,yb)):
                    return True
            elif test is not None:
                (xi, yi) = test
                if xi>=min(x1, x2) and xi<=max(x1, x2) and yi>=min(y1, y2) and yi<=max(y1, y2):
                    if xi>=min(xa, xb) and xi<=max(xa, xb) and yi>=min(ya, yb) and yi<=max(ya, yb):
                        return True

        return False

    def h_euclidean(self, node):
        state = node.state
        return euclidean_distance(state, self.goal)

class ShortcutProblem(Problem):

    def __init__(self, navProblem, minGridPath):
        if not isinstance(navProblem, NavigationProblem):
            raise Error("Problem must be of type NavigationProblem")

        self.navProblem = navProblem
        self.path = self.delRedundance([self.navProblem.initial] + minGridPath)

        Problem.__init__(self, (navProblem.initial, tuple(self.path)))

    def delRedundance(self, rawPath):
        path = list(rawPath)
        toremove = []
        for i in range(len(path)-2):
            P0 = path[i]
            P1 = path[i+1]
            P2 = path[i+2]

            if self.aligned(P0, P1, P2):
                toremove.append(P1)

        for i in range(len(toremove)):
            path.remove(toremove[i])

        return path

    def aligned(self, P0, P1, P2):
        (x0, y0) = P0
        (x1, y1) = P1
        (x2, y2) = P2

        if x0==x2:
            return (x1==x0)

        return (y1==((y2-y0)/(x2-x0))*(x1-x0)+y0)

    def actions(self, state):
        current = state[0]
        path = list(state[1])
        index = path.index(current)
        actions = []
        actions.append((path[index+1], tuple(path)))
        for i in range(index+2, len(path)):
            if not self.navProblem.intersects(current, path[i]):
                actions.append((path[i], tuple(path[0:index+1] + path[i:])))
        return actions

    def result(self, state, action):
        return action

    def goal_test(self, state):
        return state[0]==self.navProblem.goal

    def path_cost(self, c, state1, action, state2):
        tot = 0
        path = state2[1]
        for i in range(len(path)-1):
            tot += euclidean_distance(path[i], path[i+1])
        return tot

    def h_euclidean(self, node):
        state = node.state
        return euclidean_distance(state[0], self.navProblem.goal)
