from DataStructures import *
import time

class SimpleProblemSolvingAgentProgram:

	"""Abstract framework for a problem-solving agent."""

	def __init__(self, initial_state=None):
		"""State is an abstract representation of the state
		of the world, and seq is the list of actions required
		to get to a particular state from the initial state(root)."""
		self.state = initial_state
		self.seq = []

	def __call__(self, percept):
		"""Formulate a goal and problem, then
		search for a sequence of actions to solve it."""
		self.state = self.update_state(self.state, percept)
		if not self.seq:
			goal = self.formulate_goal(self.state)
			problem = self.formulate_problem(self.state, goal)
			self.seq = self.search(problem)
			if not self.seq:
				return None
		return self.seq.pop(0)

	def update_state(self, percept):
		"""Updates the state of the agent based on the current percept it perceives."""
		raise NotImplementedError

	def formulate_goal(self, state):
		"""Given the state which goal should the agent formulate?"""
		raise NotImplementedError

	def formulate_problem(self, state, goal):
		"""Given the goal and the current state which problem does
		the agent has to solve?"""
		raise NotImplementedError

	def search(self, problem):
		"""Solve a problem by searching using a search method."""
		raise NotImplementedError

class SolutionNotFound:

	def __str__(self):
		return "SolutionNotFound"

class LimitReached:

	def __str__(self):
		return "LimitReached"

def tree_search(problem, frontier, limit=None, verbose=False):
	# Create initial node
	node = Node(problem.initial)
	
	# Immediate check for solution
	if problem.goal_test(node.state):
		return node.solution()
	
	# Init frontier
	frontier.push(node)
	
	while True:
		# Failure case
		if len(frontier)==0 and limit is None:
			return SolutionNotFound()
		elif len(frontier)==0 and limit is not None:
			return LimitReached()
		
		node = frontier.pop()
		
		for child in node.expand(problem):
			if(verbose):
				print(child.state)
			
			if problem.goal_test(child.state):
				return child.solution()
		
		if limit is not None and node.depth<limit:
			frontier.extend(node.expand(problem))
		elif limit is None:
			frontier.extend(node.expand(problem))

def graph_search(problem, frontier, verbose=False):
	# Create initial node
	node = Node(problem.initial)

	# Immediate check for solution
	if problem.goal_test(node.state):
		return node.solution()

	# Init frontier
	frontier.push(node)

	# Init empty explored set
	explored = set()

	while True:
		#Failure
		if len(frontier)==0:
			return SolutionNotFound()

		node = frontier.pop()
		explored.add(node.state)

		for child in node.expand(problem):
			if verbose:
				print(child)

			if child.state not in explored and child not in frontier:
				if problem.goal_test(child.state):
					return child.solution()
				else:
					frontier.push(child)

def best_first_tree_search(problem, f, verbose=False):
	# Create initial node
	node = Node(problem.initial)

	# Immediate check for solution
	if problem.goal_test(node.state):
		return node.solution()

	# Init frontier that minimizes f
	frontier = PriorityQueue(min, f)
	frontier.push(node)

	while True:
		#Failure
		if len(frontier)==0:
			return SolutionNotFound()

		node = frontier.pop()

		if problem.goal_test(node.state):
			return node.solution()

		for child in node.expand(problem):
			if verbose:
				print(child.state)

			frontier.push(child)
			if child in frontier and f(frontier[child]) > f(child):
				del frontier[child]
				frontier.push(child)

def best_first_graph_search(problem, f, verbose=False):
	# Create initial node
	node = Node(problem.initial)

	# Immediate check for solution
	if problem.goal_test(node.state):
		return node.solution()

	# Init frontier that minimizes f
	frontier = PriorityQueue(min, f)
	frontier.push(node)

	# Init empty explored set
	explored = set()

	while True:
		#Failure
		if len(frontier)==0:
			return SolutionNotFound()

		node = frontier.pop()

		if problem.goal_test(node.state):
			return node.solution()

		explored.add(node.state)

		for child in node.expand(problem):
			if verbose:
				print(child.state)

			if child.state not in explored and child not in frontier:
				frontier.push(child)
			elif child in frontier and f(frontier[child]) > f(child):
				del frontier[child]
				frontier.push(child)

# Main Search algorithms
def breadth_first(problem, tree=True, verbose=False):
	if tree:
		return tree_search(problem, FIFOQueue(), verbose=verbose)
	else:
		return graph_search(problem, FIFOQueue(), verbose=verbose)

def uniform_cost(problem, tree=True, verbose=False):
	if tree:
		return best_first_tree_search(problem, lambda node: node.path_cost, verbose=verbose)
	else:
		return best_first_graph_search(problem, lambda node: node.path_cost, verbose=verbose)

def depth_first(problem, tree=True, verbose=False):
	if tree:
		return tree_search(problem, LIFOQueue(), verbose=verbose)
	else:
		return graph_search(problem, LIFOQueue(), verbose=verbose)

def depth_limited(problem, limit, tree=True, verbose=False):
	if tree:
		return tree_search(problem, LIFOQueue(), limit=limit, verbose=verbose)

def iterative_deepening(problem, tree=True, verbose=False):
	i = 1
	while i:
		solution = depth_limited(problem, i, tree, verbose=verbose)
		if not isinstance(solution, LimitReached):
			return solution
		i += 1

def greedy_best_first(problem, heuristic, verbose=False):
	"""Only greedy best-first graph search will be
	implemented because the tree-search version is incomplete
	even in finite state spaces."""
	return best_first_graph_search(problem, heuristic, verbose=verbose)

def a_star_best_first(problem, heuristic, tree=True, verbose=False):
	if tree:
		return best_first_tree_search(problem, lambda n: heuristic(n)+n.path_cost, verbose=verbose)
	else:
		return best_first_graph_search(problem, lambda n: heuristic(n)+n.path_cost, verbose=verbose)
