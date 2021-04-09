import collections
import bisect

class Node:

	"""A Node for the exploration of solution space."""

	def __init__(self, state, parent=None, action=None, path_cost=0):
		 """Create a search tree Node, derived from a parent by an action."""
		 self.state = state
		 self.parent = parent
		 self.action = action
		 self.path_cost = path_cost

		 # Useful when limiting depth
		 self.depth = 0
		 if parent is not None:
		 	self.depth = parent.depth+1
	
	def __repr__(self):
		return "<Node: {}>".format(self.state)
	
	def expand(self, problem):
		"""List the nodes reachable from this node."""
		return [self.child_node(problem, action) 
				for action in problem.actions(self.state)]
	
	def child_node(self, problem, action):
		"""Creates new child node."""
		next_state = problem.result(self.state,action)
		return Node(next_state, self, action,
					problem.path_cost(self.path_cost,
									  self.state, action,
									  next_state))
	
	def solution(self):
		"""Return the sequence of actions to go from the root to this node."""
		return [node.action for node in self.path()[1:]]

	def path(self):
		"""Return a list of nodes forming the path from the root to this node."""
		node, path_back = self, []
		while node:
			path_back.append(node)
			node = node.parent
		return list(reversed(path_back))
	
	# We want for a queue of nodes in breadth_first_search or
	# astar_search to have no duplicated states, so we treat nodes
	# with the same state as equal. [Problem: this may not be what you
	# want in other contexts.]

	def __eq__(self, other):
		return isinstance(other, Node) and self.state == other.state

	def __hash__(self):
		return hash(self.state)
	
	def __str__(self):
		return "{ State: "+str(self.state)+(', Parent: '+str(self.parent.state) if self.parent is not None else '')+(', Action: '+str(self.action) if self.action is not None else '')+(', Cost: '+str(self.path_cost) if self.path_cost is not None else '')+", Depth: "+str(self.depth)+" }"

	def __lt__(self, other):
		return isinstance(other, Node) and self.path_cost<other.path_cost

class Graph:

	"""A graph connects nodes (verticies) by edges (links).  Each edge can also
	have a length associated with it.  The constructor call is something like:
		g = Graph({'A': {'B': 1, 'C': 2})
	this makes a graph with 3 nodes, A, B, and C, with an edge of length 1 from
	A to B,  and an edge of length 2 from A to C.  You can also do:
		g = Graph({'A': {'B': 1, 'C': 2}, directed=False)
	This makes an undirected graph, so inverse links are also added. The graph
	stays undirected; if you add more links with g.connect('B', 'C', 3), then
	inverse link is also added.  You can use g.nodes() to get a list of nodes,
	g.get('A') to get a dict of links out of A, and g.get('A', 'B') to get the
	length of the link from A to B.  'Lengths' can actually be any object at
	all, and nodes can be any hashable object."""

	def __init__(self, dict=None, directed=True):
		self.dict = dict or {}
		self.directed = directed
		if not directed:
			self.make_undirected()

	def make_undirected(self):
		"""Make a digraph into an undirected graph by adding symmetric edges."""
		for a in list(self.dict.keys()):
			for (b, dist) in self.dict[a].items():
				self.connect1(b, a, dist)

	def connect(self, A, B, distance=1):
		"""Add a link from A and B of given distance, and also add the inverse
		link if the graph is undirected."""
		self.connect1(A, B, distance)
		if not self.directed:
			self.connect1(B, A, distance)

	def connect1(self, A, B, distance):
		"""Add a link from A to B of given distance, in one direction only."""
		self.dict.setdefault(A, {})[B] = distance

	def get(self, a, b=None):
		"""Return a link distance or a dict of {node: distance} entries.
		.get(a,b) returns the distance or None;
		.get(a) returns a dict of {node: distance} entries, possibly {}."""
		links = self.dict.get(a)
		if b is None:
			return links
		else:
			return links.get(b)

	def nodes(self):
		"""Return a list of nodes in the graph."""
		return list(self.dict.keys())
	
	def __str__(self):
		"""Prints the graph."""
		out = ''
		for key in self.dict:
			out += key+' => '
			for dest in self.dict[key]:
				out += ' '*4+dest+'('+str(self.dict[key][dest])+')'
			out += '\n'
		return out


class FIFOQueue:
	
	""" A Classical First In First Out queue."""
	
	def __init__(self, max_len=None, initial_items=[]):
		"""Init an empty queue."""
		self.data = collections.deque(initial_items, max_len)# deque is a fast to pop push list-like collection
		self.max_len = max_len if max_len is not None and max_len>0 else None
	
	
	def push(self, data):
		"""Inserts an element at the end of the queue."""
		if self.max_len is None or len(self.data)<self.max_len:
			self.data.append(data)
		else:
			raise Exception('Error cannot insert the element: the FIFOQueue number {0} is full.'.format(id(self)))
	
	def extend(self, items):
		"""Adds all items to te queue if possible."""
		if self.max_len is None or len(items)+len(self.data)<=self.max_len:
			self.data.extend(items)
		else:
			raise Exception('Error cannot insert elements: the FIFOQueue number {0} hasn\'t got enough space.'.format(id(self)))
	
	def pop(self):
		"""Returns the first element of the queue."""
		if len(self.data)==0:
			raise Exception('Error cannot pop element: the FIFOQueue number {0} is empty.'.format(id(self)))
		else:
			return self.data.popleft()
	
	def __len__(self):
		"""The length of the fifo queue is the length of data."""
		return len(self.data)
	
	def __contains__(self, item):
		return item in self.data
	
	def __str__(self):
		"""Returns queue to string."""
		return '[ '+', '.join([str(item) for item in self.data])+' ]'

class LIFOQueue:
	
	""" A Classical Last In First Out queue or Stack."""
	
	def __init__(self, max_len=None, initial_items=[]):
		"""Init an empty queue."""
		self.data = collections.deque(initial_items, max_len)# deque is a fast to pop push list-like collection
		self.max_len = max_len if max_len is not None and max_len>0 else None
	
	
	def push(self, data):
		"""Inserts an element at the end of the queue."""
		if self.max_len is None or len(self.data)<self.max_len:
			self.data.appendleft(data)
		else:
			raise Exception('Error cannot insert the element: the FIFOQueue number {0} is full.'.format(id(self)))
	
	def extend(self, items):
		"""Adds all items to te queue if possible."""
		if self.max_len is None or len(items)+len(self.data)<=self.max_len:
			self.data.extendleft(items)
		else:
			raise Exception('Error cannot insert elements: the FIFOQueue number {0} hasn\'t got enough space.'.format(id(self)))
	
	def pop(self):
		"""Returns the first element of the queue."""
		if len(self.data)==0:
			raise Exception('Error cannot pop element: the FIFOQueue number {0} is empty.'.format(id(self)))
		else:
			return self.data.pop()
	
	def __len__(self):
		"""The length of the fifo queue is the length of data."""
		return len(self.data)
	
	def __contains__(self, item):
		return item in self.data
	
	def __str__(self):
		"""Returns queue to string."""
		return '[ '+', '.join([str(item) for item in self.data])+' ]'

class PriorityQueue:

	"""A queue in which the minimum (or maximum) element (as determined by f and
	order) is returned first. If order is min, the item with minimum f(x) is
	returned first; if order is max, then it is the item with maximum f(x).
	Also supports dict-like lookup."""

	def __init__(self, order=min, f=lambda x: x):
		self.A = []
		self.order = order
		self.f = f

	def push(self, item):
		bisect.insort(self.A, (self.f(item), item))

	def __len__(self):
		return len(self.A)

	def pop(self):
		if self.order == min:
			return self.A.pop(0)[1]
		else:
			return self.A.pop()[1]

	def __contains__(self, item):
		return any(item == pair[1] for pair in self.A)

	def __getitem__(self, key):
		for _, item in self.A:
			if item == key:
				return item

	def __delitem__(self, key):
		for i, (value, item) in enumerate(self.A):
			if item == key:
				self.A.pop(i)
