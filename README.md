# Assignment-4-Graphs

1----------Breadth First Traversal for a Graph
def bfsOfGraph(V, adj):
 
    bfs_traversal = []
    vis = [False]*V
    for i in range(V):
 
        # To check if already visited
        if (vis[i] == False):
            q = []
            vis[i] = True
            q.append(i)
 
            # BFS starting from ith node
            while (len(q) > 0):
                g_node = q.pop(0)
 
                bfs_traversal.append(g_node)
                for it in adj[g_node]:
                    if (vis[it] == False):
                        vis[it] = True
                        q.append(it)
 
    return bfs_traversal
    
    
    
    2----------Depth First Traversal for a Graph
    '''Python3 program to print DFS traversal for complete graph'''
from collections import defaultdict
 
# this class represents a directed graph using adjacency list representation
 
 
class Graph:
    # Constructor
    def __init__(self):
        # default dictionary to store graph
        self.graph = defaultdict(list)
 
    # Function to add an edge to graph
    def addEdge(self, u, v):
        self.graph[u].append(v)
    # A function used by DFS
 
    def DFSUtil(self, v, visited):
        # Mark the current node as visited and print it
        visited.add(v)
        print(v, end=" ")
 
        # recur for all the vertices adjacent to this vertex
        for neighbour in self.graph[v]:
            if neighbour not in visited:
                self.DFSUtil(neighbour, visited)
        # The function to do DFS traversal. It uses recursive DFSUtil
 
    def DFS(self):
        # create a set to store all visited vertices
        visited = set()
        # call the recursive helper function to print DFS traversal starting from all
        # vertices one by one
        for vertex in self.graph:
            if vertex not in visited:
                self.DFSUtil(vertex, visited)
# Driver's code
# create a graph given in the above diagram
 
if __name__ == "__main__":
  print("Following is Depth First Traversal \n")
  g = Graph()
  g.addEdge(0, 1)
  g.addEdge(0, 2)
  g.addEdge(1, 2)
  g.addEdge(2, 0)
  g.addEdge(2, 3)
  g.addEdge(3, 3)
 
  # Function call
  g.DFS()
  
  
  
  3-----------Count the number of nodes at given level in a tree using BFS
   Python3 program to print
# count of nodes at given level.
from collections import deque
  
adj = [[] for i in range(1001)]
  
def addEdge(v, w):
     
    # Add w to v’s list.
    adj[v].append(w)
  
    # Add v to w's list.
    adj[w].append(v)
  
def BFS(s, l):
     
    V = 100
     
    # Mark all the vertices
    # as not visited
    visited = [False] * V
    level = [0] * V
  
    for i in range(V):
        visited[i] = False
        level[i] = 0
  
    # Create a queue for BFS
    queue = deque()
  
    # Mark the current node as
    # visited and enqueue it
    visited[s] = True
    queue.append(s)
    level[s] = 0
  
    while (len(queue) > 0):
         
        # Dequeue a vertex from
        # queue and print
        s = queue.popleft()
        #queue.pop_front()
  
        # Get all adjacent vertices
        # of the dequeued vertex s.
        # If a adjacent has not been
        # visited, then mark it
        # visited and enqueue it
        for i in adj[s]:
            if (not visited[i]):
  
                # Setting the level
                # of each node with
                # an increment in the
                # level of parent node
                level[i] = level[s] + 1
                visited[i] = True
                queue.append(i)
  
    count = 0
    for i in range(V):
        if (level[i] == l):
            count += 1
             
    return count
  
# Driver code
if __name__ == '__main__':
     
    # Create a graph given
    # in the above diagram
    addEdge(0, 1)
    addEdge(0, 2)
    addEdge(1, 3)
    addEdge(2, 4)
    addEdge(2, 5)
  
    level = 2
  
    print(BFS(0, level))
    
    
    
    4---------Count number of trees in a forest
    # Python3 program to count number 
# of trees in a forest.
 
# A utility function to add an
# edge in an undirected graph.
def addEdge(adj, u, v):
    adj[u].append(v)
    adj[v].append(u)
 
# A utility function to do DFS of graph
# recursively from a given vertex u.
def DFSUtil(u, adj, visited):
    visited[u] = True
    for i in range(len(adj[u])):
        if (visited[adj[u][i]] == False):
            DFSUtil(adj[u][i], adj, visited)
 
# Returns count of tree is the
# forest given as adjacency list.
def countTrees(adj, V):
    visited = [False] * V
    res = 0
    for u in range(V):
        if (visited[u] == False):
            DFSUtil(u, adj, visited)
            res += 1
    return res
 
# Driver code
if __name__ == '__main__':
 
    V = 5
    adj = [[] for i in range(V)]
    addEdge(adj, 0, 1)
    addEdge(adj, 0, 2)
    addEdge(adj, 3, 4)
    print(countTrees(adj, V))
    
    
    
    5---------Detect Cycle in a Directed Graph
    # Python program to detect cycle
# in a graph
 
from collections import defaultdict
 
class Graph():
    def __init__(self,vertices):
        self.graph = defaultdict(list)
        self.V = vertices
 
    def addEdge(self,u,v):
        self.graph[u].append(v)
 
    def isCyclicUtil(self, v, visited, recStack):
 
        # Mark current node as visited and
        # adds to recursion stack
        visited[v] = True
        recStack[v] = True
 
        # Recur for all neighbours
        # if any neighbour is visited and in
        # recStack then graph is cyclic
        for neighbour in self.graph[v]:
            if visited[neighbour] == False:
                if self.isCyclicUtil(neighbour, visited, recStack) == True:
                    return True
            elif recStack[neighbour] == True:
                return True
 
        # The node needs to be popped from
        # recursion stack before function ends
        recStack[v] = False
        return False
 
    # Returns true if graph is cyclic else false
    def isCyclic(self):
        visited = [False] * (self.V + 1)
        recStack = [False] * (self.V + 1)
        for node in range(self.V):
            if visited[node] == False:
                if self.isCyclicUtil(node,visited,recStack) == True:
                    return True
        return False
 
g = Graph(4)
g.addEdge(0, 1)
g.addEdge(0, 2)
g.addEdge(1, 2)
g.addEdge(2, 0)
g.addEdge(2, 3)
g.addEdge(3, 3)
if g.isCyclic() == 1:
    print "Graph contains cycle"
else:
    print "Graph doesn't contain cycle"
