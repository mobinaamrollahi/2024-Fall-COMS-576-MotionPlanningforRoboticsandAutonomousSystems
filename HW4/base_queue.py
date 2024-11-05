import heapq
from collections import deque

class Queue:
    def __init__(self):
        # The queue
        self.queue = []

        # Parent dictionary        
        self.parent = {}  

    def __len__(self):
        """Return the length of the queue"""
        return len(self.queue)

    def insert(self, x, parent):
        """Insert an element into the queue."""
        raise NotImplementedError

    def pop(self):
        """Remove and return the first element in the queue"""
        return self.queue.pop(0)
    
    def get_visited(self):
        """Return the list of elements that have been inserted into the queue"""
        return list(self.parent.keys())
    
    def get_path(self, xI, xG):
        """Trace back parents to return a path from xI to xG"""
        path = [xG]
        x = xG
        while x is not xI:
            x = self.parent.get(x)
            if x in None:
                return []
            path.insert(0, x)
        return path
    
    def _is_visited(self, x):
        """Return whether x has been visited (i.e., added to the queue at some point)"""
        return x in self.parent

    def _add_element_at(self, ind, x, parent):
        """Add x to the queue at index ind. Also, update the parent of x."""
        self.queue.insert(ind, x)
        self.parent[x] = parent
    
    

    

    
