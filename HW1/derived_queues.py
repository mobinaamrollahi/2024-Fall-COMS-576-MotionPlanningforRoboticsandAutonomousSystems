from base_queue import Queue

class QueueBFS(Queue):
    def insert(self, x, parent):
        if x not in self.parent:
            self.queue.append(x)
            self.parent[x] = parent

    def pop(self):
        return self.queue.pop(0)  

class QueueAstar(Queue):
    def __init__(self, X, XG):
        super().__init__()
        self.X = X
        self.XG = XG
        self.queue = []
        self.parent = {}
        self.f_scores = {}

    def insert(self, x, parent):
        self.queue.insert(0, x)        
        self.parent[x] = parent
        if parent is not None:
            self.f_scores[x] = self.f_scores[parent] + 1
        else:
            self.f_scores[x] = 0
        for goal in self.XG:
            self.f_scores[x] += self.X.get_distance_lower_bound(x, goal)

    def pop(self):
        index = self.queue.index(min(self.queue, key=lambda x: self.f_scores[x]))
        x = self.queue.pop(index)
        return x
    