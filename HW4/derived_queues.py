from base_queue import Queue

class QueueBFS(Queue):
    def insert(self, x, parent):
        if x in self.parent:
            return False
        
        self._add_element_at(len(self.queue), x, parent)
        return True

class QueueAstar(Queue):
    """The queue that implements the insert function for A*"""

    def __init__(self, cost_to_go_estimator):
        # For A*, we need to keep track of cost-to-come and estimated cost-to-go for each element
        super().__init__()
        self.cost_to_go_estimator = cost_to_go_estimator
        self.costs = {}

    def insert(self, x, parent, edge_cost=1):
        # The root has cost-to-come = 0, so its parent should have cost-to-come = -1
        parent_cost_to_come = -edge_cost
        if parent is not None:
            parent_cost_to_come = self.costs[parent][0]

        # Cost is a tuple (cost-to-come, cost-to-go)
        new_cost = (
            parent_cost_to_come + edge_cost,
            self.cost_to_go_estimator.get_lower_bound(x),
        )
        current_cost = self.costs.get(x)

        # Do nothing if the new cost is not smaller than the current cost
        if current_cost is not None and self._get_total_cost(
            current_cost
        ) <= self._get_total_cost(new_cost):
            return False

        # Resolve duplicate element by updating the cost of the element
        # To do this, we simply remove the element from the queue and re-insert it with the new cost
        if current_cost is not None:
            self.queue.remove(x)

        self._add_element_at(
            self._find_insert_index(self._get_total_cost(new_cost)), x, parent
        )
        self.costs[x] = new_cost
        return True

    def _get_total_cost(self, cost):
        """Return the sum of the element in cost"""
        return sum(cost)

    def _find_insert_index(self, cost):
        """Find the first index in the queue such that the cost of the corresponding element is greater than the given cost"""
        ind = 0
        while ind < len(self.queue):
            element_cost = self.costs[self.queue[ind]]
            if self._get_total_cost(element_cost) > cost:
                return ind
            ind += 1
        return ind

    