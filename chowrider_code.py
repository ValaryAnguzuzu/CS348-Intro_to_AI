from expand import expand
from collections import deque
import heapq

# we are given a map JSON iwth two components
    # time_map: actual travle times btwn ADJACENT nodes
    # dis_map: straight-line distances btwn ANY pair of nodes
# to use expand() fn that given a node and the time_map, yields neighbors in a FIXED l-r order
# need to always keep track of:
    # fringe(queue/stack/pq)
    # visited set to avoid revisiting
    # parent dict to reconstruct the final path once we reach the goal

# reconstructing the path: once we pop the goal, reconstruct path
    # ie follow the parent pointers back from goal -> sart
    # reverse the list

#all need path reconstruction (helper)
def reconstruct_path(parents, start, goal):
    """
    helper: rebuild path from start to goal using parent pointers
    """
    path = []
    cur = goal
    while cur:
        path.append(cur)
        # if cur == start:
        #     break
        cur = parents.get(cur)
    path.reverse()
    return path


# TO DO: Implement Breadth-first Search.

# approach: (layer-by-layer)
    # BFS will traverse from l to r, level by level starting from 'start' node
    # fringe: queue(FIFO order)
    # track - visited nodes and parent mapping
    # when to stop? - when we pop the goal node


def breadth_first_search(time_map, start, end):
    """
    Breadth-first Search

    Args:
        time_map (dict): A map containing travel times between connected nodes (places or intersections), where every
        node is a dictionary key, and every value is an inner dictionary whose keys are the children of that node and
        values are travel times. Travel times are "null" for nodes that are not connected.
        start (str): The name of the node from where to start traversal
        end (str): The name of the node where to end traversal

    Returns:
        visited (list): A list of visited nodes in the order in which they were visited
        path (list): The final path found by the search algorithm
    """
    #pseudocode:
    # start at the source node.
        # Mark it as visited
        # Put it in a queue
    # While the queue isn’t empty:
        # Remove the first node from the queue.
        # “Visit” it (process it or record it).
        # Look at all of its neighbors.
        # For each neighbor that hasn’t been visited yet:
            # Mark it visited.
            # Put it at the back of the queue.
        # Repeat until either:
        # The goal node is found, or
        # There are no more nodes left.
    if start == end:
        return [start], [start]

    queue = deque([start])
    visited_set = {start}
    parents = {start: None} # map child to parent for path reconstruction
    visited_order = [] #list of nodes in the order in which they were expanded

    while queue:
        parent = queue.popleft()
        visited_order.append(parent)

        if parent == end: # goal found
            path = reconstruct_path(parents, start, end)
            return visited_order, path

        #we havent seen goal yet, so expand neighbors
        for neighbor in expand(parent, time_map):
            if neighbor not in visited_set:
                visited_set.add(neighbor)
                parents[neighbor] = parent
                queue.append(neighbor)

    # no path found
    return visited_order, []
    

# TO DO: Implement Depth-first Search.

# approach: (deep-before-wide)
    # DFS will go s deep as possible, then backtrack
    # fringe: stack(LIFO)
    # order detail: push neighbors in the same order as yielded by expand()
    # track - visited nodes
    # stop? when we pop the goal node

def depth_first_search(time_map, start, end):
    """
    Depth-first Search

    Args:
        time_map (dict): A map containing travel times between connected nodes (places or intersections), where every
        node is a dictionary key, and every value is an inner dictionary whose keys are the children of that node and
        values are travel times. Travel times are "null" for nodes that are not connected.
        start (str): The name of the node from where to start traversal
        end (str): The name of the node where to end start traversal

    Returns:
        visited (list): A list of visited nodes in the order in which they were visited
        path (list): The final path found by the search algorithm
    """

    # pseudocode:
    #Start at the source node.
        # Mark it as visited.
        # Put it on a stack.
    # While the stack isn’t empty:
        # Remove the top node from the stack.
        # “Visit” it (process it or record it).
        # Look at all of its neighbors.
        # For each neighbor that hasn’t been visited yet:
            # Mark it visited.
            # Put it on the top of the stack.
    # Repeat until either:
        # The goal node is found, or
        # There are no more nodes left.

    if start == end:
        return [start], [start]

    stack = [start]
    visited_set = set()
    parents = {start: None}
    visited_order = []

    while stack:
        parent = stack.pop()
        #print(f"\nPopped: {parent} | stack now: {stack}")
        if parent in visited_set:
            continue
        visited_set.add(parent)
        visited_order.append(parent)

        if parent == end:
            path = reconstruct_path(parents, start, end)
            return visited_order, path
        all_neighbors = expand(parent, time_map)
        #print(f"Neighbors of {parent}: {all_neighbors}")

        for neighbor in all_neighbors:
            if neighbor not in visited_set:
                #visited_set.add(neighbor)
                parents[neighbor] = parent
                stack.append(neighbor)
    #no path found
    return visited_order, []
    

# TO DO: Implement Greedy Best-first Search.

# approach: (heuristic-only priority)
    # GBFS always chhoses the node that looks closest to the goal
    # fringe: priority queueu(min_heap, sorted by heuristic h(n))
    # h(n) - straight line dist from curr node to goal node using dis_map
    # track - mark visited as we expand to avoid cycles
    # tie breaking? - 
def best_first_search(dis_map, time_map, start, end):
    """
    Greedy Best-first Search

    Args:
        time_map (dict): A map containing travel times between connected nodes (places or intersections), where every
        node is a dictionary key, and every value is an inner dictionary whose keys are the children of that node and
        values are travel times. Travel times are "null" for nodes that are not connected.
        dis_map (dict): A map containing straight-line (Euclidean) distances between every pair of nodes (places or
        intersections, connected or not), where every node is a dictionary key, and every value is an inner dictionary whose keys are the
        children of that node and values are straight-line distances.
        start (str): The name of the node from where to start traversal
        end (str): The name of the node where to end traversal

    Returns:
        visited (list): A list of visited nodes in the order in which they were visited
        path (list): The final path found by the search algorithm
    """
    if start == end:
        return [start], [start]

    visited = []
    visited_set = set()

    # heap entries are (h, node, path_so_far)
    heap = [(dis_map[start][end], start, [start])]

    while heap:
        _, node, path = heapq.heappop(heap)

        if node in visited_set:
            continue

        visited_set.add(node)
        visited.append(node)

        if node == end:
            return visited, path

        for neighbor in expand(node, time_map):
            if neighbor not in visited_set:
                h = dis_map[neighbor][end]
                heapq.heappush(heap, (h, neighbor, path + [neighbor]))

    return visited, []
        
# TO DO: Implement A* Search.

# approach: (cost-so-far + heuristic)
    # A* will match cost so far (g) with estimated cost to go to h
    # fringe: priority queue with key = f(n) = g(n) + h(n)
        # g(n) = sum of travel times so far (from time_map) 
        # h(n) = straight liune dist to goal (from dis_map)
    # tie breaking? - multiple nodes have same f, choose the one with the smaller g
    # if g also ties, choose the one expanded first (expand() order)
    #visited - a 'best g' dictionary so we don't lock out a better path

    # path with two routes: one with shorter in time but longer in distance vs one longer in time but shorter in distance - A* must choose the shorter path truly?
def a_star_search(dis_map, time_map, start, end):
    """
    A* Search

    Args:
        time_map (dict): A map containing travel times between connected nodes (places or intersections), where every
        node is a dictionary key, and every value is an inner dictionary whose keys are the children of that node and
        values are travel times. Travel times are "null" for nodes that are not connected.
        dis_map (dict): A map containing straight-line (Euclidean) distances between every pair of nodes (places or
        intersections, connected or not), where every node is a dictionary key, and every value is an inner dictionary whose keys are the
        children of that node and values are straight-line distances.
        start (str): The name of the node from where to start traversal
        end (str): The name of the node where to end traversal

    Returns:
        visited (list): A list of visited nodes in the order in which they were visited
        path (list): The final path found by the search algorithm
    """

    # A* keeps track of best cost to reach node so far + estimated future cost to get to goal
    # we will exoand the ide with the lowest f(n) = g(n) -> cost from start to n + h(n) -> estimated cost from n to goal
    if start == end:
        return [start], [start]

    pq = []
    tie_order = 0
    parents = {start: None}
    visited_order = []
    best_g = {start: 0} # best known cost so far

    #compute f(n) for start = g(start) + h(start) = 0 + heuristic
    h_start = dis_map[start][end]
    f_start = h_start # best cost so far (g(n)) at start is 0
    heapq.heappush(pq, (f_start, 0, tie_order, start))
    tie_order += 1

    while pq:
        # pop node with smallest f(n)=g(n) + h(n)
        f, g, _, parent = heapq.heappop(pq)

        # we want to skip if this entry is stale(g isnt the current best)
        if g != best_g.get(parent, float('inf')):
            continue
        
        visited_order.append(parent)

        #found goal? reconstruct path
        if parent == end:
            path = reconstruct_path(parents, start, end)
            return visited_order, path

        for neighbor in expand(parent, time_map):
            #commpute new cost so far if we go through parent
            g_new = g + time_map[parent][neighbor]

            #update best_g if this path is cheaper than any seen before
            if g_new < best_g.get(neighbor, float('inf')):
                best_g[neighbor] = g_new

                parents[neighbor] = parent

                # compute new_f = g_new + h
                f_new = g_new + dis_map[neighbor][end]

                heapq.heappush(pq, (f_new, g_new, tie_order, neighbor))
                tie_order += 1

    #no path
    return visited_order, []
            