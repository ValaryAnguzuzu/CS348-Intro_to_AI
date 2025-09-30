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
        if cur == start:
            break
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

    pass

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

    pass

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

    pass
