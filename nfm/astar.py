from shapely.geometry import Point
from warnings import warn
import heapq

# Original Source Code: https://gist.github.com/Nicholas-Swift/003e1932ef2804bebef2710527008f44


class Node:
    """
    A node class for A* Pathfinding
    """

    def __init__(self, parent=None, position=None, cost=None):
        self.parent = parent
        self.position = position
        self.cost = cost

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __repr__(self):
        return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
        return self.f < other.f

    # defining greater than for purposes of heap queue
    def __gt__(self, other):
        return self.f > other.f


def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path


def astar(maze, start, end, discretization, height_penalty=None,
          allow_diagonal_movement=True, boundary=None, len_penalty=5e-5):
    """
    Returns a list of tuples as a path from the given start to the given end in the given maze
    """

    # Create start and end node
    start_node = Node(None, start, maze[start[0], start[1]])
    end_node = Node(None, end, maze[end[0], end[1]])

    start_node.g = start_node.h = start_node.f = 0
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Heapify the open_list and Add the start node
    heapq.heapify(open_list)
    heapq.heappush(open_list, start_node)

    # Adding a stop condition
    outer_iterations = 0
    max_iterations = (len(maze[0]) * len(maze) // 2)
    # max_iterations = 1000000

    # Possible squares
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),)
    if allow_diagonal_movement:
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),)

    # Loop until you find the end
    while len(open_list) > 0:
        print("\rExploring grid {}".format(outer_iterations), end="")
        outer_iterations += 1

        if outer_iterations > max_iterations:
            # if we hit this point return the path such as it is
            # it will not contain the destination
            print("")
            warn("giving up on pathfinding too many iterations")
            return return_path(current_node)

        # Get the current node, r
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            print("")
            return return_path(current_node)

        # Generate children
        children = []
        for new_position in adjacent_squares:

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure position is within grid range
            if node_position[0] > (len(maze) - 1) or \
               node_position[0] < 0 or \
               node_position[1] > (len(maze[len(maze) - 1]) - 1) or \
               node_position[1] < 0:
                continue

            # Make sure exploration is within the folding workspace
            if boundary is not None and \
               not boundary.contains(Point(node_position[0]*discretization, node_position[1]*discretization)):
                continue

            # Create new node
            cost = maze[node_position[0], node_position[1]]
            new_node = Node(current_node, node_position, cost)
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            """ Currently, not using a heuristic. A* then degenerates to uniform cost search (UCS) """
            child.g = current_node.g + maze[child.position[0], child.position[1]] + len_penalty

            # Height penalties to avoid gripper collision with table
            if height_penalty is not None and child.position[1] * discretization < height_penalty:
                child.g += 100

            """ Euclidean heuristic. Does not work well due to manifold's non-linearity. """
            # curr_position = np.array([child.position[0], child.position[1]], dtype=np.int32)
            # end_position = np.array([end_node.position[0], end_node.position[1]], dtype=np.int32)
            # child.h = 0
            # while not np.all(curr_position == end_position):
            #     direction = end_position - curr_position
            #     direction = direction / np.linalg.norm(direction)
            #     direction = direction.reshape((1, 2))
            #     angle = np.rad2deg(angle_between(np.array([1.0, 0]), direction))
            #     curr_position = curr_position + get_move_dir(angle)
            #     child.h += maze[curr_position[0], curr_position[1]]
            #     child.h += len_penalty

            child.f = child.g + child.h

            # Child is already in the open list
            if len([open_node for open_node in open_list if
                    child.position == open_node.position and child.g > open_node.g]) > 0:
                continue

            # Add the child to the open list
            heapq.heappush(open_list, child)

    print("")
    warn("Couldn't get a path to destination")
    return None
