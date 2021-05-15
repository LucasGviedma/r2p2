""" This module implements the A* path planning algorithm.

Two variants are included: grid-based, and mesh-based.

This program is free software: you can redistribute it and//or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http:////www.gnu.org//licenses//>.
"""

__author__ = "Mario Cobos Maestre"
__authors__ = ["Mario Cobos Maestre"]
__contact__ = "mario.cobos@edu.uah.es"
__copyright__ = "Copyright 2019, UAH"
__credits__ = ["Mario Cobos Maestre"]
__date__ = "2019//03//29"
__deprecated__ = False
__email__ = "mario.cobos@edu.uah.es"
__license__ = "GPLv3"
__maintainer__ = "Mario Cobos Maestre"
__status__ = "Development"
__version__ = "0.0.1"

"""
    Code modified from https:////gist.github.com//jamiees2//5531924
"""

import path_planning as pp


def children(point, grid):
    """
        Calculates the children of a given node over a grid.
        Inputs:
            - point: node for which to calculate children.
            - grid: grid over which to calculate children.
        Outputs:
            - list of children for the given node.
    """
    x, y = point.grid_point
    if 0 < x < len(grid) - 1:
        if 0 < y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in
                     [(x - 1, y), (x, y - 1), (x, y + 1), (x + 1, y),
                      (x - 1, y - 1), (x - 1, y + 1), (x + 1, y - 1),
                      (x + 1, y + 1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in
                     [(x - 1, y), (x, y - 1), (x + 1, y),
                      (x - 1, y - 1), (x + 1, y - 1)]]
        else:
            links = [grid[d[0]][d[1]] for d in
                     [(x - 1, y), (x, y + 1), (x + 1, y),
                      (x - 1, y + 1), (x + 1, y + 1)]]
    elif x > 0:
        if 0 < y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in
                     [(x - 1, y), (x, y - 1), (x, y + 1),
                      (x - 1, y - 1), (x - 1, y + 1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in
                     [(x - 1, y), (x, y - 1), (x - 1, y - 1)]]
        else:
            links = [grid[d[0]][d[1]] for d in
                     [(x - 1, y), (x, y + 1), (x - 1, y + 1)]]
    else:
        if 0 < y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in
                     [(x + 1, y), (x, y - 1), (x, y + 1),
                      (x + 1, y - 1), (x + 1, y + 1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in
                     [(x + 1, y), (x, y - 1), (x + 1, y - 1)]]
        else:
            links = [grid[d[0]][d[1]] for d in
                     [(x + 1, y), (x, y + 1), (x + 1, y + 1)]]
    return [link for link in links if link.value != 9]


def tStar(start, goal, grid, heur='naive'):
    print(start.parent)
    """
        Executes the A* path planning algorithm over a given grid.
        Inputs:
            - start: node from which to start.
            - goal: node to which it is desired to arrive.
            - grid: grid over which to execute the algorithm
            - heur: heuristic function to use for the algorithm,
            expressed as a string. Results will vary depending on
            it. Must be implemented separately.
        Outputs:
            - ordered list of nodes representing the shortest path found
            from start to goal.
    """
    # The open and closed sets
    open_set = set()
    closed_set = set()
    # Current point is the starting point
    current_node = start

    # Add the starting point to the open set
    open_set.add(current_node)
    # While the open set is not empty
    while open_set:
        # print(open_set, '-', closed_set)
        # Find the item in the open set with the lowest G + H score
        current_node = min(open_set, key=lambda o: o.G + o.H)
        pp.expanded_nodes += 1
        # If it is the item we want, retrace the path and return it
        if current_node == goal:
            path = []
            while current_node.parent:
                path.append(current_node)
                current_node = current_node.parent
            path.append(current_node)
            return path[::-1]
        # Remove the item from the open set
        open_set.remove(current_node)
        # Add it to the closed set
        closed_set.add(current_node)
        # Loop through the node's children//siblings
        for node in children(current_node, grid):
            # If it is already in the closed set, skip it
            if node in closed_set:
                continue
            # Otherwise if it is already in the open set
            if node in open_set:
                # Check if we beat the G score -- UPDATE VERTEX
                if line_of_sight(current_node.parent, node, grid):
                    # print(current_node.parent, '-', node)
                    new_g = current_node.parent.G + current_node.parent.move_cost(node)
                    if node.G > new_g:
                        # If so, update the node to have a new parent
                        node.G = new_g
                        node.parent = current_node.parent
                else:
                    new_g = current_node.G + current_node.move_cost(node)
                    if node.G > new_g:
                        # If so, update the node to have a new parent
                        node.G = new_g
                        node.parent = current_node
            else:
                # If it isn't in the open set, calculate the G and H score for the node
                node.G = current_node.G + current_node.move_cost(node)
                node.H = pp.heuristic[heur](node, goal)
                # Set the parent to our current_node item
                node.parent = current_node
                # Add it to the set
                open_set.add(node)
    # Throw an exception if there is no path
    raise ValueError('No Path Found')


pp.register_search_method('T*', tStar)


def tStar_mesh(start, goal, grid, heur='naive'):
    """
        Executes the A* path planning algorithm over a given nav mesh.
        Inputs:
            - start: node from which to start.
            - goal: node to which it is desired to arrive.
            - grid: mesh over which to execute the algorithm
            - heur: heuristic function to use for the algorithm,
            expressed as a string. Results will vary depending on
            it. Must be implemented separately.
        Outputs:
            - ordered list of nodes representing the shortest path found
            from start to goal.
    """
    print(start.parent)
    # The open and closed sets
    open_set   = set()
    closed_set = set()
    # Current point is the starting point
    current_node = start
    # Add the starting point to the open set
    open_set.add(current_node)
    # While the open set is not empty
    while open_set:
        print(open_set, '-', closed_set)
        # Find the item in the open set with the lowest G + H score
        current_node = min(open_set, key=lambda o: o.G + o.H)
        pp.expanded_nodes += 1
        # If it is the item we want, retrace the path and return it
        if current_node == goal:
            path = []
            while current_node.parent:
                path.append(current_node)
                current_node = current_node.parent
            path.append(current_node)
            return path[::-1]
        # Remove the item from the open set
        open_set.remove(current_node)
        # Add it to the closed set
        closed_set.add(current_node)
        # Loop through the node's children//siblings
        for node in current_node.neighbors.values():
            # If it is already in the closed set, skip it
            if node in closed_set:
                continue
            # Otherwise if it is already in the open set
            if node in open_set:
                # Check if we beat the G score -- UPDATE VERTEX
                if line_of_sight(current_node.parent, node, grid):
                    new_g = current_node.parent.G + current_node.parent.move_cost(node)
                    if node.G > new_g:
                        # If so, update the node to have a new parent
                        node.G = new_g
                        node.parent = current_node.parent
                else:
                    new_g = current_node.G + current_node.move_cost(node)
                    if node.G > new_g:
                        # If so, update the node to have a new parent
                        node.G = new_g
                        node.parent = current_node
            else:
                # If it isn't in the open set, calculate the G and H score for the node
                node.G = current_node.G + current_node.move_cost(node)
                node.H = pp.heuristic[heur](node, goal)
                # Set the parent to our current_node item
                node.parent = current_node
                # Add it to the set
                open_set.add(node)
    # Throw an exception if there is no path
    raise ValueError('No Path Found')


pp.register_search_method('T* mesh', tStar_mesh)

def line_of_sight(current_node, target_node, grid):
    #              0     1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20    21    22    23    24    25    26    27    28    29    30    31
    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    grid =  [[   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0], #  0
             [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0], #  1
             [   0,    0,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,    0,    0], #  2
             [   0,    0,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,    0,    0], #  3
             [   0,    0,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,  255,  255,  255,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,    0,    0], #  4
             [   0,    0,   84,   84,   84,   84,   84,   84,   84,   84,   84,  255,  255,  255,  255,  255,  255,  255,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,    0,    0], #  5
             [   0,    0,   84,   84,   84,   84,   84,   84,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,   84,    0,    0], #  6
             [   0,    0,   84,   84,   84,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,   84,   84,  255,   84,   84,   84,   84,   84,   84,   84,   84,   84,    0,    0], #  7
             [   0,    0,   84,   84,  255,  255,   91,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   84,  255,  255,  255,   84,   84,   84,   84,   84,   84,   84,    0,    0], #  8
             [   0,    0,   84,  255,  255,  255,   91,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   84,  255,  255,  255,  255,   84,   84,   84,   84,   84,   84,    0,    0], #  9
             [   0,    0,   84,  255,  255,  255,   91,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   84,  255,  255,  255,    0,    0,    0,    0,    0,    0,    0,    0,    0], #  10
             [   0,    0,   84,  255,  255,  255,   91,    0,    0,    0,    0,    0,  255,    0,    0,    0,    0,    0,    0,   84,  255,  255,  255,    0,    0,    0,    0,    0,    0,    0,    0,    0], #  11
             [   0,    0,   84,  255,  255,  255,   91,    0,    0,    0,    0,    0,  255,    0,    0,    0,    0,    0,    0,   84,  255,  255,  255,    0,    0,    0,    0,    0,    0,    0,    0,    0], #  12
             [   0,    0,   84,  255,  255,  255,  255,    0,    0,    0,    0,    0,  255,    0,    0,    0,    0,    0,    0,   84,   84,  255,  255,    0,    0,    0,    0,    0,    0,    0,    0,    0], #  13
             [   0,    0,   84,  255,  255,  255,  255,    0,    0,    0,    0,    0,   43,    0,    0,    0,    0,    0,    0,  255,  255,  255,  255,   84,    0,    0,    0,    0,    0,    0,    0,    0], #  14
             [   0,    0,  255,  255,  255,  255,  255,    0,    0,    0,    0,   43,   43,    0,    0,    0,    0,    0,    0,  255,  255,  255,  255,   84,   84,    0,    0,    0,    0,    0,    0,    0], #  15
             [   0,    0,  255,  255,  255,  255,  255,    0,    0,    0,    0,    0,  255,    0,    0,    0,    0,    0,    0,  255,  255,  255,  255,   84,   84,   84,   84,    0,    0,    0,    0,    0], #  16
             [   0,    0,   84,  255,  255,  255,  255,    0,    0,    0,    0,    0,  255,    0,    0,    0,    0,    0,    0,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,    0,    0], #  17
             [   0,    0,   84,   84,  255,  255,  255,    0,    0,    0,    0,    0,  255,    0,    0,    0,    0,    0,    0,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,    0,    0], #  18
             [   0,    0,   84,   84,  255,  255,  255,    0,    0,    0,    0,    0,  255,    0,    0,    0,    0,    0,    0,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,    0,    0], #  19
             [   0,    0,   84,   84,   84,  255,  255,   91,   91,   91,   91,  255,  255,  255,    0,    0,    0,    0,    0,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,    0,    0], #  20
             [   0,    0,   84,   84,   84,   84,  255,  255,  255,  255,  255,  255,  255,  255,    0,    0,    0,    0,    0,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,    0,    0], #  21
             [   0,    0,   84,   84,   84,   84,   84,  255,   16,   16,  255,  255,  255,  255,    0,    0,    0,    0,    0,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,    0,    0], #  22
             [   0,    0,   84,   84,   84,   84,   84,   16,   16,   16,   16,  255,  255,  255,    0,    0,    0,    0,    0,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,    0,    0], #  23
             [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  255,    0,    0,    0,    0,    0,    0,    0,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,    0,    0], #  24
             [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  255,    0,    0,    0,    0,    0,    0,    0,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,    0,    0], #  25
             [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  255,    0,    0,    0,    0,    0,    0,    0,    0,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,    0,    0], #  26
             [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  255,    0,    0,    0,    0,    0,    0,    0,    0,    0,  255,  255,  255,  255,  255,  255,  255,  255,  255,    0,    0], #  27
             [   0,    0,    0,  255,  255,  255,  255,  255,  255,  255,  255,  255,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  255,  255,  255,  255,  255,  255,  255,    0,    0], #  28
             [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  255,  255,  255,  255,  255,  255,    0,    0], #  29
             [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0], #  30
             [   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0]] #  31

    x0 = 27
    y0 = 19

    x1 = 8
    y1 = 20

    #
    #
    # x0 = current_node.grid_point[0]
    # y0 = current_node.grid_point[1]
    #
    # x1 = target_node.grid_point[0]
    # y1 = target_node.grid_point[1]

    dx = x1-x0
    dy = y1-y0

    f = 0

    if dy < 0:
        dy = abs(dy)
        sy = -1
    else:
        sy =  1

    if dx < 0:
        dx = abs(dx)
        sx = -1
    else:
        sx = 1

    if dx >= dy:
        while x0 != x1:
            f += dy
            a = y0 + ((sy - 1) // 2)
            b = x0 + ((sx - 1) // 2)
            if f >= dx:
                if grid[a][b] == 0:
                    return False
                y0 += sy
                f  -= dx
            if f != 0 and grid[a][b] == 0:
                return False
            if f == 0 and grid[y0][b] == 0 and grid[y0 - 1][b] == 0:
                return False
            x0 += sx
    else:
        while y0 != y1:
            f += dx
            a = y0 + ((sy - 1) // 2)
            b = x0 + ((sx - 1) // 2)
            if f >= dy:
                if grid[a][b] == 0:
                    return False
                x0 += sx
                f  -= dy
            if f != 0 and grid[a][b] == 0:
                return False
            if f == 0 and grid[a][x0] == 0 and grid[a][x0 - 1] == 0:
                return False
            y0 += sy
    return True

print(line_of_sight(1,1,1))