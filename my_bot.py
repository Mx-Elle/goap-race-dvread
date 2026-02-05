import random
from game_world.racetrack import RaceTrack
import heapq
from typing import Callable

import pygame
import pygame.locals
import sys
from collections import defaultdict


Point = tuple[int, int]

def reconstruct_path(cameFrom, current, end):
    total_path =[]
    current = end
    while current != None :
        total_path.append(current)
        current = cameFrom[current]
    return total_path[::-1]

def astar(start_point, end_point, track: RaceTrack) -> list[Point] | None:
    """
    Find the shortest path between two cells using the A* pathfinding algorithm
    Do not access any variable that starts with an underscore, use the properties instead.
    You can access a cell's coordinates with cell.coord and its neighbors with cell.neighbors.
    If you need to get the distance between two cells, use cell.distance(other_cell)

    Args:
        start_cell (NavMeshCell): The starting location
        end_cell (NavMeshCell): The ending location

    Returns:
        list[NavMeshCell] | None: A list of NavMeshCells (representing the shortest path) or None
            if there is no valid path between the two locations.
    """
    # TODO: Replace this dummy return with the correct code!
    open = []
    
    heapq.heappush(open,((abs(start_point[0]-end_point[0])+abs(start_point[1]-end_point[1])), start_point))

    cameFrom =  {start_point: None}


    gScore = defaultdict(lambda: float("inf"))
    gScore[start_point]=0



    while open !=[]:
        current = heapq.heappop(open)[1]
        if current == end_point:
            return reconstruct_path(cameFrom, current, end_point)
        safe = track.find_traversable_cells()
        options = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = {opt: (current[0] + opt[0], current[1] + opt[1]) for opt in options}
        safe_options = [neighbors[opt] for opt in neighbors if neighbors[opt] in safe]
        for neighbor in safe_options:
            temp_gScore = gScore[current] + (abs(current[0]-neighbor[0])+abs(current[1]-neighbor[1]))
            if temp_gScore < gScore[neighbor]:
                cameFrom[neighbor] = current
                gScore[neighbor] = temp_gScore
                if not any(neighbor in sublist for sublist in open):
                    heapq.heappush(open, ((temp_gScore + (abs(neighbor[0]-end_point[0])+abs(neighbor[1]-end_point[1]))), neighbor))

    return None

def random_move(loc: Point, track: RaceTrack) -> Point:
   #print(track.get_grid_coord(loc[0], loc[1]))
    safe = track.find_traversable_cells()
    options = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = {opt: (loc[0] + opt[0], loc[1] + opt[1]) for opt in options}
    safe_options = [opt for opt in neighbors if neighbors[opt] in safe]
    #print(astar(loc, track.target, track))
    return astar(loc, track.target, track)[0]

#astar through game states