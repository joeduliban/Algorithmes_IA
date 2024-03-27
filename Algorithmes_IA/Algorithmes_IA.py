#Planification de trajets pour les opérations de sauvetage(Ambulance)
#
#            11111111112222
#  012345678901234567890123
#
#0 ######################## 0
#1 G  ##  ####        ##  # 1
#2 #  ##  ####  ####  ##  # 2
#3 #  ##  ####  ####  ##  S 3
#4 #                      # 4
#5 ######################## 5

from collections import deque
import random
import heapq
#import tkinter

# Directions possibles : haut, bas, gauche, droite
directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

def heuristic(position, end):
    # Estimation du coût restant (ici, distance de Manhattan)
    return abs(position[0] - end[0]) + abs(position[1] - end[1])

def astar(start, end):
    frontier = [(0 + heuristic(start, end), 0, start, [])]
    visited = {start: None}
    
    while frontier:
        _, cost, current, path = heapq.heappop(frontier)
        if current == end:
            return creatPath(current, visited)
        for direction in directions:
            next_position = (current[0] + direction[0], current[1] + direction[1])
            # Vérifier si la prochaine position est valide
            if labyrinth[next_position[0]][next_position[1]] == ' ' and next_position not in visited:
                new_path = path + [current]
                new_cost = cost + 1  # Coût uniforme
                heapq.heappush(frontier, (new_cost + heuristic(next_position, end), new_cost, next_position, new_path))
                visited[next_position] = current
    

#def branch_and_bound(labyrinthe, start, end):

def uniform_cost_search(start, end):
    current = start
    frontier = [(0, [start])]
    visited = {start: None}
    
    while frontier and current != end:
        cost, path = heapq.heappop(frontier)
        if current == end:
            return creatPath(current, visited)
        for direction in directions:
            next_position = (current[0] + direction[0], current[1] + direction[1])
            # Vérifier si la prochaine position est valide
            if labyrinth[next_position[0]][next_position[1]] == ' ' and next_position not in visited:
                new_path = path + [next_position]
                new_cost = cost + 1  # Coût uniforme
                est_cost = new_cost + heuristic(next_position, end)
                heapq.heappush(frontier, (est_cost, new_path))
                visited[next_position] = current
                current = next_position
    
    # Si aucun chemin n'atteint l'objectif
    return creatPath(current, visited)

def beam_search(start, end, beam_width):   
    # Initialisation de la position actuelle
    current = start
    
    # Initialisation du beam avec le nœud de départ
    beam = [(0, [start])]
    visited = {start: None}
    
    while beam and current != end:
        beam.sort(key=lambda x: x[0])
        new_beam = []
        for cost, path in beam:
            for direction in directions:
                next_position = (path[-1][0] + direction[0], path[-1][1] + direction[1])
                # Vérifier si la prochaine position est valide
                if labyrinth[next_position[0]][next_position[1]] == ' ' and next_position not in visited:
                    new_path = path + [next_position]
                    new_beam.append((cost + 1, new_path))
                    visited[next_position] = current
                    current = next_position
        if current == end:
                return creatPath(current, visited)
        beam = new_beam[:beam_width]
    
    # Si aucun chemin n'atteint l'objectif
    return creatPath(current, visited)

def greedy_search(start, end):
    current = start
    visited = {start: None}
    
    while current != end:
        heuristiques = []
        
        for direction in directions:
            next_position = (current[0] + direction[0], current[1] + direction[1])
            # Calculer le coût heuristique (distance manhattan) jusqu'à l'objectif
            heuristique = abs(next_position[0] - end[0]) + abs(next_position[1] - end[1])
            heuristiques.append((heuristique, next_position))
        
        heuristiques.sort()
        
        for heuristique in heuristiques:
            min_heuristic, next_position = heuristique
            if labyrinth[next_position[0]][next_position[1]] == ' ':
                visited[next_position] = current    
                current = next_position
                break;
    
    return creatPath(current, visited)

def random_search(start, end):
    current = start
    visited = {start: None}
    
    while current != end:
        direction = random.choice(directions)
        next_position = (current[0] + direction[0], current[1] + direction[1])
        
        if 0 <= next_position[0] < len(labyrinth) and 0 <= next_position[1] < len(labyrinth[0]) \
                and labyrinth[next_position[0]][next_position[1]] != '#':
            if next_position not in visited:
                visited[next_position] = current
            current = next_position

    return creatPath(current, visited)

def bfs(start, end):
    queue = deque([start])
    visited = {start: None}
    current = None
    
    while queue and current != end:
        current = queue.popleft()

        for direction in directions:
            next_position = (current[0] + direction[0], current[1] + direction[1])
            
            if 0 <= next_position[0] < len(labyrinth) and 0 <= next_position[1] < len(labyrinth[0]) \
                    and labyrinth[next_position[0]][next_position[1]] != '#' and next_position not in visited:
                queue.append(next_position)
                visited[next_position] = current

    return creatPath(current, visited)

def creatPath(current, visited):
    path = []
    while current is not None:
        path.append(current)
        current = visited[current]
    path.reverse()
    return path

def printPath(name, path):
    print("\n"+name)
    for row in range(len(labyrinth)):
        for col in range(len(labyrinth[0])):
            if (row, col) in path:
                print('*', end=' ')
            else:
                print(labyrinth[row][col], end=' ')
        print()

labyrinth = [
    "########################",
    "   ##  ####        ##  #",
    "   ##  ####  ####  ##  #",
    "#  ##  ####  ####  ##   ",
    "#                       ",
    "#                      #",
    "########################"
]

start = (1, 0)
end = (4, 23)

printPath("BFS", bfs(start, end))
printPath("Random Search", random_search(start, end))
printPath("Greedy Search", greedy_search(start, end))
printPath("Beam Search", beam_search(start, end, 4))
printPath("Estimate-extended Uniform Cost", uniform_cost_search(start, end))
printPath("A*", astar(start, end))