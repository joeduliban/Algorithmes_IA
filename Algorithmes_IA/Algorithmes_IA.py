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
    frontier = [(0, 0, start, [])]
    visited = {start: None}
    cost_so_far = {start: 0}
    
    while frontier:
        _, cost, current, path = heapq.heappop(frontier)
        if current == end:
            return creatPath(current, visited)
        
        for direction in directions:
            next_position = (current[0] + direction[0], current[1] + direction[1])
            if 0 <= next_position[0] < len(labyrinth) and 0 <= next_position[1] < len(labyrinth[0]) \
                    and labyrinth[next_position[0]][next_position[1]] != '#' and next_position not in visited:
                new_path = path + [current]
                new_cost = cost_so_far[current] + 1
                if next_position not in cost_so_far or new_cost < cost_so_far[next_position]:
                    cost_so_far[next_position] = new_cost
                    priority = new_cost + heuristic(next_position, end)
                    heapq.heappush(frontier, (priority, new_cost, next_position, new_path))
                    visited[next_position] = current
    
    return creatPath(current, visited)
    

def branch_and_bound(start, end):
    # Crée une queue pour stocker les nœuds à explorer
    queue = [(0, [start], start)]

    # Crée un dictionnaire pour stocker les coûts et les visiteurs
    costs = {start: 0}
    visited = {start: None}

    while queue:
        # Dépile le nœud avec le coût le plus bas
        cost, path, current = heapq.heappop(queue)

        # Si nous avons atteint l'objectif, retourne le chemin
        if current == end:
            return creatPath(current, visited)

        # Explore les voisins du nœud courant
        for direction in directions:
            next_position = (current[0] + direction[0], current[1] + direction[1])
            if 0 <= next_position[0] < len(labyrinth) and 0 <= next_position[1] < len(labyrinth[0]) \
                    and labyrinth[next_position[0]][next_position[1]] != '#':
                new_path = path + [next_position]
                new_cost = costs[current] + 1
                if next_position not in costs or new_cost < costs[next_position]:
                    costs[next_position] = new_cost
                    visited[next_position] = current
                    heapq.heappush(queue, (new_cost + heuristic(next_position, end), new_path, next_position))

    # Si nous n'avons pas trouvé de chemin, retourne le chemin bloqué
    return creatPath(current, visited)

def uniform_cost_search(start, end):
    frontier = [(0, [start])]
    visited = {start: None}
    cost_so_far = {start: 0}
    
    while frontier:
        cost, path = heapq.heappop(frontier)
        current = path[-1]
        if current == end:
            return creatPath(current, visited)
        
        for direction in directions:
            next_position = (current[0] + direction[0], current[1] + direction[1])
            if 0 <= next_position[0] < len(labyrinth) and 0 <= next_position[1] < len(labyrinth[0]) \
                    and labyrinth[next_position[0]][next_position[1]] != '#' and next_position not in visited:
                new_path = path + [next_position]
                new_cost = cost_so_far[current] + 1
                if next_position not in cost_so_far or new_cost < cost_so_far[next_position]:
                    cost_so_far[next_position] = new_cost
                    heapq.heappush(frontier, (new_cost, new_path))
                    visited[next_position] = current
    
    return creatPath(current, visited)

def beam_search(start, end, beam_width):
    beam = [[start]]
    visited = {start: None}
    
    while beam:
        new_beam = []
        for path in beam:
            current = path[-1]
            if current == end:
                return creatPath(current, visited)
            for direction in directions:
                next_position = (current[0] + direction[0], current[1] + direction[1])
                if 0 <= next_position[0] < len(labyrinth) and 0 <= next_position[1] < len(labyrinth[0]) \
                        and labyrinth[next_position[0]][next_position[1]] != '#' and next_position not in visited:
                    new_path = path + [next_position]
                    new_beam.append(new_path)
                    visited[next_position] = current
        new_beam.sort(key=lambda x: len(x))
        beam = new_beam[:beam_width]
    
    return creatPath(current, visited)

def greedy_search(start, end):
    current = start
    path = [current]
    visited = {current: None}
    
    while current != end:
        best_score = float('inf')
        best_next = None
        for direction in directions:
            next_position = (current[0] + direction[0], current[1] + direction[1])
            if 0 <= next_position[0] < len(labyrinth) and 0 <= next_position[1] < len(labyrinth[0]) \
                    and labyrinth[next_position[0]][next_position[1]] != '#' and next_position not in visited:
                score = heuristic(next_position, end)
                if score < best_score:
                    best_score = score
                    best_next = next_position
        path.append(best_next)
        visited[best_next] = current
        current = best_next
    
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
    ['#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'],
    [' ', ' ', '#', '#', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '],
    [' ', ' ', ' ', ' ', '#', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '],
    ['#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', '#', '#', ' '],
    ['#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', '#', ' ', ' '],
    ['#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' '],
    ['#', ' ', ' ', '#', ' ', ' ', '#', ' ', ' ', ' ', '#', ' ', ' ', ' '],
    ['#', ' ', ' ', '#', ' ', ' ', '#', ' ', '#', ' ', ' ', '#', ' ', ' '],
    ['#', ' ', ' ', '#', ' ', ' ', '#', ' ', ' ', '#', ' ', ' ', '#', ' '],
    ['#', ' ', ' ', '#', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' '],
    ['#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#']
]

start = (1, 0)
end = (9, 13)

printPath("BFS", bfs(start, end))
printPath("Random Search", random_search(start, end))
printPath("Greedy Search", greedy_search(start, end))
printPath("Beam Search", beam_search(start, end, 4))
printPath("Estimate-extended Uniform Cost", uniform_cost_search(start, end))
printPath("Branch & Bound", branch_and_bound(start, end))
printPath("A*", astar(start, end))