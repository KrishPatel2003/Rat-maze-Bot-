import random
import numpy as np
import heapq
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional, Dict

#Ship 
class Ship:
    def __init__(self, dimension: int = 50, dead_end_factor: float = 0.5):
        self.D = dimension
        self.dead_end_ratio = dead_end_factor
        self.grid = np.zeros((self.D, self.D), dtype=int)
        self.generate_layout()

    def generate_layout(self): #start with grid of walls --> carve out open paths
        self.grid.fill(0)

        #random starting point
        sx = random.randint(1, self.D - 2)
        sy = random.randint(1, self.D - 2)
        self.grid[sx, sy] = 1


        #open cells from walls
        while True:
            frontier = []
            for x in range(1, self.D - 1):
                for y in range(1, self.D - 1):
                    if self.grid[x, y] == 0 and self._count_open_neighbors(x, y) == 1:
                        frontier.append((x, y))
            if not frontier:
                break
            cx, cy = random.choice(frontier)
            self.grid[cx, cy] = 1


        #add walls to dead ends
        total_ends = self._count_dead_ends()
        goal_ends = int(total_ends * self.dead_end_ratio)
        while self._count_dead_ends() > goal_ends:
            ends = self._list_dead_ends()
            if not ends:
                break
            ex, ey = random.choice(ends)
            walls = []
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                nx, ny = ex + dx, ey + dy
                if 0 <= nx < self.D and 0 <= ny < self.D and self.grid[nx, ny] == 0:
                    walls.append((nx, ny))
            if walls:
                wx, wy = random.choice(walls)
                self.grid[wx, wy] = 1


    def _count_open_neighbors(self, x: int, y: int) -> int:
        return sum(
            1 for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]
            if 0 <= x + dx < self.D and 0 <= y + dy < self.D and self.grid[x + dx, y + dy] == 1
        )


#Dead ends --> replanning 
    
    def _list_dead_ends(self):
        return [(i, j) for i in range(self.D) for j in range(self.D)
                if self.grid[i, j] == 1 and self._count_open_neighbors(i, j) == 1]

    def _count_dead_ends(self):  
        return len(self._list_dead_ends())

    def is_open(self, x, y): #check bounds + open cell
        return 0 <= x < self.D and 0 <= y < self.D and self.grid[x, y] == 1

    def get_open_cells(self):
        return [(i, j) for i in range(self.D) for j in range(self.D) if self.grid[i, j] == 1]

    def visualize(self, bot_pos=None, known=None):
        vis = np.zeros((self.D, self.D, 3), dtype=float)
        if known is not None:
            for i in range(self.D):
                for j in range(self.D):
                    if known[i, j] == 1:
                        vis[i, j] = [1, 1, 1]
                    elif known[i, j] == -1:
                        vis[i, j] = [0, 0, 0]
                    else:
                        vis[i, j] = [0.5, 0.5, 0.5]
        else:
            for i in range(self.D):
                for j in range(self.D):
                    vis[i, j] = [1, 1, 1] if self.grid[i, j] == 1 else [0, 0, 0]

        plt.figure(figsize=(6, 6))
        plt.imshow(vis, interpolation="nearest")
        if bot_pos:
            plt.scatter(bot_pos[1], bot_pos[0], c='r', s=50)
        plt.axis('off')
        plt.show()

#----------------------------------
#Bot 
class Bot:
    def __init__(self, ship, mode="basic"):
        self.ship = ship
        self.mode = mode
        self.position = random.choice(self.ship.get_open_cells()) #rendom position statrt
        self.known_grid = np.zeros((ship.D, ship.D), dtype=int) #start pos = known
        self.known_grid[self.position] = 1
        self._update_known_grid()

    def _heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) #manhattan distance

    def _find_path(self, goal): #shortest opath to goal

        #A* algo for pathfinding:
            #builds the open_set priority queue
            #calculates g_score and f_score
            #tracks the optimal path using came_from

        start = self.position
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}

        while open_set: #sgortest path using parent links
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]

            #explore neighbors

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.ship.is_open(*neighbor):
                    continue
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return [] #no path found
    

    #update known grid
    #get all 0 and open cells

    def _update_known_grid(self):
        x, y = self.position
        if self.mode == "sensory":
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.ship.D and 0 <= ny < self.ship.D:
                        self.known_grid[nx, ny] = 1 if self.ship.is_open(nx, ny) else -1
        else:
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.ship.D and 0 <= ny < self.ship.D:
                    self.known_grid[nx, ny] = 1 if self.ship.is_open(nx, ny) else -1

    def get_unidentified_cells(self):
        return [(i, j) for i in range(self.ship.D) for j in range(self.ship.D)
                if self.known_grid[i, j] == 0 and self.ship.is_open(i, j)]

#---------------------------------------
#Strategies
    
class ExplorationStrategy:
    def __init__(self, bot):
        self.bot = bot

    def get_next_target(self): #new target cell
        raise NotImplementedError

    def execute_strategy(self):
        total_moves = 0
        total_nodes = 0
        while True:
            target = self.get_next_target()  #chosen until all cells are known
            if not target:
                break

                #path finding using A*:
                    #get a path to the target
                    #move the bot
                    #update knowledge
                    #mark the target as blocked if unreachable
            

            path = self.bot._find_path(target)

            if path:
                #reachable = move the bot there and reveal neighbors

                total_moves += len(path)
                total_nodes += 1
                self.bot.position = target
                self.bot.known_grid[target] = 1
                self.bot._update_known_grid()
            else:
                #not reachable
                x, y = target
                self.bot.known_grid[x, y] = -1
        return total_moves, total_nodes

#1. 
    
class FarthestStrategy(ExplorationStrategy):
    def get_next_target(self):
        unk = self.bot.get_unidentified_cells()
        if not unk:
            return None
        return max(unk, key=lambda c: self.bot._heuristic(self.bot.position, c))

#2. 
    
class ClosestStrategy(ExplorationStrategy):
    def get_next_target(self):
        unk = self.bot.get_unidentified_cells()
        if not unk:
            return None
        return min(unk, key=lambda c: self.bot._heuristic(self.bot.position, c))

#3. 
     #Center Strategy:
        #prioritize cells near unknown areas
        #pick closer cells
        #pick cells that connect to known open space
        #penalty for possible dead ends
    

class CenterStrategy(ExplorationStrategy):
    def get_next_target(self):
        unk = self.bot.get_unidentified_cells()
        if not unk:
            return None
        
        def frontier_score(cell):
            x, y = cell
            unknown_neighbors = 0
            wall_neighbors = 0
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.bot.ship.D and 0 <= ny < self.bot.ship.D:
                        val = self.bot.known_grid[nx, ny]
                        if val == 0:
                            unknown_neighbors += 1
                        elif val == -1:
                            wall_neighbors += 1
            dist_to_center = self.bot._heuristic(cell, (self.bot.ship.D // 2, self.bot.ship.D // 2))
            return (-unknown_neighbors + wall_neighbors * 0.5, dist_to_center)
        return min(unk, key=frontier_score) #pick targewt cell closest to center based on manhattan dist
    

#Testing ship sizes: 
    #size 20 for sensory 
    #30 for basic with dead end factor = 0.8
    #ship = Ship(size=10, dead_end_factor=0.9) for basic

#-----------------------------------------
#Trials
    
    #RUNNING THE EXPERIMENT:
        #new ship each experiment 
        #multiple treials 
        #compare diff strategies
        #Both bot types

def run_experiment(size=50, dead_end_ratio=0.5, trials=5):
    results = {'basic': {}, 'sensory': {}}
    names = [('Farthest', FarthestStrategy), ('Closest', ClosestStrategy), ('Center', CenterStrategy)]
    for m in results:
        for n, _ in names:
            results[m][n] = []

    print("\nRunning 5 trails...hold pls :)") #Running each trial
    for t in range(trials):
        ship = Ship(size, dead_end_ratio)
        for mode in results:
            for name, Strat in names:
                bot = Bot(ship, mode)
                moves, nodes = Strat(bot).execute_strategy()
                results[mode][name].append((moves, nodes))

    summary = {m: {} for m in results}
    for mode in results:
        for name in results[mode]:
            runs = results[mode][name]
            avg_moves = sum(r[0] for r in runs) / len(runs)
            avg_nodes = sum(r[1] for r in runs) / len(runs)
            summary[mode][name] = (avg_moves, avg_nodes)
    return summary

def analyze_results(data):
    print("\nExploration Performance Results:")

    for mode in ['basic', 'sensory']:
        print(f"\n{mode.capitalize()} Bot:")
        for strat, (mv, nd) in data[mode].items():
            print(f"  {strat} strategy: {int(mv)} moves, {int(nd)} nodes")

    best_moves = min((data[m][s][0], m, s) for m in data for s in data[m])
    best_nodes = min((data[m][s][1], m, s) for m in data for s in data[m])

    print("\nFewest moves:", best_moves[1], "bot with", best_moves[2], "-", int(best_moves[0]), "moves")
    print("Fewest nodes:", best_nodes[1], "bot with", best_nodes[2], "-", int(best_nodes[0]), "nodes")


#-----------------REPORT ANALYSIS STATS: 
    
    #print("Exploration results: (Mode -- Strategy -- AvgMoves -- AvgNodes)")
    #for mode in ['basic', 'sensory']:
    #    for strat, (mv, nd) in data[mode].items():
    #        print(f"{mode.capitalize():<8} {strat:<8} {mv:>8.1f} {nd:>8.1f}")

   # best_moves = min((data[m][s][0], m, s) for m in data for s in data[m])
    #best_nodes = min((data[m][s][1], m, s) for m in data for s in data[m])
    #print("\n Fewest moves:", best_moves)
   # print("Fewest nodes:", best_nodes)


    #percentage improvements
    #print("\n improvement from basic to sensory:")
   # for strat in data['basic']:
    #    b_mv, b_nd = data['basic'][strat]
    #    s_mv, s_nd = data['sensory'][strat]
       # mv_imp = (b_mv - s_mv)/b_mv*100 if b_mv else 0
      #  nd_imp = (b_nd - s_nd)/b_nd*100 if b_nd else 0
      #  print(f"  {strat:<8}: {mv_imp:>5.1f}% fewer moves, {nd_imp:>5.1f}% fewer nodes")



 #Main method------------------------
if __name__ == "__main__":
    results = run_experiment(50, 0.5, 5)
    analyze_results(results)
