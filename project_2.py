# main environment 
# RUN THIS FILE 

import random
import numpy as np
import heapq
import matplotlib.pyplot as plt

#----------------------------------
# Ship - Same layout as project 1 
class Ship:
    def __init__(self, dimension: int = 50, dead_end_factor: float = 0.5):
        self.D = dimension
        self.dead_end_ratio = dead_end_factor
        self.grid = np.zeros((self.D, self.D), dtype=int)
        self.generate_layout()

    def generate_layout(self):
        self.grid.fill(0)
        sx = random.randint(1, self.D - 2)
        sy = random.randint(1, self.D - 2)
        self.grid[sx, sy] = 1

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

    def _list_dead_ends(self):
        return [(i, j) for i in range(self.D) for j in range(self.D)
                if self.grid[i, j] == 1 and self._count_open_neighbors(i, j) == 1]

    def _count_dead_ends(self):
        return len(self._list_dead_ends())

    def is_open(self, x, y):
        return 0 <= x < self.D and 0 <= y < self.D and self.grid[x, y] == 1

    def get_open_cells(self):
        return [(i, j) for i in range(self.D) for j in range(self.D) if self.grid[i, j] == 1]



#-----------------------------------------0--------------
# Basic Hard Sensor Bot
    
class HardSensorBot:
    def __init__(self, ship, rat_position, k):
        self.ship = ship
        self.position = random.choice(ship.get_open_cells())
        self.rat_position = rat_position
        self.k = k
        self.possible_cells = set(ship.get_open_cells())
        print(f"Rat is hidden at: {rat_position}")
        print(f"Basic Bot starting at: {self.position}\n")

    def manhattan(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def sense(self):
        return self.manhattan(self.position, self.rat_position) <= self.k

    def update_knowledge(self, beep):
        new_set = set()
        for cell in self.possible_cells:
            dist = self.manhattan(cell, self.position)
            if beep and dist <= self.k:
                new_set.add(cell)
            elif not beep and dist > self.k:
                new_set.add(cell)
        self.possible_cells = new_set

    def find_closest_possible(self):
        if not self.possible_cells:
            return None
        return min(self.possible_cells, key=lambda c: self.manhattan(self.position, c))

    def a_star_path(self, goal):
        open_set = [(0, self.position)]
        came_from = {}
        g = {self.position: 0}
        f = {self.position: self.manhattan(self.position, goal)}


        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)
                if not self.ship.is_open(nx, ny):
                    continue
                temp_g = g[current] + 1

                if neighbor not in g or temp_g < g[neighbor]:
                    came_from[neighbor] = current
                    g[neighbor] = temp_g
                    f[neighbor] = temp_g + self.manhattan(neighbor, goal)
                    heapq.heappush(open_set, (f[neighbor], neighbor))
        return []



    def run(self):
        moves = 0
        senses = 0
        while True:
            if self.position == self.rat_position:
                return moves, senses

            beep = self.sense()
            senses += 1
            if beep and self.position == self.rat_position:
                return moves, senses

            self.update_knowledge(beep)
            target = self.find_closest_possible()
            if not target:
                return moves, senses

            path = self.a_star_path(target)
            if not path:
                self.possible_cells.discard(target)
                continue


            for step in path:
                self.position = step
                moves += 1
                if self.position == self.rat_position:
                    return moves, senses

#----------------------------------
# InfoGain sdtrategy bot improved: 
                             
#Start Randomly

#Maintain Belief

#Sense:(based on Manhattan distance k); get a yes/no beep: whether the rat is within range.

#Info Gain Calculation: for each trget cell estimates how much sensing there would help reduce uncertainty.

#Path Planning: A*

#Repeat


class InfoGainHardBot:
    def __init__(self, ship, rat_position, k):
        self.senses = 0  # count how many times the bot sensed
        self.ship = ship
        self.k = 6  #  12, 13, 15
        self.position = random.choice(ship.get_open_cells())
        self.rat_position = rat_position
        self.possible_cells = set(ship.get_open_cells())
        self.visited_targets = set()  # track visited targets
        # print(f"Rat is hidden at: {rat_position}")
        print(f"Improved Bot starting at: {self.position}\n")


    def manhattan(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])


    def sense(self):
        return self.manhattan(self.position, self.rat_position) <= self.k


    def update_knowledge(self, beep):
        new_set = set()
        for cell in self.possible_cells:
            dist = self.manhattan(cell, self.position)
            if (beep and dist <= self.k) or (not beep and dist > self.k):
                new_set.add(cell)
        self.possible_cells = new_set



    def expected_info_gain(self, cell):
        in_range = sum(1 for c in self.possible_cells if self.manhattan(cell, c) <= self.k)
        out_range = len(self.possible_cells) - in_range
        gain = in_range * out_range
        dist = self.manhattan(self.position, cell) + 1  # +1 to avoid division by zero
        return gain / dist  #high info + low cost


    def find_best_sensing_cell(self):
        candidates = [c for c in self.possible_cells if c not in self.visited_targets]
        if not candidates:
            return self.position  # stay if nothing new
        return max(candidates, key=self.expected_info_gain)

    def a_star_path(self, goal):
        open_set = [(0, self.position)]
        came_from = {}
        g = {self.position: 0}
        f = {self.position: self.manhattan(self.position, goal)}


        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]
            


            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)
                if not self.ship.is_open(nx, ny):
                    continue
                temp_g = g[current] + 1

                if neighbor not in g or temp_g < g[neighbor]:
                    came_from[neighbor] = current
                    g[neighbor] = temp_g
                    f[neighbor] = temp_g + self.manhattan(neighbor, goal)
                    heapq.heappush(open_set, (f[neighbor], neighbor))
        return []

    def run(self):
        steps = 0
        max_steps = 3000  # infinite loop debug
        while steps < max_steps:
            if self.position == self.rat_position:
               print("Improved Hard Sensor Bot found the space rat in", steps, "steps.")
               return steps, self.senses

            beep = self.sense()
            self.senses += 1
            self.update_knowledge(beep)

            if not self.possible_cells:
                print("No possible rat locations left!")
                return steps, self.senses

            target = self.find_best_sensing_cell()
            self.visited_targets.add(target)
            if self.manhattan(self.position, target) > 10:
                self.possible_cells.discard(target)
                continue


            if target == self.position:
                continue



            path = self.a_star_path(target)
            if not path:
                self.possible_cells.discard(target)
                continue

            warned = False
            for step in path:
                self.position = step
                steps += 1
                if steps > 200 and not warned:
                    print("Warning: exceeded 200 steps.")
                    warned = True



                if self.position == self.rat_position:
                    print("Improved Hard Sensor Bot found the space rat in", steps, "steps.")
                    return steps, self.senses
        return steps, self.senses


# PLEASE NOTE: BOTH PLOTS HAVE BEEN MOVED TO THE END COMMENTS FOR FASTER RUN
#restored
    
def compare_hard_sensor_strategies(trials= 7): #7 trials 
    k = 6
    total_moves = 0
    total_senses = 0
    total_steps = 0
    total_improved_senses = 0


    print(f"\nRunning {trials} trials, please hold :)")  

    for _ in range(trials):
        ship = Ship(30, 0.8)
        rat = random.choice(ship.get_open_cells())

        # turn off printing 
        original_print = __builtins__.print
        __builtins__.print = lambda *args, **kwargs: None

        basic_bot = HardSensorBot(ship, rat, k)
        moves, senses = basic_bot.run()
        total_moves += moves
        total_senses += senses

        improved_bot = InfoGainHardBot(ship, rat, k)
        steps, senses = improved_bot.run()
        total_steps += steps
        total_improved_senses += senses



        # turn print on
        __builtins__.print = original_print

    avg_moves = total_moves / trials
    avg_senses = total_senses / trials
    avg_total_basic = avg_moves + avg_senses
    avg_improved = total_steps / trials
    avg_improved_senses = total_improved_senses / trials
    avg_total_improved = avg_improved + avg_improved_senses


    # final outpuut
    print("\nStrategy Comparison")
    print(f"Basic Bot:     {avg_moves:.1f} steps, {avg_senses:.1f} senses = {avg_total_basic:.1f} total")
    print(f"Improved Bot:  {avg_improved:.1f} steps, {avg_improved_senses:.1f} senses = {avg_total_improved:.1f} total\n")
    
#--------HARD BOT MULTIPLE TRIALS + PLOT -----------------------------\

#PLOT
def plot_hard_sensor_metrics(k_values, basic_moves, basic_senses, improved_steps, improved_senses):
    """
    Plots 3 performance graphs for Hard Sensor Bots:
    a) Average moves vs k
    b) Average senses vs k
    c) Average total actions (moves + senses) vs k
    """
    basic_total = [m + s for m, s in zip(basic_moves, basic_senses)]
    improved_total = [s + m for s, m in zip(improved_steps, improved_senses)]

    plt.figure(figsize=(18, 4))

    # a) Moves vs k
    plt.subplot(1, 3, 1)
    plt.plot(k_values, basic_moves, marker='o', label='Basic Bot Moves')
    plt.plot(k_values, improved_steps, marker='s', label='Improved Bot Steps')
    plt.title("Avg Moves / Steps vs k")
    plt.xlabel("k (Sensor Radius)")
    plt.ylabel("Avg Moves / Steps")
    plt.legend()
    plt.grid(True)

    # b) Senses vs k
    plt.subplot(1, 3, 2)
    plt.plot(k_values, basic_senses, marker='o', label='Basic Bot Senses')
    plt.plot(k_values, improved_senses, marker='s', label='Improved Bot Senses')
    plt.title("Avg Senses vs k")
    plt.xlabel("k (Sensor Radius)")
    plt.ylabel("Avg Senses")
    plt.legend()
    plt.grid(True)

    # c) Total Actions vs k
    plt.subplot(1, 3, 3)
    plt.plot(k_values, basic_total, marker='o', label='Basic Bot Total Actions')
    plt.plot(k_values, improved_total, marker='s', label='Improved Bot Total Actions')
    plt.title("Avg Total Actions vs k")
    plt.xlabel("k (Sensor Radius)")
    plt.ylabel("Avg Total (Moves + Senses)")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


#TRIALs----------------------------
def run_hard_bot_k_sweep():
    k_values = list(range(1, 20))
    trials = 3

    basic_moves_list = []
    basic_senses_list = []
    improved_steps_list = []
    improved_senses_list = []

    for k in k_values:
        print(f"\n--- Running k = {k} ---")
        total_moves = 0
        total_senses = 0
        total_steps = 0
        total_improved_senses = 0

        for _ in range(trials):
            ship = Ship(30, 0.8)
            rat = random.choice(ship.get_open_cells())

            bot1 = HardSensorBot(ship, rat, k)
            moves, senses = bot1.run()
            total_moves += moves
            total_senses += senses

            bot2 = InfoGainHardBot(ship, rat, k)
            steps, senses2 = bot2.run()
            total_steps += steps
            total_improved_senses += senses2

        basic_moves_list.append(total_moves / trials)
        basic_senses_list.append(total_senses / trials)
        improved_steps_list.append(total_steps / trials)
        improved_senses_list.append(total_improved_senses / trials)

        print(f"k = {k} | Basic: {total_moves / trials:.1f} total | Improved: {total_steps / trials + total_improved_senses / trials:.1f} total")

    plot_hard_sensor_metrics(k_values, basic_moves_list, basic_senses_list, improved_steps_list, improved_senses_list)

# ----------------------------------------------------------------------
# Soft Sensor Bots

# A* algo
def astar_path(ship, start, goal):
    """Standard A*: returns full path from start to goal."""
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: abs(start[0] - goal[0]) + abs(start[1] - goal[1])}
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        for dx, dy in [(0,1),(1,0),(0,-1),(-1,0)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if not ship.is_open(*neighbor):
                continue
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + abs(neighbor[0] - goal[0]) + abs(neighbor[1] - goal[1])
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return []



class SoftSensorBot:
    def __init__(self, ship, alpha=0.5):
        self.ship = ship
        self.alpha = alpha
        self.position = random.choice(ship.get_open_cells())
        self.trail = [self.position]
        self.D = ship.D
        self.open_mask = (ship.grid == 1)
        self.belief = np.zeros((self.D, self.D))
        self.belief[self.open_mask] = 1.0
        self.normalize()
        self.moves = 0
        self.senses = 0


    def normalize(self):
        total = self.belief.sum()
        if total > 0:
            self.belief /= total

    def manhattan(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def sensor_update(self, beep):
        self.senses += 1
        x, y = self.position
        xs, ys = np.ogrid[0:self.D, 0:self.D]
        dist = np.abs(xs - x) + np.abs(ys - y)
        p = np.exp(-self.alpha * (dist - 1))
        if beep:
            self.belief[self.open_mask] *= p[self.open_mask]
        else:
            self.belief[self.open_mask] *= (1 - p[self.open_mask])
        self.belief[~self.open_mask] = 0
        self.normalize()



    def at_target(self, target):
        """Return True if bot is at the target location."""
        return self.position == target

    def move_guess(self, rat):
        # pick highest belief cell
        best = tuple(np.unravel_index(self.belief.argmax(), self.belief.shape))
        if best == self.position:

            # if stucjk: random neighbor step 
            neighbors = [
                (self.position[0]+dx, self.position[1]+dy)
                for dx,dy in [(0,1),(1,0),(0,-1),(-1,0)]
                if self.ship.is_open(self.position[0]+dx, self.position[1]+dy)
            ]
            if not neighbors:
                return 0
            step = random.choice(neighbors)
            self.position = step
            self.trail.append(step)
            self.moves += 1
            return 1
        
        
        full_path = astar_path(self.ship, self.position, best) # Get A* path to best cell, take up to 5 steps
        path = full_path[:5]
        for p in path:
            self.position = p
            self.trail.append(p)
            self.moves += 1
            if self.position == rat:
                break
        return len(path)
    
    def run(self, rat, max_steps= 2500):
        steps = 0
        d0 = self.manhattan(self.position, rat)
        beep = random.random() < (np.exp(-self.alpha*(d0-1)) if d0>0 else 1)
        self.sensor_update(beep)


        # main loop----------------------------------

        while self.position != rat and steps < max_steps:
            # if confident, move straight to rat
            if self.belief[rat] > 0.85:
                path = astar_path(self.ship, self.position, rat)
                for p in path:
                    if steps >= max_steps:
                        break
                    self.position = p
                    self.trail.append(p)
                    self.moves += 1
                    steps += 1
                    d = self.manhattan(p, rat)
                    beep = random.random() < np.exp(-self.alpha*(d-1))
                    self.sensor_update(beep)
                break
            moved = self.move_guess(rat)
            steps += max(1, moved)
          
          #sense aftewr each move
            d = self.manhattan(self.position, rat)
            beep = random.random() < np.exp(-self.alpha*(d-1))
            self.sensor_update(beep)


        if steps >= max_steps:
            print("SoftSensorBot: max_steps reached, stopping.")
        print(f"SoftSensorBot → Moves={self.moves}, Senses={self.senses}, Steps={steps}")
        return self.moves, self.senses, steps

#$-------------------

#InfoGainSoftBot
class InfoGainSoftBot(SoftSensorBot):
    def __init__(self, ship, alpha=0.5):
        super().__init__(ship, alpha)
        self.visited = set()

    def exp_gain(self, cell):
        xs, ys = np.ogrid[0:self.D, 0:self.D]
        d = np.abs(xs - cell[0]) + np.abs(ys - cell[1])
        p = np.exp(-self.alpha * (d - 1))
        q = 1 - p
        ent = -p * np.log(np.clip(p, 1e-10, 1)) - q * np.log(np.clip(q, 1e-10, 1))
        return np.sum(self.belief[self.open_mask] * ent[self.open_mask])

    def find_best(self):
        flat = self.belief.flatten()
       
        topk = np.argpartition(-flat, 50)[:50]  #highest expected information gain top 50
        cands = [tuple(divmod(i, self.D)) for i in topk
                 if self.manhattan(self.position, tuple(divmod(i, self.D))) <= 5]
        if not cands:
            return self.position
        
        return max([(self.exp_gain(c) / (1 + self.manhattan(self.position, c)), c) # Filter out already visited cells
                    for c in cands])[1]

    def move_guess(self, rat):
        best = self.find_best()
        if best == self.position:
            # Random neighbor step if stuck
            neighbors = [
                (self.position[0] + dx, self.position[1] + dy)
                for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]
                if self.ship.is_open(self.position[0] + dx, self.position[1] + dy)
            ]
            if not neighbors:
                return 0
            step = random.choice(neighbors)
            self.position = step
            self.trail.append(step)
            self.moves += 1
            return 1


        full_path = astar_path(self.ship, self.position, best) # Get A* path to best cell, take up to 5 steps
        path = full_path[:5]
        for p in path:
            self.position = p
            self.trail.append(p)
            self.moves += 1
            if self.position == rat:
                break
        return len(path)
    

    def run(self, rat, max_steps=2500):
        steps = 0
        d0 = self.manhattan(self.position, rat)
        beep = random.random() < (np.exp(-self.alpha * (d0 - 1)) if d0 > 0 else 1)
        self.sensor_update(beep)

        #Main loop
        while self.position != rat and steps < max_steps:
            if self.belief[rat] > 0.5:
                # If confident, move straight to rat
                path = astar_path(self.ship, self.position, rat)
                for p in path:
                    if steps >= max_steps:
                        break
                    self.position = p
                    self.trail.append(p)
                    self.moves += 1
                    steps += 1
                    d = self.manhattan(p, rat)
                    beep = random.random() < np.exp(-self.alpha * (d - 1))
                    self.sensor_update(beep)
                break
            moved = self.move_guess(rat)
            steps += max(1, moved)
            # Sense after each move
            d = self.manhattan(self.position, rat)
            beep = random.random() < np.exp(-self.alpha * (d - 1))
            self.sensor_update(beep)

        if steps >= max_steps:
            print("InfoGainSoftBot: max_steps reached.")
            print(f"InfoGainSoftBot → Moves={self.moves}, Senses={self.senses}, Steps={steps}")
        return self.moves, self.senses, steps
    
# -----SOFT PLOT--------
    
    # soft bot plot 

def plot_soft_sensor_metrics(trials, bm, bs, im, is_, bsteps, isteps):
    total_b = [m + s for m, s in zip(bm, bs)]
    total_i = [m + s for m, s in zip(im, is_)]
    fig, ax = plt.subplots(1, 4, figsize=(24, 4))
    
    ax[0].plot(trials, bm, 'o-', label='Basic')
    ax[0].plot(trials, im, 's-', label='InfoGain')
    ax[0].set_title('Moves')
    ax[0].legend()
    ax[0].grid()
    

    ax[1].plot(trials, bs, 'o-', label='Basic')
    ax[1].plot(trials, is_, 's-', label='InfoGain')
    ax[1].set_title('Senses')
    ax[1].legend()
    ax[1].grid()
    

    ax[2].plot(trials, total_b, 'o-', label='Basic')
    ax[2].plot(trials, total_i, 's-', label='InfoGain')
    ax[2].set_title('Total (Moves+Senses)')
    ax[2].legend()
    ax[2].grid()


    ax[3].plot(trials, bsteps, 'o-', label='Basic')
    ax[3].plot(trials, isteps, 's-', label='InfoGain')
    ax[3].set_title('Steps')
    ax[3].legend()
    ax[3].grid()

    plt.tight_layout()
    plt.show()

#-----------Soft sensor comparison -------------0-----------
def compare_soft_sensor_strategies(trials=7, alpha=0.5, max_steps=2500):
    print(f"Running {trials} trials for Soft Sensor Bots (alpha={alpha}):\n")
    
    basic_moves, basic_senses, basic_steps = [], [], []
    info_moves, info_senses, info_steps = [], [], []

    for trial in range(1, trials + 1):
        ship = Ship()
        rat = random.choice(ship.get_open_cells())
        print(f"[Trial {trial}] Rat at {rat}")

        # Basic Soft Sensor Bot
        basic_bot = SoftSensorBot(ship, alpha=alpha)
        bm, bs, bst = basic_bot.run(rat, max_steps=max_steps)
        if bst >= max_steps:
            print("SoftSensorBot: max_steps reached, stopping.")
        print(f"SoftSensorBot → Moves={bm}, Senses={bs}, Steps={bst}")
        basic_moves.append(bm)
        basic_senses.append(bs)
        basic_steps.append(bst)

        # InfoGain Soft Sensor Bot
        info_bot = InfoGainSoftBot(ship, alpha=alpha)
        im, is_, ist = info_bot.run(rat, max_steps=max_steps)
        if ist >= max_steps:
            print("InfoGainSoftBot: max_steps reached.")
        print(f"InfoGainSoftBot → Moves={im}, Senses={is_}, Steps={ist}")

        print(f"  Basic Soft → Moves={bm}, Senses={bs}, Steps={bst}")
        print(f"  InfoGain Soft → Moves={im}, Senses={is_}, Steps={ist}\n")

        info_moves.append(im)
        info_senses.append(is_)
        info_steps.append(ist)
        # 
        #average
    avg_bm = round(sum(basic_moves) / trials, 1)
    avg_bs = round(sum(basic_senses) / trials, 1)
    avg_bst = round(sum(basic_steps) / trials, 1)
    avg_im = round(sum(info_moves) / trials, 1)
    avg_is = round(sum(info_senses) / trials, 1)
    avg_ist = round(sum(info_steps) / trials, 1)

    print(f"Averages: BasicMoves({avg_bm}), BasicSenses({avg_bs}), BasicSteps({avg_bst})")
    print(f"InfoMoves({avg_im}), InfoSenses({avg_is}), InfoSteps({avg_ist})")
    
    plot_soft_sensor_metrics(
    list(range(1, trials + 1)),
    basic_moves, basic_senses, info_moves, info_senses,
    basic_steps, info_steps
    )

#--------------------------------------------------
# Main
if __name__ == "__main__":
    compare_hard_sensor_strategies()
    compare_soft_sensor_strategies()
    print("\nGenerating hard bot plot... This may take a few minutes ⏳")
    run_hard_bot_k_sweep()


#----------THE END OMG------------------------------------------------