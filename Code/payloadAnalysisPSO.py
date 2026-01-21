import math 
import random
import matplotlib.pyplot as plt
import kinematics as kin
import torqueAnalysis as ta

def force_magnitude(footholds): # footholds = 12x1 array of footholds in hip frame in cm

    num_footholds = len(footholds) // 3

    for leg in range(num_footholds):
        x, y, z = footholds[leg*3:leg*3+3]
        try:
            kin.inv_kin(x, y, z, leg)
        except Exception:
            # print(f"Kinematic constraint violated for leg {leg} at foothold ({x}, {y}, {z})")
            return float('inf')
        
    footholds_b = [ta.leg_to_body(leg, footholds[leg*3:leg*3+3]) for leg in range(num_footholds)]
    xy_legs = [(footholds_b[leg][0], footholds_b[leg][1]) for leg in range(num_footholds)]

    if num_footholds == 3:
        F_legs = ta.compute_forces(xy_legs[0], xy_legs[1], xy_legs[2])
        F_mag = math.sqrt(F_legs[0]**2 + F_legs[1]**2 + F_legs[2]**2)
    else: 
        F_legs = ta.compute_forces(xy_legs[0], xy_legs[1], xy_legs[2], xy_legs[3])
        F_mag = math.sqrt(F_legs[0]**2 + F_legs[1]**2 + F_legs[2]**2 + F_legs[3]**2)

    return -F_mag # we want to maximize the force, so return negative

def init_population(pop_size, bounds):

    dim = len(bounds) # the number of bounds is the dimension of the input, e.g. for a 3D function, there would be 2 bounds on the input
    positions = []
    velocities = []

    for _ in range(pop_size):
        position = [random.uniform(bounds[d][0], bounds[d][1]) for d in range(dim)] # uniformly generate a position in the bounds
        velocity = [random.uniform(-1, 1) for _ in range(dim)] # between -1 and 1 so there is no velocity such that it always goes out of bounds no matter the position, doesn't make much difference
        positions.append(position)
        velocities.append(velocity)

    return positions, velocities

def update(positions, velocities, p_best, g_best, w, c1, c2, bounds):

    dim = len(bounds)

    for i in range(len(positions)):
        for d in range(dim): 
            
            positions[i][d] += velocities[i][d]
            
            # since the bounds are exclusive, offset the position by a small amount to ensure it is within the bounds
            if positions[i][d] < bounds[d][0]:
                positions[i][d] = bounds[d][0] + 0.001 
            elif positions[i][d] > bounds[d][1]:
                positions[i][d] = bounds[d][1] - 0.001
            
            r1 = random.random()
            r2 = random.random()

            velocities[i][d] = w*velocities[i][d] + c1*r1*(p_best[i][d] - positions[i][d]) + c2*r2*(g_best[d] - positions[i][d])

            # clamp the velocity to be between -1 and 1
            if velocities[i][d] < -1:
                velocities[i][d] = -1
            elif velocities[i][d] > 1:
                velocities[i][d] = 1

def fitness(positions, p_best, g_best, fitness_func):
    
    for i in range(len(positions)):
        fit = fitness_func(positions[i]) 
        if fit != float('inf'): # valid fitness value
            if fit < p_best[i][-1]:
                p_best[i][0:-1] = positions[i]
                p_best[i][-1] = fit
                if fit < g_best[-1]:
                    g_best[0:-1] = positions[i]
                    g_best[-1] = fit
        else:
            # print("Invalid fitness value")
            pass

def init_pso(pop_size, bounds, fitness_func):
    
    positions, velocities = init_population(pop_size, bounds)
    
    p_best = []
    for i in range(pop_size):
        p_best.append(positions[i] + [fitness_func(positions[i])])
    g_best = p_best[0] # assume the first particle is the fittest

    for i in range(1, pop_size):
        if p_best[i][-1] < g_best[-1]: # if the ith particle is more fit than the currently assumed best 
            g_best[0:-1] = p_best[i][0:-1]
            g_best[-1] = p_best[i][-1]

    return positions, velocities, p_best, g_best

POPULATION_SIZE = 1000
MAX_ITERATIONS = 250 
W = 0.5 # inertia weight
C1 = 1.5 # cognitive coefficient
C2 = 2 # social coefficient

BOUNDS = [(-5, 20), (-15, 15), (-15, 15), (-5, 20), (-15, 15), (-15, 15), (-5, 20), (-15, 15), (-15, 15)] # overestimates of x, y, and z bounds in hip frame
positions, velocities, p_best, g_best = init_pso(POPULATION_SIZE, BOUNDS, force_magnitude)

iters = 0
bests = []
averages = []

while iters < MAX_ITERATIONS:
    update(positions, velocities, p_best, g_best, W, C1, C2, BOUNDS)
    fitness(positions, p_best, g_best, force_magnitude)

    iters += 1
    bests.append(g_best.copy())
    averages.append(sum(p[-1] for p in p_best) / len(p_best))
    print(f"Iteration: {iters} | Best foothold: {bests[-1][0:-1]} | Best fitness: {bests[-1][-1]} | Average fitness: {averages[-1]}")

bests_fitness = [b[-1] for b in bests]
iterations = list(range(1, len(bests_fitness) + 1))
plt.plot(iterations, bests_fitness, label='Best Fitness')
plt.plot(iterations, averages, label='Average Fitness')
plt.xlabel('Iterations')
plt.ylabel('Fitness')
plt.title('Fitness vs Iterations')
plt.legend()
plt.grid()
plt.show()

xyz_legs = [(bests[-1][0], bests[-1][1], bests[-1][2]),
            (bests[-1][3], bests[-1][4], bests[-1][5]),
            (bests[-1][6], bests[-1][7], bests[-1][8])]
torques = ta.compute_torque(xyz_legs, leg_indices=[0, 1, 2])
print(f"\n\nWorst Footholds (hip frame in cm): {xyz_legs}")
print(f"Torques at Worst Footholds (leg frame in Nm): {torques}")