import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import copy

# Greenhouse parameters
LENGTH, WIDTH, HEIGHT = 4.0, 2.0, 2.5  # Length (L), Width (l), Height (z)
GRID_RES = 0.1  # Grid resolution for coverage calculation
PLANT_MIN_HEIGHT = 1.0  # Plant table height - focus area starts here
PLANT_MAX_HEIGHT = 1.2  # Maximum plant height for monitoring
SENSOR_RANGE = 1.5  # Sensor detection range
CONE_ANGLE_DEG = 45  # Sensor cone angle in degrees
COMMUNICATION_RANGE = 2.0  # Communication range between sensors
MIN_SENSORS = 2  # Minimum number of sensors
MAX_SENSORS = 10  # Maximum number of sensors

# NSGA-II parameters
POPULATION_SIZE = 100
GENERATIONS = 50
MUTATION_RATE = 0.1
CROSSOVER_RATE = 0.8


class Sensor:
    """Represents a single sensor with position and orientation."""

    def __init__(self, x, y, z, dx, dy, dz):
        self.x = x  # Position
        self.y = y
        self.z = z
        self.dx = dx  # Direction vector
        self.dy = dy
        self.dz = dz

    def to_array(self):
        return np.array([self.x, self.y, self.z, self.dx, self.dy, self.dz])


class Individual:
    """Represents a solution with variable number of sensors."""

    def __init__(self, sensors=None):
        if sensors is None:
            # Generate random number of sensors
            num_sensors = random.randint(MIN_SENSORS, MAX_SENSORS)
            self.sensors = self.generate_random_sensors(num_sensors)
        else:
            self.sensors = sensors

        self.coverage_rate = 0
        self.over_coverage_rate = 0
        self.communication_rate = 0
        self.sensor_density_rate = 0
        self.separation_distance = 0
        self.num_sensors = len(self.sensors)
        self.fitness = [0, 0, 0, 0, 0]  # Multi-objective fitness
        self.rank = 0
        self.crowding_distance = 0

    def generate_random_sensors(self, num_sensors):
        """Generate random sensor positions and orientations."""
        sensors = []
        for _ in range(num_sensors):
            # Random position within greenhouse bounds
            x = random.uniform(0, WIDTH)
            y = random.uniform(0, LENGTH)
            z = random.uniform(PLANT_MIN_HEIGHT + 0.2, HEIGHT)  # Above plant table

            # Random direction vector (normalized) - bias towards downward monitoring
            dx = random.uniform(-0.5, 0.5)
            dy = random.uniform(-0.5, 0.5)
            dz = random.uniform(-1, -0.3)  # Downward bias for plant monitoring

            # Normalize direction vector
            norm = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            if norm > 0:
                dx, dy, dz = dx / norm, dy / norm, dz / norm

            sensors.append(Sensor(x, y, z, dx, dy, dz))

        return sensors

    def evaluate(self):
        """Calculate fitness objectives based on the mathematical criteria."""
        # Calculate all evaluation criteria
        coverage_rate = self.calculate_coverage_rate()
        over_coverage_rate = self.calculate_over_coverage_rate()
        communication_rate = self.calculate_communication_rate()
        sensor_density_rate = self.calculate_sensor_density_rate()
        separation_distance = self.calculate_separation_distance()

        # Store metrics
        self.coverage_rate = coverage_rate
        self.over_coverage_rate = over_coverage_rate
        self.communication_rate = communication_rate
        self.sensor_density_rate = sensor_density_rate
        self.separation_distance = separation_distance

        # Multi-objective fitness:
        # [maximize coverage, minimize over-coverage, maximize communication, minimize sensor density, maximize separation]
        self.fitness = [
            coverage_rate,  # Maximize coverage
            -over_coverage_rate,  # Minimize over-coverage
            communication_rate,  # Maximize communication
            -sensor_density_rate,  # Minimize sensor density
            separation_distance  # Maximize separation distance
        ]

        return self.fitness

    def calculate_coverage_rate(self):
        """Calculate coverage rate using equation (3): Cvr = (sum_x sum_y Det(x,y)) / (L × l) × 100"""
        if not self.sensors:
            return 0

        # Create detection matrix for plant monitoring area
        x_points = int(WIDTH / GRID_RES) + 1
        y_points = int(LENGTH / GRID_RES) + 1
        z_points = int((PLANT_MAX_HEIGHT - PLANT_MIN_HEIGHT) / GRID_RES) + 1

        detection_matrix = np.zeros((x_points, y_points, z_points))

        # Calculate detection coverage for each sensor
        for sensor in self.sensors:
            pos = np.array([sensor.x, sensor.y, sensor.z])
            direction = np.array([sensor.dx, sensor.dy, sensor.dz])

            for i in range(x_points):
                for j in range(y_points):
                    for k in range(z_points):
                        point = np.array([
                            i * GRID_RES,
                            j * GRID_RES,
                            PLANT_MIN_HEIGHT + k * GRID_RES
                        ])

                        # Check if point is within plant monitoring zone
                        if (0 <= point[0] <= WIDTH and
                                0 <= point[1] <= LENGTH and
                                PLANT_MIN_HEIGHT <= point[2] <= PLANT_MAX_HEIGHT):

                            if self.is_in_detection_cone(pos, direction, point):
                                detection_matrix[i, j, k] = 1

        # Calculate coverage rate
        total_area = WIDTH * LENGTH
        covered_area = np.sum(np.max(detection_matrix, axis=2)) * (GRID_RES ** 2)
        coverage_rate = (covered_area / total_area) * 100

        return coverage_rate

    def calculate_over_coverage_rate(self):
        """Calculate over-coverage rate using equation (4): Ov-Cvr = (sum_x sum_y O-Det(x,y)) / (L × l) × 100"""
        if not self.sensors:
            return 0

        # Create over-coverage detection matrix
        x_points = int(WIDTH / GRID_RES) + 1
        y_points = int(LENGTH / GRID_RES) + 1
        z_points = int((PLANT_MAX_HEIGHT - PLANT_MIN_HEIGHT) / GRID_RES) + 1

        coverage_count = np.zeros((x_points, y_points, z_points))

        # Count coverage by each sensor
        for sensor in self.sensors:
            pos = np.array([sensor.x, sensor.y, sensor.z])
            direction = np.array([sensor.dx, sensor.dy, sensor.dz])

            for i in range(x_points):
                for j in range(y_points):
                    for k in range(z_points):
                        point = np.array([
                            i * GRID_RES,
                            j * GRID_RES,
                            PLANT_MIN_HEIGHT + k * GRID_RES
                        ])

                        if (0 <= point[0] <= WIDTH and
                                0 <= point[1] <= LENGTH and
                                PLANT_MIN_HEIGHT <= point[2] <= PLANT_MAX_HEIGHT):

                            if self.is_in_detection_cone(pos, direction, point):
                                coverage_count[i, j, k] += 1

        # Calculate over-coverage rate - FIXED LINE
        total_area = WIDTH * LENGTH
        # First get the over-coverage count (subtract 1 for single coverage, keep only positive)
        over_coverage_3d = np.maximum(coverage_count - 1, 0)
        # Sum over z-axis to get 2D over-coverage map
        over_coverage_2d = np.sum(over_coverage_3d, axis=2)
        # Calculate total over-covered area
        over_covered_area = np.sum(over_coverage_2d) * (GRID_RES ** 2)
        over_coverage_rate = (over_covered_area / total_area) * 100

        return over_coverage_rate

    def calculate_communication_rate(self):
        """Calculate communication rate using equation (5): Con = (sum_x sum_y Cn(x,y)) / (L × l) × 100"""
        if len(self.sensors) < 2:
            return 0

        # Create connectivity matrix
        x_points = int(WIDTH / GRID_RES) + 1
        y_points = int(LENGTH / GRID_RES) + 1

        connectivity_matrix = np.zeros((x_points, y_points))

        # Calculate connectivity for each grid point
        for i in range(x_points):
            for j in range(y_points):
                point = np.array([i * GRID_RES, j * GRID_RES, PLANT_MIN_HEIGHT])

                # Count sensors that can communicate from this point
                connected_sensors = 0
                for sensor in self.sensors:
                    sensor_pos = np.array([sensor.x, sensor.y, sensor.z])
                    distance = np.linalg.norm(point - sensor_pos)
                    if distance <= COMMUNICATION_RANGE:
                        connected_sensors += 1

                # Point is connected if at least 2 sensors can communicate
                if connected_sensors >= 2:
                    connectivity_matrix[i, j] = 1

        # Calculate communication rate
        total_area = WIDTH * LENGTH
        connected_area = np.sum(connectivity_matrix) * (GRID_RES ** 2)
        communication_rate = (connected_area / total_area) * 100

        return communication_rate

    def calculate_sensor_density_rate(self):
        """Calculate sensor density rate using equation (1): Nsr = (Ns / (L × l)) × 100"""
        total_area = WIDTH * LENGTH
        sensor_density_rate = (self.num_sensors / total_area) * 100
        return sensor_density_rate

    def calculate_separation_distance(self):
        """Calculate average separation distance using equation (2): Dist = sum sqrt((xi - xk)² + (yi - yk)²)"""
        if len(self.sensors) < 2:
            return 0

        total_distance = 0
        count = 0

        # Calculate pairwise distances
        for i in range(len(self.sensors)):
            for j in range(i + 1, len(self.sensors)):
                sensor_i = self.sensors[i]
                sensor_j = self.sensors[j]

                distance = np.sqrt((sensor_i.x - sensor_j.x) ** 2 +
                                   (sensor_i.y - sensor_j.y) ** 2)
                total_distance += distance
                count += 1

        # Return average separation distance
        return total_distance / count if count > 0 else 0

    def is_in_detection_cone(self, sensor_pos, direction, point):
        """Check if point is within sensor detection cone."""
        vec = point - sensor_pos
        dist = np.linalg.norm(vec)

        if dist == 0 or dist > SENSOR_RANGE:
            return False

        unit_vec = vec / dist
        unit_direction = direction / np.linalg.norm(direction)

        cos_angle = np.dot(unit_vec, unit_direction)
        return cos_angle >= np.cos(np.deg2rad(CONE_ANGLE_DEG))


class NSGA2:
    """NSGA-II algorithm implementation."""

    def __init__(self, population_size=POPULATION_SIZE, generations=GENERATIONS):
        self.population_size = population_size
        self.generations = generations
        self.population = []
        self.best_solutions = []
        self.generation_stats = []

    def initialize_population(self):
        """Create initial random population."""
        self.population = []
        for _ in range(self.population_size):
            individual = Individual()
            individual.evaluate()
            self.population.append(individual)

    def non_dominated_sort(self, population):
        """Perform non-dominated sorting."""
        fronts = [[]]

        for i, individual in enumerate(population):
            individual.domination_count = 0
            individual.dominated_solutions = []

            for j, other in enumerate(population):
                if i != j:
                    if self.dominates(individual, other):
                        individual.dominated_solutions.append(j)
                    elif self.dominates(other, individual):
                        individual.domination_count += 1

            if individual.domination_count == 0:
                individual.rank = 0
                fronts[0].append(i)

        front_idx = 0
        while len(fronts[front_idx]) > 0:
            next_front = []
            for i in fronts[front_idx]:
                for j in population[i].dominated_solutions:
                    population[j].domination_count -= 1
                    if population[j].domination_count == 0:
                        population[j].rank = front_idx + 1
                        next_front.append(j)

            front_idx += 1
            fronts.append(next_front)

        return fronts[:-1]  # Remove empty last front

    def dominates(self, ind1, ind2):
        """Check if ind1 dominates ind2."""
        at_least_one_better = False
        for i in range(len(ind1.fitness)):
            if ind1.fitness[i] < ind2.fitness[i]:
                return False
            elif ind1.fitness[i] > ind2.fitness[i]:
                at_least_one_better = True
        return at_least_one_better

    def calculate_crowding_distance(self, population, front):
        """Calculate crowding distance for individuals in a front."""
        if len(front) == 0:
            return

        # Initialize crowding distance
        for i in front:
            population[i].crowding_distance = 0

        # For each objective
        for obj_idx in range(len(population[0].fitness)):
            # Sort front by objective value
            front_sorted = sorted(front, key=lambda x: population[x].fitness[obj_idx], reverse=True)

            # Set boundary points to infinite distance
            population[front_sorted[0]].crowding_distance = float('inf')
            population[front_sorted[-1]].crowding_distance = float('inf')

            # Calculate distance for intermediate points
            if len(front_sorted) > 2:
                obj_range = (population[front_sorted[0]].fitness[obj_idx] -
                             population[front_sorted[-1]].fitness[obj_idx])

                if obj_range > 0:
                    for i in range(1, len(front_sorted) - 1):
                        distance = (population[front_sorted[i - 1]].fitness[obj_idx] -
                                    population[front_sorted[i + 1]].fitness[obj_idx]) / obj_range
                        population[front_sorted[i]].crowding_distance += distance

    def tournament_selection(self):
        """Tournament selection for parent selection."""
        k = 2
        tournament = random.sample(self.population, k)

        # Select based on rank first, then crowding distance
        tournament.sort(key=lambda x: (x.rank, -x.crowding_distance))
        return tournament[0]

    def crossover(self, parent1, parent2):
        """Crossover operation to create offspring."""
        if random.random() > CROSSOVER_RATE:
            return copy.deepcopy(parent1), copy.deepcopy(parent2)

        # Variable-length crossover
        min_sensors = min(len(parent1.sensors), len(parent2.sensors))

        if min_sensors <= 1:
            return copy.deepcopy(parent1), copy.deepcopy(parent2)

        # Random crossover point
        crossover_point = random.randint(1, min_sensors - 1)

        # Create offspring
        child1_sensors = (parent1.sensors[:crossover_point] +
                          parent2.sensors[crossover_point:min_sensors])
        child2_sensors = (parent2.sensors[:crossover_point] +
                          parent1.sensors[crossover_point:min_sensors])

        # Add remaining sensors randomly
        if len(parent1.sensors) > min_sensors:
            if random.random() < 0.5:
                child1_sensors.extend(parent1.sensors[min_sensors:])
            else:
                child2_sensors.extend(parent1.sensors[min_sensors:])

        if len(parent2.sensors) > min_sensors:
            if random.random() < 0.5:
                child1_sensors.extend(parent2.sensors[min_sensors:])
            else:
                child2_sensors.extend(parent2.sensors[min_sensors:])

        return Individual(child1_sensors), Individual(child2_sensors)

    def mutate(self, individual):
        """Mutation operation."""
        if random.random() > MUTATION_RATE:
            return individual

        mutation_type = random.choice(['position', 'direction', 'add_sensor', 'remove_sensor'])

        if mutation_type == 'position' and len(individual.sensors) > 0:
            # Mutate sensor position
            sensor_idx = random.randint(0, len(individual.sensors) - 1)
            sensor = individual.sensors[sensor_idx]

            sensor.x = np.clip(sensor.x + random.gauss(0, 0.2), 0, WIDTH)
            sensor.y = np.clip(sensor.y + random.gauss(0, 0.2), 0, LENGTH)
            sensor.z = np.clip(sensor.z + random.gauss(0, 0.2), PLANT_MIN_HEIGHT + 0.2, HEIGHT)

        elif mutation_type == 'direction' and len(individual.sensors) > 0:
            # Mutate sensor direction
            sensor_idx = random.randint(0, len(individual.sensors) - 1)
            sensor = individual.sensors[sensor_idx]

            sensor.dx += random.gauss(0, 0.3)
            sensor.dy += random.gauss(0, 0.3)
            sensor.dz += random.gauss(0, 0.3)

            # Normalize direction
            norm = np.sqrt(sensor.dx ** 2 + sensor.dy ** 2 + sensor.dz ** 2)
            if norm > 0:
                sensor.dx, sensor.dy, sensor.dz = sensor.dx / norm, sensor.dy / norm, sensor.dz / norm

        elif mutation_type == 'add_sensor' and len(individual.sensors) < MAX_SENSORS:
            # Add new sensor
            new_sensors = individual.generate_random_sensors(1)
            individual.sensors.extend(new_sensors)
            individual.num_sensors = len(individual.sensors)

        elif mutation_type == 'remove_sensor' and len(individual.sensors) > MIN_SENSORS:
            # Remove sensor
            sensor_idx = random.randint(0, len(individual.sensors) - 1)
            individual.sensors.pop(sensor_idx)
            individual.num_sensors = len(individual.sensors)

        return individual

    def evolve(self):
        """Main evolution loop."""
        print("Initializing NSGA-II for Plant Monitoring Sensor Optimization...")
        print(f"Plant monitoring zone: {PLANT_MIN_HEIGHT}m - {PLANT_MAX_HEIGHT}m height")
        print(f"Greenhouse dimensions: {LENGTH}m × {WIDTH}m × {HEIGHT}m")
        self.initialize_population()

        for generation in range(self.generations):
            # Create offspring
            offspring = []
            for _ in range(self.population_size // 2):
                parent1 = self.tournament_selection()
                parent2 = self.tournament_selection()

                child1, child2 = self.crossover(parent1, parent2)
                child1 = self.mutate(child1)
                child2 = self.mutate(child2)

                child1.evaluate()
                child2.evaluate()

                offspring.extend([child1, child2])

            # Combine population and offspring
            combined_population = self.population + offspring

            # Non-dominated sorting
            fronts = self.non_dominated_sort(combined_population)

            # Calculate crowding distance
            for front in fronts:
                self.calculate_crowding_distance(combined_population, front)

            # Select next generation
            new_population = []
            for front in fronts:
                if len(new_population) + len(front) <= self.population_size:
                    new_population.extend([combined_population[i] for i in front])
                else:
                    # Sort by crowding distance and take the best
                    remaining = self.population_size - len(new_population)
                    front_sorted = sorted(front, key=lambda x: combined_population[x].crowding_distance, reverse=True)
                    new_population.extend([combined_population[i] for i in front_sorted[:remaining]])
                    break

            self.population = new_population

            # Store statistics
            best_coverage = max(ind.coverage_rate for ind in self.population)
            avg_coverage = np.mean([ind.coverage_rate for ind in self.population])
            best_over_coverage = min(ind.over_coverage_rate for ind in self.population)
            avg_communication = np.mean([ind.communication_rate for ind in self.population])
            avg_sensors = np.mean([ind.num_sensors for ind in self.population])

            self.generation_stats.append({
                'generation': generation,
                'best_coverage': best_coverage,
                'avg_coverage': avg_coverage,
                'best_over_coverage': best_over_coverage,
                'avg_communication': avg_communication,
                'avg_sensors': avg_sensors
            })

            if generation % 10 == 0:
                print(f"Generation {generation}: Coverage = {best_coverage:.2f}%, "
                      f"Over-coverage = {best_over_coverage:.2f}%, "
                      f"Communication = {avg_communication:.2f}%, "
                      f"Avg Sensors = {avg_sensors:.1f}")

        # Extract Pareto front
        fronts = self.non_dominated_sort(self.population)
        self.best_solutions = [self.population[i] for i in fronts[0]]

        print(f"\nOptimization completed! Found {len(self.best_solutions)} Pareto optimal solutions.")
        return self.best_solutions

    def print_results(self):
        """Print optimization results."""
        print(f"\n{'=' * 100}")
        print("NSGA-II PLANT MONITORING OPTIMIZATION RESULTS")
        print(f"{'=' * 100}")

        print(f"🏆 PARETO OPTIMAL SOLUTIONS ({len(self.best_solutions)} found):")
        print(f"{'=' * 100}")

        # Sort solutions by coverage for display
        sorted_solutions = sorted(self.best_solutions, key=lambda x: x.coverage_rate, reverse=True)

        for i, solution in enumerate(sorted_solutions):
            print(f"\n📊 Solution {i + 1}:")
            print(f"   🔢 Number of sensors: {solution.num_sensors}")
            print(f"   ✅ Coverage rate: {solution.coverage_rate:.2f}%")
            print(f"   ⚠️  Over-coverage rate: {solution.over_coverage_rate:.2f}%")
            print(f"   📡 Communication rate: {solution.communication_rate:.2f}%")
            print(f"   📊 Sensor density rate: {solution.sensor_density_rate:.2f} sensors/m²")
            print(f"   📏 Avg separation distance: {solution.separation_distance:.2f}m")
            print(f"   📍 Sensor positions (x, y, z) → direction (dx, dy, dz):")

            for j, sensor in enumerate(solution.sensors):
                print(f"      S{j + 1}: ({sensor.x:.2f}, {sensor.y:.2f}, {sensor.z:.2f}) "
                      f"→ ({sensor.dx:.2f}, {sensor.dy:.2f}, {sensor.dz:.2f})")

        # Recommendations
        print(f"\n{'=' * 100}")
        print("RECOMMENDATIONS FOR PLANT MONITORING:")
        print(f"{'=' * 100}")

        # Best coverage solution
        best_coverage_sol = max(self.best_solutions, key=lambda x: x.coverage_rate)
        print(f"🎯 For MAXIMUM PLANT COVERAGE ({best_coverage_sol.coverage_rate:.2f}%):")
        print(f"   Use {best_coverage_sol.num_sensors} sensors")
        print(f"   Over-coverage: {best_coverage_sol.over_coverage_rate:.2f}%")
        print(f"   Communication: {best_coverage_sol.communication_rate:.2f}%")

        # Best balance solution (high coverage, low over-coverage)
        balance_scores = [sol.coverage_rate - sol.over_coverage_rate + sol.communication_rate for sol in
                          self.best_solutions]
        best_balance_sol = self.best_solutions[balance_scores.index(max(balance_scores))]
        print(f"\n🎯 For BEST OVERALL BALANCE:")
        print(f"   Use {best_balance_sol.num_sensors} sensors")
        print(f"   Coverage: {best_balance_sol.coverage_rate:.2f}%")
        print(f"   Over-coverage: {best_balance_sol.over_coverage_rate:.2f}%")
        print(f"   Communication: {best_balance_sol.communication_rate:.2f}%")

        # Minimum sensors solution
        min_sensors_sol = min(self.best_solutions, key=lambda x: x.num_sensors)
        print(f"\n🎯 For MINIMUM SENSORS ({min_sensors_sol.num_sensors} sensors):")
        print(f"   Coverage: {min_sensors_sol.coverage_rate:.2f}%")
        print(f"   Over-coverage: {min_sensors_sol.over_coverage_rate:.2f}%")
        print(f"   Communication: {min_sensors_sol.communication_rate:.2f}%")

    def plot_evolution(self):
        """Plot evolution statistics."""
        if not self.generation_stats:
            return

        generations = [stat['generation'] for stat in self.generation_stats]
        best_coverage = [stat['best_coverage'] for stat in self.generation_stats]
        avg_coverage = [stat['avg_coverage'] for stat in self.generation_stats]
        best_over_coverage = [stat['best_over_coverage'] for stat in self.generation_stats]
        avg_communication = [stat['avg_communication'] for stat in self.generation_stats]
        avg_sensors = [stat['avg_sensors'] for stat in self.generation_stats]

        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))

        # Coverage evolution
        ax1.plot(generations, best_coverage, 'b-', label='Best Coverage', linewidth=2)
        ax1.plot(generations, avg_coverage, 'b--', label='Average Coverage', alpha=0.7)
        ax1.set_xlabel('Generation')
        ax1.set_ylabel('Coverage Rate (%)')
        ax1.set_title('Plant Area Coverage Evolution')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Over-coverage evolution
        ax2.plot(generations, best_over_coverage, 'r-', label='Best (Minimum) Over-coverage', linewidth=2)
        ax2.set_xlabel('Generation')
        ax2.set_ylabel('Over-coverage Rate (%)')
        ax2.set_title('Over-coverage Evolution')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Communication evolution
        ax3.plot(generations, avg_communication, 'g-', label='Average Communication', linewidth=2)
        ax3.set_xlabel('Generation')
        ax3.set_ylabel('Communication Rate (%)')
        ax3.set_title('Communication Network Evolution')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Pareto front - Coverage vs Over-coverage
        coverages = [sol.coverage_rate for sol in self.best_solutions]
        over_coverages = [sol.over_coverage_rate for sol in self.best_solutions]
        communications = [sol.communication_rate for sol in self.best_solutions]

        scatter = ax4.scatter(over_coverages, coverages, c=communications, cmap='viridis',
                              s=100, alpha=0.7, edgecolors='black')
        ax4.set_xlabel('Over-coverage Rate (%)')
        ax4.set_ylabel('Coverage Rate (%)')
        ax4.set_title('Pareto Front: Coverage vs Over-coverage')
        ax4.grid(True, alpha=0.3)

        # Add colorbar
        cbar = plt.colorbar(scatter, ax=ax4)
        cbar.set_label('Communication Rate (%)')

        plt.tight_layout()
        plt.show()


# Main execution
if __name__ == "__main__":
    # Initialize and run NSGA-II
    nsga2 = NSGA2(population_size=80, generations=40)
    pareto_solutions = nsga2.evolve()

    # Print results
    nsga2.print_results()

    # Plot evolution
    nsga2.plot_evolution()

    print(f"\n{'=' * 100}")
    print("PLANT MONITORING SENSOR OPTIMIZATION COMPLETED")
    print(f"Plant monitoring zone: {PLANT_MIN_HEIGHT}m - {PLANT_MAX_HEIGHT}m height")
    print(f"Greenhouse dimensions: {LENGTH}m × {WIDTH}m × {HEIGHT}m")
    print(f"Sensor specifications: {SENSOR_RANGE}m range, {CONE_ANGLE_DEG}° cone angle")
    print(f"Communication range: {COMMUNICATION_RANGE}m")
    print(f"{'=' * 100}")