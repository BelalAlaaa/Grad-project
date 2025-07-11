import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D

# Greenhouse parameters
LENGTH, WIDTH, HEIGHT = 4.0, 2.0, 2.5  # Length (L), Width (l), Height (z)
GRID_RES = 0.1  # Grid resolution for coverage calculation
PLANT_MIN_HEIGHT = 1.  # Plant table height - focus area starts here
PLANT_MAX_HEIGHT = 1.2  # Maximum plant height for monitoring
SENSOR_RANGE = 1.5  # Sensor detection range
CONE_ANGLE_DEG = 45  # Sensor cone angle in degrees
COMMUNICATION_RANGE = 2.0  # Communication range between sensors


class Sensor:
    """Represents a single sensor with position and orientation."""

    def __init__(self, x, y, z, dx, dy, dz):
        self.x = x  # Position
        self.y = y
        self.z = z
        self.dx = dx  # Direction vector
        self.dy = dy
        self.dz = dz


class SensorEvaluator:
    """Evaluates fixed sensor configuration using the mathematical criteria."""

    def __init__(self, sensors):
        self.sensors = sensors
        self.num_sensors = len(sensors)
        self.total_area = LENGTH * WIDTH  # Total greenhouse floor area in m²

    def calculate_sensor_density_rate(self):
        """Calculate sensor density rate using equation (1): Nsr = (Ns / (L × l)) × 100"""
        sensor_density_rate = (self.num_sensors / self.total_area) * 100
        return sensor_density_rate

    def calculate_separation_distance(self):
        """Calculate separation distance using equation (2): Dist = sum sqrt((xi - xk)² + (yi - yk)²)"""
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

        # Return total separation distance (sum of all pairwise distances)
        return total_distance

    def calculate_coverage_rate(self):
        """Calculate coverage rate using equation (3): Cvr = (sum_x sum_y Det(x,y)) / (L × l) × 100"""
        if not self.sensors:
            return 0, 0

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
        covered_area = np.sum(np.max(detection_matrix, axis=2)) * (GRID_RES ** 2)
        coverage_rate = (covered_area / self.total_area) * 100

        return coverage_rate, covered_area

    def calculate_over_coverage_rate(self):
        """Calculate over-coverage rate using equation (4): Ov-Cvr = (sum_x sum_y O-Det(x,y)) / (L × l) × 100"""
        if not self.sensors:
            return 0, 0

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

        # Calculate over-coverage rate
        # Get the over-coverage count (subtract 1 for single coverage, keep only positive)
        over_coverage_3d = np.maximum(coverage_count - 1, 0)
        # Sum over z-axis to get 2D over-coverage map
        over_coverage_2d = np.sum(over_coverage_3d, axis=2)
        # Calculate total over-covered area
        over_covered_area = np.sum(over_coverage_2d) * (GRID_RES ** 2)
        over_coverage_rate = (over_covered_area / self.total_area) * 100

        return over_coverage_rate, over_covered_area

    def calculate_communication_rate(self):
        """Calculate communication rate using equation (5): Con = (sum_x sum_y Cn(x,y)) / (L × l) × 100"""
        if len(self.sensors) < 2:
            return 0, 0

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
        connected_area = np.sum(connectivity_matrix) * (GRID_RES ** 2)
        communication_rate = (connected_area / self.total_area) * 100

        return communication_rate, connected_area

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

    def evaluate_all_criteria(self):
        """Calculate all evaluation criteria and return results."""
        results = {}

        # Calculate each criterion (now returns both percentage and area)
        results['sensor_density_rate'] = self.calculate_sensor_density_rate()
        results['separation_distance'] = self.calculate_separation_distance()

        coverage_rate, covered_area = self.calculate_coverage_rate()
        results['coverage_rate'] = coverage_rate
        results['covered_area'] = covered_area

        over_coverage_rate, over_covered_area = self.calculate_over_coverage_rate()
        results['over_coverage_rate'] = over_coverage_rate
        results['over_covered_area'] = over_covered_area

        communication_rate, connected_area = self.calculate_communication_rate()
        results['communication_rate'] = communication_rate
        results['connected_area'] = connected_area

        return results

    def print_detailed_results(self, results):
        """Print detailed results with equations and areas."""
        print("=" * 80)
        print("FIXED 4-SENSOR CONFIGURATION EVALUATION")
        print("=" * 80)

        print(f"Greenhouse dimensions: {LENGTH}m × {WIDTH}m × {HEIGHT}m")
        print(f"Total floor area: {self.total_area:.2f} m²")
        print(f"Plant monitoring zone: {PLANT_MIN_HEIGHT}m - {PLANT_MAX_HEIGHT}m height")
        print(f"Sensor range: {SENSOR_RANGE}m, Cone angle: {CONE_ANGLE_DEG}°")
        print(f"Communication range: {COMMUNICATION_RANGE}m")

        print("\n" + "=" * 80)
        print("SENSOR POSITIONS:")
        print("=" * 80)
        for i, sensor in enumerate(self.sensors):
            print(f"Sensor {i + 1}: Position ({sensor.x:.2f}, {sensor.y:.2f}, {sensor.z:.2f})")
            print(f"           Direction ({sensor.dx:.2f}, {sensor.dy:.2f}, {sensor.dz:.2f})")

        print("\n" + "=" * 80)
        print("EVALUATION CRITERIA RESULTS:")
        print("=" * 80)

        # Equation 1: Sensor density rate
        print(f"\n📊 Equation (1) - Sensor Density Rate:")
        print(f"   Formula: Nsr = (Ns / (L × l)) × 100")
        print(f"   Calculation: Nsr = ({self.num_sensors} / ({LENGTH} × {WIDTH})) × 100")
        print(f"   Result: {results['sensor_density_rate']:.2f} sensors/m²")

        # Equation 2: Separation distance
        print(f"\n📏 Equation (2) - Sensor Separation Distance:")
        print(f"   Formula: Dist = Σ√((xi - xk)² + (yi - yk)²)")
        print(f"   Individual distances:")
        distances = []
        for i in range(len(self.sensors)):
            for j in range(i + 1, len(self.sensors)):
                sensor_i = self.sensors[i]
                sensor_j = self.sensors[j]
                distance = np.sqrt((sensor_i.x - sensor_j.x) ** 2 +
                                   (sensor_i.y - sensor_j.y) ** 2)
                distances.append(distance)
                print(f"     S{i + 1} to S{j + 1}: {distance:.2f}m")
        print(f"   Total separation distance: {results['separation_distance']:.2f}m")

        # Equation 3: Coverage rate
        print(f"\n✅ Equation (3) - Coverage Rate:")
        print(f"   Formula: Cvr = (Σx Σy Det(x,y)) / (L × l) × 100")
        print(f"   Result: {results['coverage_rate']:.2f}%")
        print(f"   Covered area: {results['covered_area']:.2f} m²")

        # Equation 4: Over-coverage rate
        print(f"\n⚠️  Equation (4) - Over-coverage Rate:")
        print(f"   Formula: Ov-Cvr = (Σx Σy O-Det(x,y)) / (L × l) × 100")
        print(f"   Result: {results['over_coverage_rate']:.2f}%")
        print(f"   Over-covered area: {results['over_covered_area']:.2f} m²")

        # Equation 5: Communication rate
        print(f"\n📡 Equation (5) - Communication Rate:")
        print(f"   Formula: Con = (Σx Σy Cn(x,y)) / (L × l) × 100")
        print(f"   Result: {results['communication_rate']:.2f}%")
        print(f"   Connected area: {results['connected_area']:.2f} m²")

        print("\n" + "=" * 80)
        print("SUMMARY:")
        print("=" * 80)
        print(f"• Total greenhouse floor area: {self.total_area:.2f} m²")
        print(f"• Number of sensors: {self.num_sensors}")
        print(f"• Sensor density: {results['sensor_density_rate']:.2f} sensors/m²")
        print(f"• Total separation distance: {results['separation_distance']:.2f}m")
        print(f"• Plant area coverage: {results['coverage_rate']:.2f}% ({results['covered_area']:.2f} m²)")
        print(f"• Over-coverage: {results['over_coverage_rate']:.2f}% ({results['over_covered_area']:.2f} m²)")
        print(f"• Communication coverage: {results['communication_rate']:.2f}% ({results['connected_area']:.2f} m²)")

        print("\n" + "=" * 80)
        print("AREA BREAKDOWN:")
        print("=" * 80)
        uncovered_area = self.total_area - results['covered_area']
        print(f"• Total area: {self.total_area:.2f} m²")
        print(f"• Covered area: {results['covered_area']:.2f} m²")
        print(f"• Uncovered area: {uncovered_area:.2f} m²")
        print(f"• Over-covered area: {results['over_covered_area']:.2f} m²")
        print(f"• Communication-connected area: {results['connected_area']:.2f} m²")

    def visualize_coverage(self):
        """Visualize the 3D sensor coverage."""
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Draw greenhouse boundaries
        # Bottom face
        ax.plot([0, WIDTH, WIDTH, 0, 0], [0, 0, LENGTH, LENGTH, 0], [0, 0, 0, 0, 0], 'k-', alpha=0.3)
        # Top face
        ax.plot([0, WIDTH, WIDTH, 0, 0], [0, 0, LENGTH, LENGTH, 0], [HEIGHT, HEIGHT, HEIGHT, HEIGHT, HEIGHT], 'k-',
                alpha=0.3)
        # Vertical edges
        for x, y in [(0, 0), (WIDTH, 0), (WIDTH, LENGTH), (0, LENGTH)]:
            ax.plot([x, x], [y, y], [0, HEIGHT], 'k-', alpha=0.3)

        # Draw plant monitoring zone
        plant_corners = np.array([
            [0, 0, PLANT_MIN_HEIGHT], [WIDTH, 0, PLANT_MIN_HEIGHT],
            [WIDTH, LENGTH, PLANT_MIN_HEIGHT], [0, LENGTH, PLANT_MIN_HEIGHT]
        ])
        ax.plot(plant_corners[:, 0], plant_corners[:, 1], plant_corners[:, 2], 'g-', linewidth=2,
                label='Plant Zone Bottom')

        plant_corners_top = np.array([
            [0, 0, PLANT_MAX_HEIGHT], [WIDTH, 0, PLANT_MAX_HEIGHT],
            [WIDTH, LENGTH, PLANT_MAX_HEIGHT], [0, LENGTH, PLANT_MAX_HEIGHT]
        ])
        ax.plot(plant_corners_top[:, 0], plant_corners_top[:, 1], plant_corners_top[:, 2], 'g-', linewidth=2,
                label='Plant Zone Top')

        # Draw sensors
        for i, sensor in enumerate(self.sensors):
            ax.scatter(sensor.x, sensor.y, sensor.z, color='red', s=100, marker='o')
            ax.text(sensor.x, sensor.y, sensor.z + 0.1, f'S{i + 1}', fontsize=10)

            # Draw direction vector
            ax.quiver(sensor.x, sensor.y, sensor.z,
                      sensor.dx * 0.3, sensor.dy * 0.3, sensor.dz * 0.3,
                      color='blue', alpha=0.7, arrow_length_ratio=0.1)

        ax.set_xlabel('Width (m)')
        ax.set_ylabel('Length (m)')
        ax.set_zlabel('Height (m)')
        ax.set_title('3D Greenhouse with Fixed 4-Sensor Configuration')
        ax.legend()
        ax.set_xlim(0, WIDTH)
        ax.set_ylim(0, LENGTH)
        ax.set_zlim(0, HEIGHT)

        plt.tight_layout()
        plt.show()


# Define the 4 fixed sensors with correct positions
fixed_sensors = [
    # Sensor 1: Right-front wall at [2.0, 0.0, 1.5]
    Sensor(x=2.0, y=0.0, z=1.5, dx=-0.7, dy=0.8, dz=-0.5),

    # Sensor 2: Right-back wall at [2.0, 4.0, 1.5]
    Sensor(x=2.0, y=4.0, z=1.5, dx=-0.7, dy=-0.8, dz=-0.5),

    # Sensor 3: Left-middle wall at [0.0, 2.0, 1.5]
    Sensor(x=0.0, y=0.0, z=1.5, dx=0.7, dy=0.8, dz=-0.5),

    # Sensor 4: Center position at [1.0, 2.0, 1.5]
    Sensor(x=0.0, y=4.0, z=1.5, dx=0.7, dy=-0.8, dz=-0.5)
]

# Normalize direction vectors
for sensor in fixed_sensors:
    norm = np.sqrt(sensor.dx ** 2 + sensor.dy ** 2 + sensor.dz ** 2)
    if norm > 0:
        sensor.dx, sensor.dy, sensor.dz = sensor.dx / norm, sensor.dy / norm, sensor.dz / norm

if __name__ == "__main__":
    # Create evaluator with fixed 4 sensors
    evaluator = SensorEvaluator(fixed_sensors)

    # Calculate all criteria
    results = evaluator.evaluate_all_criteria()

    # Print detailed results
    evaluator.print_detailed_results(results)

    # Visualize the configuration
    evaluator.visualize_coverage()

    print("\n" + "=" * 80)
    print("EVALUATION COMPLETED")
    print("=" * 80)