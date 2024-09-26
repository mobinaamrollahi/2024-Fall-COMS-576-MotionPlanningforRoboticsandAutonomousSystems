import argparse
import math
import matplotlib.pyplot as plt

def argument_parser():
    parser = argparse.ArgumentParser(
        prog='chain.py',
        description='Visualize a 2D kinematic chain based on given parameters.',
        epilog='Example: %(prog)s -W 1 -L 5 -D 0.5 30 45 60'
    )
    parser.add_argument("-W", type=float, required=True, help="Width of each link")
    parser.add_argument("-L", type=float, required=True, help="Length of each link")
    parser.add_argument("-D", type=float, required=True, help="Distance between attachment points")
    parser.add_argument("thetas", nargs='+', type=float, help="Angles defining the chain configuration")
    return parser.parse_args()

def kinematic_model(W, L, D, thetas):
    positions = [(0, 0)]  # Start position
    current_x, current_y = 0, 0
    current_angle = 0

    for theta in thetas:
        current_angle += theta
        current_x += L * math.cos(math.radians(current_angle))
        current_y += L * math.sin(math.radians(current_angle))
        positions.append((current_x, current_y))

    return positions

def plot_chain(positions, W, L, D):
    plt.figure(figsize=(10, 6))
    for i in range(len(positions) - 1):
        x1, y1 = positions[i]
        x2, y2 = positions[i + 1]
        plt.plot([x1, x2], [y1, y2], 'b-', linewidth=2)
        plt.plot(x1, y1, 'ro')  # Joint
    
    plt.plot(positions[-1][0], positions[-1][1], 'ro')  # Last joint
    plt.title("2D Kinematic Chain")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis('equal')
    plt.grid(True)
    plt.show()

def main():
    args = argument_parser()
    positions = kinematic_model(args.W, args.L, args.D, args.thetas)
    plot_chain(positions, args.W, args.L, args.D)

if __name__ == "__main__":
    main()