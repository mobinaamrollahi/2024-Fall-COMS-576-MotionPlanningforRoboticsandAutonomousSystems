import argparse
import math
import matplotlib.pyplot as plt

def argument_parser():
    parser = argparse.ArgumentParser(
        prog='chain.py',
        description='Visualize a 2D kinematic chain based on given parameters.',
        epilog='Example: %(prog)s -W 1 -L 7 -D 5 30 45 60'
    )
    parser.add_argument("-W", type=float, required=True, help="Width of each link")
    parser.add_argument("-L", type=float, required=True, help="Length of each link")
    parser.add_argument("-D", type=float, required=True, help="Distance between attachment points")
    parser.add_argument("thetas", nargs='+', type=float, help="Angles defining the chain configuration")
    return parser.parse_args()

def kinematic_model(W, L, D, thetas):
    joint_coordinates = [(0, 0)]  # Start position
    links = []
    current_x, current_y = 0, 0
    current_angle = 0

    for theta in thetas:
        current_angle += theta
        angle_rad = math.radians(current_angle)

        # Calculate next joint position
        dx = D * math.cos(angle_rad)
        dy = D * math.sin(angle_rad)
        next_x = current_x + dx
        next_y = current_y + dy

        # Calculate link vertices (in correct order for a rectangle)
        vertices = [
            (current_x+(-(L-D)/2)*math.cos(angle_rad) -W/2*math.sin(angle_rad), current_y+((-(L-D)/2)*math.sin(angle_rad) -(-W/2)*math.cos(angle_rad))),
            (next_x+((L-D)/2)*math.cos(angle_rad) -W/2*math.sin(angle_rad), next_y+(((L-D)/2)*math.sin(angle_rad) -(-W/2)*math.cos(angle_rad))),
            (next_x+((L-D)/2)*math.cos(angle_rad) +W/2*math.sin(angle_rad), next_y+(((L-D)/2)*math.sin(angle_rad) -(W/2)*math.cos(angle_rad))),
            (current_x+(-(L-D)/2)*math.cos(angle_rad) +W/2*math.sin(angle_rad), current_y+((-(L-D)/2)*math.sin(angle_rad) -(W/2)*math.cos(angle_rad))),
        ]
        links.append(vertices)

        current_x, current_y = next_x, next_y
        joint_coordinates.append((current_x, current_y))

    return joint_coordinates, links

def linksplot(links, ax):
    for i, vertices in enumerate(links):
        x = [vertex[0] for vertex in vertices]
        y = [vertex[1] for vertex in vertices]
        x.append(vertices[0][0])  # Add first point again to close the rectangle
        y.append(vertices[0][1])
        ax.plot(x, y, "k-", linewidth=2)
        
        # Label vertices
        for j, (x, y) in enumerate(vertices):
            ax.text(x, y, f'A{i+1}{j+1}', fontsize=9, ha='right', va='bottom')

def joinplot(joint_coordinates, ax):
    x = [pos[0] for pos in joint_coordinates]
    y = [pos[1] for pos in joint_coordinates]
    ax.plot(x, y, "ro", markersize=10)

    # Label joints
    for i, (x, y) in enumerate(joint_coordinates):
        ax.text(x, y, f'J{i+1}', fontsize=9, ha='left', va='bottom', color='black')

def chainplot(joint_coordinates, links, W, L, D):
    fig, ax = plt.subplots(figsize=(12, 8))
    
    linksplot(links, ax)
    joinplot(joint_coordinates, ax)

    plt.title("2D Kinematic Chain")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis('equal')
    plt.grid(True)
    plt.show()

def main():
    args = argument_parser()
    joint_coordinates, links = kinematic_model(args.W, args.L, args.D, args.thetas)
    chainplot(joint_coordinates, links, args.W, args.L, args.D)

if __name__ == "__main__":
    main()