import numpy as np
import matplotlib.pyplot as plt
import math
import time
def circle_coordinates(num_samples, reverse=False):
    coordinates = []
    radius = 1
    for i in range(num_samples):
        angle = 2 * math.pi * i / num_samples
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        coordinates.append((x, y))
    coordinates.append(coordinates[0])  # Repeat the first coordinate at the end
    if reverse:
        coordinates.reverse()
    return coordinates

def square_coordinates(num_samples, edge_length=1, reverse=False):
    samples_per_edge = num_samples // 4

    top = np.column_stack((
        np.linspace(-edge_length/2, edge_length/2, samples_per_edge),
        np.full(samples_per_edge, edge_length/2)
    ))

    right = np.column_stack((
        np.full(samples_per_edge, edge_length/2),
        np.linspace(edge_length/2, -edge_length/2, samples_per_edge)
    ))

    bottom = np.column_stack((
        np.linspace(edge_length/2, -edge_length/2, samples_per_edge),
        np.full(samples_per_edge, -edge_length/2)
    ))

    left = np.column_stack((
        np.full(samples_per_edge, -edge_length/2),
        np.linspace(-edge_length/2, edge_length/2, samples_per_edge)
    ))

    coordinates = np.vstack((top, right, bottom, left))
    coordinates = np.append(coordinates, [coordinates[0]], axis=0)
    if reverse:
        coordinates = np.flip(coordinates, axis=0)
    return coordinates

def infinity_coordinates(num_samples, reverse=False):
    coordinates = []
    a = 1
    b = 0.3
    for i in range(num_samples):
        t = 2 * math.pi * i / num_samples
        x = a * math.cos(t)
        y = b * math.sin(t) * math.cos(t)
        coordinates.append((x, y))
    coordinates.append(coordinates[0])  # Repeat the first coordinate at the end
    if reverse:
        coordinates.reverse()
    return coordinates

# def plot_coordinates(coordinates, pause_time=0.1):
#     x = np.array([c[0] for c in coordinates])
#     y = np.array([c[1] for c in coordinates])
#     plt.ion()  # Turn on interactive mode
#     for i in range(len(x)):
#         plt.plot(x[i], y[i], 'o')
#         plt.axis([2*x.min(), 2*x.max(), 2*y.min(), 2*y.max()])
#         plt.draw()
#         plt.pause(pause_time)
#     plt.ioff()  # Turn off interactive mode
#     plt.show()

# num_samples = 40
# circle_coords = circle_coordinates(num_samples, reverse=True)
# square_coords = square_coordinates(num_samples, reverse=False)
# infinity_coords = infinity_coordinates(num_samples, reverse=False)

# plot_coordinates(circle_coords)
# time.sleep(0.5)
# plot_coordinates(square_coords)
# time.sleep(0.5)
# plot_coordinates(infinity_coords)
# time.sleep(0.5)