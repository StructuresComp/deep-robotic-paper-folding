import torch
import numpy as np
import pickle
from scipy.spatial import Delaunay


global ON_GPU
if torch.cuda.is_available():
    float_tensor = torch.cuda.FloatTensor
    get_numpy = lambda x: x.detach().cpu().numpy()
    ON_GPU = True
else:
    float_tensor = torch.FloatTensor
    get_numpy = lambda x: x.data.numpy()
    ON_GPU = False


""" 
    Below is code to generate concave hulls for a set of points.
    This is useful for visualizing both the l_s penalty boundary as well the folding workspace. 
"""


# from https://karobben.github.io/2022/03/07/Python/point_outline/
def alpha_shape(points, alpha, only_outer=True):
    """
    Compute the alpha shape (concave hull) of a set of points.
    :param points: np.array of shape (n,2) points.
    :param alpha: alpha value.
    :param only_outer: boolean value to specify if we keep only the outer border
    or also inner edges.
    :return: set of (i,j) pairs representing edges of the alpha-shape. (i,j) are
    the indices in the points array.
    """
    assert points.shape[0] > 3, "Need at least four points"

    def add_edge(edges, i, j):
        """
        Add an edge between the i-th and j-th points,
        if not in the list already
        """
        if (i, j) in edges or (j, i) in edges:
            # already added
            assert (j, i) in edges, "Can't go twice over same directed edge right?"
            if only_outer:
                # if both neighboring triangles are in shape, it's not a boundary edge
                edges.remove((j, i))
            return
        edges.add((i, j))

    tri = Delaunay(points)
    edges = set()
    # Loop over triangles:
    # ia, ib, ic = indices of corner points of the triangle
    for ia, ib, ic in tri.vertices:
        pa = points[ia]
        pb = points[ib]
        pc = points[ic]
        # Computing radius of triangle circumcircle
        # www.mathalino.com/reviewer/derivation-of-formulas/derivation-of-formula-for-radius-of-circumcircle
        a = np.sqrt((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2)
        b = np.sqrt((pb[0] - pc[0]) ** 2 + (pb[1] - pc[1]) ** 2)
        c = np.sqrt((pc[0] - pa[0]) ** 2 + (pc[1] - pa[1]) ** 2)
        s = (a + b + c) / 2.0
        area = np.sqrt(s * (s - a) * (s - b) * (s - c))
        circum_r = a * b * c / (4.0 * area)
        if circum_r < alpha:
            add_edge(edges, ia, ib)
            add_edge(edges, ib, ic)
            add_edge(edges, ic, ia)
    return edges


def get_penalty_outline(penalty_points):
    edges = alpha_shape(penalty_points, alpha=1)

    edges = list(edges)
    node_ids = [0]
    counter = 0
    while counter < len(edges):
        curr_element = node_ids[-1]
        for edge in edges:
            if edge[0] == curr_element:
                node_ids.append(edge[1])
                counter += 1
                break

    boundary_shape = np.zeros((len(node_ids), 2), dtype=np.float64)
    for n, i in enumerate(node_ids):
        boundary_shape[n, 0] = penalty_points[i, 0]
        boundary_shape[n, 1] = penalty_points[i, 1]

    return boundary_shape


def get_workspace_outline():
    with open("interpolation_data.pkl", "rb") as f:
        dataset, _ = pickle.load(f)

    xz = dataset[:, :2]
    edges = alpha_shape(xz, alpha=1)

    edges = list(edges)
    node_ids = [0]
    counter = 0
    while counter < len(edges):
        curr_element = node_ids[-1]
        for edge in edges:
            if edge[0] == curr_element:
                node_ids.append(edge[1])
                counter += 1
                break

    boundary_shape = np.zeros((len(node_ids), 2), dtype=np.float64)
    for n, i in enumerate(node_ids):
        boundary_shape[n, 0] = xz[i, 0]
        boundary_shape[n, 1] = xz[i, 1]

    with open("workspace_outline.pkl", "wb") as f:
        pickle.dump(boundary_shape, f)


if __name__ == "__main__":
    get_workspace_outline()
