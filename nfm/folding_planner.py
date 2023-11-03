import numpy as np
import pickle
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from matplotlib.cm import ScalarMappable
from scipy.spatial.transform import Rotation as R
from shapely.geometry import Polygon
from nfm.neural_force_manifold import NeuralForceManifold
from nfm.utils import get_penalty_outline, float_tensor, get_numpy, ON_GPU


class FoldingPlanner:
    def __init__(self, lgb_o, lgb_n):
        self._nn = NeuralForceManifold()
        if ON_GPU:
            self._nn = self._nn.cuda()
            self._device = 'cuda'
        else:
            self._device = 'cpu'

        with open("nfm/misc_data/workspace_outline.pkl", "rb") as f:
            self._ws_edges = pickle.load(f)

        self.lgb_o = lgb_o
        self.lgb_n = lgb_n

        self._ws_edges /= self.lgb_o

        self._ws = Polygon(self._ws_edges[:-1])

    def get_folding_workspace(self):
        return self._ws

    def construct_discretized_manifold(self, discretization=0.01, x_bound=0.4, z_bound=0.2, ls_threshold=0.035):
        num_x = int(x_bound / discretization) + 1
        num_z = int(z_bound / discretization) + 1
        x_vals = [discretization*i for i in range(num_x)]
        z_vals = [discretization*i for i in range(num_z)]
        nn_inp = np.zeros((num_x * num_z, 2))
        for i, x in enumerate(x_vals):
            for j, z in enumerate(z_vals):
                nn_inp[i * num_z + j, 0] = x
                nn_inp[i * num_z + j, 1] = z

        # Generate neural network outputs
        inp = float_tensor(self._nn.preprocess_input(nn_inp))[0]
        all_outputs = get_numpy(self._nn(inp))  # alpha, ft, fn, ls
        ft_fn = (all_outputs[:, 1] / all_outputs[:, 2]).reshape((num_x, num_z))

        # Generate l_s penalty
        alpha_penalty = all_outputs[:, 0] / all_outputs[:, 3]**2
        alpha_penalty[all_outputs[:, -1] > ls_threshold] = 0
        ap = alpha_penalty[all_outputs[:, -1] < ls_threshold].mean()
        idx, idy = np.where((all_outputs[:, -1] < ls_threshold).reshape(num_x, num_z))
        penalty_points = np.zeros((idx.shape[0], 2))
        penalty_points[:, 0] = idx
        penalty_points[:, 1] = idy
        penalty_boundary = get_penalty_outline(penalty_points) * discretization
        alpha_penalty = alpha_penalty.reshape((num_x, num_z))
        fp = ft_fn.ravel()[all_outputs[:, -1] < ls_threshold].mean()

        manifold = np.abs(ft_fn)
        manifold_with_penalty = manifold + 10 * fp / ap * alpha_penalty

        return manifold, manifold_with_penalty, penalty_boundary

    def smooth_trajectory(self, traj, start_alpha=None, num_points=100, weight_data=0.3, weight_smooth=0.7, tolerance=0.000001):
        prev_len = traj.shape[0]
        if num_points is not None:
            new_points = np.zeros((num_points, 3))
            new_points[0, :2] = traj[0, :2]
            new_points[-1, :2] = traj[-1, :2]
            total_dist = 0
            for i in range(traj.shape[0] - 1):
                total_dist += np.linalg.norm(traj[i + 1] - traj[i])
            curr_dist = 0
            fd = []  # fractional distances
            for i in range(traj.shape[0] - 1):
                curr_dist += np.linalg.norm(traj[i + 1] - traj[i])
                fd.append(curr_dist / total_dist)
            fd = np.array(fd)
            traj_dim = traj.shape[1]

            if prev_len > num_points:
                # First we space the coordinates of the trajectory to be equidistant
                # source: https://math.stackexchange.com/questions/321293/find-coordinates-of-equidistant-points-in-bezier-curve
                for i in range(1, num_points-1):
                    d = i / num_points
                    bot = np.where(fd <= d)[0][-1]
                    up = bot + 1

                    u = (d - fd[bot]) / (fd[up] - fd[bot])
                    new_points[i, :traj_dim] = traj[bot] + u * (traj[up] - traj[bot])
            else:
                for i in range(1, num_points-1):
                    d = i / num_points
                    if np.all(fd > d):
                        bot = 0
                    else:
                        bot = np.where(fd <= d)[0][-1]
                    up = bot + 1

                    u = (d - fd[bot]) / (fd[up] - fd[bot])
                    new_points[i, :traj_dim] = traj[bot] + u * (traj[up] - traj[bot])
        else:
            new_points = np.zeros((traj.shape[0], 3))
            new_points[:, :2] = traj

        # Second, we apply a smoothing algorithm to the trajectory
        # source: https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4
        new_traj = new_points.copy()
        dims = 2
        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, new_points.shape[0]-1):
                for j in range(dims):
                    x_i = new_points[i, j]
                    y_i, y_prev, y_next = new_traj[i, j], new_traj[i-1, j], new_traj[i+1, j]

                    y_i_saved = y_i
                    y_i += weight_data * (x_i - y_i) + weight_smooth * (y_next + y_prev - (2 * y_i))
                    new_traj[i, j] = y_i

                    change += abs(y_i - y_i_saved)

        new_traj_tensor = self._nn.preprocess_input(new_traj[:, :2] * self.lgb_o)[1]
        if start_alpha is None:
            new_traj[:, 2] = get_numpy(self._nn(new_traj_tensor)[:, 0])
        else:
            new_alphas = get_numpy(self._nn(new_traj_tensor)[:, 0])
            alpha_diff = start_alpha - new_alphas[0]
            new_traj[:, 2] = new_alphas + np.linspace(1, 0, new_alphas.shape[0]) * alpha_diff

        return new_traj

    def plot_trajectory(self, start, goal, trajectory, penalty_boundary, figsize=(10, 5), ratio=0.225,
                        discretization=0.0005, x_max=0.40, z_max=0.22, threshold_region=None, font_size=18, tick_size=15):
        fig = plt.figure(figsize=figsize)
        ax = fig.add_subplot(111)

        # Generate neural force manifold
        num_x = int(1 / discretization * x_max)
        num_z = int(1 / discretization * z_max)
        x = np.linspace(0, x_max, num_x)
        z = np.linspace(0, z_max, num_z)
        x, z = np.meshgrid(x, z)
        nn_inp = np.zeros((num_x*num_z, 2))
        x = x.ravel()
        z = z.ravel()
        nn_inp[:, 0] = x
        nn_inp[:, 1] = z
        nn_inp -= self._nn.means
        nn_inp /= self._nn.stds
        nn_inp = float_tensor(nn_inp)
        all_outputs = get_numpy(self._nn(nn_inp))
        ft_fn = (all_outputs[:, 1] / all_outputs[:, 2]).reshape((num_z, num_x))
        manifold = np.abs(ft_fn)

        x /= self.lgb_o
        z /= self.lgb_o
        penalty_boundary /= self.lgb_o

        # Visualize neural force manifold
        ax.imshow(manifold, origin='lower', cmap=plt.cm.jet, norm=colors.PowerNorm(ratio, vmin=0, vmax=25), zorder=0,
                  interpolation='bilinear', extent=[0, x.max(), 0, z.max()])

        # Plot concave hull of l_s penalty boundary
        plt.plot(penalty_boundary[:, 0], penalty_boundary[:, 1], c='darkred', linewidth=2, zorder=1)

        # Plot concave hull of workspace boundary
        plt.plot(self._ws_edges[:, 0], self._ws_edges[:, 1], c='magenta', linewidth=2, zorder=1)

        if threshold_region is not None:
            idx = np.where(manifold < threshold_region)
            ax.scatter(x[idx], z[idx], c='red', s=10)

        # Plot trajectory
        ax.plot(trajectory[:, 0], trajectory[:, 1], '--', c='black', linewidth=3, zorder=2)
        ax.scatter(start[0], start[1], c='green', s=50, zorder=3, label="Start State")
        ax.scatter(goal[0], goal[1], c='red', s=50, zorder=3, label="Goal State")

        ax.set_title("Neural Network Force Manifold", fontsize=font_size)
        ax.set_xlabel("$\\bar x$", fontsize=font_size)
        ax.set_ylabel("$\\bar z$", fontsize=font_size)
        ax.tick_params(axis='x', labelsize=tick_size)
        ax.tick_params(axis='y', labelsize=tick_size)
        cbar = fig.colorbar(ScalarMappable(norm=colors.PowerNorm(ratio, vmin=0, vmax=25),
                                           cmap=plt.cm.jet), ax=ax)
        cbar.set_label("$\\lambda$", fontsize=font_size, rotation=270)
        cbar.ax.tick_params(labelsize=tick_size)
        plt.tight_layout()
        plt.show()

    # This function redimensionalizes the generated trajectory and
    # then transforms according to the robot's reference frame.
    # Therefore, this method will be system specific.
    @staticmethod
    def transform_nn_traj_to_robot_traj(traj, Lgb, init_rotation=np.array([[0.0, 0.0, 1.0],
                                                                           [-1.0, 0.0, 0.0],
                                                                           [0.0, -1.0, 0.0]])):
        robot_trajectory = np.zeros((traj.shape[0], 7), dtype=np.float64)
        robot_trajectory[:, 1] = -traj[:, 0]   # robot negative y-axis is the nn positive x-axis
        robot_trajectory[:, 2] = traj[:, 1]    # robot z-axis is equivalent to nn z-axis

        # Redimensionalize the neural force manifold trajectory
        robot_trajectory[:, :3] *= Lgb

        # Convert gripper alphas to quaternions
        last_alpha = traj[:, 2][-1]
        rot_vecs = np.zeros((traj.shape[0], 3), dtype=np.float64)
        rot_vecs[:, 2] = traj[:, 2]
        r = R.from_rotvec(-rot_vecs).as_matrix()
        for i in range(r.shape[0]):
            final = init_rotation @ r[i]
            robot_trajectory[i, 3:] = R.from_matrix(final).as_quat()

        return robot_trajectory, last_alpha
