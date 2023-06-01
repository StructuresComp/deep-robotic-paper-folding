import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from nfm.utils import float_tensor, get_numpy, ON_GPU


""" Network is outfitted with default args for use of the pretrained model from the manuscript. """


class NeuralForceManifold(nn.Module):
    def __init__(self, input_size=2, output_size=4, hidden_layer_units=(392,)*4,
                 model_weights="nfm/models/model_weights.pth", lgb=0.036546586993887034,
                 means=np.array([0.14337862625005493, 0.0668542666984968]).reshape((1, 2)),
                 stds=np.array([0.12099335598019705, 0.05941409095964373]).reshape((1, 2))):
        super(NeuralForceManifold, self).__init__()
        self.input_size = input_size
        self.output_size = output_size
        self.layers = nn.ModuleList()
        self.layers.append(nn.Linear(self.input_size, hidden_layer_units[0]))
        for i in range(1, len(hidden_layer_units)):
            self.layers.append(nn.Linear(hidden_layer_units[i-1], hidden_layer_units[i]))
        self.layers.append(nn.Linear(hidden_layer_units[-1], self.output_size))

        # Lgb of original training dataset. We trained using a single material + shape.
        self.lgb = lgb

        # Mean and std of the original training dataset
        self.means = means
        self.stds = stds

        # Load pretrained weights
        if ON_GPU:
            self.load_state_dict(torch.load(model_weights))
        else:
            self.load_state_dict(torch.load(model_weights, map_location=torch.device('cpu')))
        self.eval()

    def get_data_params(self):
        return self.means, self.stds

    def preprocess_input(self, orig_data):
        data = orig_data.copy()
        # data[:] *= self.lgb
        data[:] -= self.means
        data[:] /= self.stds
        return data, float_tensor(data)

    def revert_data(self, data):
        data[:] *= self.stds
        data[:] += self.means
        # data[:] /= self.lgb

    # The gradients of NFM were not used in the final method.
    # This is some code that was used when exploring a gradient descent based path planning approach.
    def get_gradient(self, point):
        point = point.reshape((1, 2))
        point.requires_grad = True
        out = self.forward(point)
        theta = out[0, 0].reshape((1, 1))
        ft = out[0, 1].reshape((1, 1))
        gradient = torch.autograd.grad(ft, point)[0]
        gradient /= torch.linalg.norm(gradient)
        return gradient, float(get_numpy(ft)), float(get_numpy(theta))

    # def get_gradient(self, point):
    #     point = point.reshape((1, 2))
    #     point.requires_grad = True
    #     out = self.forward(point)[0, 1].reshape((1, 1))
    #     return torch.autograd.grad(out, point)[0], float(get_numpy(out)), float(get_numpy(out))

    def forward(self, state):
        x_c = state
        for i in range(len(self.layers)-1):
            x_c = F.relu(self.layers[i](x_c))
        output = self.layers[-1](x_c)
        return output
