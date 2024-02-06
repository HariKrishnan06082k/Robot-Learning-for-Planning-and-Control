from torch import nn


class MLP(nn.Module):
    """
    Regression approximation via 3-FC NN layers.
    The network input features are one-dimensional as well as the output features.
    The network hidden sizes are 100 and 100.
    Activations are ReLU
    """
    def __init__(self):
        super().__init__()
        # --- Your code here
        self.layers = nn.Sequential(
          nn.Linear(3,128),
          nn.ReLU(),
          nn.Linear(128,128),
          nn.ReLU(),
          nn.Linear(128,2)
        )


        # ---

    def forward(self, x):
        """
        :param x: Tensor of size (N, 3)
        :return: y_hat: Tensor of size (N, 2)
        """
        y_hat = None
        # --- Your code here
        y_hat = self.layers(x)
        # ---
        return y_hat
