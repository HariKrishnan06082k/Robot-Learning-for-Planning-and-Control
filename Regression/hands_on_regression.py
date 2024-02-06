import torch
from typing import List, Tuple
from torch import Tensor
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.autograd import Variable
from tqdm import tqdm


##############################################################################
# HANDS ON REGRESSION


def polynomial_basis_functions(xs: Tensor, d: int) -> Tensor:
    """
    Extends the input array to a series of polynomial basis functions of it.
    Args:
        xs: torch.Tensor (N, num_feats)
        d: Integer representing the degree of the polynomial basis functions
    Returns:
        Xs: torch.Tensor of shape (N, d*num_feats) containing the basis functions for the
        i.e. [1, x, x**2, x**3,...., x**d]
    """
    Xs = None
    # --- Your code here
    one_col = torch.ones((xs.shape[0],1),dtype = xs.dtype)
    Xs = torch.cat([one_col,xs],axis=1)
    for i in range(1,d):
      x_pow = xs.pow(i+1)
      Xs = torch.cat([Xs,x_pow],axis=1)
    # ---
    return Xs


def compute_least_squares_solution(Xs: Tensor, ys: Tensor) -> Tensor:
    """
    Compute the Least Squares solution that minimizes the MSE(Xs@coeffs, ys)
    Args:
        Xs: torch.Tensor shape (N,m)
        ys: Torch.Tensor of shape (N,1)
    Returns:
        coeffs: torch.Tensor of shape (m,) containing the optimal coefficients
    
    NOTE: You may need to compute the inverse of a matrix. Typically, computing 
    matrix inverses are a costly operation. Instead, given a linear system Ax = b,
    the solution can be computed much more efficient as x = torch.linalg.solve(A, b)
    """
    coeffs = None
    # --- Your code here
    xdx_inv = (Xs.T.mm(Xs)).inverse()
    xdy = Xs.T.mm(ys)
    coeffs = xdx_inv.mm(xdy).squeeze()
    # ---

    return coeffs


def get_normalization_constants(Xs: Tensor) -> Tuple:
    mean_i = None
    std_i = None
    # --- Your code here
    mean_i = torch.mean(Xs,dim=0)
    std_i = torch.std(Xs,dim=0)
    # ---
    return mean_i, std_i


def normalize_tensor(Xs: Tensor, mean_i: Tensor, std_i: Tensor) -> Tensor:
    """
    Normalize the given tensor Xs
    :param Xs: torch.Tensor of shape (batch_size, num_features)
    :return: Normalized version of Xs
    """
    Xs_norm = None
    # --- Your code here
    Xs_norm = (Xs - mean_i) / (std_i)
    # ---
    Xs_norm = torch.nan_to_num(Xs_norm, nan=0.0) # avoid NaNs.
    return Xs_norm


def denormalize_tensor(Xs_norm: Tensor, mean_i: Tensor, std_i: Tensor) -> Tensor:
    """
        Normalize the given tensor Xs
        :param Xs: torch.Tensor of shape (batch_size, num_features)
        :return: Normalized version of Xs
        """
    Xs_denorm = None
    # --- Your code here
    Xs_denorm = (Xs_norm * std_i) + mean_i


    # ---
    return Xs_denorm


class LinearRegressor(nn.Module):
    """
    Linear regression implemented as a neural network.
    The learnable coefficients can be easily implemented via linear layers without bias.
    The network regression output is one-dimensional.

    """
    def __init__(self, num_in_feats):
        super().__init__()
        self.num_in_feats = num_in_feats # number of regression input features
        # Define trainable
        self.coeffs = None # TODO: Override with the learnable regression coefficients
        # --- Your code here

        self.coeffs = nn.Linear(num_in_feats,1,bias=False)

        # ---

    def forward(self, x):
        """
        :param x: Tensor of size (N, num_in_feats)
        :return: y_hat: Tensor of size (N, 1)
        """
        y_hat = None
        # --- Your code here
        y_hat = self.coeffs(x)
        # ---
        return y_hat

    def get_coeffs(self):
        return self.coeffs.weight.data


class GeneralNN(nn.Module):
    """
    Regression approximation via 3-FC NN layers.
    The network input features are one-dimensional as well as the output features.
    The network hidden sizes are 100 and 100.
    Activations are Tanh
    """
    def __init__(self):
        super().__init__()
        # --- Your code here
        self.layers = nn.Sequential(
          nn.Linear(1,100),
          nn.Tanh(),
          nn.Linear(100,100),
          nn.Tanh(),
          nn.Linear(100,1)
        )
        # ---

    def forward(self, x):
        """
        :param x: Tensor of size (N, 1)
        :return: y_hat: Tensor of size (N, 1)
        """
        y_hat = None
        #import pdb; pdb.set_trace()
        # --- Your code here
        y_hat = self.layers(x)
        # ---
        return y_hat


def train_step(model, train_loader, optimizer) -> float:
    """
    Performs an epoch train step.
    :param model: Pytorch nn.Module
    :param train_loader: Pytorch DataLoader
    :param optimizer: Pytorch optimizer
    :return: train_loss <float> representing the average loss among the different mini-batches.
        Loss needs to be MSE loss.
    """
    train_loss = 0. # TODO: Modify the value
    # Initialize the train loop
    # --- Your code here
    model.train()
    # ---
    for batch_idx, (data, target) in enumerate(train_loader):
        # --- Your code here
        output = model(data)
        loss = F.mse_loss(output,target)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        # ---
        train_loss += loss.item()
    return train_loss/len(train_loader)


def val_step(model, val_loader) -> float:
    """
    Perfoms an epoch of model performance validation
    :param model: Pytorch nn.Module
    :param train_loader: Pytorch DataLoader
    :param optimizer: Pytorch optimizer
    :return: val_loss <float> representing the average loss among the different mini-batches
    """
    val_loss = 0. # TODO: Modify the value
    # Initialize the validation loop
    # --- Your code here

    model.eval()

    # ---
    for batch_idx, (data, target) in enumerate(val_loader):
        loss = None
        # --- Your code here
        output = model(data)
        loss = F.mse_loss(output,target)
        # ---
        val_loss += loss.item()
    return val_loss/len(val_loader)


def train_model(model, train_dataloader, val_dataloader, num_epochs=100, lr=1e-3):
    """
    Trains the given model for `num_epochs` epochs. Use SGD as an optimizer.
    You may need to use `train_step` and `val_step`.
    :param model: Pytorch nn.Module.
    :param train_dataloader: Pytorch DataLoader with the training data.
    :param val_dataloader: Pytorch DataLoader with the validation data.
    :param num_epochs: <int> number of epochs to train the model.
    :param lr: <float> learning rate for the weight update.
    :return:
    """
    optimizer = None
    # Initialize the optimizer
    # --- Your code here
    optimizer  = optim.SGD(model.parameters(), lr=lr)
    # ---
    pbar = tqdm(range(num_epochs))
    train_losses = []
    val_losses = []
    for epoch_i in pbar:
        train_loss_i = None
        val_loss_i = None
        # --- Your code here
        train_loss_i = train_step(model,train_dataloader,optimizer)
        val_loss_i  = val_step(model,val_dataloader)
        # ---
        pbar.set_description(f'Train Loss: {train_loss_i:.4f} | Validation Loss: {val_loss_i:.4f}')
        train_losses.append(train_loss_i)
        val_losses.append(val_loss_i)
    return train_losses, val_losses



