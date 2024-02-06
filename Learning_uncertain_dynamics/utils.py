import torch
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


def batch_cov(points):
    """
    Estimates covariance matrix of batches of sets of points
    Args:
        points: torch.tensor of shape (B, N, D), where B is the batch size, N is sample size,
                and D is the dimensionality of the data

    Returns:
        bcov: torch.tensor of shape (B, D, D) of B DxD covariance matrices
    """
    B, N, D = points.size()
    mean = points.mean(dim=1).unsqueeze(1)
    diffs = (points - mean).reshape(B * N, D)
    prods = torch.bmm(diffs.unsqueeze(2), diffs.unsqueeze(1)).reshape(B, N, D, D)
    bcov = prods.sum(dim=1) / (N - 1)  # Unbiased estimate
    return bcov  # (B, D, D)


def rollout_uncertain(model, initial_state, actions):
    pred_mu = [initial_state.clone().reshape(1, 2)]
    pred_sigma = [torch.zeros(1, 2, 2)]

    with torch.no_grad():
        for action in actions:
            mu, sigma = model.propagate_uncertainty(pred_mu[-1],
                                                    pred_sigma[-1],
                                                    action.reshape(-1, 3))
            pred_mu.append(mu)
            pred_sigma.append(sigma)

    return torch.cat(pred_mu, dim=0), torch.cat(pred_sigma, dim=0)


def get_ellipsoid_params_from_cov(cov):
    # 2 x 2 covariance matrix - get w, h, rotation from covariance
    l, v = torch.linalg.eig(cov)
    # find rotation from
    e_x = torch.tensor([1.0, 0.0])

    # just use first eigen vector
    angle = torch.arccos(e_x.reshape(1, 2) @ v[:, 0].real / (torch.linalg.norm(v[:, 0].real)))

    width = l[0].real
    height = l[1].real

    return torch.sqrt(width).item(), torch.sqrt(height).item(), 180 * angle.item() / torch.pi


def plot_uncertainty_propagation(states_true, states_mu, states_sigma, title):
    for i, states in enumerate(states_true):
        if i == 0:
            plt.plot(states[:, 0], states[:, 1], color='b', label='ground truth', alpha=0.7)
        else:
            plt.plot(states[:, 0], states[:, 1], color='b', alpha=0.7)

    plt.plot(states_mu[:, 0], states_mu[:, 1], color='g', label='prediction')
    ax = plt.gca()

    for mu, sigma in zip(states_mu, states_sigma):
        w, h, theta = get_ellipsoid_params_from_cov(sigma)
        ellipse = Ellipse(xy=(mu[0], mu[1]),
                          width=2 * w, height=2 * h, angle=theta,
                          color='g', alpha=0.5)
        ax.add_patch(ellipse)

    ax.add_patch(Ellipse(xy=(states_true[0, 0, 0], states_true[0, 0, 1]),
                         width=0.1, height=0.1, color='k',
                         alpha=0.5, label='Cylinder start'))

    plt.xlim([0.34, 0.625])
    plt.ylim([-0.1, 0.1])
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(title)
    plt.legend()
    plt.show()
