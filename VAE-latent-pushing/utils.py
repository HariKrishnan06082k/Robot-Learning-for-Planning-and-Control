import torch
import numpy as np
import matplotlib.pyplot as plt

def view_states(states):
    fig, axes = plt.subplots(1, len(states),sharey=True, figsize=(2*len(states),2))

    for i, state_i in enumerate(states):
        ax_i = axes[i]
        ax_i.imshow(state_i, cmap='gray')
    plt.show()


def evaluate_model_plot(model, traj, norm_tr):
    states = torch.tensor(traj['states']).permute(0, 3, 1, 2).to(torch.float32)
    actions = torch.tensor(traj['actions'])
    states_norm = norm_tr.normalize_state(states)
    traj_len = actions.shape[0]

    # prediction:
    pred_states_multistep = []
    prev_state = states_norm[0].unsqueeze(0)

    for i, action_i in enumerate(actions):
        next_state = model(prev_state, action_i.unsqueeze(0))
        pred_states_multistep.append(next_state.squeeze(0))
        prev_state = next_state

    pred_states_singlestep = model(states_norm[:-1], actions)

    pred_states_multistep = [norm_tr.denormalize_state(s) for s in pred_states_multistep]
    pred_states_singlestep = norm_tr.denormalize_state(pred_states_singlestep)

    fig = plt.figure(constrained_layout=True, figsize=(22, 6))

    subfigs = fig.subfigures(nrows=3, ncols=1)
    subfigs[0].suptitle('Ground truth image', fontsize=24)
    subfigs[1].suptitle('Single-step reconstruction', fontsize=24)
    subfigs[2].suptitle('Multi-step reconstruction', fontsize=24)

    # fig, axes = plt.subplots(3,traj_len+1, sharex=True, sharey=True, figsize=(2*(traj_len+1), 6))
    axes = []
    for subfig in subfigs:
        axes.append(subfig.subplots(1, traj_len + 1, sharex=True, sharey=True))
    for i in range(traj_len + 1):
        if i == 0:
            pred_state_ss = states[0]
            pred_state_ms = states[0]
        else:
            pred_state_ss = pred_states_singlestep[i - 1]
            pred_state_ms = pred_states_multistep[i - 1]
        state_gth = states[i]

        pred_state_ss = pred_state_ss.detach().cpu().numpy().transpose(1, 2, 0).astype(np.uint8)
        pred_state_ms = pred_state_ms.detach().cpu().numpy().transpose(1, 2, 0).astype(np.uint8)
        state_gth = state_gth.detach().cpu().numpy().transpose(1, 2, 0).astype(np.uint8)

        axes[0][i].imshow(state_gth, cmap='gray')
        axes[1][i].imshow(pred_state_ss, cmap='gray')
        axes[2][i].imshow(pred_state_ms, cmap='gray')