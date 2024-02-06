import pdb

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader, random_split
from tqdm import tqdm
from mppi import MPPI
from panda_pushing_env import TARGET_POSE_FREE, TARGET_POSE_OBSTACLES, OBSTACLE_HALFDIMS, OBSTACLE_CENTRE, BOX_SIZE
import control

TARGET_POSE_FREE_TENSOR = torch.as_tensor(TARGET_POSE_FREE, dtype=torch.float32)
TARGET_POSE_OBSTACLES_TENSOR = torch.as_tensor(TARGET_POSE_OBSTACLES, dtype=torch.float32)
OBSTACLE_CENTRE_TENSOR = torch.as_tensor(OBSTACLE_CENTRE, dtype=torch.float32)[:2]
OBSTACLE_HALFDIMS_TENSOR = torch.as_tensor(OBSTACLE_HALFDIMS, dtype=torch.float32)[:2]


def collect_data_random_trajectory(env, num_trajectories=1000, trajectory_length=10):
    """
    Collect data from the provided environment using uniformly random exploration.
    :param env: Gym Environment instance.
    :param num_trajectories: <int> number of data to be collected.
    :param trajectory_length: <int> number of state transitions to be collected
    :return: collected data: List of dictionaries containing the state-action trajectories.
    Each trajectory dictionary should have the following structure:
        {'states': states,
        'actions': actions}
    where
        * states is a numpy array of shape (trajectory_length+1, 32, 32, num_channels) containing the states [x_0, ...., x_T]
        * actions is a numpy array of shape (trajectory_length, actions_size) containing the actions [u_0, ...., u_{T-1}]
    Each trajectory is:
        x_0 -> u_0 -> x_1 -> u_1 -> .... -> x_{T-1} -> u_{T_1} -> x_{T}
        where x_0 is the state after resetting the environment with env.reset()
    All data elements must be encoded as np.float32.
    """
    collected_data = None
    # --- Your code here
    # collected_data = []
    # for i in range(num_trajectories):
    #     states = np.zeros((trajectory_length + 1, 32, 32, 3), dtype=np.uint8)
    #     actions = np.zeros((trajectory_length, 3), dtype=np.float32)
    #     state = env.reset()
    #     states[0] = state
    #     # print(i)
    #     for t in range(trajectory_length):
    #         action = env.action_space.sample()
    #         next_state, _, done, _ = env.step(action)
    #         while done:
    #             state = env.reset()
    #             next_state, _, done, _ = env.step(env.action_space.sample())
    #         states[t + 1] = next_state
    #         actions[t] = action
    #     collected_data.append({'states': states, 'actions': actions})
    collected_data = list()
    

    for i in range(num_trajectories):
      state_temp = np.zeros((trajectory_length+1,32,32,1),dtype=np.uint8)
      action_temp = np.zeros((trajectory_length,3),dtype=np.float32)
      state_0 = env.reset()
      state_temp[0,:,:,:] = state_0
      #print(i)
      for j in range(trajectory_length):
        action_j = env.action_space.sample()
        state_temp[j+1,:,:,:],_,done,_ = env.step(action_temp[j,:])
        if done==True:
          continue
      state_temp = np.asarray(state_temp,dtype=np.uint8)
      action_temp = np.asarray(action_temp,dtype=np.float32)
      collected_data.append({"states":state_temp,"actions":action_temp})


    # ---
    return collected_data


class NormalizationTransform(object):

    def __init__(self, norm_constants):
        self.norm_constants = norm_constants
        self.mean = norm_constants['mean']
        self.std = norm_constants['std']

    def __call__(self, sample):
        """
        Transform the sample by normalizing the 'states' using the provided normalization constants.
        :param sample: dictionary containing {'states', 'actions'}
        :return:
        """
        # --- Your code here

        sample["states"] = self.normalize_state(sample["states"])
        # ---
        return sample

    def inverse(self, sample):
        """
        Transform the sample by de-normalizing the 'states' using the provided normalization constants.
        :param sample: dictionary containing {'states', 'actions'}
        :return:
        """
        # --- Your code here
        sample["states"] = self.denormalize_state(sample["states"])
        # ---
        return sample

    def normalize_state(self, state):
        """
        Normalize the state using the provided normalization constants.
        :param state: <torch.tensor> of shape (..., num_channels, 32, 32)
        :return: <torch.tensor> of shape (..., num_channels, 32, 32)
        """
        # --- Your code here

        #state_norm = (torch.tensor(state, dtype=torch.float32)- torch.tensor(self.mean, dtype=torch.float32)) / torch.tensor(self.std, dtype=torch.float32)
        state = (state - self.mean)/self.std
        # ---
        return state

    def denormalize_state(self, state_norm):
        """
        Denormalize the state using the provided normalization constants.
        :param state_norm: <torch.tensor> of shape (..., num_channels, 32, 32)
        :return: <torch.tensor> of shape (..., num_channels, 32, 32)
        """
        # --- Your code here
        
        state = (state_norm*self.std) + self.mean

        # ---
        return state


def process_data_multiple_step(collected_data, batch_size=500, num_steps=4):
    """
    Process the collected data and returns a DataLoader for train and one for validation.
    The data provided is a list of trajectories (like collect_data_random output).
    Each DataLoader must load dictionary as
    {'states': x_t,x_{t+1}, ... , x_{t+num_steps}
     'actions': u_t, ..., u_{t+num_steps-1},
    }
    where:
     states: torch.float32 tensor of shape (batch_size, num_steps+1, state_size)
     actions: torch.float32 tensor of shape (batch_size, num_steps, state_size)

    Each DataLoader must load dictionary dat
    The data should be split in a 80-20 training-validation split.

    :param collected_data:
    :param batch_size: <int> size of the loaded batch.
    :param num_steps: <int> number of steps to load the multistep data.

    :return train_loader: <torch.utils.data.DataLoader> for training
    :return val_loader: <torch.utils.data.DataLoader> for validation
    :return normalization_constants: <dict> containing the mean and std of the states.

    Hints:
     - Pytorch provides data tools for you such as Dataset and DataLoader and random_split
     - You should implement MultiStepDynamicsDataset below.
        This class extends pytorch Dataset class to have a custom data format.
    """
    train_data = None
    val_data = None
    normalization_constants = {
        'mean': None,
        'std': None,
    }
    # Your implemetation needs to do the following:
    #  1. Initialize dataset
    #  2. Split dataset,
    #  3. Estimate normalization constants for the train dataset states.
    # --- Your code here
    dataset = MultiStepDynamicsDataset(collected_data,num_steps)
    train_size = int(0.8 * len(dataset))
    val_size = len(dataset) - train_size
    train_data, val_data = random_split(dataset,[train_size,val_size],generator=torch.Generator().manual_seed(42))
    # 3. Estimate normalization constants for the train dataset states.
    normalization_constants["mean"] = 0
    normalization_constants["std"] = 0
    
    for data in train_data:
      states = data["states"]
      normalization_constants["mean"] += states.mean()
      normalization_constants["std"] += states.std()

    normalization_constants["mean"] /= train_size
    normalization_constants["std"] /= train_size

    # ---
    norm_tr = NormalizationTransform(normalization_constants)
    train_data.dataset.transform = norm_tr
    val_data.dataset.transform = norm_tr

    train_loader = DataLoader(train_data, batch_size=batch_size, shuffle=True)
    # for i,ele in enumerate(train_loader):

    #   print(ele['states'].shape)
    val_loader = DataLoader(val_data, batch_size=batch_size)


    return train_loader, val_loader, normalization_constants


class MultiStepDynamicsDataset(Dataset):
    """
    Dataset containing multi-step dynamics data.
    Each data sample is a dictionary containing (state, action, next_state) in the form:
    {'states':[x_{t}, x_{t+1},..., x_{t+num_steps} ] -- initial state of the multipstep torch.float32 tensor of shape (state_size,)
     'actions': [u_t,..., u_{t+num_steps-1}] -- actions applied in the muli-step.
                torch.float32 tensor of shape (num_steps, action_size)
    }

    Observation: If num_steps=1, this dataset is equivalent to SingleStepDynamicsDataset.
    """

    def __init__(self, collected_data, num_steps=4, transform=None):
        self.data = collected_data
        self.trajectory_length = self.data[0]['actions'].shape[0] - num_steps + 1 # 7
        self.num_steps = num_steps
        self.transform = transform

    def __len__(self):
        return len(self.data) * (self.trajectory_length)

    def __iter__(self):
        for i in range(self.__len__()):
            yield self.__getitem__(i)

    def __getitem__(self, item):
        """
        Return the data sample corresponding to the index <item>.
        :param item: <int> index of the data sample to produce.
            It can take any value in range 0 to self.__len__().
        :return: data sample corresponding to encoded as a dictionary with keys (states, actions).
        The class description has more details about the format of this data sample.
        """
        sample = {
            'states': None,
            'actions': None,
        }
        # --- Your code here
        state_idx = item - ((self.trajectory_length)*(item//(self.trajectory_length)))
        action_idx = item - ((self.trajectory_length)*(item//(self.trajectory_length)))
        sample["states"] = torch.from_numpy(self.data[item//(self.trajectory_length)]["states"][state_idx:state_idx+self.num_steps+1]).permute(0,3,1,2).to(dtype=torch.float)
        sample["actions"] = torch.from_numpy(self.data[item//(self.trajectory_length)]["actions"][action_idx:action_idx+(self.num_steps)]).to(dtype=torch.float)
        if self.transform:
            sample = self.transform(sample)           
        # ---
        return sample


class VAELoss(nn.Module):
    def __init__(self, beta=1.0):
        super().__init__()
        self.beta = beta  # Weight of the KL divergence term

    def forward(self, x_hat, x, mu, logvar):
        """
        Compute the VAE loss.
        vae_loss = MSE(x, x_hat) + beta * KL(N(\mu, \sigma), N(0, 1))
        where KL(N(\mu, \sigma), N(0, 1)) = \log \frac{1}{\sigma} + \frac{\sigma^2 + \mu^2}{2} - \frac{1}{2}
        :param x: <torch.tensor> ground truth tensor of shape (batch_size, state_size)
        :param x_hat: <torch.tensor> reconstructed tensor of shape (batch_size, state_size)
        :param mu: <torch.tensor> of shape (batch_size, state_size)
        :param logvar: <torch.tensor> of shape (batch_size, state_size)
        :return: <torch.tensor> scalar loss
        """
        loss = None
        # --- Your code here
        lossfcn = nn.MSELoss()
        a = torch.log(1/torch.sqrt(torch.exp(logvar)))
        b = (torch.exp(logvar) + mu**2)/2
        c = (1/2)
        KL = a+b-c
        KL = torch.mean(torch.sum(KL, dim=1))
        # KL = torch.mean(KL)

        # KL = torch.mean(KL)     ##CHECK IT AGAIN maybe times 16 or sum and then mean 
        loss = lossfcn(x,x_hat)+(self.beta * KL)
        #print(loss)

        # ---
        return loss


class MultiStepLoss(nn.Module):
    def __init__(self, state_loss_fn, latent_loss_fn, alpha=0.1):
        super().__init__()
        self.state_loss = state_loss_fn
        self.latent_loss = latent_loss_fn
        self.alpha = alpha

    def forward(self, model, states, actions):
        """
        Compute the multi-step loss resultant of multi-querying the model from (state, action) and comparing the predictions with targets.
        Here the loss is computed based on 3 terms:
         - reconstruction loss: enforces good encoding and reconstruction of the states (no dynamics).
         - latent loss: enforces dynamics on latent space by matching the state encodings with the dynamics on latent space.
         - prediction loss: enforces reconstruction of the predicted states by matching the predictions with the targets.

         :param model: <nn.Module> model to be trained.
         :param states: <torch.tensor> tensor of shape (batch_size, traj_size + 1, state_size)
         :param actions: <torch.tensor> tensor of shape (batch_size, traj_size, action_size)
        """
        # compute reconstruction loss -- compares the encoded-decoded states with the original states
        rec_loss = 0.
        # --- Your code here
        T = actions.shape[1]
        latent_values = model.encode(states)
        reconstruction = model.decode(latent_values)
        rec_loss = self.state_loss(reconstruction,states)
        # ---
        # propagate dynamics on latent space as well as reconstructed states
        pred_latent_values = []
        pred_states = []
        prev_z = latent_values[:, 0, :]  # get initial latent value
        prev_state = states[:, 0, :]  # get initial state value
        for t in range(actions.shape[1]):
            next_z = None
            next_state = None
            # --- Your code here
            # print(states.shape)
            # print(actions.shape)
            next_z = model.latent_dynamics(prev_z,actions[:,t,:])
            next_state = model(prev_state, actions[:,t,:])
            pred_latent_values.append(next_z)
            pred_states.append(next_state)
            # ---
            prev_z = next_z
            prev_state = next_state

        pred_states = torch.stack(pred_states, dim=1)
        pred_latent_values = torch.stack(pred_latent_values, dim=1)
        # compute prediction loss -- compares predicted state values with the given states
        pred_loss = 0.
        # --- Your code here

        pred_loss = self.state_loss(pred_states,states[:,1:])

        # ---

        # compute latent loss -- compares predicted latent values with the encoded latent values for states
        lat_loss = 0.
        # --- Your code here

        lat_loss = self.latent_loss(pred_latent_values,latent_values[:,1:])

        # ---

        multi_step_loss = rec_loss + pred_loss + self.alpha * lat_loss

        return multi_step_loss


class StateEncoder(nn.Module):
    """
    Embeds the state into a latent space.
    State shape: (..., num_channels, 32, 32)
    latent shape: (..., latent_dim)
    Check the notebook for more details about the architecture.
    """

    def __init__(self, latent_dim, num_channels=3):
        super().__init__()
        self.latent_dim = latent_dim
        self.num_channels = num_channels
        # --- Your code here

        # HARI #
        sample_net_conv = []
        sample_net_conv.append(nn.Conv2d(in_channels=self.num_channels, out_channels=4, kernel_size=5, stride=1, padding=0, dilation=1))
        sample_net_conv.append(nn.ReLU(inplace=True))
        sample_net_conv.append(nn.MaxPool2d(kernel_size=2))
        sample_net_conv.append(nn.Conv2d(in_channels=4, out_channels=4, kernel_size=5, stride=1, padding=0, dilation=1))
        sample_net_conv.append(nn.ReLU(inplace=True))
        sample_net_conv.append(nn.MaxPool2d(kernel_size=2))
        
        #declaring the linear layers of encoder architecture
        sample_net_fc = []
        sample_net_fc.append(nn.Linear(in_features=100,out_features=100))
        sample_net_fc.append(nn.ReLU(inplace=True))
        sample_net_fc.append(nn.Linear(in_features=100,out_features=self.latent_dim))
        #sample_net_fc.append(nn.ReLU(inplace=True))

        #init layers
        #not given any specific info on how to init the conv layers!
        self.conv_enc = nn.Sequential(*sample_net_conv)
        self.linear_enc = nn.Sequential(*sample_net_fc)
        # ---

    def forward(self, state):
        """
        :param state: <torch.Tensor> of shape (..., num_channels, 32, 32)
        :return latent_state: <torch.Tensor> of shape (..., latent_dim)
        """
        latent_state = None
        input_shape = state.shape
        state = state.reshape(-1, self.num_channels, 32, 32)
        # --- Your code here
        out_feats = self.conv_enc(state) #should be (B,4,5,5)
        out = out_feats.flatten(1) #should be (B,100)
        latent_state = self.linear_enc(out) #should be (B,100)
        # ---
        # convert to original multi-batch shape
        latent_state = latent_state.reshape(*input_shape[:-3], self.latent_dim)
        return latent_state


class StateVariationalEncoder(nn.Module):
    """
    Embeds the state into a latent space.
    State shape: (..., num_channels, 32, 32)
    latent shape: (..., latent_dim)
    Check the notebook for more details about the architecture.
    """

    def __init__(self, latent_dim, num_channels=3):
        super().__init__()
        self.latent_dim = latent_dim
        self.num_channels = num_channels
        # --- Your code here

        # HARI #
        sample_net_conv = []
        sample_net_conv.append(nn.Conv2d(in_channels=self.num_channels, out_channels=4, kernel_size=5, stride=1, padding=0, dilation=1))
        sample_net_conv.append(nn.ReLU(inplace=True))
        sample_net_conv.append(nn.MaxPool2d(kernel_size=2))
        sample_net_conv.append(nn.Conv2d(in_channels=4, out_channels=4, kernel_size=5, stride=1, padding=0, dilation=1))
        sample_net_conv.append(nn.ReLU(inplace=True))
        sample_net_conv.append(nn.MaxPool2d(kernel_size=2))
        
        #declaring the linear layers of encoder architecture
        sample_net_fc = []

        sample_net_fc.append(nn.Linear(in_features=100,out_features=100))
        sample_net_fc.append(nn.ReLU(inplace=True))

        #ModuleDict for using mu and log_var
        self.dist = nn.ModuleDict({
          'mu': nn.Linear(in_features=100,out_features=self.latent_dim),
          'log_var': nn.Linear(in_features=100,out_features=self.latent_dim),
          'relu': nn.ReLU(inplace=True)
        })
       
        #init layers
        #not given any specific info on how to init the conv layers!
        self.conv_enc = nn.Sequential(*sample_net_conv)
        self.linear_enc = nn.Sequential(*sample_net_fc)
        # ---

    def forward(self, state):
        """
        :param state: <torch.Tensor> of shape (..., num_channels, 32, 32)
        :return: 2 <torch.Tensor>
          :mu: <torch.Tensor> of shape (..., latent_dim)
          :log_var: <torch.Tensor> of shape (..., latent_dim)
        """
        mu = None
        log_var = None
        input_shape = state.shape
        state = state.reshape(-1, self.num_channels, 32, 32)
        # --- Your code here
        out_feats = self.conv_enc(state) #should be (B,4,5,5)
        out = out_feats.flatten(1) #should be (B,100)
        latent_state = self.linear_enc(out) #should be (B,100)

        mu = self.dist['mu'](latent_state) #should be of size (B,100)
        #mu = self.dist['relu'](mean) #final mu should be again (B,100)

        #linear outputs of mean and log var separately after passing 
        #through ReLU
        log_var = self.dist['log_var'](latent_state) #should be of size (B,100)
        #log_var = self.dist['relu'](var) #should be of size (B,100)
        # ---
        # convert to original multi-batch shape
        mu = mu.reshape(*input_shape[:-3], self.latent_dim)
        log_var = log_var.reshape(*input_shape[:-3], self.latent_dim)
        return mu, log_var

    def reparameterize(self, mu, logvar):
        """
        Reparametrization trick to sample from N(mu, std) from N(0,1)
        :param mu: <torch.Tensor> of shape (..., latent_dim)
        :param logvar: <torch.Tensor> of shape (..., latent_dim)
        :return: <torch.Tensor> of shape (..., latent_dim)
        """
        sampled_latent_state = None
        # --- Your code here
        std = torch.exp(logvar)
        std = torch.sqrt(std)
        #eps = torch.randn_like(std)
        eps = torch.randn(logvar.size())

        #should be of size (B,100) again. 
        sampled_latent_state = mu + (eps*std)


        # ---
        return sampled_latent_state


class StateDecoder(nn.Module):
    """
    Reconstructs the state from a latent space.
    Check the notebook for more details about the architecture.
    """

    def __init__(self, latent_dim, num_channels=3):
        super().__init__()
        self.latent_dim = latent_dim
        self.num_channels = num_channels
        # --- Your code here
        #declaring the decoder architecture
        sample_net_decoder = []  #latent should be (B,100)
        sample_net_decoder.append(nn.Linear(in_features=self.latent_dim, out_features=500))
        sample_net_decoder.append(nn.ReLU(inplace=True))
        sample_net_decoder.append(nn.Linear(in_features=500 , out_features=500))
        sample_net_decoder.append(nn.ReLU(inplace=True))
        sample_net_decoder.append(nn.Linear(in_features=500, out_features=self.num_channels*32*32))

        #doubtful but according to architecture the final output comes out of 
        #ReLU if KL loss isn't doing well might have to comment below line
        #sample_net_decoder.append(nn.ReLU(inplace=True))
        
        #init the seq list built based on above decoder arch list
        self.decoder = nn.Sequential(*sample_net_decoder)

        # ---

    def forward(self, latent_state):
        """
        :param latent_state: <torch.Tensor> of shape (..., latent_dim)
        :return decoded_state: <torch.Tensor> of shape (..., num_channels, 32, 32)
        """
        decoded_state = None
        input_shape = latent_state.shape
        latent_state = latent_state.reshape(-1, self.latent_dim)

        # --- Your code here
        #should be of size (B,3*32*32)
        decoded_state = self.decoder(latent_state) 
        # ---
        #below line should reshape the linear layer to dimension of image
        #shape of this should be (B,num_channels(3),32,32)
        decoded_state = decoded_state.reshape(*input_shape[:-1], self.num_channels, 32, 32)
        # ---
        return decoded_state


class StateVAE(nn.Module):
    """
    State AutoEncoder
    """

    def __init__(self, latent_dim, num_channels=3):
        super().__init__()
        self.latent_dim = latent_dim
        self.num_channels = num_channels
        self.encoder = StateVariationalEncoder(latent_dim, num_channels)
        self.decoder = StateDecoder(latent_dim, num_channels)

    def forward(self, state):
        """
        :param state: <torch.Tensor> of shape (..., num_channels, 32, 32)
        :return:
            reconstructed_state: <torch.Tensor> of shape (..., num_channels, 32, 32)
            mu: <torch.Tensor> of shape (..., latent_dim)
            log_var: <torch.Tensor> of shape (..., latent_dim)
            latent_state: <torch.Tensor> of shape (..., latent_dim)
        """
        reconstructed_state = None # decoded states from the latent_state
        mu, log_var = None, None # mean and log variance obtained from encoding state
        latent_state = None # sample from the latent space feeded to the decoder
        # --- Your code here
        #get mu and log_var from self.encoder
        mu , log_var = self.encoder(state)

        #get sampleed_latent_state from self.encode
        latent_state = self.encode(state)

        #get reconstructed image by sending latent_state to self.decoder
        reconstructed_state = self.decode(latent_state)
        # ---
        return reconstructed_state, mu, log_var, latent_state

    def encode(self, state):
        """
        :param state: <torch.Tensor> of shape (..., num_channels, 32, 32)
        :return: <torch.Tensor> of shape (..., latent_dim)
        """
        latent_state = None
        # --- Your code here
        mu,log_var = self.encoder(state)
        latent_state = self.reparameterize(mu,log_var)
        # ---
        return latent_state

    def decode(self, latent_state):
        """
        :param latent_state: <torch.Tensor> of shape (..., latent_dim)
        :return: <torch.Tensor> of shape (..., num_channels, 32, 32)
        """
        reconstructed_state = None
        # --- Your code here
        reconstructed_state = self.decoder(latent_state)
        # ---
        return reconstructed_state

    def reparameterize(self, mu, logvar):
        return self.encoder.reparameterize(mu, logvar)


class LatentDynamicsModel(nn.Module):
    """
    Model the dynamics in latent space via residual learning z_{t+1} = z_{t} + f(z_{t},a_{t})
    Use StateEncoder and StateDecoder encoding-decoding the state into latent space.
    where
        z_{t}  = encoder(x_{t})
        z_{t+1} = z_{t} + latent_dynamics_model(z_{t}, a_{t})
        x_{t+1} = decoder(z_{t+1})

    Latent dynamics model must be a Linear 3-layer network with 100 units in each layer and ReLU activations.
    The input to the latent_dynamics_model must be the latent states and actions concatentated along the last dimension.
    """

    def __init__(self, latent_dim, action_dim, num_channels=3):
        super().__init__()
        self.latent_dim = latent_dim
        self.action_dim = action_dim
        self.num_channels = num_channels
        self.encoder = None
        self.decoder = None
        self.latent_dynamics_model = None
        # --- Your code here
        self.encoder = StateEncoder(self.latent_dim, self.num_channels)
        self.decoder = StateDecoder(self.latent_dim, self.num_channels)
        self.latent_dynamics_model = nn.Sequential(
                              nn.Linear(self.latent_dim + self.action_dim,100),
                              nn.ReLU(),
                              nn.Linear(100,100),
                              nn.ReLU(),
                              nn.Linear(100,self.latent_dim))

        # ---

    def forward(self, state, action):
        """
        Compute next_state resultant of applying the provided action to provided state
        :param state: torch tensor of shape (..., num_channels, 32, 32)
        :param action: torch tensor of shape (..., action_dim)
        :return: next_state: torch tensor of shape (..., num_channels, 32, 32)
        """
        next_state = None
        # --- Your code here
        zt = self.encode(state)
        zt_1 = self.latent_dynamics(zt, action)
        next_state = self.decode(zt_1)
        # ---
        return next_state

    def encode(self, state):
        """
        Encode a state into the latent space
        :param state: torch tensor of shape (..., num_channels, 32, 32)
        :return: latent_state: torch tensor of shape (..., latent_dim)
        """
        latent_state = None
        # --- Your code here
        latent_state = self.encoder(state)
        # ---
        return latent_state

    def decode(self, latent_state):
        """
        Decode a latent state into the original space.
        :param latent_state: torch tensor of shape (..., latent_dim)
        :return: state: torch tensor of shape (..., num_channels, 32, 32)
        """
        state = None
        # --- Your code here
        state = self.decoder(latent_state)
        # ---
        return state

    def latent_dynamics(self, latent_state, action):
        """
        Compute the dynamics in latent space
        z_{t+1} = z_{t} + latent_dynamics_model(z_{t}, a_{t})
        :param latent_state: torch tensor of shape (..., latent_dim)
        :param action: torch tensor of shape (..., action_dim)
        :return: next_latent_state: torch tensor of shape (..., latent_dim)
        """
        next_latent_state = None
        # --- Your code here
        next_latent_state = latent_state + self.latent_dynamics_model(torch.cat([latent_state,action],dim=-1))
        # ---
        return next_latent_state


def latent_space_pushing_cost_function(latent_state, action, target_latent_state):
    """
    Compute the state cost for MPPI on a setup without obstacles in latent space.
    :param state: torch tensor of shape (B, latent_dim)
    :param action: torch tensor of shape (B, action_size)
    :param target_latent_state: torch tensor of shape (latent_dim,)
    :return: cost: torch tensor of shape (B,) containing the costs for each of the provided states
    """
    cost = None
    # --- Your code here
    cost = torch.sum(torch.square(latent_state - target_latent_state),dim=1)
    # ---
    return cost


def img_space_pushing_cost_function(state, action, target_state):
    """
    Compute the state cost for MPPI on a setup without obstacles in state space (images).
    :param state: torch tensor of shape (B, w, h, num_channels)
    :param action: torch tensor of shape (B, action_size)
    :param target_state: torch tensor of shape (w, h, num_channels)
    :return: cost: torch tensor of shape (B,) containing the costs for each of the provided states
    """
    cost = None
    # --- Your code here

    cost = torch.mean(torch.square(state-target_state),dim=(1,2,3))

    # ---
    return cost


class PushingImgSpaceController(object):
    """
    MPPI-based controller
    Since you implemented MPPI on HW2, here we will give you the MPPI for you.
    You will just need to implement the dynamics and tune the hyperparameters and cost functions.
    """

    def __init__(self, env, model, cost_function, norm_constants, num_samples=100, horizon=10):
        self.env = env
        self.model = model
        self.norm_constants = norm_constants
        self.target_state = torch.as_tensor(self.env.get_target_state(), dtype=torch.float32).permute(2, 0, 1)
        self.target_state_norm = (self.target_state - self.norm_constants['mean']) / self.norm_constants['std']
        self.cost_function = cost_function
        # MPPI Hyperparameters:
        # --- You may need to tune them
        state_dim = env.observation_space.shape[0]
        u_min = torch.from_numpy(env.action_space.low)
        u_max = torch.from_numpy(env.action_space.high)
        noise_sigma = 0.1 * torch.eye(env.action_space.shape[0])
        lambda_value = 0.01
        # ---
        self.mppi = MPPI(self._compute_dynamics,
                         self._compute_costs,
                         nx=state_dim,
                         num_samples=num_samples,
                         horizon=horizon,
                         noise_sigma=noise_sigma,
                         lambda_=lambda_value,
                         u_min=u_min,
                         u_max=u_max)

    def _compute_dynamics(self, state, action):
        """
        Compute next_state using the dynamics model self.model and the provided state and action tensors
        :param state: torch tensor of shape (B, wrapped_state_size)
        :param action: torch tensor of shape (B, action_size)
        :return: next_state: torch tensor of shape (B, wrapped_state_size) containing the predicted states from the learned model.
        """
        next_state = None
        # --- Your code here
        unwrap_state = self._unwrap_state(state)
        next_state = self.model(unwrap_state,action)
        next_state = self._wrap_state(next_state)
        # ---
        return next_state

    def _compute_costs(self, state, action):
        """
        Compute the cost for each state-action pair.
        You need to call self.cost_function to compute the cost.
        :param state: torch tensor of shape (B, wrapped_state_size)
        :param action: torch tensor of shape (B, action_size)
        :return: cost: torch tensor of shape (B,) containing the costs for each of the provided states
        """
        cost = None
        # --- Your code here
        unwrap_state = self._unwrap_state(state)
        cost = self.cost_function(unwrap_state,action,self.target_state_norm)
        # ---
        return cost

    def control(self, state):
        """
        Query MPPI and return the optimal action given the current state <state>
        :param state: numpy array of shape (height, width, num_channels) representing current state
        :return: action: numpy array of shape (action_size,) representing optimal action to be sent to the robot.
        TO DO:
         - Prepare the state so it can be sent to the mppi controller. Note that MPPI works with torch tensors.
         - You may need to normalize the state to the same space used for training the model.
         - Unpack the mppi returned action to the desired format.
        """
        action = None
        state_tensor = None
        # --- Your code here
        state_tensor = torch.as_tensor(state,dtype = torch.float32).permute(2,0,1)
        # ---
        action_tensor = self.mppi.command(state_tensor)
        # --- Your code here
        action = action_tensor.detach().cpu().numpy()


        # ---
        return action

    def _wrap_state(self, state):
        # convert state from shape (..., num_channels, height, width) to shape (..., num_channels*height*width)
        wrapped_state = None
        # --- Your code here
        self.B = state.shape[0]
        self.NC = state.shape[1]
        self.H = state.shape[2]
        self.W = state.shape[3]

        wrapped_state = torch.flatten(state,start_dim=-3)

        # ---
        return wrapped_state

    def _unwrap_state(self, wrapped_state):
        # convert state from shape (..., num_channels*height*width) to shape (..., num_channels, height, width)
        state = None
        # --- Your code here
        state = wrapped_state.reshape(-1,*self.target_state.shape) #nc,h,w

        # ---
        return state


class PushingLatentController(object):
    """
    MPPI-based controller
    Since you implemented MPPI on HW2, here we will give you the MPPI for you.
    You will just need to implement the dynamics and tune the hyperparameters and cost functions.
    """

    def __init__(self, env, model, cost_function, norm_constants, num_samples=100, horizon=10):
        self.env = env
        self.model = model
        self.norm_constants = norm_constants
        self.target_state = torch.as_tensor(self.env.get_target_state(), dtype=torch.float32).permute(2, 0, 1)
        self.target_state_norm = (self.target_state - self.norm_constants['mean']) / self.norm_constants['std']
        self.latent_target_state = self.model.encode(self.target_state_norm)
        self.cost_function = cost_function
        # MPPI Hyperparameters:
        # --- You may need to tune them
        state_dim = model.latent_dim  # Note that the state size is the latent dimension of the model
        u_min = torch.from_numpy(env.action_space.low)
        u_max = torch.from_numpy(env.action_space.high)
        noise_sigma = 0.1 * torch.eye(env.action_space.shape[0])
        lambda_value = 0.01
        # ---
        self.mppi = MPPI(self._compute_dynamics,
                         self._compute_costs,
                         nx=state_dim,
                         num_samples=num_samples,
                         horizon=horizon,
                         noise_sigma=noise_sigma,
                         lambda_=lambda_value,
                         u_min=u_min,
                         u_max=u_max)

    def _compute_dynamics(self, state, action):
        """
        Compute next_state using the dynamics model self.model and the provided state and action tensors
        :param state: torch tensor of shape (B, latent_dim)
        :param action: torch tensor of shape (B, action_size)
        :return: next_state: torch tensor of shape (B, latent_dim) containing the predicted states from the learned model.
        """
        next_state = None
        # --- Your code here
        next_state = self.model.latent_dynamics(state,action)


        # ---
        return next_state

    def _compute_costs(self, state, action):
        """
        Compute the cost for each state-action pair.
        You need to call self.cost_function to compute the cost.
        :param state: torch tensor of shape (B, latent_dim)
        :param action: torch tensor of shape (B, action_size)
        :return: cost: torch tensor of shape (B,) containing the costs for each of the provided states
        """
        cost = None
        # --- Your code here
        cost = self.cost_function(state,action,self.latent_target_state)


        # ---
        return cost

    def control(self, state):
        """
        Query MPPI and return the optimal action given the current state <state>
        :param state: numpy array of shape (height, width, num_channels) representing current state
        :return: action: numpy array of shape (action_size,) representing optimal action to be sent to the robot.
        TO DO:
         - Prepare the state so it can be sent to the mppi controller. Note that MPPI works with torch tensors.
         - You may need to normalize the state to the same space used for training the model.
         - Unpack the mppi returned action to the desired format.
        """
        action = None
        state_tensor = None
        # --- Your code here
        state = torch.as_tensor(state,dtype=torch.float32).permute(2,0,1)
        state_norm = (state - self.norm_constants["mean"]) / self.norm_constants["std"]
        latent_tensor = self.model.encode(state_norm)


        # ---
        action_tensor = self.mppi.command(latent_tensor)
        # --- Your code here
        action = action_tensor.detach().cpu().numpy()


        # ---
        return action

# =========== AUXILIARY FUNCTIONS AND CLASSES HERE ===========
# --- Your code here



# ---
# ============================================================
