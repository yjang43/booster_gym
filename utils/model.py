import torch
import torch.nn.functional as F


class ActorCritic(torch.nn.Module):

    def __init__(self, num_act, num_obs, num_privileged_obs):
        super().__init__()
        self.critic = torch.nn.Sequential(
            torch.nn.Linear(num_obs + num_privileged_obs, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, 1),
        )
        self.actor = torch.nn.Sequential(
            torch.nn.Linear(num_obs, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, num_act),
        )
        self.logstd = torch.nn.parameter.Parameter(torch.full((1, num_act), fill_value=-2.0), requires_grad=True)

    def act(self, obs):
        action_mean = self.actor(obs)
        action_std = torch.exp(self.logstd).expand_as(action_mean)
        return torch.distributions.Normal(action_mean, action_std)

    def est_value(self, obs, privileged_obs):
        critic_input = torch.cat((obs, privileged_obs), dim=-1)
        return self.critic(critic_input).squeeze(-1)


class ActorCriticV2(torch.nn.Module):

    def __init__(self, num_act, num_obs, num_privileged_obs, upper_body_dof_indices):
        super().__init__()

        self.upper_body_dof_indices = upper_body_dof_indices
        self.num_act_full = num_act
        self.num_act_reduced = num_act - len(upper_body_dof_indices)

        # Create mask for lower body indices (non-upper body)
        all_indices = set(range(num_act))
        upper_body_set = set(upper_body_dof_indices)
        self.lower_body_dof_indices = sorted(list(all_indices - upper_body_set))

        self.critic = torch.nn.Sequential(
            torch.nn.Linear(num_obs + num_privileged_obs, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, 1),
        )
        self.actor = torch.nn.Sequential(
            torch.nn.Linear(num_obs, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, self.num_act_reduced),  # Only output for lower body
        )
        self.logstd = torch.nn.parameter.Parameter(torch.full((1, self.num_act_reduced), fill_value=-2.0), requires_grad=True)

    def act(self, obs):
        action_mean_reduced = self.actor(obs)
        action_std_reduced = torch.exp(self.logstd).expand_as(action_mean_reduced)

        action_mean_full = torch.zeros(action_mean_reduced.shape[:-1] + (self.num_act_full,), device=action_mean_reduced.device)
        action_std_full = torch.full(action_std_reduced.shape[:-1] + (self.num_act_full,), 1e-6, device=action_std_reduced.device)

        action_mean_full[..., self.lower_body_dof_indices] = action_mean_reduced
        action_std_full[..., self.lower_body_dof_indices] = action_std_reduced

        return torch.distributions.Normal(action_mean_full, action_std_full)

    def est_value(self, obs, privileged_obs):
        critic_input = torch.cat((obs, privileged_obs), dim=-1)
        return self.critic(critic_input).squeeze(-1)
