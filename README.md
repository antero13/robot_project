# Robot Project ROS

ROS 2 workspace for the robot competition.

## RL model checkpoint

The pushed RL policy checkpoint is here:

```text
mission_manager/models/rl_avoid_search_best.pt
```

Before changing the RL runner, read this document:

```text
rl_model_policy/MODEL_CHECKPOINT_README.md
```

Important current contract:

```text
checkpoint input dimension: 18
checkpoint action dimension: 2
first policy layer: Linear(18, 128)
state_preprocessor running_mean/running_variance length: 18
```

If a Codex/developer updates `rl_model_policy_node.py`, the observation order
and tensor shapes in `rl_model_policy/MODEL_CHECKPOINT_README.md` must be kept
matched to `mission_manager/models/rl_avoid_search_best.pt`.

YOLO target classes used by the project:

```text
'12' / '20' / '6' / '8' / apple / banana / orange / pineapple
```

