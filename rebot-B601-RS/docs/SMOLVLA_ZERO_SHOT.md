# SmolVLA zero-shot mode (v3.5, experimental)

**What it is:** the app's policy panel can run the public base model
[`lerobot/smolvla_base`](https://huggingface.co/lerobot/smolvla_base) (about 450 M parameters, a SmolVLM2 vision-language
backbone plus a flow-matching action expert) with a free text command. This is a "just for fun" pipeline test:
the base model was pretrained on community SO-100/SO-101 datasets, never on the reBot B601-RS, our cameras or our
joint zero points. **Expect random or tiny motions, not a solved task.** Real use needs fine-tuning on our demos.

## How our robot is mapped onto the base model

| Base model expects | We give it |
|---|---|
| `observation.images.camera1` (top/front view) | iPhone feed 1 `overview`, 640x360 (model resizes+pads to 512x512) |
| `observation.images.camera2` (wrist view) | iPhone feed 2 `gripper`, 640x360 |
| `observation.images.camera3` | not provided (left out; the model simply uses 2 images) |
| `observation.state` [6] (SO-100: pan, lift, elbow, wrist_flex, wrist_roll, gripper) | default `--smolvla-map so100`: our joints without `wrist_yaw` (6 values, degrees). `wrist_yaw` is held at its start value. `--smolvla-map pad7`: all 7 joints. The model zero-pads to 32 dims internally in both cases. |
| `action` [6] | same 6 (or 7) joints, absolute target positions in degrees |
| language | the text command from the app (default `pick up the box and put it in the bin`) |

**Normalization (honest caveat):** the base checkpoint ships normalization statistics for its own pretraining data,
not for our robot. We use **our** dataset statistics (`datasets/pick_place_all7/meta/stats.json`, MEAN_STD) for
state normalization and action un-normalization. That puts our joint values into roughly the numeric range the model
saw, but the meaning of each joint (direction, zero point, scale) still differs from SO-100. Images use IDENTITY
normalization as in the base config.

**Weights:** loaded from the `smolvla_base` checkpoint only (`load_vlm_weights=False`, so the separate SmolVLM2 weights
are not downloaded again). The play log prints how many model tensors were not found in the checkpoint (should be 0).

## Safety (same as ACT, plus)
- Collision guard v3.2 on, max 2 deg/cycle on arm joints, 10 deg gripper, 60 s limit, STOP = torque off immediately,
  rest position + torque off at the end. Refused while teleop or training runs.
- Actions are clipped to the min/max joint range of our recorded dataset (margin `PLAY_RANGE_MARGIN_DEG`, default 0 for SmolVLA).
- The app only allows a real SmolVLA run after a successful SmolVLA dry run (no motors) with the same command in the same app session.

## Timing
SmolVLA predicts a chunk of 50 actions per inference (`n_action_steps=50`, env `PLAY_N_ACTION_STEPS`). One inference is
slow (logged as "SmolVLA Aktionsblock N: Inferenz X ms"); the chunk is then executed at 30 Hz. During an inference the
arm holds its last target, so expect a short pause roughly every 1.7 s.

## Command line
```
./play_policy.sh --policy smolvla --task "pick up the box and put it in the bin" --dry-run --seconds 5
./play_policy.sh --policy smolvla --task "..." --device cpu --smolvla-map pad7
```

## Next step for real results
Fine-tune `smolvla_base` on our own demos (30+ clean episodes), then load that checkpoint instead of the base model.
