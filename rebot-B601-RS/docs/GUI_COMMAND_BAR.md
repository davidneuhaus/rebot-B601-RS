# Policy command bar (v3.6)

The teleop GUI policy panel has a dedicated command bar for SmolVLA text commands. This is a UI
change only; the playback process is still `play_policy.sh` / `play_policy.py` from v3.5.

## Layout

- **Policy:** `ACT (eigenes Training)` or `SmolVLA (Text-Befehl)`.
- **ACT:** the v3.4 button **▶ Policy abspielen** (learned checkpoint). No text command.
- **SmolVLA:** the play button is hidden. Instead a framed bar is shown:
  - a large text field, labelled "Was soll der Roboter tun?"
  - **🧪 Testen (ohne Motoren)** — runs `play_policy.sh --policy smolvla --task "…" --dry-run`
  - **▶ Ausführen (Motoren an)** — same without `--dry-run`, only after a successful Testen
    with *exactly* the same text in this app session
  - **Vorschläge:** clickable presets (English, which the base model expects), including
    `pick up the box and put it in the bin`
  - **Verlauf:** last 8 commands, stored in `logs/smolvla_history.json` next to the app
- Enter in the field = Testen, or Ausführen once that text has already been tested.
- Duration spinbox and **■ STOPP (Motoren sofort aus)** stay on the row above (STOPP still
  cuts torque immediately). Escape still stops a running policy.

## Safety (unchanged from v3.5)

- Collision guard v3.2, 2°/cycle cap, gripper 10°, time limit, STOP = torque off, rest
  position at the end. Refused while teleop or training runs (a dry Testen may run during
  training; `play_policy.py` then uses the CPU).
- A real SmolVLA run is blocked until Testen with the same command exits 0 in this session.
  Changing the text disables Ausführen again. The confirm dialog for a real run shows the
  command text.
- Zero-shot `lerobot/smolvla_base` has never seen this arm. Expect odd or tiny motions;
  actions are still normalized and clipped with our demo statistics. See `docs/SMOLVLA_ZERO_SHOT.md`.

## After install

Stop teleop if it is running, then press **App neu laden** (or wait for the auto-reload when
idle). Rollback: `./rollback.sh before_gui_v3_6`.
