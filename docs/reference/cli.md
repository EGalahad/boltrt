# CLI

```text
boltrt [--task <name>] --robot <name|yaml> --policy <yaml> --backend <name|yaml>
```

| Option | Meaning |
|---|---|
| `--task tracking` | Select a task; default is the base loop |
| `--motion <npz\|tcp://host:port>` | Tracking reference (tracking task only) |
| `--validate` | Check configs, assets, observations and ONNX bindings; start nothing |
| `--start` | Enter the policy and play the motion at startup |
| `--rate <hz>` | Policy rate (default 50) |
| `--steps <n>` / `--run-once` | Stop after n ticks / after one pass of the motion |
| `--headless` | No window, no real-time pacing |
| `--no-keyboard` | Do not take over the terminal |
| `--record <jsonl>` | Per-tick state, observations, model I/O, actions and commands |
| `--timings <csv>` | Per-tick policy and inference latency |
| `--events <json>` | Scripted key events for unattended runs |
| `--ort-threads <n>` | ONNX Runtime threads (default 1) |
| `--init-steps <n>` | Ticks used by the init mode to reach the default pose (default 500) |
| `--robot-model <xml\|hf://...>` | Override the robot model |
| `--config-root <dir>` | Directory of builtin robot and backend configs |
| `--print-config` | Print the resolved configuration |
| `--list-robots`, `--list-backends`, `--list-observations`, `--list-tasks` | List registered names |
