# Backends

A backend is where robot state comes from and where joint commands go. Every
backend implements the same four calls:

```cpp
class RobotBackend {
public:
  virtual void start() = 0;
  virtual std::optional<RobotState> read_state() = 0;  // nullopt: session closed
  virtual void submit(const JointCommand &) = 0;       // PD targets, gains, feedforward
  virtual void stop() noexcept = 0;
};
```

The policy loop does not know which backend it runs on, so moving from
simulation to hardware is a flag:

```bash
boltrt --robot g1 --policy policy.yaml --backend mujoco
boltrt --robot g1 --policy policy.yaml --backend g1
```

## MuJoCo

```yaml
type: mujoco
physics_dt: 0.005
realtime: false
floor: true
```

- **Online** (default with a window): physics runs on its own thread at
  `physics_dt` using the latest command; the policy keeps its own clock.
- **Headless** (`--headless`): no window and no pacing; physics and policy step
  deterministically, for batch evaluation and regression checks.
- The native viewer owns its shortcuts (`Space` pause, `Backspace` reset).
  These affect physics only.

## Hardware: G1 and RP1

Hardware backends run a high-frequency command writer with a watchdog in the
same process. If commands stop arriving within `command_timeout_s`, or state
stops within `state_timeout_s`, the backend fails safe. Details, safety notes and
configuration: [Real robots](../real-robots.md).

## Replay

`replay` feeds recorded robot states into the loop without simulating anything.
It is used for latency measurement and same-state parity checks.

## Adding a backend

Implement `RobotBackend`, register a factory in `BackendRegistry`, and keep SDK
headers and build rules inside `src/backend/<name>/`. See
[Add a backend](../extending/add-backend.md).
