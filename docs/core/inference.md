# Inference

Policies run on the CPU with ONNX Runtime, single-threaded by default
(`--ort-threads`).

## Bindings

The runtime maps observation groups to ONNX inputs and ONNX outputs to the
action through `onnx.in_keys` and `onnx.out_keys` in `policy.yaml`:

```yaml
model_path: policy.onnx
onnx:
  in_keys: [policy, command]
  out_keys: [action]
```

Older exports provide the same bindings in a JSON sidecar, which is still read.

## Supported model shapes

- Typed inputs: `float32`, `int64` and `bool` tensors.
- Recurrent models: hidden state outputs are fed back as the next tick's inputs
  and reset on policy entry.
- Transformer caches: KV-cache inputs/outputs and position counters are carried
  across ticks and reset with the policy (HoloMotion).
- Extra diagnostic outputs that the deployment does not consume are allowed.

`--validate` loads the model and checks every binding's name, shape and dtype
against the observation graph without starting a backend.

GPU execution providers (CUDA, TensorRT) are not supported yet.
