# Cat Comm Template Bridge Protocol (Upload-Ready)

This document defines the bridge contract expected by the Windows Cat Comm adapter in [src/io/vendor_cat_comm.rs](src/io/vendor_cat_comm.rs).

The repository includes a functional reference bridge implementation at [src/bin/cat_comm_bridge.rs](src/bin/cat_comm_bridge.rs) and a staging helper script at [scripts/build_cat_comm_template_bridge.ps1](scripts/build_cat_comm_template_bridge.ps1).

## 1. Startup modes

The bridge executable must support:

1. `--mode=stdio-jsonl`

The host process launches the bridge and communicates with JSON lines over stdin/stdout.

## 2. Transport contract

- One JSON object per line.
- UTF-8 encoded.
- Requests are sent by the host.
- Responses are returned by the bridge.
- Every request requires one response line.

## 3. Request schema

`cmd` is required.

### init

```json
{
  "cmd": "init",
  "dry_run": true,
  "enable_write": false,
  "can_interface": "can0",
  "serial_port": "COM5",
  "template_dir": "C:/path/to/template"
}
```

### ping

```json
{ "cmd": "ping" }
```

### read

```json
{
  "cmd": "read",
  "timeout_ms": 1000
}
```

### try_read

```json
{ "cmd": "try_read" }
```

### send

```json
{
  "cmd": "send",
  "frame": {
    "Can": {
      "id": 418381568,
      "dlc": 8,
      "data": [1,2,3,4,5,6,7,8],
      "len": 8,
      "timestamp_ms": 1723520000000
    }
  }
}
```

Serial frame variant:

```json
{
  "cmd": "send",
  "frame": {
    "Serial": {
      "bytes": [62,0],
      "protocol_hint": "uds-raw",
      "timestamp_ms": 1723520000000
    }
  }
}
```

### close

```json
{ "cmd": "close" }
```

## 4. Response schema

Base fields:

```json
{
  "ok": true,
  "frame": null,
  "error": null,
  "kind": null
}

Optional capability negotiation fields (recommended):

```json
{
  "ok": true,
  "protocol_version": "1.0",
  "capabilities": ["can", "serial", "write", "uds_flash"]
}
```

Optional transport diagnostics fields (for flash telemetry parity):

```json
{
  "ok": true,
  "diag": {
    "wait_frame_count_delta": 1,
    "sequence_error_count_delta": 0,
    "flowcontrol_timeout_count_delta": 0,
    "fc_blocksize": 8,
    "fc_stmin_ms": 5
  }
}
```

When provided, host appends these deltas to `log_dir/cat_comm_bridge_diag.jsonl` and merges them into live flash transport diagnostics.
```

On `read`/`try_read`, include `frame` when data exists.

On failure:

```json
{
  "ok": false,
  "error": "human-readable detail",
  "kind": "timeout"
}
```

Supported `kind` values mapped by host:

1. `timeout`
2. `rate_limited`
3. `bus_off`
4. `transceiver`
5. `permission`
6. `write_blocked`
7. `parse`

Any other value maps to generic `Unknown` host error.

Compatibility notes:

1. `protocol_version` is optional for backward compatibility.
2. If present, host currently accepts major version `1`.
3. `capabilities` is optional.
4. If `capabilities` is provided, host enforces strict checks for requested transport and write mode.

## 5. Upload layout options

Option A: explicit executable path

1. Put bridge executable anywhere.
2. Launch AutoBreaking with `--vendor-name=cat_comm --vendor-bridge-exe=<full_path_to_exe>`.

Option B: template directory convention

1. Put executable as `cat_comm_bridge.exe` inside your uploaded template folder.
2. Launch with `--vendor-name=cat_comm --vendor-template-dir=<folder>`.

## 6. Safety expectations

The bridge must fail closed:

1. Never claim success if transport write/read failed.
2. Return `ok=false` + `error` + `kind` on faults.
3. Respect `enable_write` from init request.
4. Preserve deterministic timeout behavior for `read`.
