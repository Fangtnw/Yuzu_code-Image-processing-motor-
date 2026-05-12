# Reverse-engineer a new MEXE02 motor register

Use this skill whenever you need to add control of a new AZD-KEP / AZ-series
motor parameter (e.g. jerk limit, torque limit, settling window) that is not yet
implemented in `motor_controller_node.py`.

## What you need before starting

- MEXE02 software connected to the motor on COM11 (19200 baud, 8E1)
- The existing `diff_dump.py` tool (`motion_cmd/dump trial/diff_dump.py`)
- A baseline dump (ideally the most recent `dump_writedata.txt`)

---

## Step 1 — Capture a targeted dump

1. Open MEXE02 → **Communication Monitor** → start logging.
2. Change **only one parameter** to a known value (e.g. acceleration = 0.7 mm/s²).
   Keep everything else identical to the baseline session.
3. Trigger one complete write-to-driver cycle (one move command is enough).
4. Stop logging → save as `dump_<param>_<value>.txt`.
5. Repeat for a **second distinct value** (e.g. 0.4 mm/s²) → save as
   `dump_<param>_<value2>.txt`.

> Tip: give MEXE02 a few seconds between the parameter change and the trigger,
> so the "Write to Driver" bulk frames definitely contain the new value.

---

## Step 2 — Find the changed bytes

```bash
python diff_dump.py dump_<param>_baseline.txt dump_<param>_<value>.txt
```

Look for lines like:
```
SPEED_FRAME  (baseline block #55  vs  new block #55)
  offset 142: 0xe8 -> 0x20   <- int32 LE: 1000 -> 8000  (as mm/s2 if x0.001: ...)
```

If the frame is misidentified as POSITION_FRAME or SPEED_FRAME (same sizes),
search the raw bytes directly:

```python
# Find value in Written blocks
python -X utf8 -c "
import re, struct
data = open('dump_file.txt', encoding='utf-8', errors='ignore').read()
# search for known value bytes, e.g. 300000 = e0 93 04 00
target = bytes([0xe0, 0x93, 0x04, 0x00])
...
"
```

---

## Step 3 — Identify the register address

In each `[reg_lo reg_hi val_b0 val_b1 val_b2 val_b3]` 6-byte entry:
- **Register** = the 2 bytes immediately before the changed value bytes (LE uint16).
- AZ operation-data register map (data slot n, spacing 0x20):

| Parameter     | DS0 address | Formula          |
|---------------|-------------|------------------|
| Position      | 0x0C01      | 0x0C01 + 0x20×n  |
| Speed         | 0x0C02      | 0x0C02 + 0x20×n  |
| Acceleration  | 0x0C03      | 0x0C03 + 0x20×n  |
| Deceleration  | 0x0C04      | 0x0C04 + 0x20×n  |

---

## Step 4 — Confirm the value encoding

Compare the known physical value to the raw int32:

| Parameter  | Unit      | Multiply from mm (or mm/s, mm/s²)  |
|------------|-----------|------------------------------------|
| Position   | µm        | × 1 000                            |
| Speed      | µm/s      | × 1 000                            |
| Accel/Decel| nm/s²     | × 1 000 000                        |

Verify: `raw_value / unit_factor ≈ physical_value` for both test values.

---

## Step 5 — Derive the new frame template

**If the new parameter uses the same 240-byte frame size as speed/accel:**

Start from `_ACCEL_FRAME` and increment all 12 DS register-low bytes by 1
(e.g., X3 → X4 for decel).  Then recompute the 3 XOR checksums:

```python
from motor_controller_node import _ACCEL_FRAME
import struct

AF = bytearray(_ACCEL_FRAME)
reg_lo_offsets = [140, 146, 152, 166, 172, 178, 184, 190, 204, 210, 216, 222]
NEW_FRAME = bytearray(AF)

# Increment register low byte for all 12 DS entries
for off in reg_lo_offsets:
    NEW_FRAME[off] += 1          # X3->X4, or X4->X5, etc.

# Fix CS[159]: covers bytes [0:159).  Count how many reg changes are < 159.
delta159 = 0
for o in reg_lo_offsets:
    if o < 159:
        delta159 ^= (AF[o] ^ NEW_FRAME[o])
NEW_FRAME[159] ^= delta159

# Fix CS[229]: covers bytes [1:229) excluding byte 159.
delta229 = 0
for o in reg_lo_offsets:
    if 1 <= o < 229 and o != 159:
        delta229 ^= (AF[o] ^ NEW_FRAME[o])
NEW_FRAME[229] ^= delta229

# Fix CS[239]: covers bytes [0:239) including byte 159 (which changed above).
delta239 = 0
for o in reg_lo_offsets:
    if o < 239:
        delta239 ^= (AF[o] ^ NEW_FRAME[o])
delta239 ^= (AF[159] ^ NEW_FRAME[159])   # CS[159] change propagates here
NEW_FRAME[239] ^= delta239

# Verify
def xor_r(d, s, e, excl=set()):
    r = 0
    for i in range(s, e):
        if i not in excl: r ^= d[i]
    return r

assert NEW_FRAME[159] == xor_r(NEW_FRAME, 0, 159),           "CS159 fail"
assert NEW_FRAME[229] == xor_r(NEW_FRAME, 1, 229, {159}),    "CS229 fail"
assert NEW_FRAME[239] == xor_r(NEW_FRAME, 0, 239),           "CS239 fail"
print("All checksums OK")
```

**If the new parameter uses a 320-byte frame** (same size as position):
Use checksums at indices **(159, 311, 319)** instead of (159, 229, 239).

---

## Step 6 — Implement in `motor_controller_node.py`

Add after the last `_*_FRAME` constant block:

```python
# ── <Parameter> frame (240 bytes) ─────────────────────────────────────────────
_NEW_FRAME = bytes([...])          # paste bytes from Step 5
assert len(_NEW_FRAME) == 240, "<Parameter> frame length mismatch"

_NEW_LO_IDX   = 142               # same as accel/decel for 240B frames
_NEW_HI_IDX   = 144
_NEW_BASE_RAW = 700_000           # raw value in baseline frame
_NEW_CS_IDX   = (159, 229, 239)   # 240B frames; use (159,311,319) for 320B
```

Add the builder function (copy from `_build_accel_frame`, change names/constants):

```python
def _build_new_frame(value_mms2: float) -> bytes:
    raw        = int(round(value_mms2 * 1_000_000))  # adjust unit factor
    new_bytes  = struct.pack("<i", raw)
    base_bytes = struct.pack("<i", _NEW_BASE_RAW)
    xor_delta  = _xor_of(base_bytes) ^ _xor_of(new_bytes)
    frame = bytearray(_NEW_FRAME)
    frame[_NEW_LO_IDX : _NEW_LO_IDX + 2] = new_bytes[0:2]
    frame[_NEW_HI_IDX : _NEW_HI_IDX + 2] = new_bytes[2:4]
    for idx in _NEW_CS_IDX:
        frame[idx] ^= xor_delta
    return bytes(frame)
```

Update `_move_payloads()` to include the new frame in the correct order.

---

## Step 7 — Verify

1. **Import check**: `python -c "import motor_controller_node"` — the `assert` fires at
   load time if byte counts are wrong.
2. **Replay test**: paste the new frame into `replay_frames.py`, send it with a known
   value, capture a new dump, and check via `parse_dump.py` that the physical
   behaviour (ramp slope, settling, etc.) matches the commanded value.
3. **Diff confirmation**: run `diff_dump.py baseline.txt new_capture.txt` — only the
   expected offsets should change.

---

## Checksum quick-reference

| Frame size | CS indices       | CS[n] covers             |
|------------|------------------|--------------------------|
| 240B       | 159, 229, 239    | [0:159), [1:229) excl 159, [0:239) |
| 320B       | 159, 311, 319    | [0:159), [0:311), [0:319) |
