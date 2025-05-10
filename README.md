# Serial Command Protocol (Host → Controller)

## Command Format
```plaintext 
<CMD>[<PARAM>]\n
```
- Single uppercase letter command
- Optional numeric parameter (for movement commands)
- Must end with newline (`\n`)

## Command Reference

| Command | Description               | Parameter       | Valid Values       | Example       |
|---------|---------------------------|-----------------|--------------------|---------------|
| `H`     | Home motor                | None            | -                  | `H\n`         |
| `S`     | Emergency stop            | None            | -                  | `S\n`         |
| `U<n>`  | Move up (relative)        | Steps           | 1 - 4,294,967,295  | `U500\n`      |
| `D<n>`  | Move down (relative)      | Steps           | 1 - 4,294,967,295  | `D1000\n`     |
| `M<n>`  | Move to absolute position | Target position | ±2,147,483,647     | `M2500\n`     |
| `P`     | Get current position      | None            | -                  | `P\n`         |
| `LL`    | Check lower limit switch  | None            | -                  | `LL\n`        |
| `LU`    | Check upper limit switch  | None            | -                  | `LU\n`        |

## Command Details
1. **Movement Commands** (`U`, `D`, `M`)
   - Execute in chunks of `STEP_SIZE` (500 steps)
   - Send periodic `PENDING` status updates
   - Can be interrupted by `S` command

2. **Sensor Commands** (`LL`, `LU`)
   - Immediate response with sensor state:
     - `REACHED_SENSOR` (active)
     - `UNREACHED_SENSOR` (inactive)

3. **Special Cases**
   - Empty command: No response
   - Unknown command: `UNKNOWN_CMD` status
   - Malformed number: `ERROR_CMD` status

## Examples
```plaintext
Move up 1500 steps → "U1500\n"
Move to position -200 → "M-200\n"
Stop immediately → "S\n"
Query position → "P\n"
