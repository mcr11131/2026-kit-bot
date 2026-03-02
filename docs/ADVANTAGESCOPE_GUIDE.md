# AdvantageScope Guide — 2026 Kit Bot

## What is AdvantageScope?

AdvantageScope is a free tool from Team 6328 for visualizing robot data. It reads the `.wpilog` data log files your robot records during matches and practice sessions. Think of it as a DVR for your robot — you can replay everything that happened.

**Download:** https://github.com/Mechanical-Advantage/AdvantageScope/releases

---

## Getting Your Log Files

The robot saves data logs automatically to the roboRIO at `/home/lvuser/logs/`. To retrieve them:

### Option 1: USB Drive
1. Plug a USB drive into the roboRIO
2. Logs auto-copy to the USB drive
3. Grab the `.wpilog` files

### Option 2: Network Transfer
1. Connect to the robot's network
2. Open a browser: `http://roborio-XXXX-frc.local` (your team number)
3. Navigate to `/home/lvuser/logs/`
4. Download the `.wpilog` files

### Option 3: SCP/SFTP
```
scp lvuser@roborio-XXXX-frc.local:/home/lvuser/logs/*.wpilog .
```
Password is blank by default.

---

## Opening Logs

1. Launch AdvantageScope
2. **File → Open Log File** (or drag and drop a `.wpilog` file)
3. The left panel shows all logged data fields in a tree

---

## Our Logged Data

Here's everything the robot logs and what to look for:

### Drive Motors (`/drive/`)

| Field | What It Is | What to Watch For |
|-------|-----------|-------------------|
| `leftCurrent` / `rightCurrent` | Motor current draw (amps) | Spikes above 60A = near breaker trip |
| `leftVoltage` / `rightVoltage` | Applied voltage | Should match joystick commands |
| `leftTemp` / `rightTemp` | Motor temperature (°C) | Above 80°C = overheating, back off |
| `leftVelocity` / `rightVelocity` | Wheel speed (m/s) | Left/right should match when driving straight |
| `leftPosition` / `rightPosition` | Distance traveled (m) | Useful for verifying auto distances |
| `batteryVoltage` | Battery voltage | Below 8V = brownout risk, charge batteries |

### NavX Gyro (`/navx/`)

| Field | What It Is | What to Watch For |
|-------|-----------|-------------------|
| `yaw` | Heading -180° to 180° | Should be 0 at start, stable when still |
| `pitch` | Tilt forward/back | Should be ~0 on flat ground |
| `roll` | Tilt left/right | Should be ~0 on flat ground |
| `accelX` / `accelY` | Linear acceleration (g) | Spikes = collisions or hard stops |
| `angle` | Continuous angle (doesn't wrap) | Good for tracking total rotation |
| `rate` | Turn rate (deg/s) | How fast you're spinning |

### Fuel System (`/fuel/`)

| Field | What It Is | What to Watch For |
|-------|-----------|-------------------|
| `launcherCurrent` / `feederCurrent` | Motor current draw | Spikes when ball contacts rollers |
| `launcherVoltage` / `feederVoltage` | Applied voltage | Should match command values |
| `launcherTemp` / `feederTemp` | Motor temperature | Monitor during extended use |

### DriverStation Data (auto-logged)

| Field | What It Is |
|-------|-----------|
| Joystick axes | Raw controller inputs |
| Match time | Time remaining in match |
| Robot mode | Disabled/Auto/Teleop/Test |
| Alliance | Red/Blue |

---

## Common Analysis Tasks

### 1. "Why did the robot stop moving?"

**Check these in order:**
1. `batteryVoltage` — Did it brownout? (drops below 6.8V = roboRIO reboots)
2. `leftCurrent` / `rightCurrent` — Did a breaker trip? (current drops to 0 suddenly)
3. `leftTemp` / `rightTemp` — Did motors overheat?
4. DriverStation data — Was it disabled?

**How to view:**
- Add `batteryVoltage`, `leftCurrent`, `rightCurrent` to a Line Chart
- Look for the moment things go wrong and zoom in

### 2. "The robot isn't driving straight"

**Check:**
1. `leftVelocity` vs `rightVelocity` — Are they matched?
2. `leftCurrent` vs `rightCurrent` — Is one side drawing more?
3. `navx/yaw` — Is heading drifting?

**How to view:**
- Line Chart with `leftVelocity` and `rightVelocity` overlaid
- They should track together when driving straight

### 3. "The 180 spin isn't accurate"

**Check:**
1. `navx/yaw` — Does it actually reach 180° from start?
2. `navx/rate` — Is it spinning too fast to stop accurately?
3. `navx/Connected` and `navx/Calibrating` — Was the gyro working?

**How to view:**
- Line Chart with `navx/yaw` and `navx/rate`
- Mark the start and end of the spin command
- Check if yaw settles at the target or overshoots

### 4. "The TurnToAngle command oscillates"

**Check SmartDashboard data:**
1. `TurnToAngle/Current` — Current heading during the turn
2. `TurnToAngle/Error` — PID error (should decrease to 0)

**If oscillating:** `TURN_KD` is too low or `TURN_KP` is too high. Tune in Constants.java.

### 5. "Launcher isn't shooting consistently"

**Check:**
1. `fuel/launcherCurrent` — Is it reaching full speed before feeding?
2. `fuel/feederCurrent` — Spikes show when balls are being fed
3. `batteryVoltage` — Voltage sag during launch affects speed

### 6. "Robot tipped or got hit"

**Check:**
1. `navx/pitch` and `navx/roll` — Sudden changes = tipping
2. `navx/accelX` and `navx/accelY` — Spikes = collisions
3. Correlate with `leftCurrent`/`rightCurrent` for full picture

---

## AdvantageScope View Types

### Line Chart (most useful)
- Drag fields from the left panel onto the chart
- Zoom with scroll wheel, pan with click-drag
- Right-click for options (color, axis, etc.)
- **Tip:** Put related fields on the same chart to see correlations

### Table
- See exact values at any timestamp
- Good for finding precise moments

### Video (if you have match video)
- Sync match video with data for the full picture
- **File → Add Video** then sync the timestamps

---

## Tips & Tricks

1. **Zoom to a match period** — Click the Auto/Teleop markers in the timeline to jump to that section

2. **Compare matches** — Open multiple log files in tabs and compare the same data across matches

3. **Find brownouts fast** — Add `batteryVoltage` and look for dips below 7V

4. **Verify auto routines** — Plot `leftPosition` and `rightPosition` to see if the robot drove the expected distances

5. **Check NavX health** — If `navx/yaw` drifts when the robot is sitting still, the gyro may need recalibration or there's excessive vibration

6. **Export data** — Right-click a chart → Export CSV for analysis in Excel/Google Sheets

7. **Keyboard shortcuts:**
   - `Space` — Play/pause
   - `←` / `→` — Step frame by frame
   - `Ctrl+Scroll` — Zoom timeline

---

## Pre-Match Checklist

Before each match, verify on SmartDashboard:
- [ ] `NavX/Connected` = true
- [ ] `NavX/Calibrating` = false (wait for it!)
- [ ] `NavX/Yaw` ≈ 0 (don't move robot during calibration)
- [ ] `Drive/Battery Voltage` > 12.0V
- [ ] `Drive/Left Temperature` and `Right Temperature` < 40°C

---

## Troubleshooting

**"No data in log file"**
- Make sure `DataLogManager.start()` is in `robotInit()` (it is in our code)
- Check that the roboRIO has free disk space

**"NavX data shows 0 for everything"**
- Check `NavX/Connected` — if false, check the MXP cable
- NavX takes ~3 seconds to calibrate on boot — don't move the robot

**"Data looks choppy"**
- Data is logged at 50Hz (every 20ms). This is normal.
- If it's worse than that, the robot loop may be overrunning — check for loop time warnings in the console

**"Log file is huge"**
- Normal! A 2.5 minute match generates ~5-10MB
- Old logs in `/home/lvuser/logs/` should be cleaned periodically
