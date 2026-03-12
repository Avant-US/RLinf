# Troubleshooting: Arm Moves Around But Doesn’t Grasp

Two main failure sources: **control pipeline** (our code) and **model/scene** (VLA + environment).

---

## 1. Control pipeline (code)

**Symptom:** Logs show high `track_err` (e.g. > 0.25), many ticks with `CLAMP`, arm keeps changing direction.

**Cause:** The command loop clamps the target to at most `max_track_error` rad from current pose. If that is too small, the robot never catches up to the planned trajectory. Every re-inference then plans from the current (lagged) pose, so the commanded direction keeps changing → arm wanders.

**Fixes (already applied in `run_test.sh`):**

- **`MAX_TRACK_ERROR`** – Allow command up to 0.35 rad from current so the robot can follow the trajectory. (Too small, e.g. 0.06–0.10, causes wandering.)
- **`MAX_CMD_STEP`** – Per-tick limit (e.g. 0.06 rad) so we still limit how fast the command moves; 0.05 is safe, 0.06 gives a bit more follow-through.
- **`OPEN_LOOP_HORIZON`** – Number of chunk steps before re-inferring (e.g. 12). Larger = same trajectory longer = less direction change = less wandering. 8 is minimum; 12 is a good balance.

If you still see a lot of `track_clamped` and high `track_err` in logs, try:

- Increase `MAX_TRACK_ERROR` to 0.40 (watch for reflex if the model outputs bad actions).
- Increase `OPEN_LOOP_HORIZON` to 15 (full chunk before re-infer).

---

## 2. Model / scene (VLA + environment)

**Symptom:** Low or moderate `track_err`, few clamps, but arm motion is not goal-directed (no approach to object, gripper doesn’t close on grasp).

**Cause:** The policy (e.g. PI0.5 DROID) is not getting useful visual/state information or the scene is too far from what it was trained on.

**Checks:**

1. **Observation images**  
   After a run, check `logs/obs_wrist.png` and `logs/obs_exterior.png`.  
   - Exterior: table and object (e.g. cup) clearly visible and well lit.  
   - Wrist: gripper/hand in view.  
   If they are reversed, run with `SWAP_CAMERAS=1` in `run_test.sh` or `--swap-cameras`.

2. **Lighting**  
   Training data is usually bright. If images are dark (e.g. mean pixel &lt; 100), add light so the scene is clearly visible.

3. **Prompt**  
   Use a clear, specific prompt, e.g. `Pick up the cup` (match your object).

4. **Object / scene**  
   Model may perform poorly on objects or backgrounds very different from training (e.g. flat towel, heavy clutter). Prefer a simple table and a common object (e.g. cup, bottle).

**If control looks good but behavior is still wrong:** the bottleneck is likely the **VLA model or scene**, not the station code. Options:

- Try another VLA (e.g. different OpenPI-compatible policy) with the same observation format.
- Improve scene (lighting, camera pose, less clutter) and ensure observation keys match the policy’s expected inputs (see `station/run.py` `build_observation` and the policy server’s expected keys).

---

## 3. Image quality sent to the model

**Yes — poor image quality can directly cause failure.** The model only sees what you send: if images are too dark, blurry, or the wrong views, it will output weak or wrong actions.

**What we send:** 224×224 RGB (uint8), one wrist and one exterior image, after `resize_with_pad` from the RealSense stream (default 640×480). Auto-exposure and auto-white-balance are enabled on the cameras.

**Check quality at run time:** When you use `--save-obs-images logs`, the station prints **image quality stats** for the first inference:

- **mean_brightness** – Mean pixel value (0–255). Training data is usually well lit; if this is **&lt; 80**, the scene is likely too dark for the model.
- **dark_pct** – Fraction of pixels with value &lt; 25. If this is **&gt; 20%**, large parts of the image are near-black and the model may not see the object or table.

**If stats look bad:**

1. **Lighting** – Add a lamp or move the arm so the table and object are clearly lit. Avoid strong backlight.
2. **Cameras** – Confirm which camera is wrist vs exterior: open `logs/obs_wrist.png` and `logs/obs_exterior.png`. Wrist = gripper/hand close-up; exterior = full table/scene. If they’re reversed, set `SWAP_CAMERAS=1` in `run_test.sh` or pass `--swap-cameras`.
3. **Resolution** – We send 224×224; the policy expects this. Don’t change `--image-size` unless your policy is trained for another size.

So: **image quality can be the cause of failure**; use the printed stats and the saved PNGs to verify what the model is seeing.

---

## Quick log check

From a `run_test` JSONL log:

```bash
# High track_err and many clamps → control pipeline (Section 1)
grep '"event":"tick"' logs/run_test_*.jsonl | head -100 | python3 -c "
import sys, json
clamped = 0
total = 0
errs = []
for line in sys.stdin:
    d = json.loads(line)
    total += 1
    errs.append(d.get('track_err', 0))
    if d.get('track_clamped'): clamped += 1
if total:
    print('Clamped ratio:', round(100*clamped/total, 1), '%')
    print('Track err mean/max:', round(sum(errs)/len(errs), 3), round(max(errs), 3))
"
```

- High clamped ratio and high track err → tune Section 1 (and keep Section 2 in mind).
- Low clamped ratio, low track err, but no grasp → focus on Section 2 (model/scene).
