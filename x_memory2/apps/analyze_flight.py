import csv
import os
import sys

def analyze(log_path):
    if not os.path.exists(log_path):
        print(f"Error: Log file not found at {log_path}")
        return

    timestamps = []
    ekf_alts = []
    ekf_pitches = []
    ekf_rolls = []
    pwm_frs = []
    pwm_rls = []
    pwm_fls = []
    pwm_rrs = []

    with open(log_path, 'r') as f:
        reader = csv.reader(f)
        header = None
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            if header is None:
                header = [col.strip() for col in row]
                continue
            
            try:
                # Zip and map columns to floats (ignore header row)
                d = dict(zip(header, map(float, row)))
                timestamps.append(d['timestamp_ns'])
                ekf_alts.append(d['ekf_alt'])
                ekf_pitches.append(d['ekf_pitch_deg'])
                ekf_rolls.append(d['ekf_roll_deg'])
                pwm_frs.append(d['pwm_fr'])
                pwm_rls.append(d['pwm_rl'])
                pwm_fls.append(d['pwm_fl'])
                pwm_rrs.append(d['pwm_rr'])
            except Exception as e:
                pass

    if not ekf_alts:
        print("No valid telemetry data found in log file.")
        return

    # Find airborne segment (altitude > 1.0m)
    airborne_indices = [i for i, alt in enumerate(ekf_alts) if alt > 1.0]
    
    if not airborne_indices:
        print("=" * 60)
        print("FLIGHT TELEMETRY DIAGNOSTIC REPORT (takeoff failed)")
        print("=" * 60)
        print("  - Drone did not stay airborne (altitude > 1.0m) long enough.")
        max_pitch = max(ekf_pitches, key=abs)
        max_roll = max(ekf_rolls, key=abs)
        print(f"  - Peak Attitude Roll: {max_roll:+.1f} deg, Pitch: {max_pitch:+.1f} deg")
        if abs(max_pitch) > 50.0 or abs(max_roll) > 50.0:
            print("  - [Diagnostic] Flipped over detected!")
            if abs(max_pitch) > abs(max_roll):
                if max_pitch > 0:
                    print("    --> PITCHED BACKWARD: The drone tilted backward. Increase 'pitch_trim' in parameters.")
                else:
                    print("    --> PITCHED FORWARD: The drone tilted forward. Decrease 'pitch_trim' in parameters.")
            else:
                if max_roll > 0:
                    print("    --> ROLLED RIGHT: The drone tilted right. Decrease roll trim (unsupported yet).")
                else:
                    print("    --> ROLLED LEFT: The drone tilted left. Increase roll trim (unsupported yet).")
        print("=" * 60)
        return

    # Calculate average trims during airborne segment
    pitch_trims = []
    roll_trims = []
    
    for i in airborne_indices:
        fr = pwm_frs[i]
        rl = pwm_rls[i]
        fl = pwm_fls[i]
        rr = pwm_rrs[i]
        
        # pitch trim = (rl + rr - fr - fl) / 4
        # roll trim = (fl + rl - fr - rr) / 4
        p_t = (rl + rr - fr - fl) / 4.0
        r_t = (fl + rl - fr - rr) / 4.0
        pitch_trims.append(p_t)
        roll_trims.append(r_t)

    avg_pitch_trim = sum(pitch_trims) / len(pitch_trims)
    avg_roll_trim = sum(roll_trims) / len(roll_trims)
    
    avg_pitch = sum(ekf_pitches[i] for i in airborne_indices) / len(airborne_indices)
    avg_roll = sum(ekf_rolls[i] for i in airborne_indices) / len(airborne_indices)
    
    print("=" * 60)
    print("FLIGHT TELEMETRY DIAGNOSTIC REPORT (Auto-Tuning)")
    print("=" * 60)
    print(f"Airborne time analyzed: {len(airborne_indices) / 4000.0:.2f} seconds")
    print(f"Average Roll: {avg_roll:+.2f} deg, Pitch: {avg_pitch:+.2f} deg")
    print("-" * 60)
    print("ESTIMATED TRIM CORRECTIONS (To balance CG offset):")
    print(f"  --> pitch_trim (Pitch FF) : {avg_pitch_trim:+.5f} (Current recommendation)")
    print(f"  --> roll_trim  (Roll FF)  : {avg_roll_trim:+.5f}")
    print("-" * 60)
    print("Instructions:")
    print("  1. Open build/pid_params_z30.txt")
    print(f"  2. Update 'pitch_trim' value to: {avg_pitch_trim:.4f}")
    print("  3. Run 'SendCmd pid' to load new parameters live.")
    print("  4. Take off again to verify stability!")
    print("=" * 60)

if __name__ == '__main__':
    log_path = "d:/xlab/sim25/logs/silapp/flight_log_4000hz.csv"
    if len(sys.argv) > 1:
        log_path = sys.argv[1]
    analyze(log_path)
