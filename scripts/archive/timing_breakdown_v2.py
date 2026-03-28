#!/usr/bin/env python3
"""
Timing Breakdown Analysis — Where is the loop time going?
Uses wpiutil.log.DataLogReader for proper wpilog parsing.
Compares "bad" session (3/25) vs "good" session (3/26+) vs Idaho match.
"""
import os, glob, datetime, statistics, sys
from wpiutil.log import DataLogReader

LOG_DIRS = [
    os.path.expanduser("~/Desktop/Logs2"),
    r"D:\logs",
]

# Keys to extract
TIMING_KEYS = [
    "/RealOutputs/LoggedRobot/FullCycleMS",
    "/RealOutputs/LoggedRobot/UserCodeMS",
    "/RealOutputs/LoggedRobot/GCTimeMS",
    "/RealOutputs/LoggedRobot/LogPeriodicMS",
    "/RealOutputs/LoggedRobot/GCCounts",
    "/RealOutputs/Logger/AutoLogMS",
    "/RealOutputs/Logger/ConduitCaptureMS",
    "/RealOutputs/Logger/ConduitSaveMS",
    "/RealOutputs/Logger/EntryUpdateMS",
    "/RealOutputs/Logger/DashboardInputsMS",
    "/RealOutputs/Logger/DriverStationMS",
    "/RealOutputs/Logger/ConsoleMS",
    "/RealOutputs/Logger/AlertLogMS",
    "/RealOutputs/Logger/RadioLogMS",
    "/RealOutputs/Logger/QueuedCycles",
]

REPRESENTATIVE_LOGS = [
    # (filename, label)
    ("akit_26-03-25_00-53-10.wpilog", "Early-3/25 (BAD, 75% overrun)"),
    ("akit_26-03-25_17-43-32.wpilog", "Late-3/25 (BAD, 83% overrun)"),
    ("akit_26-03-26_02-51-20.wpilog", "3/26-morning (GOOD, 9% overrun)"),
    ("akit_26-03-27_00-53-36.wpilog", "3/27-early (BEST, 4% overrun)"),
    ("akit_26-03-27_01-12-49.wpilog", "3/27-pre-match (12% overrun)"),
    ("akit_26-03-27_01-50-13_idbo.wpilog", "IDAHO MATCH (13% overrun)"),
]

def find_all_log_files():
    """Build map of basename -> full path"""
    result = {}
    for d in LOG_DIRS:
        if not os.path.isdir(d):
            continue
        for f in glob.glob(os.path.join(d, "**", "*.wpilog"), recursive=True):
            base = os.path.basename(f)
            if base not in result:
                result[base] = f
    return result


def extract_timing(path):
    """Extract timing data using wpiutil DataLogReader."""
    reader = DataLogReader(path)
    
    entry_map = {}  # entry_id -> key_name
    data = {k: [] for k in TIMING_KEYS}
    
    for record in reader:
        if record.isStart():
            sd = record.getStartData()
            if sd.name in TIMING_KEYS:
                entry_map[sd.entry] = sd.name
        elif not record.isControl():
            eid = record.getEntry()
            if eid in entry_map:
                key = entry_map[eid]
                ts = record.getTimestamp() / 1e6  # microseconds to seconds
                try:
                    val = record.getDouble()
                    data[key].append((ts, val))
                except Exception:
                    try:
                        val = record.getInteger()
                        data[key].append((ts, float(val)))
                    except Exception:
                        pass
    
    return data


def stats_line(arr, name, indent="    "):
    if not arr:
        return f"{indent}{name:30s}  -- NO DATA --"
    # Skip first 5 cycles (startup)
    clean = arr[5:] if len(arr) > 10 else arr
    if not clean:
        clean = arr
    mn = statistics.mean(clean)
    md = statistics.median(clean)
    s = sorted(clean)
    p95 = s[int(len(s) * 0.95)] if len(s) > 20 else max(s)
    p99 = s[int(len(s) * 0.99)] if len(s) > 100 else max(s)
    mx = max(clean)
    return f"{indent}{name:30s}  mean={mn:7.2f}  median={md:6.2f}  p95={p95:7.2f}  p99={p99:7.2f}  max={mx:9.1f}  (n={len(clean)})"


def analyze_log(path, basename, label, out):
    out.append(f"\n  {'-' * 74}")
    out.append(f"  {basename} -- {label}")
    out.append(f"  {'-' * 74}")
    
    print(f"  Parsing: {basename} ...", flush=True)
    data = extract_timing(path)
    
    # Extract value arrays
    fc = [v for _, v in data["/RealOutputs/LoggedRobot/FullCycleMS"]]
    uc = [v for _, v in data["/RealOutputs/LoggedRobot/UserCodeMS"]]
    gc = [v for _, v in data["/RealOutputs/LoggedRobot/GCTimeMS"]]
    lp = [v for _, v in data["/RealOutputs/LoggedRobot/LogPeriodicMS"]]
    gcc = [v for _, v in data["/RealOutputs/LoggedRobot/GCCounts"]]
    
    al = [v for _, v in data["/RealOutputs/Logger/AutoLogMS"]]
    cc = [v for _, v in data["/RealOutputs/Logger/ConduitCaptureMS"]]
    cs = [v for _, v in data["/RealOutputs/Logger/ConduitSaveMS"]]
    eu = [v for _, v in data["/RealOutputs/Logger/EntryUpdateMS"]]
    di = [v for _, v in data["/RealOutputs/Logger/DashboardInputsMS"]]
    ds = [v for _, v in data["/RealOutputs/Logger/DriverStationMS"]]
    co = [v for _, v in data["/RealOutputs/Logger/ConsoleMS"]]
    alg = [v for _, v in data["/RealOutputs/Logger/AlertLogMS"]]
    rl = [v for _, v in data["/RealOutputs/Logger/RadioLogMS"]]
    qc = [v for _, v in data["/RealOutputs/Logger/QueuedCycles"]]
    
    out.append("    --- Main Timing (ms) ---")
    out.append(stats_line(fc, "FullCycleMS"))
    out.append(stats_line(uc, "UserCodeMS"))
    out.append(stats_line(gc, "GCTimeMS"))
    out.append(stats_line(lp, "LogPeriodicMS"))
    if gcc:
        clean_gcc = gcc[5:] if len(gcc) > 10 else gcc
        out.append(f"    {'GCCounts':30s}  mean={statistics.mean(clean_gcc):7.2f}  max={max(clean_gcc):9.1f}")
    
    out.append("")
    out.append("    --- Logger Sub-Components (ms) ---")
    out.append(stats_line(al, "AutoLogMS"))
    out.append(stats_line(cc, "ConduitCaptureMS"))
    out.append(stats_line(cs, "ConduitSaveMS"))
    out.append(stats_line(eu, "EntryUpdateMS"))
    out.append(stats_line(di, "DashboardInputsMS"))
    out.append(stats_line(ds, "DriverStationMS"))
    out.append(stats_line(co, "ConsoleMS"))
    out.append(stats_line(alg, "AlertLogMS"))
    out.append(stats_line(rl, "RadioLogMS"))
    if qc:
        clean_qc = qc[5:] if len(qc) > 10 else qc
        out.append(f"    {'QueuedCycles':30s}  mean={statistics.mean(clean_qc):7.1f}  max={max(clean_qc):.0f}")
    
    # Budget breakdown: What % of FullCycle is each component?
    n = min(len(fc), len(uc), len(gc) if gc else 999999, len(lp) if lp else 999999)
    if n > 10:
        skip = 5
        out.append("")
        out.append("    --- Time Budget (% of FullCycle, skipping first 5 cycles) ---")
        fc_s = fc[skip:n]
        uc_s = uc[skip:n]
        gc_s = gc[skip:n] if gc and len(gc) >= n else None
        lp_s = lp[skip:n] if lp and len(lp) >= n else None
        
        total_fc = sum(fc_s)
        total_uc = sum(uc_s)
        total_gc = sum(gc_s) if gc_s else 0
        total_lp = sum(lp_s) if lp_s else 0
        total_other = total_fc - total_uc - total_gc - total_lp
        
        out.append(f"      UserCode:    {total_uc/total_fc*100:5.1f}%  ({total_uc/len(fc_s):.2f} ms/cycle avg)")
        out.append(f"      GC:          {total_gc/total_fc*100:5.1f}%  ({total_gc/len(fc_s):.2f} ms/cycle avg)")
        out.append(f"      LogPeriodic: {total_lp/total_fc*100:5.1f}%  ({total_lp/len(fc_s):.2f} ms/cycle avg)")
        out.append(f"      Other/Sched: {total_other/total_fc*100:5.1f}%  ({total_other/len(fc_s):.2f} ms/cycle avg)")
        out.append(f"      TOTAL:       100.0%  ({total_fc/len(fc_s):.2f} ms/cycle avg)")
    
    # Logger sub-budget
    logger_keys = [
        ("AutoLogMS", al), ("ConduitCaptureMS", cc), ("ConduitSaveMS", cs),
        ("EntryUpdateMS", eu), ("DashboardInputsMS", di), ("DriverStationMS", ds),
        ("ConsoleMS", co), ("AlertLogMS", alg), ("RadioLogMS", rl),
    ]
    avail = [(k, d) for k, d in logger_keys if d]
    if avail and lp:
        n2 = min(len(lp), *(len(d) for _, d in avail))
        if n2 > 10:
            skip = 5
            total_lp2 = sum(lp[skip:n2])
            out.append("")
            out.append("    --- LogPeriodic Sub-Budget (% of LogPeriodicMS) ---")
            for kname, kdata in avail:
                total_k = sum(kdata[skip:n2])
                out.append(f"      {kname:25s}  {total_k/total_lp2*100:5.1f}%  ({total_k/(n2-skip):.3f} ms/cycle avg)")
    
    # Overrun deep-dive: When FullCycle > 25ms, where is the excess?
    if fc and uc and n > 10:
        skip = 5
        overrun_data = []
        for i in range(skip, n):
            if fc[i] > 25:
                row = {"fc": fc[i], "uc": uc[i]}
                if gc and i < len(gc): row["gc"] = gc[i]
                if lp and i < len(lp): row["lp"] = lp[i]
                overrun_data.append(row)
        
        if len(overrun_data) > 5:
            out.append("")
            out.append(f"    --- Overrun Breakdown (FullCycle>25ms, n={len(overrun_data)}/{n-skip}) ---")
            fc_o = [r["fc"] for r in overrun_data]
            uc_o = [r["uc"] for r in overrun_data]
            gc_o = [r["gc"] for r in overrun_data if "gc" in r]
            lp_o = [r["lp"] for r in overrun_data if "lp" in r]
            out.append(f"      FullCycleMS:   mean={statistics.mean(fc_o):.1f}  p95={sorted(fc_o)[int(len(fc_o)*0.95)]:.1f}")
            out.append(f"      UserCodeMS:    mean={statistics.mean(uc_o):.1f}  p95={sorted(uc_o)[int(len(uc_o)*0.95)]:.1f}")
            if gc_o:
                out.append(f"      GCTimeMS:      mean={statistics.mean(gc_o):.1f}  p95={sorted(gc_o)[int(len(gc_o)*0.95)]:.1f}")
            if lp_o:
                out.append(f"      LogPeriodicMS: mean={statistics.mean(lp_o):.1f}  p95={sorted(lp_o)[int(len(lp_o)*0.95)]:.1f}")
            if gc_o and lp_o:
                unaccounted = [r["fc"] - r["uc"] - r.get("gc", 0) - r.get("lp", 0) for r in overrun_data if "gc" in r and "lp" in r]
                if unaccounted:
                    out.append(f"      Unaccounted:   mean={statistics.mean(unaccounted):.1f}  p95={sorted(unaccounted)[int(len(unaccounted)*0.95)]:.1f}")
    
    # GC spike analysis
    if gc and len(gc) > 10:
        gc_clean = gc[5:]
        gc_spikes = [(i+5, v) for i, v in enumerate(gc_clean) if v > 2.0]
        if gc_spikes:
            out.append("")
            out.append(f"    --- GC Spikes (>2ms): {len(gc_spikes)} occurrences ---")
            gc_vals = [v for _, v in gc_spikes]
            out.append(f"      mean={statistics.mean(gc_vals):.1f}ms  max={max(gc_vals):.1f}ms")
            # Are GC spikes correlated with overruns?
            if fc and len(fc) >= len(gc):
                gc_spike_fc = [fc[i] for i, _ in gc_spikes if i < len(fc)]
                if gc_spike_fc:
                    out.append(f"      FullCycle during GC spikes: mean={statistics.mean(gc_spike_fc):.1f}ms  max={max(gc_spike_fc):.1f}ms")
    
    return fc, uc, gc, lp


def main():
    all_logs = find_all_log_files()
    out = []
    
    out.append("=" * 80)
    out.append("  TIMING BREAKDOWN ANALYSIS -- Where is the loop time going?")
    out.append(f"  Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    out.append("=" * 80)
    
    summaries = []
    for basename, label in REPRESENTATIVE_LOGS:
        if basename in all_logs:
            fc, uc, gc, lp = analyze_log(all_logs[basename], basename, label, out)
            if fc:
                skip = min(5, len(fc) - 1) if len(fc) > 1 else 0
                fc_c = fc[skip:]
                summaries.append((label, statistics.mean(fc_c), 
                                  statistics.mean(uc[skip:len(fc_c)+skip]) if uc and len(uc) >= len(fc_c)+skip else 0,
                                  statistics.mean(gc[skip:len(fc_c)+skip]) if gc and len(gc) >= len(fc_c)+skip else 0,
                                  statistics.mean(lp[skip:len(fc_c)+skip]) if lp and len(lp) >= len(fc_c)+skip else 0))
        else:
            out.append(f"\n  SKIPPED: {basename} -- not found")
    
    # Summary table
    out.append("")
    out.append("=" * 80)
    out.append("  COMPARATIVE SUMMARY")
    out.append("=" * 80)
    out.append(f"  {'Label':45s} {'Full':>7s} {'User':>7s} {'GC':>7s} {'Log':>7s} {'Other':>7s}")
    out.append(f"  {'-'*45} {'-'*7} {'-'*7} {'-'*7} {'-'*7} {'-'*7}")
    for label, fc_mean, uc_mean, gc_mean, lp_mean in summaries:
        other = fc_mean - uc_mean - gc_mean - lp_mean
        out.append(f"  {label:45s} {fc_mean:6.1f}  {uc_mean:6.1f}  {gc_mean:6.1f}  {lp_mean:6.1f}  {other:6.1f}")
    
    # Root cause analysis
    out.append("")
    out.append("=" * 80)
    out.append("  ROOT CAUSE ANALYSIS")
    out.append("=" * 80)
    
    # Write output
    out_text = "\n".join(out)
    print(out_text)
    
    out_path = os.path.join(os.path.dirname(__file__), "timing_breakdown.txt")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(out_text)
    print(f"\n  Output written to: {out_path}")


if __name__ == "__main__":
    main()
