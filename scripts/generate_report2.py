"""
MRT3216 2026 Season — Advanced Diagnostics Report (Report 2)
Covers: Command timeline, shooter velocity timing, drive speed utilization,
        hood angle units, loop cycle time, vision acceptance, battery discharge,
        match-to-match learning curve, CAN bus health, CPU/temp.

Usage:
    .venv/Scripts/python.exe scripts/generate_report2.py
Output:
    scripts/MRT3216_Diagnostics_Report.pdf
"""

import os, re, bisect, io, sys, datetime, struct as pystruct, math
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from matplotlib.ticker import MaxNLocator

from reportlab.lib import colors
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Image, Table, TableStyle,
    PageBreak, HRFlowable, KeepTogether
)
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.platypus import Flowable

from wpiutil.log import DataLogReader

# ── Paths ────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIRS   = sys.argv[1:] if len(sys.argv) > 1 else [os.path.expanduser("~/Desktop/Logs"), os.path.expanduser("~/Desktop/Logs2")]
OUTPUT_PDF = os.path.join(SCRIPT_DIR, "MRT3216_Diagnostics_Report.pdf")

# ── Signal catalogue ─────────────────────────────────────────────────────────
TELEOP = "/DriverStation/Enabled"
AUTON  = "/DriverStation/Autonomous"

# Signals we want and how to read them
SIGNALS = {
    TELEOP:                                          "boolean",
    AUTON:                                           "boolean",
    # Loop timing
    "/RealOutputs/LoggedRobot/FullCycleMS":          "double",
    "/RealOutputs/LoggedRobot/UserCodeMS":           "double",
    "/RealOutputs/Logger/QueuedCycles":              "int64",
    # Drive speed
    "/RealOutputs/SwerveChassisSpeeds/Measured":     "struct",   # getDoubleArray → [vx,vy,omega]
    # Hood
    "/Hood/Angle":                                   "double",
    "/Hood/Setpoint":                                "double",
    "/RealOutputs/Hood/FX/PositionDegrees":          "double",
    "/RealOutputs/Hood/FX/ReferenceDegrees":         "double",
    # Flywheel / Kicker / Spindexer velocity
    "/Shooter/Flywheel/Velocity":                    "double",
    "/Shooter/Flywheel/Setpoint":                    "double",
    "/RealOutputs/Flywheel/FX/VelocityRPM":         "double",
    "/RealOutputs/Flywheel/FX/ReferenceRPM":        "double",
    "/Kicker/Velocity":                              "double",
    "/Kicker/Setpoint":                              "double",
    "/Spindexer/Velocity":                           "double",
    "/Spindexer/Setpoint":                           "double",
    # Vision (Pose3d[] arrays — count length/56 for pose count)
    "/RealOutputs/Vision/Camera0/RobotPoses":        "struct_arr",
    "/RealOutputs/Vision/Camera0/RobotPosesAccepted":"struct_arr",
    "/RealOutputs/Vision/Camera0/RobotPosesRejected":"struct_arr",
    "/RealOutputs/Vision/Camera1/RobotPoses":        "struct_arr",
    "/RealOutputs/Vision/Camera1/RobotPosesAccepted":"struct_arr",
    "/RealOutputs/Vision/Camera2/RobotPoses":        "struct_arr",
    "/RealOutputs/Vision/Camera2/RobotPosesAccepted":"struct_arr",
    "/RealOutputs/Vision/Camera3/RobotPoses":        "struct_arr",
    "/RealOutputs/Vision/Camera3/RobotPosesAccepted":"struct_arr",
    # Battery discharge
    "/SystemStats/BatteryVoltage":                   "double",
    "/SystemStats/BatteryCurrent":                   "double",
    "/PowerDistribution/TotalEnergy":                "double",
    "/PowerDistribution/TotalPower":                 "double",
    # CAN bus
    "/SystemStats/CANBus/Utilization":               "float",
    "/SystemStats/CANBus/OffCount":                  "int64",
    # CPU & temp
    "/SystemStats/CPUTempCelsius":                   "double",
    # Commands (all boolean)
    "/RealOutputs/CommandsAll/FlywheelStopHold":                           "boolean",
    "/RealOutputs/CommandsAll/HoodSubsystem SetAngle":                     "boolean",
    "/RealOutputs/CommandsAll/Intake.Deploy":                              "boolean",
    "/RealOutputs/CommandsAll/IntakePivotSubsystem SetDutyCycle":          "boolean",
    "/RealOutputs/CommandsAll/IntakeRollersStopHold":                      "boolean",
    "/RealOutputs/CommandsAll/IntakeRollersSubsystem IntakeRollersMech SetSpeed": "boolean",
    "/RealOutputs/CommandsAll/IntakeRollersSubsystem SetDutyCycle":        "boolean",
    "/RealOutputs/CommandsAll/InterruptShooting":                          "boolean",
    "/RealOutputs/CommandsAll/KickerStopHold":                             "boolean",
    "/RealOutputs/CommandsAll/Left Bum Rush (No SOTF)":                    "boolean",
    "/RealOutputs/CommandsAll/ParallelCommandGroup":                       "boolean",
    "/RealOutputs/CommandsAll/SpindexerStopHold":                          "boolean",
    "/RealOutputs/CommandsAll/TurretSubsystem SetAngle":                   "boolean",
}

POSE3D_BYTES = 56   # 7 doubles × 8 bytes (x,y,z, qw,qx,qy,qz)

MAX_SAMPLES = 4000

# ── Log parser ────────────────────────────────────────────────────────────────
def parse_log(path):
    entry_map, type_map = {}, {}
    raw_v = defaultdict(list)
    raw_t = defaultdict(list)

    for record in DataLogReader(path):
        if record.isStart():
            d = record.getStartData()
            if d.name in SIGNALS:
                entry_map[d.entry] = d.name
                type_map[d.entry]  = d.type
            continue
        eid = record.getEntry()
        if eid not in entry_map:
            continue
        name = entry_map[eid]
        t    = record.getTimestamp() / 1e6
        typ  = SIGNALS[name]
        try:
            if   typ == "double":    val = record.getDouble()
            elif typ == "float":     val = float(record.getDouble())
            elif typ == "boolean":   val = 1.0 if record.getBoolean() else 0.0
            elif typ == "int64":     val = float(record.getInteger())
            elif typ == "struct":
                arr = record.getDoubleArray()
                val = arr   # store list, will be [vx, vy, omega]
            elif typ == "struct_arr":
                raw = record.getRaw()
                val = len(raw) // POSE3D_BYTES   # number of poses in this frame
            else:
                continue
        except Exception:
            continue
        raw_v[name].append(val)
        raw_t[name].append(t)

    # Build teleop windows
    en_pts = list(zip(raw_t[TELEOP], raw_v[TELEOP]))
    au_pts = list(zip(raw_t[AUTON],  raw_v[AUTON]))
    change_times = sorted(set(t for t,_ in en_pts) | set(t for t,_ in au_pts))
    en_s=sorted(en_pts); au_s=sorted(au_pts)
    tt=[]; tv=[]; en=au=0.0; ei=ai=0
    for ct in change_times:
        while ei<len(en_s) and en_s[ei][0]<=ct: en=en_s[ei][1]; ei+=1
        while ai<len(au_s) and au_s[ai][0]<=ct: au=au_s[ai][1]; ai+=1
        tt.append(ct); tv.append(en==1.0 and au==0.0)

    def in_tele(t):
        idx=bisect.bisect_right(tt,t)-1
        return idx>=0 and tv[idx]

    # Compute teleop start time (for relative-time timelines)
    tele_start = next((t for t,v in zip(tt,tv) if v), None)

    result = {"tele_start": tele_start, "tele_filter": in_tele}
    for sig in SIGNALS:
        if sig in (TELEOP, AUTON): continue
        ts  = raw_t[sig]
        vs  = raw_v[sig]
        typ = SIGNALS[sig]
        # Filter to teleop
        tele_ts=[]; tele_vs=[]
        for t,v in zip(ts,vs):
            if in_tele(t): tele_ts.append(t); tele_vs.append(v)
        # Downsample plain scalars; keep full precision for commands/vision
        if typ in ("double","float","int64") and len(tele_vs)>MAX_SAMPLES:
            step=len(tele_vs)//MAX_SAMPLES
            tele_ts=tele_ts[::step]; tele_vs=tele_vs[::step]
        result[sig] = {"ts": tele_ts, "vs": tele_vs}
    return result


# ── Discover match logs ───────────────────────────────────────────────────────
def get_match_type(fn):
    m=re.search(r'_azfg_(q|e|p)(\d+)',fn,re.I)
    return (m.group(1).upper(),int(m.group(2))) if m else ("?",0)

seen=set(); match_logs=[]
for d in LOG_DIRS:
    for f in os.listdir(d):
        if "_azfg_" in f and f.endswith(".wpilog") and f not in seen:
            seen.add(f)
            mt,mn=get_match_type(f)
            fp=os.path.join(d,f); sz=os.path.getsize(fp)/1e6
            match_logs.append({"type":mt,"num":mn,"name":f,"path":fp,"mb":sz})
match_logs.sort(key=lambda x:(x["type"],x["num"]))

print("Parsing logs...")
for m in match_logs:
    lbl=f"{m['type']}{m['num']}"
    print(f"  {lbl} ({m['mb']:.0f}MB)...", flush=True)
    m["parsed"]=parse_log(m["path"])
    m["label"]=lbl

# Keep only matches with teleop data
active=[m for m in match_logs if m["parsed"]["tele_start"] is not None]
# Qual matches only (for learning curve, sorted chronologically by match number)
quals=[m for m in active if m["type"]=="Q"]
quals.sort(key=lambda x:x["num"])
print(f"Done. {len(active)} matches with teleop data.")

# ── Style helpers ─────────────────────────────────────────────────────────────
PAL={"bg":"#1A1A2E","mid":"#16213E","accent":"#E94560","light":"#EAEAEA","grid":"#2D2D4E"}
plt.rcParams.update({
    "figure.facecolor":PAL["bg"],"axes.facecolor":PAL["mid"],
    "axes.edgecolor":PAL["light"],"axes.labelcolor":PAL["light"],
    "xtick.color":PAL["light"],"ytick.color":PAL["light"],
    "text.color":PAL["light"],"grid.color":PAL["grid"],
    "grid.linestyle":"--","grid.alpha":0.6,"font.family":"DejaVu Sans",
    "axes.titlesize":10,"axes.labelsize":9,"xtick.labelsize":8,
    "ytick.labelsize":8,"legend.fontsize":8,"legend.framealpha":0.4,
})

styles=getSampleStyleSheet()
def ms(name,parent="Normal",**kw):
    s=ParagraphStyle(name,parent=styles[parent],**kw); styles.add(s); return s

TITLE =ms("DTitle","Title",   fontSize=26,textColor=colors.HexColor("#E94560"),alignment=TA_CENTER,fontName="Helvetica-Bold")
SUB   =ms("DSub",  "Normal",  fontSize=13,textColor=colors.HexColor("#A0A0C0"),alignment=TA_CENTER,spaceAfter=4)
H1    =ms("DH1",   "Heading1",fontSize=15,textColor=colors.HexColor("#E94560"),spaceBefore=10,spaceAfter=5,fontName="Helvetica-Bold")
H2    =ms("DH2",   "Heading2",fontSize=11,textColor=colors.HexColor("#60A0E9"),spaceBefore=7,spaceAfter=3,fontName="Helvetica-Bold")
BODY  =ms("DBody", "Normal",  fontSize=9, textColor=colors.HexColor("#CCCCCC"),spaceAfter=4,leading=13)
SMALL =ms("DSmall","Normal",  fontSize=7.5,textColor=colors.HexColor("#AAAAAA"),spaceAfter=2)
CAP   =ms("DCap",  "Normal",  fontSize=8, textColor=colors.HexColor("#888888"),alignment=TA_CENTER,spaceAfter=6)

TBL=[ ("BACKGROUND",(0,0),(-1,0),colors.HexColor("#E94560")),
      ("TEXTCOLOR",  (0,0),(-1,0),colors.white),
      ("FONTNAME",   (0,0),(-1,0),"Helvetica-Bold"),
      ("FONTSIZE",   (0,0),(-1,-1),8),
      ("ROWBACKGROUNDS",(0,1),(-1,-1),[colors.HexColor("#1E1E3A"),colors.HexColor("#252545")]),
      ("TEXTCOLOR",  (0,1),(-1,-1),colors.HexColor("#DDDDDD")),
      ("ALIGN",      (0,0),(-1,-1),"CENTER"),
      ("VALIGN",     (0,0),(-1,-1),"MIDDLE"),
      ("GRID",       (0,0),(-1,-1),0.3,colors.HexColor("#333360")),
      ("TOPPADDING", (0,0),(-1,-1),3),
      ("BOTTOMPADDING",(0,0),(-1,-1),3)]

class ColorBar(Flowable):
    def __init__(self,h=3,c="#E94560"): super().__init__(); self.h=h; self.c=c
    def wrap(self,w,h): self.width=w; return w,self.h
    def draw(self):
        self.canv.setFillColor(colors.HexColor(self.c))
        self.canv.rect(0,0,self.width,self.h,fill=1,stroke=0)

def fig2img(fig,w=6.5,dpi=150):
    buf=io.BytesIO()
    fig.savefig(buf,format="png",dpi=dpi,bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    return Image(buf,width=w*inch,height=w*inch*(fig.get_figheight()/fig.get_figwidth()))

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 2 — Command Timeline Gantt
# ══════════════════════════════════════════════════════════════════════════════
COMMAND_SIGS = [
    ("/RealOutputs/CommandsAll/TurretSubsystem SetAngle",     "TurretSubsystem SetAngle",    "#9C27B0"),
    ("/RealOutputs/CommandsAll/HoodSubsystem SetAngle",       "HoodSubsystem SetAngle",      "#00BCD4"),
    ("/RealOutputs/CommandsAll/FlywheelStopHold",             "FlywheelStopHold",            "#4CAF50"),
    ("/RealOutputs/CommandsAll/InterruptShooting",            "InterruptShooting",           "#F44336"),
    ("/RealOutputs/CommandsAll/Left Bum Rush (No SOTF)",      "Left Bum Rush",               "#FF9800"),
    ("/RealOutputs/CommandsAll/Intake.Deploy",                "Intake.Deploy",               "#2196F3"),
    ("/RealOutputs/CommandsAll/IntakePivotSubsystem SetDutyCycle", "IntakePivot SetDuty",    "#E91E63"),
    ("/RealOutputs/CommandsAll/IntakeRollersSubsystem IntakeRollersMech SetSpeed","IntakeRollers SetSpeed","#8BC34A"),
    ("/RealOutputs/CommandsAll/KickerStopHold",               "KickerStopHold",              "#795548"),
    ("/RealOutputs/CommandsAll/SpindexerStopHold",            "SpindexerStopHold",           "#607D8B"),
    ("/RealOutputs/CommandsAll/ParallelCommandGroup",         "ParallelCommandGroup",        "#FF5722"),
]

def gantt_for_match(m, max_t=135):
    """Return a matplotlib figure with a Gantt-style command timeline."""
    parsed = m["parsed"]
    ts0    = parsed["tele_start"] or 0
    fig, ax = plt.subplots(figsize=(10, 4.2))

    plotted = []
    for row_i, (sig, label, clr) in enumerate(COMMAND_SIGS):
        d = parsed.get(sig, {})
        ts = d.get("ts", []); vs = d.get("vs", [])
        if not ts:
            continue
        # Convert to relative time, build run segments
        segs_on = []
        active = False; t_on = None
        for t, v in zip(ts, vs):
            rel = t - ts0
            if rel > max_t: break
            if v > 0.5 and not active:
                active = True; t_on = rel
            elif v < 0.5 and active:
                active = False
                segs_on.append((t_on, rel - t_on))
        if active and t_on is not None:
            segs_on.append((t_on, max_t - t_on))
        if not segs_on:
            continue
        y = len(plotted)
        for (x0, w) in segs_on:
            ax.barh(y, w, left=x0, height=0.7, color=clr, alpha=0.85, edgecolor="#0D0D1A", linewidth=0.3)
        plotted.append((y, label, clr))

    if not plotted:
        plt.close(fig)
        return None

    ys=[p[0] for p in plotted]; lbls=[p[1] for p in plotted]
    ax.set_yticks(ys); ax.set_yticklabels(lbls, fontsize=7.5)
    ax.set_xlabel("Time into Teleop (s)")
    ax.set_title(f"Command Activity — {m['label']}")
    ax.set_xlim(0, max_t)
    ax.xaxis.grid(True); ax.set_axisbelow(True)
    ax.invert_yaxis()
    fig.tight_layout()
    return fig

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 3 — Shooter Velocity Timing
# ══════════════════════════════════════════════════════════════════════════════
def chart_shooter_velocity(m):
    """Flywheel velocity vs setpoint + Kicker on/off markers for one match."""
    parsed=m["parsed"]; ts0=parsed["tele_start"] or 0
    fw_ts=[t-ts0 for t in parsed.get("/Shooter/Flywheel/Velocity",{}).get("ts",[])]
    fw_vs=parsed.get("/Shooter/Flywheel/Velocity",{}).get("vs",[])
    fs_ts=[t-ts0 for t in parsed.get("/Shooter/Flywheel/Setpoint",{}).get("ts",[])]
    fs_vs=parsed.get("/Shooter/Flywheel/Setpoint",{}).get("vs",[])
    kv_ts=[t-ts0 for t in parsed.get("/Kicker/Velocity",{}).get("ts",[])]
    kv_vs=parsed.get("/Kicker/Velocity",{}).get("vs",[])

    # Use RealOutputs RPM if subsystem signals empty
    if not fw_vs:
        fw_ts=[t-ts0 for t in parsed.get("/RealOutputs/Flywheel/FX/VelocityRPM",{}).get("ts",[])]
        fw_vs=parsed.get("/RealOutputs/Flywheel/FX/VelocityRPM",{}).get("vs",[])
        fs_ts=[t-ts0 for t in parsed.get("/RealOutputs/Flywheel/FX/ReferenceRPM",{}).get("ts",[])]
        fs_vs=parsed.get("/RealOutputs/Flywheel/FX/ReferenceRPM",{}).get("vs",[])

    if not fw_vs and not kv_vs:
        return None

    fig,ax=plt.subplots(figsize=(10,3.5))
    if fw_vs:
        ax.plot(fw_ts,fw_vs,lw=1.5,color="#4CAF50",label="Flywheel Velocity")
    if fs_vs:
        ax.step(fs_ts,fs_vs,lw=1.2,color="#FFEB3B",alpha=0.7,where="post",label="Flywheel Setpoint")
    if kv_vs:
        ax2=ax.twinx()
        ax2.plot(kv_ts,kv_vs,lw=1.2,color="#FF9800",alpha=0.8,label="Kicker Vel")
        ax2.set_ylabel("Kicker Velocity",color="#FF9800",fontsize=8)
        ax2.tick_params(colors="#FF9800",labelsize=7)
    ax.set_xlabel("Time into Teleop (s)")
    ax.set_ylabel("Flywheel (RPM or rad/s)")
    ax.set_title(f"Flywheel Velocity vs Setpoint — {m['label']}")
    ax.legend(loc="upper left",fontsize=7); ax.yaxis.grid(True); ax.set_axisbelow(True)
    fig.tight_layout()
    return fig

def chart_shooter_spinup_histogram(all_matches):
    """
    For each match, find intervals where flywheel goes from 0 to >= 90% of setpoint.
    Plot histogram of spin-up durations.
    """
    spinup_times = []
    for m in all_matches:
        parsed=m["parsed"]
        fw_ts=parsed.get("/RealOutputs/Flywheel/FX/VelocityRPM",{}).get("ts",[])
        fw_vs=parsed.get("/RealOutputs/Flywheel/FX/VelocityRPM",{}).get("vs",[])
        fs_ts=parsed.get("/RealOutputs/Flywheel/FX/ReferenceRPM",{}).get("ts",[])
        fs_vs=parsed.get("/RealOutputs/Flywheel/FX/ReferenceRPM",{}).get("vs",[])
        if not fw_vs or not fs_vs: continue
        # Find each time setpoint goes from ~0 to non-zero, then measure time to 90%
        sp_dict={}
        for t,v in zip(fs_ts,fs_vs): sp_dict[t]=v
        sp_times=sorted(sp_dict)
        prev_sp=0.0
        for i,t in enumerate(sp_times):
            sp=sp_dict[t]
            if prev_sp<100 and sp>=100:  # setpoint jumped up
                target=sp*0.90
                # Find first fw_v sample after t that exceeds target
                t_start=t
                for ft,fv in zip(fw_ts,fw_vs):
                    if ft<t_start: continue
                    if fv>=target:
                        spinup_times.append(ft-t_start)
                        break
                    if ft-t_start>5: break  # gave up
            prev_sp=sp

    if not spinup_times:
        return None
    fig,ax=plt.subplots(figsize=(7,3.5))
    bins=np.arange(0,max(spinup_times)+0.5,0.25)
    ax.hist(spinup_times,bins=bins,color="#4CAF50",edgecolor="#0D0D1A",alpha=0.85)
    ax.axvline(np.median(spinup_times),color="#FFEB3B",lw=2,ls="--",label=f"Median: {np.median(spinup_times):.2f}s")
    ax.set_xlabel("Spin-up Time (s)"); ax.set_ylabel("Count")
    ax.set_title("Flywheel Spin-up Duration (0 to 90% setpoint, all matches)")
    ax.legend(); ax.yaxis.grid(True); ax.set_axisbelow(True)
    fig.tight_layout()
    return fig

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 4 — Drive Speed Utilization
# ══════════════════════════════════════════════════════════════════════════════
MAX_SPEED = 5.05  # TunerConstants.kSpeedAt12Volts in m/s (typical Kraken swerve)

def chart_speed_histograms():
    """Stacked histogram of drive speed utilization % across all matches."""
    bins=np.array([0,0.25,0.50,0.75,1.01])   # fraction of max speed
    bin_labels=["0-25%","25-50%","50-75%","75-100%"]
    match_bins={}
    for m in active:
        parsed=m["parsed"]
        # ChassisSpeeds/Measured → [vx,vy,omega] list per timestep
        css=parsed.get("/RealOutputs/SwerveChassisSpeeds/Measured",{})
        vs=css.get("vs",[])
        if not vs: continue
        speeds=[]
        for v in vs:
            if isinstance(v,(list,tuple)) and len(v)>=2:
                speed=math.sqrt(v[0]**2+v[1]**2)
            elif isinstance(v,(int,float)):
                speed=abs(v)
            else: continue
            speeds.append(speed/MAX_SPEED)
        if not speeds: continue
        counts,_=np.histogram(speeds,bins=bins)
        pcts=100*counts/len(speeds)
        match_bins[m["label"]]=pcts

    if not match_bins:
        return None
    labels=list(match_bins.keys())
    data=np.array([match_bins[l] for l in labels])
    fig,ax=plt.subplots(figsize=(10,4))
    x=np.arange(len(labels)); w=0.18
    clrs=["#1565C0","#42A5F5","#FF9800","#F44336"]
    for i,(bn,c) in enumerate(zip(bin_labels,clrs)):
        ax.bar(x+i*w,data[:,i],w,label=bn,color=c,alpha=0.88,edgecolor="#0D0D1A",linewidth=0.3)
    ax.set_xticks(x+1.5*w); ax.set_xticklabels(labels,rotation=45,ha="right")
    ax.set_ylabel("% of Teleop Samples")
    ax.set_title("Drive Speed Utilization Distribution per Match")
    ax.legend(title="Speed Band",ncol=4); ax.yaxis.grid(True); ax.set_axisbelow(True)
    fig.tight_layout()
    return fig

def chart_speed_timeline(m):
    """Robot speed over time for one match."""
    parsed=m["parsed"]; ts0=parsed["tele_start"] or 0
    css=parsed.get("/RealOutputs/SwerveChassisSpeeds/Measured",{})
    ts_raw=css.get("ts",[]); vs_raw=css.get("vs",[])
    times=[]; speeds=[]
    for t,v in zip(ts_raw,vs_raw):
        if isinstance(v,(list,tuple)) and len(v)>=2:
            speeds.append(math.sqrt(v[0]**2+v[1]**2))
            times.append(t-ts0)
    if not times: return None
    fig,ax=plt.subplots(figsize=(9,2.8))
    ax.plot(times,speeds,lw=1.2,color="#2196F3",alpha=0.9)
    ax.axhline(MAX_SPEED*0.75,color="#FFEB3B",lw=1,ls="--",label="75% max")
    ax.set_xlabel("Time into Teleop (s)"); ax.set_ylabel("Speed (m/s)")
    ax.set_title(f"Robot Speed — {m['label']}")
    ax.set_ylim(0,MAX_SPEED*1.05); ax.legend(fontsize=7)
    ax.yaxis.grid(True); ax.set_axisbelow(True)
    fig.tight_layout()
    return fig

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 5 — Hood Angle Unit Investigation
# ══════════════════════════════════════════════════════════════════════════════
def chart_hood_angles():
    """Plot Hood/Angle vs Hood/Setpoint and RealOutputs/Hood/FX/PositionDegrees per match."""
    labels=[m["label"] for m in active]
    raw_max=[]; raw_sp_max=[]; fx_max=[]; fx_ref_max=[]
    for m in active:
        p=m["parsed"]
        rv=p.get("/Hood/Angle",{}).get("vs",[])
        sv=p.get("/Hood/Setpoint",{}).get("vs",[])
        fv=p.get("/RealOutputs/Hood/FX/PositionDegrees",{}).get("vs",[])
        rv2=p.get("/RealOutputs/Hood/FX/ReferenceDegrees",{}).get("vs",[])
        raw_max.append(max(abs(v) for v in rv) if rv else 0)
        raw_sp_max.append(max(abs(v) for v in sv) if sv else 0)
        fx_max.append(max(abs(v) for v in fv) if fv else 0)
        fx_ref_max.append(max(abs(v) for v in rv2) if rv2 else 0)

    x=np.arange(len(labels)); w=0.2
    fig,ax=plt.subplots(figsize=(10,4))
    ax.bar(x-1.5*w,raw_max,    w,label="Hood/Angle (raw)",        color="#00BCD4",alpha=0.85,edgecolor="#0D0D1A",linewidth=0.3)
    ax.bar(x-0.5*w,raw_sp_max, w,label="Hood/Setpoint (raw)",     color="#0097A7",alpha=0.85,edgecolor="#0D0D1A",linewidth=0.3)
    ax.bar(x+0.5*w,fx_max,     w,label="FX/PositionDegrees",      color="#4CAF50",alpha=0.85,edgecolor="#0D0D1A",linewidth=0.3)
    ax.bar(x+1.5*w,fx_ref_max, w,label="FX/ReferenceDegrees",     color="#8BC34A",alpha=0.85,edgecolor="#0D0D1A",linewidth=0.3)
    ax.set_xticks(x); ax.set_xticklabels(labels,rotation=45,ha="right")
    ax.set_ylabel("Maximum Absolute Value (units depend on signal)")
    ax.set_title("Hood Angle — Raw vs FX Degrees (unit discrepancy check)")
    ax.legend(ncol=2); ax.yaxis.grid(True); ax.set_axisbelow(True)
    # Add rad->deg reference
    ax.axhline(math.pi,color="#FFEB3B",lw=1.2,ls=":",label=f"pi rad = {math.pi:.3f}")
    ax.axhline(math.pi*0.25,color="#FF9800",lw=1,ls=":",label=f"pi/4 rad = {math.pi/4:.3f}")
    fig.tight_layout()
    return fig

def chart_hood_timeline(m):
    """Hood angle vs setpoint over time for a single match."""
    parsed=m["parsed"]; ts0=parsed["tele_start"] or 0
    raw_ts=[t-ts0 for t in parsed.get("/Hood/Angle",{}).get("ts",[])]
    raw_vs=parsed.get("/Hood/Angle",{}).get("vs",[])
    sp_ts =[t-ts0 for t in parsed.get("/Hood/Setpoint",{}).get("ts",[])]
    sp_vs =parsed.get("/Hood/Setpoint",{}).get("vs",[])
    fx_ts =[t-ts0 for t in parsed.get("/RealOutputs/Hood/FX/PositionDegrees",{}).get("ts",[])]
    fx_vs =parsed.get("/RealOutputs/Hood/FX/PositionDegrees",{}).get("vs",[])
    fx2_ts=[t-ts0 for t in parsed.get("/RealOutputs/Hood/FX/ReferenceDegrees",{}).get("ts",[])]
    fx2_vs=parsed.get("/RealOutputs/Hood/FX/ReferenceDegrees",{}).get("vs",[])
    if not raw_vs and not fx_vs: return None
    fig,axs=plt.subplots(2,1,figsize=(10,5),sharex=True)
    if raw_vs:
        axs[0].plot(raw_ts,raw_vs,lw=1.2,color="#00BCD4",label="Hood/Angle (raw)")
    if sp_vs:
        axs[0].step(sp_ts,sp_vs,lw=1.2,color="#FFEB3B",alpha=0.8,where="post",label="Hood/Setpoint (raw)")
    axs[0].set_ylabel("Raw Value"); axs[0].legend(fontsize=7); axs[0].yaxis.grid(True)
    axs[0].set_title(f"Hood Angle & Setpoint — {m['label']}")
    if fx_vs:
        axs[1].plot(fx_ts,fx_vs,lw=1.2,color="#4CAF50",label="FX/PositionDegrees")
    if fx2_vs:
        axs[1].step(fx2_ts,fx2_vs,lw=1.2,color="#8BC34A",alpha=0.8,where="post",label="FX/ReferenceDegrees")
    axs[1].set_ylabel("Degrees"); axs[1].set_xlabel("Time into Teleop (s)")
    axs[1].legend(fontsize=7); axs[1].yaxis.grid(True)
    fig.tight_layout()
    return fig

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 6 — Loop Cycle Time
# ══════════════════════════════════════════════════════════════════════════════
def chart_loop_times():
    """Box plot of FullCycleMS and UserCodeMS per match."""
    labels=[m["label"] for m in active]
    full_data=[]; user_data=[]
    for m in active:
        p=m["parsed"]
        fv=p.get("/RealOutputs/LoggedRobot/FullCycleMS",{}).get("vs",[])
        uv=p.get("/RealOutputs/LoggedRobot/UserCodeMS",{}).get("vs",[])
        full_data.append(fv if fv else [0])
        user_data.append(uv if uv else [0])

    fig,axs=plt.subplots(1,2,figsize=(10,4))
    bp1=axs[0].boxplot(full_data,patch_artist=True,medianprops={"color":"white","lw":2},
                       labels=labels)
    for patch in bp1["boxes"]: patch.set_facecolor("#2196F3"); patch.set_alpha(0.7)
    axs[0].axhline(20,color="#F44336",lw=1.5,ls="--",label="20ms deadline")
    axs[0].set_xticklabels(labels,rotation=45,ha="right")
    axs[0].set_ylabel("Milliseconds"); axs[0].set_title("Full Cycle Time (FullCycleMS)")
    axs[0].legend(fontsize=7); axs[0].yaxis.grid(True); axs[0].set_axisbelow(True)

    bp2=axs[1].boxplot(user_data,patch_artist=True,medianprops={"color":"white","lw":2},
                       labels=labels)
    for patch in bp2["boxes"]: patch.set_facecolor("#4CAF50"); patch.set_alpha(0.7)
    axs[1].axhline(10,color="#FF9800",lw=1.5,ls="--",label="10ms reference")
    axs[1].set_xticklabels(labels,rotation=45,ha="right")
    axs[1].set_ylabel("Milliseconds"); axs[1].set_title("User Code Time (UserCodeMS)")
    axs[1].legend(fontsize=7); axs[1].yaxis.grid(True); axs[1].set_axisbelow(True)
    fig.suptitle("Loop Cycle Time Distribution per Match",fontsize=10,color=PAL["light"])
    fig.tight_layout()
    return fig

def chart_loop_overruns():
    """Bar chart: count of cycles where FullCycleMS > 20ms (overruns)."""
    labels=[m["label"] for m in active]
    overruns=[]
    for m in active:
        fv=m["parsed"].get("/RealOutputs/LoggedRobot/FullCycleMS",{}).get("vs",[])
        overruns.append(sum(1 for v in fv if v>20))
    x=np.arange(len(labels))
    clrs=["#F44336" if c>50 else "#FF9800" if c>10 else "#4CAF50" for c in overruns]
    fig,ax=plt.subplots(figsize=(10,3))
    ax.bar(x,overruns,color=clrs,edgecolor="#0D0D1A",linewidth=0.3)
    for xi,c in zip(x,overruns):
        if c>0: ax.text(xi,c+0.5,str(c),ha="center",fontsize=7.5,color=PAL["light"])
    ax.set_xticks(x); ax.set_xticklabels(labels,rotation=45,ha="right")
    ax.set_ylabel("Overrun Count")
    ax.set_title("Loop Cycle Overruns per Match (FullCycleMS > 20ms)")
    red_p=mpatches.Patch(color="#F44336",label=">50 overruns")
    ora_p=mpatches.Patch(color="#FF9800",label="10-50 overruns")
    grn_p=mpatches.Patch(color="#4CAF50",label="<10 overruns")
    ax.legend(handles=[red_p,ora_p,grn_p],fontsize=7)
    ax.yaxis.grid(True); ax.set_axisbelow(True)
    fig.tight_layout()
    return fig

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 7 — Vision Acceptance Rate
# ══════════════════════════════════════════════════════════════════════════════
def chart_vision_acceptance():
    """Bar chart: accepted / total poses per camera per match."""
    labels=[m["label"] for m in active]
    cameras=["Camera0","Camera1","Camera2","Camera3"]
    cam_colors=["#2196F3","#4CAF50","#FF9800","#9C27B0"]
    fig,axs=plt.subplots(2,2,figsize=(10,6),sharey=False)
    axs=axs.flatten()
    for ci,cam in enumerate(cameras):
        acc_key=f"/RealOutputs/Vision/{cam}/RobotPosesAccepted"
        all_key=f"/RealOutputs/Vision/{cam}/RobotPoses"
        pcts=[]
        totals=[]
        for m in active:
            p=m["parsed"]
            acc_vs=p.get(acc_key,{}).get("vs",[])
            all_vs=p.get(all_key,{}).get("vs",[])
            total_obs=sum(all_vs)   # each sample value = number of poses in that frame
            total_acc=sum(acc_vs)
            totals.append(total_obs)
            pcts.append(100*total_acc/total_obs if total_obs>0 else 0)
        x=np.arange(len(labels))
        ax=axs[ci]
        bars=ax.bar(x,pcts,color=cam_colors[ci],alpha=0.85,edgecolor="#0D0D1A",linewidth=0.3)
        # Annotate with total obs count
        for xi,p,tot in zip(x,pcts,totals):
            if tot>0:
                ax.text(xi,p+1.5,f"{tot:.0f}",ha="center",fontsize=6.5,color=PAL["light"])
        ax.set_xticks(x); ax.set_xticklabels(labels,rotation=45,ha="right",fontsize=7)
        ax.set_ylabel("Accept Rate (%)"); ax.set_title(cam,fontsize=9)
        ax.set_ylim(0,115); ax.yaxis.grid(True); ax.set_axisbelow(True)
        ax.axhline(80,color="#FFEB3B",lw=0.8,ls="--",alpha=0.6)
    fig.suptitle("Vision Pose Accept Rate per Camera per Match\n(numbers = total raw observations; yellow line = 80%)",
                 fontsize=9,color=PAL["light"])
    fig.tight_layout()
    return fig

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 8 — Battery Discharge Curve
# ══════════════════════════════════════════════════════════════════════════════
def chart_battery_discharge(m):
    """Voltage vs cumulative Ah discharged for one match."""
    parsed=m["parsed"]; ts0=parsed["tele_start"] or 0
    bv_ts=parsed.get("/SystemStats/BatteryVoltage",{}).get("ts",[])
    bv_vs=parsed.get("/SystemStats/BatteryVoltage",{}).get("vs",[])
    bc_ts=parsed.get("/SystemStats/BatteryCurrent",{}).get("ts",[])
    bc_vs=parsed.get("/SystemStats/BatteryCurrent",{}).get("vs",[])
    if not bv_vs or not bc_vs: return None

    # Integrate current to get Ah (Trapezoidal)
    bc_sorted=sorted(zip(bc_ts,bc_vs)); bv_dict=dict(zip(bv_ts,bv_vs))
    ah=0.0; prev_t=bc_sorted[0][0]; ah_list=[]; bv_interp=[]
    for t,i in bc_sorted:
        dt=(t-prev_t)/3600.0
        ah+=i*dt; prev_t=t
        # Interpolate voltage
        bv_t=min(bv_dict.keys(),key=lambda x:abs(x-t))
        ah_list.append(ah); bv_interp.append(bv_dict[bv_t])
    fig,ax=plt.subplots(figsize=(8,3.5))
    ax.plot(ah_list,bv_interp,lw=1.5,color="#4CAF50",alpha=0.9)
    ax.axhline(6.3,color="#F44336",lw=1.2,ls="--",label="Brownout (6.3V)")
    ax.axhline(10.0,color="#FFEB3B",lw=1,ls=":",label="Healthy floor (10V)")
    ax.set_xlabel("Cumulative Charge Discharged (Ah)")
    ax.set_ylabel("Battery Voltage (V)")
    ax.set_title(f"Battery Discharge Curve — {m['label']}")
    ax.set_ylim(4,14); ax.legend(fontsize=7)
    ax.xaxis.grid(True); ax.yaxis.grid(True); ax.set_axisbelow(True)
    fig.tight_layout()
    return fig

def chart_discharge_comparison():
    """Overlay discharge curves for all matches (normalized to teleop start Ah=0)."""
    fig,ax=plt.subplots(figsize=(10,4))
    cmap=plt.cm.tab20
    for i,m in enumerate(active):
        parsed=m["parsed"]
        bv_ts=parsed.get("/SystemStats/BatteryVoltage",{}).get("ts",[])
        bv_vs=parsed.get("/SystemStats/BatteryVoltage",{}).get("vs",[])
        bc_ts=parsed.get("/SystemStats/BatteryCurrent",{}).get("ts",[])
        bc_vs=parsed.get("/SystemStats/BatteryCurrent",{}).get("vs",[])
        if not bv_vs or not bc_vs: continue
        bc_s=sorted(zip(bc_ts,bc_vs)); bv_d=dict(zip(bv_ts,bv_vs))
        ah=0.0; prev_t=bc_s[0][0]; ah_l=[]; bv_l=[]
        for t,c in bc_s:
            dt=(t-prev_t)/3600.0; ah+=c*dt; prev_t=t
            bv_t=min(bv_d.keys(),key=lambda x:abs(x-t))
            ah_l.append(ah); bv_l.append(bv_d[bv_t])
        ax.plot(ah_l,bv_l,lw=1,alpha=0.75,label=m["label"],color=cmap(i/len(active)))
    ax.axhline(6.3,color="#F44336",lw=1.5,ls="--",label="Brownout (6.3V)")
    ax.axhline(10.0,color="#FFEB3B",lw=1,  ls=":",label="10V floor")
    ax.set_xlabel("Cumulative Ah Discharged"); ax.set_ylabel("Battery Voltage (V)")
    ax.set_title("Battery Discharge Curves — All Matches (full match, not just teleop)")
    ax.set_ylim(4,14)
    ax.legend(fontsize=6.5,ncol=4,loc="lower left")
    ax.xaxis.grid(True); ax.yaxis.grid(True); ax.set_axisbelow(True)
    fig.tight_layout()
    return fig

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 9 — Match-to-Match Learning Curve
# ══════════════════════════════════════════════════════════════════════════════
def chart_learning_curve():
    """Qual matches sorted chronologically: battery avg/min, CAN utilization, loop time."""
    if not quals: return None
    q_labels=[m["label"] for m in quals]
    q_avg_v=[]; q_min_v=[]; q_can=[]; q_loop=[]; q_cpu=[]
    for m in quals:
        p=m["parsed"]
        bv=p.get("/SystemStats/BatteryVoltage",{}).get("vs",[])
        can=p.get("/SystemStats/CANBus/Utilization",{}).get("vs",[])
        lp=p.get("/RealOutputs/LoggedRobot/FullCycleMS",{}).get("vs",[])
        cpu=p.get("/SystemStats/CPUTempCelsius",{}).get("vs",[])
        q_avg_v.append(sum(bv)/len(bv) if bv else 0)
        q_min_v.append(min(bv)         if bv else 0)
        q_can.append(sum(can)/len(can)*100 if can else 0)  # fraction -> %
        q_loop.append(sum(lp)/len(lp) if lp else 0)
        q_cpu.append(max(cpu) if cpu else 0)

    x=np.arange(len(q_labels))
    fig,axs=plt.subplots(2,2,figsize=(10,7))
    # Voltage trend
    axs[0,0].plot(x,q_avg_v,"o-",color="#4CAF50",lw=2,ms=6,label="Avg Voltage")
    axs[0,0].plot(x,q_min_v,"s--",color="#F44336",lw=2,ms=6,label="Min Voltage")
    axs[0,0].axhline(6.3,color="#FF9800",lw=1,ls=":",label="Brownout")
    axs[0,0].set_title("Battery Voltage Trend (Quals)"); axs[0,0].set_ylabel("V")
    axs[0,0].set_xticks(x); axs[0,0].set_xticklabels(q_labels,rotation=45,ha="right",fontsize=7)
    axs[0,0].legend(fontsize=7); axs[0,0].yaxis.grid(True); axs[0,0].set_axisbelow(True)
    # CAN utilization
    axs[0,1].bar(x,q_can,color="#9C27B0",alpha=0.85,edgecolor="#0D0D1A",linewidth=0.3)
    axs[0,1].axhline(90,color="#F44336",lw=1.2,ls="--",label=">90% risk")
    axs[0,1].set_title("CAN Bus Utilization (Quals)"); axs[0,1].set_ylabel("%")
    axs[0,1].set_xticks(x); axs[0,1].set_xticklabels(q_labels,rotation=45,ha="right",fontsize=7)
    axs[0,1].legend(fontsize=7); axs[0,1].yaxis.grid(True); axs[0,1].set_axisbelow(True)
    # Loop time
    axs[1,0].bar(x,q_loop,color="#2196F3",alpha=0.85,edgecolor="#0D0D1A",linewidth=0.3)
    axs[1,0].axhline(20,color="#F44336",lw=1.2,ls="--",label="20ms deadline")
    axs[1,0].set_title("Avg Loop Cycle Time (Quals)"); axs[1,0].set_ylabel("ms")
    axs[1,0].set_xticks(x); axs[1,0].set_xticklabels(q_labels,rotation=45,ha="right",fontsize=7)
    axs[1,0].legend(fontsize=7); axs[1,0].yaxis.grid(True); axs[1,0].set_axisbelow(True)
    # CPU temp
    axs[1,1].bar(x,q_cpu,color="#FF9800",alpha=0.85,edgecolor="#0D0D1A",linewidth=0.3)
    axs[1,1].axhline(70,color="#F44336",lw=1.2,ls="--",label="70C warning")
    axs[1,1].set_title("Peak CPU Temperature (Quals)"); axs[1,1].set_ylabel("C")
    axs[1,1].set_xticks(x); axs[1,1].set_xticklabels(q_labels,rotation=45,ha="right",fontsize=7)
    axs[1,1].legend(fontsize=7); axs[1,1].yaxis.grid(True); axs[1,1].set_axisbelow(True)
    fig.suptitle("Match-to-Match Learning Curve — Qualification Matches",fontsize=11,color=PAL["light"])
    fig.tight_layout()
    return fig

def chart_can_health_all():
    """CAN utilization and off-count for all matches."""
    labels=[m["label"] for m in active]
    can_util=[]; can_off=[]
    for m in active:
        p=m["parsed"]
        cu=p.get("/SystemStats/CANBus/Utilization",{}).get("vs",[])
        co=p.get("/SystemStats/CANBus/OffCount",{}).get("vs",[])
        can_util.append(sum(cu)/len(cu)*100 if cu else 0)
        # OffCount is a cumulative counter; take max - min = total off events during teleop
        can_off.append(max(co)-min(co) if len(co)>1 else 0)
    x=np.arange(len(labels))
    fig,axs=plt.subplots(1,2,figsize=(10,3.5))
    clrs=["#F44336" if v>80 else "#FF9800" if v>60 else "#4CAF50" for v in can_util]
    axs[0].bar(x,can_util,color=clrs,edgecolor="#0D0D1A",linewidth=0.3)
    axs[0].axhline(90,color="#F44336",lw=1.5,ls="--",label=">90% risk")
    axs[0].axhline(70,color="#FF9800",lw=1,ls=":",label=">70% caution")
    for xi,v in zip(x,can_util):
        if v>0: axs[0].text(xi,v+0.5,f"{v:.0f}%",ha="center",fontsize=6.5,color=PAL["light"])
    axs[0].set_xticks(x); axs[0].set_xticklabels(labels,rotation=45,ha="right")
    axs[0].set_ylabel("Avg Utilization (%)"); axs[0].set_title("CAN Bus Utilization")
    axs[0].legend(fontsize=7); axs[0].yaxis.grid(True); axs[0].set_axisbelow(True)

    off_clrs=["#F44336" if v>0 else "#4CAF50" for v in can_off]
    axs[1].bar(x,can_off,color=off_clrs,edgecolor="#0D0D1A",linewidth=0.3)
    for xi,v in zip(x,can_off):
        if v>0: axs[1].text(xi,v+0.01,str(int(v)),ha="center",fontsize=6.5,color=PAL["light"])
    axs[1].set_xticks(x); axs[1].set_xticklabels(labels,rotation=45,ha="right")
    axs[1].set_ylabel("Off Events (delta during teleop)"); axs[1].set_title("CAN Bus Off Events")
    axs[1].yaxis.grid(True); axs[1].set_axisbelow(True)
    fig.suptitle("CAN Bus Health — All Matches",fontsize=10,color=PAL["light"])
    fig.tight_layout()
    return fig

# ══════════════════════════════════════════════════════════════════════════════
# PDF ASSEMBLY
# ══════════════════════════════════════════════════════════════════════════════
def build_pdf():
    doc=SimpleDocTemplate(OUTPUT_PDF,pagesize=letter,
        leftMargin=0.6*inch,rightMargin=0.6*inch,
        topMargin=0.6*inch,bottomMargin=0.6*inch)
    s=[]

    # ── Cover ─────────────────────────────────────────────────────────────────
    s+=[Spacer(1,1*inch),ColorBar(6,"#E94560"),Spacer(1,0.2*inch),
        Paragraph("MRT3216",TITLE),
        Paragraph("2026 Season — Advanced Diagnostics Report",SUB),
        Spacer(1,0.1*inch),ColorBar(2,"#60A0E9"),Spacer(1,0.3*inch),
        Paragraph("This report covers in-depth diagnostic analyses drawn from AdvantageKit "
                  ".wpilog match logs. Topics covered: command activity timelines, "
                  "shooter spin-up timing, drive speed utilization, hood angle unit "
                  "verification, loop cycle overruns, vision acceptance rates, battery "
                  "discharge curves, and match-to-match learning trends.",BODY),
        Spacer(1,0.3*inch)]
    cov_data=[["Section","Topic","Key Question"],
              ["2","Command Timeline Gantt","What was the robot actually doing each match?"],
              ["3","Shooter Velocity Timing","Was the flywheel spun up before shooting?"],
              ["4","Drive Speed Utilization","How aggressively was the robot driven?"],
              ["5","Hood Angle Unit Investigation","Are hood setpoints in degrees or radians?"],
              ["6","Loop Cycle Time","Were 20ms deadlines being met?"],
              ["7","Vision Acceptance Rate","How often did pose estimates get accepted?"],
              ["8","Battery Discharge Curve","How fast was charge depleted per match?"],
              ["9","Match-to-Match Trends","Did performance change across quals?"],
              ["Bonus","CAN Bus Health","Was the CAN bus a reliability risk?"]]
    ct=Table(cov_data,colWidths=[0.7*inch,2.4*inch,3.4*inch])
    ct.setStyle(TableStyle(TBL))
    s+=[ct,Spacer(1,0.4*inch),
        Paragraph(f"Generated {datetime.datetime.now().strftime('%B %d, %Y at %H:%M')} | "
                  "MRT3216 Programming Subteam",CAP),PageBreak()]

    # ── SECTION 2: Command Timelines ─────────────────────────────────────────
    s+=[Paragraph("Section 2 — Command Activity Timeline",H1),ColorBar(2,"#60A0E9"),
        Spacer(1,0.1*inch),
        Paragraph("Each bar represents a time interval where that command was actively scheduled "
                  "during teleop. This is read directly from <i>/RealOutputs/CommandsAll/*</i> "
                  "boolean signals. Note that 'StopHold' commands run as default commands — "
                  "they occupy the subsystem when nothing else has it.",BODY),
        Spacer(1,0.1*inch)]
    # Show Gantt for the two busiest matches (most command diversity)
    for lbl in ["E9","Q23","Q82"]:
        m_obj=next((m for m in active if m["label"]==lbl),None)
        if m_obj is None and active: m_obj=active[0]
        if m_obj:
            fig=gantt_for_match(m_obj)
            if fig:
                s+=[fig2img(fig,6.5),
                    Paragraph(f"Figure — Command activity timeline for {m_obj['label']}. "
                              "X-axis is seconds into teleop.",CAP),Spacer(1,0.1*inch)]

    # Command usage frequency table
    s+=[Paragraph("Command Usage Frequency (across all matches)",H2)]
    cmd_rows=[["Command","Matches Present","Avg % Teleop Active"]]
    for sig,label,_ in COMMAND_SIGS:
        match_count=0; pct_sum=0.0
        for m in active:
            vs=m["parsed"].get(sig,{}).get("vs",[])
            if not vs: continue
            match_count+=1
            pct_sum+=100*sum(vs)/len(vs)
        avg_pct=pct_sum/match_count if match_count else 0
        cmd_rows.append([label,str(match_count),f"{avg_pct:.1f}%"])
    ct2=Table(cmd_rows,colWidths=[2.8*inch,1.2*inch,1.5*inch])
    ct2.setStyle(TableStyle(TBL))
    s+=[ct2,PageBreak()]

    # ── SECTION 3: Shooter Velocity ──────────────────────────────────────────
    s+=[Paragraph("Section 3 — Shooter Velocity & Spin-up Timing",H1),ColorBar(2,"#4CAF50"),
        Spacer(1,0.1*inch),
        Paragraph("A critical cycle-time factor is whether the flywheel reached its target "
                  "velocity before the kicker fired. If the kicker fires while the flywheel "
                  "is still accelerating, the ball exits at reduced velocity and unpredictable "
                  "angle. The charts below show flywheel velocity vs. setpoint for selected "
                  "matches, plus a histogram of spin-up durations across all matches.",BODY),
        Spacer(1,0.1*inch)]
    for lbl in ["E4","Q23","Q82"]:
        m_obj=next((m for m in active if m["label"]==lbl),None)
        if m_obj is None and active: m_obj=active[0]
        if m_obj:
            fig=chart_shooter_velocity(m_obj)
            if fig:
                s+=[fig2img(fig,6.5),
                    Paragraph(f"Figure — Flywheel velocity (green) vs setpoint (yellow) + "
                              f"kicker velocity (orange) for {m_obj['label']}.",CAP),
                    Spacer(1,0.08*inch)]
    fig=chart_shooter_spinup_histogram(active)
    if fig:
        s+=[fig2img(fig,5.5),
            Paragraph("Figure — Distribution of flywheel spin-up times (0 to 90% setpoint) "
                      "across all matches. Shorter is better.",CAP)]
    s.append(PageBreak())

    # ── SECTION 4: Drive Speed ───────────────────────────────────────────────
    s+=[Paragraph("Section 4 — Drive Speed Utilization",H1),ColorBar(2,"#2196F3"),
        Spacer(1,0.1*inch),
        Paragraph("Understanding how aggressively the robot was driven helps distinguish "
                  "'high drive current from speed' vs. 'high drive current despite being slow'. "
                  "Speed is computed from <i>SwerveChassisSpeeds/Measured</i> as "
                  "sqrt(vx² + vy²) and normalized to the robot's theoretical maximum "
                  f"({MAX_SPEED} m/s). The joystick axis value structure "
                  "(<i>DriverStation/Joystick0/AxisValues</i>) is also available for future "
                  "driver input analysis.",BODY),
        Spacer(1,0.1*inch)]
    fig=chart_speed_histograms()
    if fig:
        s+=[fig2img(fig,6.5),
            Paragraph("Figure — Speed utilization distribution. Each group of 4 bars shows "
                      "what fraction of teleop the robot spent in each speed band.",CAP),
            Spacer(1,0.1*inch)]
    # Timeline for two representative matches
    for lbl in ["Q63","Q82"]:
        m_obj=next((m for m in active if m["label"]==lbl),None)
        if m_obj:
            fig=chart_speed_timeline(m_obj)
            if fig:
                s+=[fig2img(fig,6.5),
                    Paragraph(f"Figure — Robot speed over teleop for {m_obj['label']}.",CAP),
                    Spacer(1,0.08*inch)]
    s.append(PageBreak())

    # ── SECTION 5: Hood Angles ───────────────────────────────────────────────
    s+=[Paragraph("Section 5 — Hood Angle Unit Investigation",H1),ColorBar(2,"#00BCD4"),
        Spacer(1,0.1*inch),
        Paragraph("Earlier single-match analysis found <i>Hood/Angle</i> values near 0.227 — "
                  "which would be radians (~13°) rather than degrees. However, "
                  "<i>RealOutputs/Hood/FX/PositionDegrees</i> logs the raw TalonFX "
                  "position directly in degrees. Comparing both signals reveals whether "
                  "the YAMS-layer signal is in radians or degrees.",BODY),
        Spacer(1,0.1*inch)]
    fig=chart_hood_angles()
    s+=[fig2img(fig,6.5),
        Paragraph("Figure — Max absolute value of all four Hood signals per match. "
                  "If Hood/Angle peaks at ~0.2–0.5 while FX/PositionDegrees peaks "
                  "at 10–30+, the YAMS signal is in radians.",CAP),
        Spacer(1,0.1*inch)]
    # Detailed timeline for one match
    best_hood=next((m for m in active if m["parsed"].get("/Hood/Angle",{}).get("vs",[])),None)
    if best_hood:
        fig=chart_hood_timeline(best_hood)
        if fig:
            s+=[fig2img(fig,6.5),
                Paragraph(f"Figure — Hood angle and setpoint timelines for {best_hood['label']}. "
                          "Top panel: YAMS raw value. Bottom panel: TalonFX degrees.",CAP)]
    s.append(PageBreak())

    # ── SECTION 6: Loop Timing ───────────────────────────────────────────────
    s+=[Paragraph("Section 6 — Loop Cycle Time & Overruns",H1),ColorBar(2,"#FF9800"),
        Spacer(1,0.1*inch),
        Paragraph("The roboRIO runs the robot loop on a 20ms (50Hz) period. If <i>FullCycleMS</i> "
                  "exceeds 20ms, the scheduler misses its deadline, causing delayed sensor "
                  "reads, stale odometry, and degraded control. <i>UserCodeMS</i> is the "
                  "time spent executing user subsystem/command code. The difference between "
                  "FullCycleMS and UserCodeMS is AdvantageKit logging overhead.",BODY),
        Spacer(1,0.1*inch)]
    fig=chart_loop_times()
    s+=[fig2img(fig,6.5),
        Paragraph("Figure — Box plots of FullCycleMS (left) and UserCodeMS (right). "
                  "The red dashed line marks the 20ms deadline.",CAP),
        Spacer(1,0.1*inch)]
    fig=chart_loop_overruns()
    s+=[fig2img(fig,6.5),
        Paragraph("Figure — Count of loop cycles exceeding 20ms per match.",CAP)]

    # Summary table
    loop_rows=[["Match","Avg FullCycleMS","Max FullCycleMS","Avg UserCodeMS","Overruns (>20ms)"]]
    for m in active:
        fv=m["parsed"].get("/RealOutputs/LoggedRobot/FullCycleMS",{}).get("vs",[])
        uv=m["parsed"].get("/RealOutputs/LoggedRobot/UserCodeMS",{}).get("vs",[])
        loop_rows.append([
            m["label"],
            f"{sum(fv)/len(fv):.2f}" if fv else "N/A",
            f"{max(fv):.2f}"         if fv else "N/A",
            f"{sum(uv)/len(uv):.2f}" if uv else "N/A",
            str(sum(1 for v in fv if v>20)) if fv else "N/A",
        ])
    lt=Table(loop_rows,colWidths=[0.6*inch,1.2*inch,1.2*inch,1.2*inch,1.2*inch])
    lt.setStyle(TableStyle(TBL))
    s+=[Spacer(1,0.1*inch),lt,PageBreak()]

    # ── SECTION 7: Vision ────────────────────────────────────────────────────
    s+=[Paragraph("Section 7 — Vision Pose Acceptance Rate",H1),ColorBar(2,"#9C27B0"),
        Spacer(1,0.1*inch),
        Paragraph("Vision pose estimates are filtered by the Vision subsystem before being "
                  "passed to the pose estimator. Factors that cause rejection include: "
                  "excessive Z coordinate (camera disconnected or tag partially visible), "
                  "pose outside field boundaries, and high tag ambiguity. A low acceptance "
                  "rate means the robot is effectively driving without vision correction — "
                  "relying entirely on wheel odometry, which drifts significantly over a "
                  "2-minute match.",BODY),
        Spacer(1,0.1*inch)]
    fig=chart_vision_acceptance()
    s+=[fig2img(fig,6.5),
        Paragraph("Figure — Vision pose acceptance rate per camera per match. Numbers above "
                  "bars show total raw observations. Yellow line = 80% threshold.",CAP),
        Spacer(1,0.1*inch),
        Paragraph("Note: OrangePI-Left@2 and OrangePI-Right@1 are visible in SystemStats/NTClients, "
                  "confirming two coprocessors are connected and publishing to NetworkTables. "
                  "Camera0–3 correspond to the physical cameras on those coprocessors.",BODY),
        PageBreak()]

    # ── SECTION 8: Battery Discharge ────────────────────────────────────────
    s+=[Paragraph("Section 8 — Battery Discharge Curves",H1),ColorBar(2,"#4CAF50"),
        Spacer(1,0.1*inch),
        Paragraph("By integrating <i>SystemStats/BatteryCurrent</i> over time (trapezoidal rule), "
                  "we can construct a discharge curve: battery voltage as a function of "
                  "cumulative Ah drawn. A healthy FRC battery (18 Ah rated, ~12–14 Ah usable) "
                  "should show a relatively flat curve above 10V until the final few Ah. "
                  "A steeply dropping curve indicates either a degraded battery or excessive "
                  "current draw.",BODY),
        Spacer(1,0.1*inch)]
    fig=chart_discharge_comparison()
    s+=[fig2img(fig,6.5),
        Paragraph("Figure — All matches overlaid. A curve that drops quickly to 6–8V with only "
                  "1–2 Ah drawn indicates a heavily loaded or partially discharged battery.",CAP),
        Spacer(1,0.15*inch)]
    # Individual curve for worst match (lowest minimum battery voltage)
    worst_m=sorted(active,key=lambda m: min(m["parsed"].get("/SystemStats/BatteryVoltage",{}).get("vs",[999])))[0]
    fig=chart_battery_discharge(worst_m)
    if fig:
        s+=[fig2img(fig,5.5),
            Paragraph(f"Figure — Discharge curve for {worst_m['label']} (worst min voltage). "
                      "X-axis shows cumulative charge removed; Y-axis shows voltage.",CAP)]
    s.append(PageBreak())

    # ── SECTION 9: Learning Curve ────────────────────────────────────────────
    s+=[Paragraph("Section 9 — Match-to-Match Performance Trends",H1),ColorBar(2,"#60A0E9"),
        Spacer(1,0.1*inch),
        Paragraph("Tracking key metrics across qualification matches in chronological order "
                  "reveals whether the robot (or team) was improving during the event. "
                  "Metrics: battery voltage trend (proxy for current draw and battery quality), "
                  "CAN bus utilization (proxy for device count and communication health), "
                  "average loop cycle time (proxy for software load), and peak CPU temperature.",BODY),
        Spacer(1,0.1*inch)]
    fig=chart_learning_curve()
    if fig:
        s+=[fig2img(fig,6.5),
            Paragraph("Figure — Four key metrics across qualification matches (Q6 through Q82). "
                      "Downward voltage trends indicate worse batteries or higher loads in later matches.",CAP),
            Spacer(1,0.1*inch)]
    # CAN bus across all matches
    s+=[Paragraph("CAN Bus Health — All Matches",H2)]
    fig=chart_can_health_all()
    s+=[fig2img(fig,6.5),
        Paragraph("Figure — CAN bus utilization (left) and off-event count (right). "
                  "Utilization above 90% risks dropped packets and missed actuator commands. "
                  "Off events indicate a device briefly fell off the bus (connector issue or brownout).",CAP),
        Spacer(1,0.3*inch),
        HRFlowable(width="100%",thickness=1,color=colors.HexColor("#E94560")),
        Spacer(1,0.1*inch),
        Paragraph(f"MRT3216 Advanced Diagnostics Report | Generated {datetime.datetime.now().strftime('%B %d, %Y')} | "
                  "Data: AdvantageKit .wpilog files",CAP)]

    doc.build(s)
    print(f"DONE: Report saved to: {OUTPUT_PDF}")

if __name__=="__main__":
    build_pdf()
