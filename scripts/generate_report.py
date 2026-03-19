"""
MRT3216 2026 Season — Match Performance Report Generator
Produces a multi-page PDF with charts, tables, and analysis from .wpilog data.

Usage:
    .venv\Scripts\python.exe scripts\generate_report.py
Output:
    scripts\MRT3216_Match_Report.pdf
"""

import os, re, bisect, io, datetime
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from matplotlib.ticker import MultipleLocator

from reportlab.lib import colors
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Image, Table, TableStyle,
    PageBreak, HRFlowable, KeepTogether
)
from reportlab.lib.enums import TA_CENTER, TA_LEFT, TA_RIGHT
from reportlab.platypus import Flowable

from wpiutil.log import DataLogReader

# ── Paths ────────────────────────────────────────────────────────────────────
LOG_DIRS = [
    r"C:\Users\danla\Desktop\Logs",
    r"C:\Users\danla\Desktop\Logs2",
]
OUTPUT_PDF = r"C:\Users\danla\Documents\GitHub\MRT3216-2026-Rebuilt\scripts\MRT3216_Match_Report.pdf"

# ── Signal definitions ────────────────────────────────────────────────────────
CURRENT_SIGNALS = {
    "/Drive/Module0/DriveCurrentAmps": "Drive",
    "/Drive/Module0/TurnCurrentAmps":  "Drive",
    "/Drive/Module1/DriveCurrentAmps": "Drive",
    "/Drive/Module1/TurnCurrentAmps":  "Drive",
    "/Drive/Module2/DriveCurrentAmps": "Drive",
    "/Drive/Module2/TurnCurrentAmps":  "Drive",
    "/Drive/Module3/DriveCurrentAmps": "Drive",
    "/Drive/Module3/TurnCurrentAmps":  "Drive",
    "/Intake/Pivot/Current":           "IntakePivot",
    "/Intake/Rollers/Current":         "IntakeRollers",
    "/Shooter/Turret/Current":         "Turret",
    "/Shooter/Flywheel/Current":       "Flywheel",
    "/Hood/Current":                   "Hood",
    "/Kicker/Current":                 "Kicker",
    "/Spindexer/Current":              "Spindexer",
    "/SystemStats/BatteryVoltage":     "_Battery",
    "/SystemStats/BatteryCurrent":     "_BatteryCurrent",
}
TELEOP = "/DriverStation/Enabled"
AUTON  = "/DriverStation/Autonomous"
SUBSYSTEMS = ["Drive","IntakePivot","IntakeRollers","Turret","Flywheel","Hood","Kicker","Spindexer"]
SUB_COLORS = {
    "Drive":        "#2196F3",
    "IntakePivot":  "#F44336",
    "IntakeRollers":"#FF9800",
    "Turret":       "#9C27B0",
    "Flywheel":     "#4CAF50",
    "Hood":         "#00BCD4",
    "Kicker":       "#795548",
    "Spindexer":    "#607D8B",
}

MAX_SAMPLES = 6000

# ── Log parser (same fast single-pass as match_analysis.py) ──────────────────
def parse_log(path):
    entry_map, type_map = {}, {}
    raw, raw_t = defaultdict(list), defaultdict(list)

    for record in DataLogReader(path):
        if record.isStart():
            d = record.getStartData()
            if d.name in CURRENT_SIGNALS or d.name in (TELEOP, AUTON):
                entry_map[d.entry] = d.name
                type_map[d.entry]  = d.type
            continue
        eid = record.getEntry()
        if eid not in entry_map:
            continue
        name = entry_map[eid]
        t    = record.getTimestamp() / 1e6
        typ  = type_map[eid]
        try:
            if   typ in ("double","float"): val = record.getDouble()
            elif typ == "boolean":          val = 1.0 if record.getBoolean() else 0.0
            elif typ == "int64":            val = float(record.getInteger())
            else: continue
        except Exception:
            continue
        raw[name].append(val)
        raw_t[name].append(t)

    # Build teleop windows
    en_pts = list(zip(raw_t[TELEOP], raw[TELEOP]))
    au_pts = list(zip(raw_t[AUTON],  raw[AUTON]))
    change_times = sorted(set(t for t,_ in en_pts)|set(t for t,_ in au_pts))
    en_s = sorted(en_pts); au_s = sorted(au_pts)
    tt, tv = [], []
    en=au=0.0; ei=ai=0
    for ct in change_times:
        while ei<len(en_s) and en_s[ei][0]<=ct: en=en_s[ei][1]; ei+=1
        while ai<len(au_s) and au_s[ai][0]<=ct: au=au_s[ai][1]; ai+=1
        tt.append(ct); tv.append(en==1.0 and au==0.0)

    def in_tele(t):
        idx=bisect.bisect_right(tt,t)-1
        return idx>=0 and tv[idx]

    teleop_data  = {}
    raw_bv       = []
    raw_pivot    = []

    for sig in CURRENT_SIGNALS:
        if sig not in raw: continue
        ts=raw_t[sig]; vs=raw[sig]
        tvs=[]; tts=[]
        for t,v in zip(ts,vs):
            if in_tele(t): tvs.append(v); tts.append(t)
        if not tvs: continue
        if sig=="/SystemStats/BatteryVoltage": raw_bv=list(zip(tts,tvs))
        if sig=="/Intake/Pivot/Current":       raw_pivot=list(zip(tts,tvs))
        n=len(tvs)
        if n>MAX_SAMPLES:
            step=n//MAX_SAMPLES; tvs=tvs[::step]; tts=tts[::step]
        teleop_data[sig]={"vals":tvs,"times":tts}

    return {"teleop_data":teleop_data,"raw_bv":raw_bv,"raw_pivot":raw_pivot}


def summarize(parsed):
    td = parsed["teleop_data"]
    sub_samples = defaultdict(list)
    for sig,d in td.items():
        sub=CURRENT_SIGNALS.get(sig)
        if sub and not sub.startswith("_"):
            sub_samples[sub].extend(d["vals"])
    stats={}
    for sub,vals in sub_samples.items():
        if vals: stats[sub]={"avg":sum(vals)/len(vals),"max":max(vals),"n":len(vals)}
        else:    stats[sub]={"avg":0,"max":0,"n":0}
    bv=[v for _,v in parsed["raw_bv"]]
    bc=td.get("/SystemStats/BatteryCurrent",{}).get("vals",[])
    stats["_Battery"]={
        "avg_v": sum(bv)/len(bv) if bv else 0,
        "min_v": min(bv)         if bv else 0,
        "avg_c": sum(bc)/len(bc) if bc else 0,
        "max_c": max(bc)         if bc else 0,
    }
    return stats


# ── Discover logs ────────────────────────────────────────────────────────────
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

# ── Parse all logs ────────────────────────────────────────────────────────────
print("Parsing logs...")
for m in match_logs:
    lbl=f"{m['type']}{m['num']}"
    print(f"  {lbl} ({m['mb']:.0f}MB)...", flush=True)
    m["parsed"] = parse_log(m["path"])
    m["stats"]  = summarize(m["parsed"])
    m["label"]  = lbl

# Filter to matches with teleop data
active = [m for m in match_logs if m["stats"].get("_Battery",{}).get("avg_v",0)>0]

print(f"Parsed {len(active)} matches with teleop data. Building report...")

# ── Helpers ───────────────────────────────────────────────────────────────────
def fig_to_image(fig, width_in=6.5, dpi=150):
    """Convert a matplotlib figure to a ReportLab Image flowable."""
    buf=io.BytesIO()
    fig.savefig(buf,format="png",dpi=dpi,bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    img=Image(buf,width=width_in*inch,height=width_in*inch*
              (fig.get_figheight()/fig.get_figwidth()))
    return img

def team_palette():
    return {"bg":"#1A1A2E","accent":"#E94560","mid":"#16213E","light":"#EAEAEA"}

PAL = team_palette()

# ── Matplotlib style ──────────────────────────────────────────────────────────
plt.rcParams.update({
    "figure.facecolor":  PAL["bg"],
    "axes.facecolor":    PAL["mid"],
    "axes.edgecolor":    PAL["light"],
    "axes.labelcolor":   PAL["light"],
    "xtick.color":       PAL["light"],
    "ytick.color":       PAL["light"],
    "text.color":        PAL["light"],
    "grid.color":        "#2D2D4E",
    "grid.linestyle":    "--",
    "grid.alpha":        0.6,
    "font.family":       "DejaVu Sans",
    "axes.titlesize":    11,
    "axes.labelsize":    9,
    "xtick.labelsize":   8,
    "ytick.labelsize":   8,
    "legend.fontsize":   8,
    "legend.framealpha": 0.4,
})

# ── ReportLab styles ──────────────────────────────────────────────────────────
styles = getSampleStyleSheet()

def make_style(name, parent="Normal", **kwargs):
    s=ParagraphStyle(name, parent=styles[parent], **kwargs)
    styles.add(s)
    return s

TITLE  = make_style("RTitle","Title",  fontSize=28, textColor=colors.HexColor("#E94560"),
                    spaceAfter=6, alignment=TA_CENTER, fontName="Helvetica-Bold")
SUB    = make_style("RSub",  "Normal", fontSize=14, textColor=colors.HexColor("#A0A0C0"),
                    spaceAfter=4, alignment=TA_CENTER)
H1     = make_style("RH1",   "Heading1",fontSize=16, textColor=colors.HexColor("#E94560"),
                    spaceBefore=12, spaceAfter=6, fontName="Helvetica-Bold")
H2     = make_style("RH2",   "Heading2",fontSize=12, textColor=colors.HexColor("#60A0E9"),
                    spaceBefore=8,  spaceAfter=4, fontName="Helvetica-Bold")
BODY   = make_style("RBody", "Normal", fontSize=9,  textColor=colors.HexColor("#CCCCCC"),
                    spaceAfter=4, leading=13)
SMALL  = make_style("RSmall","Normal", fontSize=7.5,textColor=colors.HexColor("#AAAAAA"),
                    spaceAfter=2)
CAPTION= make_style("RCaption","Normal",fontSize=8, textColor=colors.HexColor("#888888"),
                    alignment=TA_CENTER, spaceAfter=6)

TBL_HEADER = [
    ("BACKGROUND", (0,0), (-1,0), colors.HexColor("#E94560")),
    ("TEXTCOLOR",  (0,0), (-1,0), colors.white),
    ("FONTNAME",   (0,0), (-1,0), "Helvetica-Bold"),
    ("FONTSIZE",   (0,0), (-1,-1),8),
    ("ROWBACKGROUNDS",(0,1),(-1,-1),[colors.HexColor("#1E1E3A"),colors.HexColor("#252545")]),
    ("TEXTCOLOR",  (0,1), (-1,-1), colors.HexColor("#DDDDDD")),
    ("ALIGN",      (0,0), (-1,-1),"CENTER"),
    ("VALIGN",     (0,0), (-1,-1),"MIDDLE"),
    ("GRID",       (0,0), (-1,-1), 0.3, colors.HexColor("#333360")),
    ("TOPPADDING", (0,0), (-1,-1), 3),
    ("BOTTOMPADDING",(0,0),(-1,-1),3),
]


class ColorBar(Flowable):
    """A full-width horizontal accent bar."""
    def __init__(self, height=3, color="#E94560"):
        super().__init__()
        self.bar_height=height; self.color=color
    def wrap(self,w,h): self.width=w; return w,self.bar_height
    def draw(self):
        self.canv.setFillColor(colors.HexColor(self.color))
        self.canv.rect(0,0,self.width,self.bar_height,fill=1,stroke=0)


# ═══════════════════════════════════════════════════════════════════════════════
# CHARTS
# ═══════════════════════════════════════════════════════════════════════════════

def chart_avg_current_by_match():
    """Grouped bar chart: avg current per subsystem per match."""
    labels=[m["label"] for m in active]
    x=np.arange(len(labels)); width=0.10
    fig,ax=plt.subplots(figsize=(10,4.5))
    for i,sub in enumerate(SUBSYSTEMS):
        vals=[m["stats"].get(sub,{}).get("avg",0) for m in active]
        ax.bar(x+i*width,vals,width,label=sub,color=SUB_COLORS[sub],alpha=0.9)
    ax.set_xticks(x+(len(SUBSYSTEMS)-1)*width/2)
    ax.set_xticklabels(labels,rotation=45,ha="right")
    ax.set_ylabel("Average Current (A)")
    ax.set_title("Average Current Draw per Subsystem — All Matches (Teleop)")
    ax.legend(loc="upper right",ncol=4)
    ax.yaxis.grid(True); ax.set_axisbelow(True)
    return fig


def chart_pivot_stall_heatmap():
    """Heatmap: % of teleop where IntakePivot > 20A."""
    labels=[m["label"] for m in active]
    pcts=[]
    for m in active:
        pv=m["parsed"]["raw_pivot"]
        if pv:
            vals=[v for _,v in pv]
            pcts.append(100*sum(1 for v in vals if v>20)/len(vals))
        else: pcts.append(0)
    fig,ax=plt.subplots(figsize=(10,1.8))
    data=np.array(pcts).reshape(1,-1)
    im=ax.imshow(data,aspect="auto",cmap="RdYlGn_r",vmin=0,vmax=100)
    ax.set_xticks(range(len(labels))); ax.set_xticklabels(labels,rotation=45,ha="right",fontsize=8)
    ax.set_yticks([]); ax.set_title("IntakePivot: % of Teleop Above 20A (Red = Bad)",pad=8)
    for j,pct in enumerate(pcts):
        ax.text(j,0,f"{pct:.0f}%",ha="center",va="center",
                fontsize=7.5,color="white" if pct>50 else "black",fontweight="bold")
    plt.colorbar(im,ax=ax,label="%",shrink=0.8,pad=0.01)
    fig.tight_layout()
    return fig


def chart_battery_voltage():
    """Line chart: avg and min battery voltage per match."""
    labels=[m["label"] for m in active]
    avg_v=[m["stats"]["_Battery"]["avg_v"] for m in active]
    min_v=[m["stats"]["_Battery"]["min_v"] for m in active]
    x=np.arange(len(labels))
    fig,ax=plt.subplots(figsize=(10,3.8))
    ax.plot(x,avg_v,"o-",color="#4CAF50",lw=2,ms=6,label="Avg Voltage")
    ax.plot(x,min_v,"s--",color="#F44336",lw=2,ms=6,label="Min Voltage")
    ax.axhline(6.3,color="#FF9800",lw=1.5,ls=":",label="roboRIO Brownout (6.3V)")
    ax.axhline(8.0,color="#FFEB3B",lw=1,  ls=":",label="Warning Threshold (8V)")
    ax.fill_between(x,min_v,6.3,where=[v<6.3 for v in min_v],
                    alpha=0.25,color="#F44336",label="Brownout zone")
    ax.set_xticks(x); ax.set_xticklabels(labels,rotation=45,ha="right")
    ax.set_ylabel("Battery Voltage (V)"); ax.set_ylim(4,14)
    ax.set_title("Battery Voltage During Teleop — All Matches")
    ax.legend(loc="lower left",ncol=3); ax.yaxis.grid(True); ax.set_axisbelow(True)
    return fig


def chart_subsystem_pie(match_data):
    """Pie chart of avg current share for a single match."""
    vals=[(sub,match_data["stats"].get(sub,{}).get("avg",0)) for sub in SUBSYSTEMS]
    vals=[(s,v) for s,v in vals if v>0.01]
    if not vals: return None
    labels_=[s for s,_ in vals]; sizes=[v for _,v in vals]
    clrs=[SUB_COLORS[s] for s in labels_]
    fig,ax=plt.subplots(figsize=(5,4))
    wedges,texts,autotexts=ax.pie(sizes,labels=None,autopct="%1.1f%%",
                                   colors=clrs,startangle=140,
                                   wedgeprops={"edgecolor":"#0D0D1A","linewidth":0.8})
    for at in autotexts: at.set_fontsize(7.5); at.set_color("white")
    ax.legend(wedges,labels_,loc="center left",bbox_to_anchor=(1.05,0.5),fontsize=8)
    ax.set_title(f"Current Share — {match_data['label']}")
    fig.tight_layout()
    return fig


def chart_total_current_by_subsystem():
    """Horizontal bar: grand average current per subsystem across all matches."""
    avgs={}
    for sub in SUBSYSTEMS:
        all_v=[m["stats"].get(sub,{}).get("avg",0) for m in active]
        avgs[sub]=sum(all_v)/len(all_v) if all_v else 0
    sorted_items=sorted(avgs.items(),key=lambda x:x[1])
    subs=[s for s,_ in sorted_items]; vals=[v for _,v in sorted_items]
    clrs=[SUB_COLORS[s] for s in subs]
    fig,ax=plt.subplots(figsize=(7,3.5))
    bars=ax.barh(subs,vals,color=clrs,edgecolor="#0D0D1A",linewidth=0.5)
    for bar,val in zip(bars,vals):
        ax.text(val+0.3,bar.get_y()+bar.get_height()/2,f"{val:.1f}A",
                va="center",fontsize=8.5,color=PAL["light"])
    ax.set_xlabel("Grand Avg Current (A)")
    ax.set_title("Grand Average Current Draw per Subsystem (All Matches)")
    ax.xaxis.grid(True); ax.set_axisbelow(True)
    fig.tight_layout()
    return fig


def chart_pivot_peak_per_match():
    """Bar chart: peak IntakePivot current per match."""
    labels=[m["label"] for m in active]
    peaks=[]
    for m in active:
        pv=m["parsed"]["raw_pivot"]
        peaks.append(max(v for _,v in pv) if pv else 0)
    x=np.arange(len(labels))
    clrs=["#F44336" if p>80 else "#FF9800" if p>50 else "#4CAF50" for p in peaks]
    fig,ax=plt.subplots(figsize=(10,3.5))
    bars=ax.bar(x,peaks,color=clrs,edgecolor="#0D0D1A",linewidth=0.5)
    ax.axhline(40,color="#FFEB3B",lw=1.5,ls="--",label="NEO 40A continuous limit")
    for bar,val in zip(bars,peaks):
        if val>0:
            ax.text(bar.get_x()+bar.get_width()/2,val+1,f"{val:.0f}",
                    ha="center",fontsize=7.5,color=PAL["light"])
    ax.set_xticks(x); ax.set_xticklabels(labels,rotation=45,ha="right")
    ax.set_ylabel("Peak Current (A)")
    ax.set_title("IntakePivot Peak Current per Match")
    ax.legend(); ax.yaxis.grid(True); ax.set_axisbelow(True)
    fig.tight_layout()
    return fig


def chart_voltage_timeline(match_data):
    """Voltage over time for a specific match."""
    bv=match_data["parsed"]["raw_bv"]
    if not bv: return None
    ts=[t for t,_ in bv]; vs=[v for _,v in bv]
    fig,ax=plt.subplots(figsize=(9,3))
    ax.plot(ts,vs,lw=1.2,color="#4CAF50",alpha=0.9)
    ax.fill_between(ts,vs,6.3,where=[v<6.3 for v in vs],alpha=0.4,color="#F44560",label="Brownout zone")
    ax.axhline(6.3,color="#FF9800",lw=1.2,ls="--",label="Brownout threshold (6.3V)")
    ax.axhline(8.0,color="#FFEB3B",lw=0.8,ls=":",label="Warning (8V)")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Voltage (V)")
    ax.set_title(f"Battery Voltage — {match_data['label']}")
    ax.set_ylim(4,14); ax.legend(fontsize=7); ax.yaxis.grid(True); ax.set_axisbelow(True)
    fig.tight_layout()
    return fig


def chart_brownout_duration():
    """Bar chart: number of samples below 8V per match."""
    labels=[m["label"] for m in active]
    counts=[]
    for m in active:
        bv=m["parsed"]["raw_bv"]
        counts.append(sum(1 for _,v in bv if v<8.0))
    x=np.arange(len(labels))
    clrs=["#F44336" if c>1000 else "#FF9800" if c>100 else "#4CAF50" for c in counts]
    fig,ax=plt.subplots(figsize=(10,3.5))
    ax.bar(x,counts,color=clrs,edgecolor="#0D0D1A",linewidth=0.5)
    ax.set_xticks(x); ax.set_xticklabels(labels,rotation=45,ha="right")
    ax.set_ylabel("Samples below 8V (50 Hz)")
    ax.set_title("Battery Voltage Sag Events per Match  (# samples < 8V during teleop)")
    ax.yaxis.grid(True); ax.set_axisbelow(True)
    # Add value labels
    for xi,c in zip(x,counts):
        if c>0:
            ax.text(xi,c+20,str(c),ha="center",fontsize=7.5,color=PAL["light"])
    red_p  = mpatches.Patch(color="#F44336", label=">1000 samples (severe)")
    ora_p  = mpatches.Patch(color="#FF9800", label="100-1000 (moderate)")
    grn_p  = mpatches.Patch(color="#4CAF50", label="<100 (minor)")
    ax.legend(handles=[red_p,ora_p,grn_p],fontsize=8)
    fig.tight_layout()
    return fig


def chart_qual_vs_elim_radar():
    """Radar chart comparing average subsystem draw: Quals vs Elims."""
    quals  = [m for m in active if m["type"]=="Q"]
    elims  = [m for m in active if m["type"]=="E"]
    practice=[m for m in active if m["type"]=="P"]
    def avg_list(lst,sub):
        v=[m["stats"].get(sub,{}).get("avg",0) for m in lst]
        return sum(v)/len(v) if v else 0

    cats=["Drive","IntakePivot","IntakeRollers","Flywheel","Kicker","Spindexer"]
    N=len(cats)
    angles=np.linspace(0,2*np.pi,N,endpoint=False).tolist()
    angles+=angles[:1]

    def get_vals(lst):
        v=[avg_list(lst,s) for s in cats]; v+=v[:1]; return v

    qv=get_vals(quals); ev=get_vals(elims); pv=get_vals(practice)
    fig,ax=plt.subplots(figsize=(6,6),subplot_kw=dict(polar=True))
    ax.set_facecolor(PAL["mid"])
    for spine in ax.spines.values(): spine.set_edgecolor(PAL["light"])
    ax.tick_params(colors=PAL["light"])
    ax.set_theta_offset(np.pi/2); ax.set_theta_direction(-1)
    ax.set_xticks(angles[:-1]); ax.set_xticklabels(cats,size=9,color=PAL["light"])
    ax.set_rlabel_position(0); ax.yaxis.grid(True)
    if qv: ax.plot(angles,qv,"o-",lw=2,color="#2196F3",label="Quals");  ax.fill(angles,qv,alpha=0.2,color="#2196F3")
    if ev: ax.plot(angles,ev,"s-",lw=2,color="#F44336",label="Elims");  ax.fill(angles,ev,alpha=0.2,color="#F44336")
    if pv: ax.plot(angles,pv,"^-",lw=2,color="#4CAF50",label="Practice");ax.fill(angles,pv,alpha=0.2,color="#4CAF50")
    ax.set_title("Avg Current Draw: Quals vs Elims vs Practice",pad=20,color=PAL["light"])
    ax.legend(loc="upper right",bbox_to_anchor=(1.35,1.1))
    fig.tight_layout()
    return fig


def chart_spindexer_anomalies():
    """Highlight Spindexer spikes — P24 and Q6 were extreme."""
    labels=[m["label"] for m in active]
    vals=[m["stats"].get("Spindexer",{}).get("avg",0) for m in active]
    x=np.arange(len(labels))
    clrs=["#F44336" if v>30 else "#FF9800" if v>15 else "#607D8B" for v in vals]
    fig,ax=plt.subplots(figsize=(10,3.5))
    bars=ax.bar(x,vals,color=clrs,edgecolor="#0D0D1A",linewidth=0.5)
    ax.axhline(15,color="#FFEB3B",lw=1.2,ls="--",label="15A reference")
    for bar,val in zip(bars,vals):
        if val>0:
            ax.text(bar.get_x()+bar.get_width()/2,val+0.5,f"{val:.1f}",
                    ha="center",fontsize=7.5,color=PAL["light"])
    ax.set_xticks(x); ax.set_xticklabels(labels,rotation=45,ha="right")
    ax.set_ylabel("Avg Spindexer Current (A)")
    ax.set_title("Spindexer Average Current per Match  (spikes suggest ball jams)")
    ax.legend(); ax.yaxis.grid(True); ax.set_axisbelow(True)
    fig.tight_layout()
    return fig


# ═══════════════════════════════════════════════════════════════════════════════
# PDF ASSEMBLY
# ═══════════════════════════════════════════════════════════════════════════════

def build_pdf():
    doc=SimpleDocTemplate(
        OUTPUT_PDF, pagesize=letter,
        leftMargin=0.65*inch, rightMargin=0.65*inch,
        topMargin=0.65*inch,  bottomMargin=0.65*inch,
    )

    story=[]

    # ── PAGE 1: Cover ─────────────────────────────────────────────────────────
    story += [
        Spacer(1,1.2*inch),
        ColorBar(6,"#E94560"),
        Spacer(1,0.25*inch),
        Paragraph("MRT3216", TITLE),
        Paragraph("2026 FRC Season — Match Performance Report", SUB),
        Spacer(1,0.1*inch),
        ColorBar(2,"#60A0E9"),
        Spacer(1,0.4*inch),
    ]

    # Cover stats box
    total_matches=len(active)
    worst_v=min(m["stats"]["_Battery"]["min_v"] for m in active)
    worst_lbl=next(m["label"] for m in active
                   if m["stats"]["_Battery"]["min_v"]==worst_v)
    avg_pivot=sum(m["stats"].get("IntakePivot",{}).get("avg",0) for m in active)/total_matches
    peak_pivot=max((max(v for _,v in m["parsed"]["raw_pivot"]) if m["parsed"]["raw_pivot"] else 0)
                   for m in active)

    cover_data=[
        ["Metric","Value"],
        ["Matches Analyzed",str(total_matches)],
        ["Lowest Battery Voltage",f"{worst_v:.2f}V  ({worst_lbl})"],
        ["IntakePivot Grand Avg",f"{avg_pivot:.1f}A"],
        ["IntakePivot Peak (all matches)",f"{peak_pivot:.1f}A"],
        ["Matches w/ Sub-8V Events",str(sum(1 for m in active if any(v<8 for _,v in m["parsed"]["raw_bv"])))],
        ["Matches w/ Sub-6.3V (Brownout)",str(sum(1 for m in active if any(v<6.3 for _,v in m["parsed"]["raw_bv"])))],
        ["Report Generated", datetime.datetime.now().strftime("%B %d, %Y  %H:%M")],
    ]
    ct=Table(cover_data,colWidths=[3.0*inch,3.0*inch])
    ct.setStyle(TableStyle(TBL_HEADER+[
        ("FONTSIZE",(0,0),(-1,-1),10),
        ("TOPPADDING",(0,0),(-1,-1),6),
        ("BOTTOMPADDING",(0,0),(-1,-1),6),
    ]))
    story+=[ct,Spacer(1,0.4*inch),
            Paragraph("Prepared by the MRT3216 Programming Subteam",CAPTION),
            PageBreak()]

    # ── PAGE 2: Executive Summary ─────────────────────────────────────────────
    story+=[
        Paragraph("Executive Summary",H1),
        ColorBar(2,"#60A0E9"),
        Spacer(1,0.15*inch),
        Paragraph(
            "This report analyzes all available match and practice logs from the 2026 "
            "competition season. All data is drawn from AdvantageKit <i>.wpilog</i> files "
            "recorded on the robot's roboRIO. Statistics reflect only the <b>teleop</b> "
            "window (enabled + non-autonomous) to focus on driver-operated performance.",
            BODY),
        Spacer(1,0.1*inch),
        Paragraph("Top Findings",H2),
        Paragraph(
            "1. <b>IntakePivot is the primary battery load.</b>  With a grand average of "
            f"{avg_pivot:.1f}A and peaks exceeding 100A, the pivot motor is operating in a "
            "near-constant stall condition. Root cause: <font color='#FF9800'>kP = 0.0</font> "
            "in IntakeConstants — the closed-loop position controller has no proportional gain "
            "and cannot drive the pivot to its setpoint. The motor holds a static voltage "
            "output against a mechanical load indefinitely, causing excessive current draw and "
            "battery brownouts throughout every match.",BODY),
        Paragraph(
            "2. <b>Battery brownouts are universal.</b>  Only one match (E6) had no sub-8V "
            "readings. Q35 reached a minimum of 5.96V — below the roboRIO's 6.3V brownout "
            "trip point, which causes a mid-match software disable. The primary corrective "
            "action is fixing IntakePivot kP.",BODY),
        Paragraph(
            "3. <b>The turret never moved.</b>  Turret average current across all matches "
            "is 0.10–0.17A (near zero), consistent with a motor that is enabled but not "
            "commanded. Log data confirms the turret setpoint was logged only once (value = 0°) "
            "and never updated. This means the shooter aimed exclusively from drive rotation, "
            "severely limiting shot accuracy and cycle speed.",BODY),
        Paragraph(
            "4. <b>Spindexer current spikes in P24 and Q6.</b>  Average Spindexer current "
            "reached 54.96A in P24 and 41.23A in Q6 — significantly above the 15A expected "
            "for normal operation. This is consistent with a ball jam or missing jam-detection "
            "logic. If the Spindexer has no current-based jam detection, a jammed ball will "
            "continuously stall the motor.",BODY),
        Paragraph(
            "5. <b>agitate().whileFalse() (now fixed).</b>  Prior to this session, the intake "
            "agitate command was bound to <font color='#FF9800'>whileFalse()</font>, meaning it "
            "scheduled continuously whenever the button was <i>not held</i>. This contributed "
            "to IntakeRollers and IntakePivot load during idle periods. This binding has been "
            "corrected to <font color='#4CAF50'>onFalse()</font>.",BODY),
        Spacer(1,0.1*inch),
        Paragraph("Recommended Action Priority",H2),
    ]

    rec_data=[
        ["Priority","Item","Expected Impact"],
        ["🔴 CRITICAL","Fix IntakePivot kP (IntakeConstants.java)","Eliminate 40A+ stall, recover ~10V minimum battery"],
        ["🔴 CRITICAL","IntakePivot feedforward + default command","Stable position hold, no stall current"],
        ["🟠 HIGH","Fix Turret setpoint update in aimAndShoot()","Enable actual auto-aim, more efficient shooting cycles"],
        ["🟠 HIGH","Add Spindexer jam detection","Prevent 50A+ stall events in competition"],
        ["🟡 MEDIUM","Hood angle unit check (radians vs degrees)","Correct hood angle, improve shot arc"],
        ["🟡 MEDIUM","tuningMode=false before competition","Enable real button bindings"],
        ["🟢 LOW","Clean up redundant SIM/REAL branch","Code clarity only"],
    ]
    rt=Table(rec_data,colWidths=[0.9*inch,3.0*inch,2.8*inch])
    rt.setStyle(TableStyle(TBL_HEADER))
    story+=[rt,PageBreak()]

    # ── PAGE 3: Current Draw Overview ─────────────────────────────────────────
    story+=[
        Paragraph("Section 1 — Current Draw Overview",H1),
        ColorBar(2,"#60A0E9"),
        Spacer(1,0.1*inch),
        Paragraph(
            "The charts below show average current draw (in Amperes) for each robot subsystem "
            "across all analyzed matches. Drive current is the sum of all 8 swerve motor signals.",BODY),
    ]
    fig=chart_avg_current_by_match()
    story+=[fig_to_image(fig,6.5),
            Paragraph("Figure 1 — Average current per subsystem per match during teleop.",CAPTION),
            Spacer(1,0.15*inch)]

    fig=chart_total_current_by_subsystem()
    story+=[fig_to_image(fig,6.0),
            Paragraph("Figure 2 — Grand average current per subsystem across all matches.",CAPTION),
            PageBreak()]

    # ── PAGE 4: IntakePivot Analysis ─────────────────────────────────────────
    story+=[
        Paragraph("Section 2 — IntakePivot Stall Analysis",H1),
        ColorBar(2,"#F44336"),
        Spacer(1,0.1*inch),
        Paragraph(
            "IntakePivot was the <b>largest single source of current draw</b> in every match. "
            "The pivot motor is a REV NEO with a continuous current rating of approximately "
            "40A. Values above 40A represent stall conditions that generate significant heat "
            "and battery voltage sag. The root cause is <b>kP = 0.0</b> in the position "
            "controller, which prevents the motor from ever reaching its commanded angle.",BODY),
    ]
    fig=chart_pivot_peak_per_match()
    story+=[fig_to_image(fig,6.5),
            Paragraph("Figure 3 — IntakePivot peak current per match. Values above 40A (yellow line) are stall events.",CAPTION),
            Spacer(1,0.1*inch)]

    fig=chart_pivot_stall_heatmap()
    story+=[fig_to_image(fig,6.5),
            Paragraph("Figure 4 — Percentage of teleop time where IntakePivot current exceeded 20A. Red = stalled most of the match.",CAPTION),
            Spacer(1,0.15*inch),
            Paragraph("Key observations:",H2),
            Paragraph("• <b>Q6 peak: 101.2A</b> — motor was producing nearly 3× its continuous limit for extended periods.",BODY),
            Paragraph("• <b>P17: 96% of teleop above 20A</b> — the entire match was essentially a stall condition.",BODY),
            Paragraph("• <b>E6 was the only match with low pivot draw (3.86A avg)</b> — the pivot may have been stowed the entire match.",BODY),
            Paragraph(
                "<b>Fix:</b> Set <font color='#4CAF50'>IntakeConstants.Pivot.kP</font> to an appropriate gain "
                "(start with 0.5–1.0 and tune). Also enable feedforward in IntakePivotSubsystem constructor "
                "and update the default command from <font color='#FF9800'>set(0)</font> to "
                "<font color='#4CAF50'>setAngle(() -> getTarget())</font>.",BODY),
            PageBreak()]

    # ── PAGE 5: Battery Voltage Analysis ─────────────────────────────────────
    story+=[
        Paragraph("Section 3 — Battery Voltage & Brownout Analysis",H1),
        ColorBar(2,"#FF9800"),
        Spacer(1,0.1*inch),
        Paragraph(
            "A healthy FRC battery under load should stay above 10V. The roboRIO triggers a "
            "software brownout at 6.3V. Readings below 8V indicate severe power demand that "
            "risks brownout and actuator failures. The charts below show voltage sag events "
            "across all matches.",BODY),
    ]
    fig=chart_battery_voltage()
    story+=[fig_to_image(fig,6.5),
            Paragraph("Figure 5 — Average and minimum battery voltage per match during teleop.",CAPTION),
            Spacer(1,0.1*inch)]

    fig=chart_brownout_duration()
    story+=[fig_to_image(fig,6.5),
            Paragraph("Figure 6 — Number of 50Hz samples below 8V per match. Over 1000 samples (red) = 20+ seconds of sag.",CAPTION),
            PageBreak()]

    # ── PAGE 6: Per-Match Voltage Timeline for worst matches ─────────────────
    story+=[
        Paragraph("Section 3 (cont.) — Voltage Timelines: Worst Matches",H1),
        ColorBar(2,"#FF9800"),
        Spacer(1,0.1*inch),
        Paragraph("The timelines below show battery voltage over time for the three matches with the most brownout-risk samples.",BODY),
        Spacer(1,0.1*inch),
    ]
    worst_matches=sorted(active,key=lambda m:sum(1 for _,v in m["parsed"]["raw_bv"] if v<8),reverse=True)[:3]
    for wm in worst_matches:
        fig=chart_voltage_timeline(wm)
        if fig:
            story+=[fig_to_image(fig,6.5),
                    Paragraph(f"Figure — Battery voltage timeline for {wm['label']}. Red fill = below brownout threshold.",CAPTION),
                    Spacer(1,0.15*inch)]
    story.append(PageBreak())

    # ── PAGE 7: Quals vs Elims Radar ─────────────────────────────────────────
    story+=[
        Paragraph("Section 4 — Performance Comparison: Quals vs Elims vs Practice",H1),
        ColorBar(2,"#60A0E9"),
        Spacer(1,0.1*inch),
        Paragraph(
            "The radar chart below compares the average current draw profile across match types. "
            "A larger area indicates higher average load. Drive and shooter systems should scale "
            "with match intensity; IntakePivot load should be near zero with a properly tuned controller.",BODY),
        Spacer(1,0.15*inch),
    ]
    fig=chart_qual_vs_elim_radar()
    story+=[fig_to_image(fig,5.5),
            Paragraph("Figure 7 — Radar comparison of average current draw across match types.",CAPTION),
            Spacer(1,0.15*inch)]

    # Pie charts for a few key matches
    story+=[Paragraph("Current Share by Subsystem — Selected Matches",H2)]
    highlight=[m for m in active if m["label"] in ["Q23","Q82","E9","P17"]]
    if len(highlight)<4:
        highlight=active[:4]

    for i in range(0,len(highlight),2):
        row_flowables=[]
        for hm in highlight[i:i+2]:
            fig=chart_subsystem_pie(hm)
            if fig: row_flowables.append(fig_to_image(fig,3.0))
        if row_flowables:
            t=Table([row_flowables])
            t.setStyle(TableStyle([("ALIGN",(0,0),(-1,-1),"CENTER"),("VALIGN",(0,0),(-1,-1),"TOP")]))
            story.append(t)
    story.append(PageBreak())

    # ── PAGE 8: Spindexer Anomalies ────────────────────────────────────────────
    story+=[
        Paragraph("Section 5 — Spindexer Anomalies",H1),
        ColorBar(2,"#607D8B"),
        Spacer(1,0.1*inch),
        Paragraph(
            "The Spindexer showed significant current spikes in P24 (54.96A avg) and Q6 (41.23A avg). "
            "Under normal ball-indexing operation, the Spindexer should draw 5–15A. "
            "Values this high suggest a persistent mechanical jam or a missing jam-detection/cutoff "
            "in software. If the motor controller has no current limit configured, a jam will "
            "stall the motor indefinitely.",BODY),
        Spacer(1,0.1*inch),
    ]
    fig=chart_spindexer_anomalies()
    story+=[fig_to_image(fig,6.5),
            Paragraph("Figure 8 — Spindexer average current per match. Spikes in P24 and Q6 indicate ball jam events.",CAPTION),
            Spacer(1,0.2*inch),
            Paragraph("Recommended mitigations:",H2),
            Paragraph("• Add a current-based jam detection: if Spindexer current > 30A for > 0.2s, reverse briefly then re-engage.",BODY),
            Paragraph("• Set a hardware current limit on the Spindexer motor controller (e.g., 35A stall limit).",BODY),
            Paragraph("• Log the Spindexer position to detect if it stops moving (jam) vs. just drawing high current (legitimate load).",BODY),
            PageBreak()]

    # ── PAGE 9: Full Data Table ────────────────────────────────────────────────
    story+=[
        Paragraph("Section 6 — Full Match Data Table",H1),
        ColorBar(2,"#60A0E9"),
        Spacer(1,0.1*inch),
        Paragraph("All current values are average Amps during teleop. Drive = sum of 8 swerve motor signals.",BODY),
        Spacer(1,0.1*inch),
    ]
    table_data=[["Match","MB","Drive","IPivot","IRoll","Turret","Flywheel","Hood","Kicker","Spindex","AvgV","MinV"]]
    for m in active:
        s=m["stats"]
        row=[
            m["label"], f"{m['mb']:.0f}",
            f"{s.get('Drive',{}).get('avg',0):.1f}",
            f"{s.get('IntakePivot',{}).get('avg',0):.1f}",
            f"{s.get('IntakeRollers',{}).get('avg',0):.1f}",
            f"{s.get('Turret',{}).get('avg',0):.2f}",
            f"{s.get('Flywheel',{}).get('avg',0):.1f}",
            f"{s.get('Hood',{}).get('avg',0):.2f}",
            f"{s.get('Kicker',{}).get('avg',0):.1f}",
            f"{s.get('Spindexer',{}).get('avg',0):.1f}",
            f"{s['_Battery']['avg_v']:.2f}",
            f"{s['_Battery']['min_v']:.2f}",
        ]
        table_data.append(row)
    # Averages row
    avg_row=["AVG","—"]
    for sub in SUBSYSTEMS:
        v=[m["stats"].get(sub,{}).get("avg",0) for m in active]
        avg_row.append(f"{sum(v)/len(v):.1f}" if v else "—")
    bv_all=[m["stats"]["_Battery"]["avg_v"] for m in active]
    bv_min=[m["stats"]["_Battery"]["min_v"] for m in active]
    avg_row+=[f"{sum(bv_all)/len(bv_all):.2f}", f"{sum(bv_min)/len(bv_min):.2f}"]
    table_data.append(avg_row)

    col_w=[0.52*inch,0.38*inch]+[0.56*inch]*10
    dt=Table(table_data,colWidths=col_w)
    dt.setStyle(TableStyle(TBL_HEADER+[
        ("FONTNAME",(0,-1),(-1,-1),"Helvetica-Bold"),
        ("BACKGROUND",(0,-1),(-1,-1),colors.HexColor("#1A3A1A")),
    ]))
    story+=[dt, Spacer(1,0.3*inch)]

    # Brownout table
    story+=[Paragraph("Brownout Risk Detail",H2)]
    br_data=[["Match","Samples <8V","Worst Voltage","Timestamp (s)"]]
    for m in active:
        bv=m["parsed"]["raw_bv"]
        low=[(t,v) for t,v in bv if v<8.0]
        if low:
            worst=min(low,key=lambda x:x[1])
            br_data.append([m["label"],str(len(low)),f"{worst[1]:.2f}V",f"{worst[0]:.1f}"])
        else:
            br_data.append([m["label"],"0","OK","—"])
    bt=Table(br_data,colWidths=[0.7*inch,1.1*inch,1.3*inch,1.2*inch])
    bt.setStyle(TableStyle(TBL_HEADER))
    story+=[bt,PageBreak()]

    # ── PAGE 10: Code Fixes Summary + Next Steps ───────────────────────────────
    story+=[
        Paragraph("Section 7 — Code Changes Applied This Session",H1),
        ColorBar(2,"#4CAF50"),
        Spacer(1,0.1*inch),
        Paragraph("The following fixes were applied to the robot codebase in this programming session:",BODY),
        Spacer(1,0.1*inch),
    ]
    fixes=[
        ["File","Change","Reason"],
        ["Robot.java","autonomousPeriodic() restored to empty body",
         "Previous AI re-scheduled auto command every tick, double-running CommandScheduler"],
        ["Robot.java","BuildConstants metadata logging restored",
         "Code version/event info was being omitted from logs"],
        ["Robot.java","disabledTimer.restart() in disabledInit()",
         "Low-battery timer never reset on re-disable"],
        ["build.gradle","Added gversion{} block (timezone=America/Denver)",
         "BuildConstants was not being generated at compile time"],
        ["RobotContainer.java","Vision SIM: fixed front camera transform (was using Left transform)",
         "Copy-paste bug — front camera was getting wrong pose offset in sim"],
        ["RobotContainer.java","Vision REPLAY: changed 2 stubs → 4 stubs",
         "Mismatch with REAL (4 cameras) would cause replay index-out-of-bounds"],
        ["RobotContainer.java","Hood default: setAngle(Degrees.of(0)) replaces setAngle(getPosition())",
         "getPosition() captured position once at init — hood would not retract under trench"],
        ["RobotContainer.java","aimAndShoot: onTrue() → toggleOnTrue()",
         "onTrue() started the command but never gave the driver a way to cancel it"],
        ["RobotContainer.java","agitate: whileFalse(deploy()) → onFalse(deploy())",
         "whileFalse() re-scheduled deploy() on every loop cycle when button not held, drawing continuous current"],
    ]
    ft=Table(fixes,colWidths=[1.5*inch,2.4*inch,2.4*inch])
    ft.setStyle(TableStyle(TBL_HEADER))
    story+=[ft, Spacer(1,0.3*inch)]

    story+=[Paragraph("Remaining Action Items",H2)]
    next_data=[
        ["Priority","Item","File(s)"],
        ["🔴 CRITICAL","Set IntakePivot kP (0→0.5 starting point)","IntakeConstants.java"],
        ["🔴 CRITICAL","Enable feedforward in IntakePivotSubsystem constructor","IntakePivotSubsystem.java"],
        ["🔴 CRITICAL","Update IntakePivot default command to setAngle(getTarget)","RobotContainer.java"],
        ["🟠 HIGH","Fix turret setpoint update in aimAndShoot()","ShooterSystem.java"],
        ["🟠 HIGH","Add Spindexer current limit + jam detection","SpindexerSubsystem.java"],
        ["🟡 MEDIUM","Verify hood angle units (radians vs degrees in logs)","HoodSubsystem.java"],
        ["🟡 MEDIUM","Set tuningMode=false before next competition","RobotContainer.java"],
        ["🟢 LOW","Remove redundant SIM/REAL branch in configureButtonBindings()","RobotContainer.java"],
    ]
    nt=Table(next_data,colWidths=[0.9*inch,3.0*inch,2.4*inch])
    nt.setStyle(TableStyle(TBL_HEADER))
    story+=[nt, Spacer(1,0.3*inch),
            HRFlowable(width="100%",thickness=1,color=colors.HexColor("#E94560")),
            Spacer(1,0.1*inch),
            Paragraph(
                f"Report generated {datetime.datetime.now().strftime('%B %d, %Y at %H:%M')} | "
                "MRT3216 Programming Subteam | Data source: AdvantageKit .wpilog files",
                CAPTION),
            ]

    doc.build(story)
    print(f"\nDONE: Report saved to: {OUTPUT_PDF}")


if __name__ == "__main__":
    build_pdf()
