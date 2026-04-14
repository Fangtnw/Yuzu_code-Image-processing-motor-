from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import mm
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    HRFlowable, PageBreak, KeepTogether
)
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_RIGHT

OUTPUT = "DR28T_AZD-KEP_Manual.pdf"

# ── Colour palette ────────────────────────────────────────────
DARK   = colors.HexColor("#1a1a2e")
ACCENT = colors.HexColor("#0f3460")
BLUE   = colors.HexColor("#2196f3")
GREEN  = colors.HexColor("#4caf50")
ORANGE = colors.HexColor("#ff9800")
RED    = colors.HexColor("#f44336")
LGREY  = colors.HexColor("#f5f5f5")
MGREY  = colors.HexColor("#e0e0e0")
WHITE  = colors.white

W, H = A4

# ── Styles ────────────────────────────────────────────────────
base = getSampleStyleSheet()

def S(name, parent="Normal", **kw):
    return ParagraphStyle(name, parent=base[parent], **kw)

sTitle   = S("sTitle",  "Title",   fontSize=26, textColor=WHITE,  alignment=TA_CENTER, spaceAfter=4)
sSub     = S("sSub",    "Normal",  fontSize=13, textColor=MGREY,  alignment=TA_CENTER, spaceAfter=2)
sH1      = S("sH1",     "Heading1",fontSize=14, textColor=WHITE,  spaceBefore=14, spaceAfter=6,
             backColor=ACCENT, borderPad=6)
sH2      = S("sH2",     "Heading2",fontSize=11, textColor=ACCENT, spaceBefore=10, spaceAfter=4)
sBody    = S("sBody",   "Normal",  fontSize=9,  leading=14,       spaceAfter=4)
sNote    = S("sNote",   "Normal",  fontSize=8,  textColor=colors.HexColor("#555555"),
             leftIndent=10, spaceAfter=4)
sCode    = S("sCode",   "Code",    fontSize=8,  backColor=LGREY,  leftIndent=10, rightIndent=10,
             borderPad=6, leading=12)
sWarn    = S("sWarn",   "Normal",  fontSize=9,  textColor=RED,    leftIndent=8, spaceAfter=4)
sTH      = S("sTH",     "Normal",  fontSize=9,  textColor=WHITE,  alignment=TA_CENTER)
sTD      = S("sTD",     "Normal",  fontSize=9,  alignment=TA_LEFT)
sTDc     = S("sTDc",    "Normal",  fontSize=9,  alignment=TA_CENTER)
sMono    = S("sMono",   "Code",    fontSize=8,  alignment=TA_LEFT, leading=11)

def th(t):  return Paragraph(t, sTH)
def td(t):  return Paragraph(str(t), sTD)
def tdc(t): return Paragraph(str(t), sTDc)
def mono(t):return Paragraph(t, sMono)

TSTYLE = [
    ("BACKGROUND",   (0,0), (-1,0),  ACCENT),
    ("ROWBACKGROUNDS",(0,1),(-1,-1), [WHITE, LGREY]),
    ("GRID",         (0,0), (-1,-1), 0.4, MGREY),
    ("TOPPADDING",   (0,0), (-1,-1), 5),
    ("BOTTOMPADDING",(0,0), (-1,-1), 5),
    ("LEFTPADDING",  (0,0), (-1,-1), 7),
    ("RIGHTPADDING", (0,0), (-1,-1), 7),
    ("VALIGN",       (0,0), (-1,-1), "MIDDLE"),
]

def make_table(data, col_widths, extra=None):
    style = list(TSTYLE) + (extra or [])
    return Table(data, colWidths=col_widths, style=TableStyle(style))


# ══════════════════════════════════════════════════════════════
# COVER
# ══════════════════════════════════════════════════════════════
def cover():
    elems = [Spacer(1, 28*mm)]
    banner = Table(
        [[Paragraph("MOTOR CONTROL SYSTEM", sTitle)],
         [Paragraph("Quick Reference Manual  —  Rev 2.0", sSub)],
         [Spacer(1, 3*mm)],
         [Paragraph("DR28T2.5B03-AZAKD  ×  AZD-KEP  |  ROS2 Humble", sSub)]],
        colWidths=[160*mm],
        style=TableStyle([
            ("BACKGROUND",   (0,0),(-1,-1), DARK),
            ("TOPPADDING",   (0,0),(-1,-1), 8),
            ("BOTTOMPADDING",(0,0),(-1,-1), 8),
            ("LEFTPADDING",  (0,0),(-1,-1), 14),
            ("RIGHTPADDING", (0,0),(-1,-1), 14),
        ])
    )
    elems.append(banner)
    elems.append(Spacer(1, 12*mm))
    elems.append(make_table(
        [[th("Rev"), th("Platform"), th("Interfaces"), th("Date")],
         [tdc("2.0"), tdc("Ubuntu 22.04 + ROS2 Humble"),
          tdc("USB Serial  &  EtherNet/IP"), tdc("April 2026")]],
        [22*mm, 60*mm, 55*mm, 30*mm]
    ))
    elems.append(Spacer(1, 8*mm))
    elems.append(Paragraph(
        "This manual documents the 5-DOF Yuzu Peeler motor system: hardware specs, "
        "both communication interfaces (USB serial legacy and EtherNet/IP production), "
        "all ROS2 nodes, launch files, GUI, and the harvest sequence orchestrator.",
        sBody))
    elems.append(Spacer(1, 6*mm))

    # System overview table
    elems.append(Paragraph("System Overview — 5 Axes", sH2))
    elems.append(make_table(
        [[th("Namespace"), th("Axis"), th("Motor"), th("Driver / Node")],
         [td("motor_z"),             td("Z linear (peeler head)"),  td("DR28T"), td("ethernet_ip_motor_node  or  motor_controller_node")],
         [td("motor_yuzu_rot"),      td("Yuzu rotation spindle"),   td("Rotary"),td("rotary_motor_stub_node  (stub)")],
         [td("motor_peeler_orbit"),  td("Peeler arm orbit"),        td("Rotary"),td("rotary_motor_stub_node  (stub)")],
         [td("motor_peeler_advance"),td("Peeler radial advance"),   td("DR28T"), td("ethernet_ip_motor_node  or  stub")],
         [td("motor_conveyor"),      td("Conveyor belt"),           td("Stepper"),td("rotary_motor_stub_node  (stub)")]],
        [42*mm, 45*mm, 22*mm, 60*mm]
    ))
    return elems


# ══════════════════════════════════════════════════════════════
# SECTION 1 – HARDWARE
# ══════════════════════════════════════════════════════════════
def section_hw():
    elems = [Paragraph("1  Hardware Specifications", sH1)]

    elems.append(Paragraph("1.1  Motor — DR28T2.5B03-AZAKD", sH2))
    elems.append(make_table(
        [[th("Parameter"),       th("Value"),                           th("Notes")],
         [td("Type"),            tdc("Compact Electric Cylinder"),      td("Ball screw, closed-loop stepper")],
         [td("Stroke"),          tdc("30 mm"),                          td("Software limit set to 29 mm")],
         [td("Lead / Pitch"),    tdc("2.5 mm / rev"),                   td("Used for steps_per_mm calculation")],
         [td("Max Speed"),       tdc("100 mm/s"),                       td("Hard cap in node parameters")],
         [td("Max Thrust"),      tdc("20 N"),                           td("")],
         [td("Repeat Accuracy"), tdc("±0.003 mm"),                      td("Ball screw shaft tip")],
         [td("Encoder"),         tdc("Absolute multi-turn"),            td("No battery required")],
         [td("Supply Voltage"),  tdc("24 / 48 VDC"),                    td("Driver dependent")]],
        [48*mm, 55*mm, 60*mm]
    ))
    elems.append(Spacer(1, 5*mm))

    elems.append(Paragraph("1.2  Driver — AZD-KEP", sH2))
    elems.append(make_table(
        [[th("Parameter"),       th("Value")],
         [td("Series"),          tdc("AZ Series Closed-Loop Stepper Driver")],
         [td("Fieldbus"),        tdc("EtherNet/IP™  (CT16 conformant, Vendor ID 187)")],
         [td("Network Speed"),   tdc("10/100 Mbps (autonegotiation)")],
         [td("USB Port"),        tdc("Mini-B  →  MEXE02 config tool (Windows only)")],
         [td("Control Power"),   tdc("24 VDC ±5%,  0.25 A")],
         [td("IP Address"),      tdc("192.168.1.x  (set via rotary switches on face)")],
         [td("EDS File"),        tdc("Download from orientalmotor.com")]],
        [60*mm, 105*mm]
    ))
    elems.append(Spacer(1, 3*mm))
    elems.append(Paragraph(
        "IP address rotary switches: IP ADDR ×16 and ×1 on the driver face. "
        "IP = 192.168.1.(16 × x16 + x1).  e.g. ×16=0, ×1=2 → 192.168.1.2.  "
        "Power-cycle after changing switches.", sNote))
    return elems


# ══════════════════════════════════════════════════════════════
# SECTION 2 – COMMUNICATION
# ══════════════════════════════════════════════════════════════
def section_comm():
    elems = [Paragraph("2  Communication Interfaces", sH1)]

    # ── 2.1 USB ──
    elems.append(Paragraph("2.1  USB Serial (Legacy — motor_controller_node.py)", sH2))
    elems.append(Paragraph(
        "Motor is controlled by replaying binary MEXE02 protocol frames captured "
        "on Windows, sent over the USB Virtual COM Port (Linux: /dev/ttyACM0).", sBody))
    elems.append(make_table(
        [[th("Setting"),    th("Value")],
         [td("Baud rate"),  tdc("19 200")],
         [td("Data bits"),  tdc("8")],
         [td("Parity"),     tdc("Even")],
         [td("Stop bits"),  tdc("1")],
         [td("Timeout"),    tdc("0.5 s")]],
        [60*mm, 105*mm]
    ))
    elems.append(Spacer(1, 3*mm))
    elems.append(Paragraph(
        "Position encoding: mm × 1000 → µm (int32 LE).  "
        "Speed encoding: mm/s × 1000 → µm/s (int32 LE).  "
        "XOR checksum patched per frame.  Template HTML files required in working dir.", sNote))
    elems.append(Spacer(1, 5*mm))

    # ── 2.2 EtherNet/IP ──
    elems.append(Paragraph("2.2  EtherNet/IP (Production — ethernet_ip_motor_node.py)", sH2))
    elems.append(Paragraph(
        "CIP explicit messaging via pycomm3.  "
        "Get/Set Attribute Single on the Assembly Object (class 0x04).  "
        "No implicit (UDP) connection required — each command is a separate TCP request.", sBody))
    elems.append(make_table(
        [[th("Direction"),            th("Instance"),    th("Attr"), th("Size"), th("Purpose")],
         [td("Scanner → Driver (cmd)"), tdc("101 (0x65)"), tdc("3"),  tdc("40 B"), td("Set Attribute Single — write motion command")],
         [td("Driver → Scanner (stat)"),tdc("100 (0x64)"), tdc("3"),  tdc("56 B"), td("Get Attribute Single — read status")]],
        [48*mm, 28*mm, 18*mm, 18*mm, 54*mm]
    ))
    elems.append(Spacer(1, 5*mm))

    # Output assembly detail
    elems.append(Paragraph("Output Assembly — 40 bytes (scanner → driver)", sH2))
    elems.append(make_table(
        [[th("Bytes"), th("Type"), th("Field"), th("Notes")],
         [tdc("0–1"),  tdc("H"),  td("Remote I/O (R-IN)"),              td("Set to 0 (unused)")],
         [tdc("2–3"),  tdc("H"),  td("Operation data number selection"), td("Set to 0")],
         [tdc("4–5"),  tdc("H"),  td("Fixed I/O (IN)  — control bits"), td("See bit table below")],
         [tdc("6–7"),  tdc("H"),  td("Direct data op type"),            td("1 = absolute positioning")],
         [tdc("8–11"), tdc("i"),  td("Direct data op position"),        td("steps  (signed int32)")],
         [tdc("12–15"),tdc("i"),  td("Direct data op speed"),           td("Hz = steps/s  (signed int32)")],
         [tdc("16–19"),tdc("i"),  td("Starting / changing rate"),       td("1 = 0.001 kHz/s  (default 1 000 000)")],
         [tdc("20–23"),tdc("i"),  td("Stopping deceleration"),          td("same unit  (default 1 000 000)")],
         [tdc("24–25"),tdc("H"),  td("Operating current"),              td("1 = 0.1%,  1000 = 100%")],
         [tdc("26–27"),tdc("H"),  td("Forwarding destination"),         td("0 = execution memory")],
         [tdc("28–35"),tdc("4H"), td("Reserved / parameter R/W area"), td("Set to 0 for motion commands")],
         [tdc("36–39"),tdc("i"),  td("Write data"),                     td("Set to 0 for motion commands")]],
        [18*mm, 14*mm, 58*mm, 74*mm]
    ))
    elems.append(Spacer(1, 3*mm))
    elems.append(Paragraph(
        "Python struct: struct.Struct(\"&lt;HHHH iiii HHHHHH i\")  \u2192  40 bytes", sCode))
    elems.append(Spacer(1, 3*mm))

    # Fixed I/O IN bits
    elems.append(Paragraph("Fixed I/O (IN) bit definitions  (output bytes 4–5):", sH2))
    elems.append(make_table(
        [[th("Bit"), th("Mask"), th("Name"),    th("Description")],
         [tdc("3"),  tdc("0x0008"), td("START"),    td("Execute stored-data operation")],
         [tdc("4"),  tdc("0x0010"), td("ZHOME"),    td("High-speed return-to-home")],
         [tdc("5"),  tdc("0x0020"), td("STOP"),     td("Deceleration stop")],
         [tdc("6"),  tdc("0x0040"), td("FREE"),     td("De-energise motor coils")],
         [tdc("7"),  tdc("0x0080"), td("ALM-RST"),  td("Reset active alarm  (pulse ON→OFF)")],
         [tdc("8"),  tdc("0x0100"), td("TRIG"),     td("Trigger direct data operation  (rising edge)")],
         [tdc("9"),  tdc("0x0200"), td("TRIG-MODE"),td("0 = ON edge (default),  1 = ON level")]],
        [14*mm, 22*mm, 28*mm, 100*mm]
    ))
    elems.append(Spacer(1, 5*mm))

    # Input assembly detail
    elems.append(Paragraph("Input Assembly — 56 bytes (driver → scanner)", sH2))
    elems.append(make_table(
        [[th("Bytes"), th("Type"), th("Field"), th("Notes")],
         [tdc("0–1"),  tdc("H"),  td("Remote I/O (R-OUT)"),             td("")],
         [tdc("2–3"),  tdc("H"),  td("Op data number selection_R"),     td("")],
         [tdc("4–5"),  tdc("H"),  td("Fixed I/O (OUT) — status bits"),  td("See bit table below")],
         [tdc("6–7"),  tdc("H"),  td("Present alarm code"),             td("0 = no alarm")],
         [tdc("8–11"), tdc("i"),  td("Feedback position"),              td("steps  (signed int32)")],
         [tdc("12–15"),tdc("I"),  td("Feedback speed"),                 td("Hz  (unsigned int32)")],
         [tdc("16–19"),tdc("i"),  td("Command position"),               td("steps  (signed int32)")],
         [tdc("20–23"),tdc("2H"), td("Torque monitor / CST current"),   td("")],
         [tdc("24–27"),tdc("I"),  td("Information code"),               td("")],
         [tdc("28–35"),tdc("4H"), td("Reserved / parameter R/W area"), td("")],
         [tdc("36–55"),tdc("5i"), td("Read data + assignable monitors 0–3"), td("")]],
        [18*mm, 14*mm, 58*mm, 74*mm]
    ))
    elems.append(Spacer(1, 3*mm))
    elems.append(Paragraph(
        "Python struct: struct.Struct(\"&lt;HHHH iIi HH I HHHH iiiii\")  \u2192  56 bytes", sCode))
    elems.append(Spacer(1, 3*mm))

    # Fixed I/O OUT bits
    elems.append(Paragraph("Fixed I/O (OUT) bit definitions  (input bytes 4–5):", sH2))
    elems.append(make_table(
        [[th("Bit"), th("Mask"), th("Name"),    th("Description")],
         [tdc("1"),  tdc("0x0002"), td("MOVE"),     td("Motor is currently operating")],
         [tdc("2"),  tdc("0x0004"), td("IN-POS"),   td("Positioning complete")],
         [tdc("4"),  tdc("0x0010"), td("HOME-END"), td("Return-to-home complete")],
         [tdc("5"),  tdc("0x0020"), td("READY"),    td("Driver ready to accept command")],
         [tdc("6"),  tdc("0x0040"), td("DCMD-RDY"), td("Ready for direct data operation  (poll before TRIG)")],
         [tdc("7"),  tdc("0x0080"), td("ALM-A"),    td("Alarm active  (normally open)")],
         [tdc("8"),  tdc("0x0100"), td("TRIG_R"),   td("TRIG acknowledged by driver")],
         [tdc("10"), tdc("0x0400"), td("SET-ERR"),  td("Error in direct data settings")],
         [tdc("11"), tdc("0x0800"), td("EXE-ERR"),  td("Direct data execution failed")]],
        [14*mm, 22*mm, 28*mm, 100*mm]
    ))
    elems.append(Spacer(1, 5*mm))

    # TRIG sequence
    elems.append(Paragraph("Direct Data Operation — TRIG ON-Edge Sequence  (manual §6-3)", sH2))
    steps = [
        ("1", "Poll input assembly until DCMD-RDY (bit 6 of FIO-OUT) is ON."),
        ("2", "Write output assembly: set op type=1, position, speed, accel, decel.  TRIG = 0."),
        ("3", "Wait 1 ms, then write output again with TRIG = 1  (rising edge triggers move)."),
        ("4", "Wait 5 ms, write output with TRIG = 0  (clear after driver latches)."),
        ("5", "Poll input until IN-POS (bit 2) or HOME-END (bit 4) → publish status='idle'."),
        ("6", "If ALM-A (bit 7), SET-ERR (bit 10), or EXE-ERR (bit 11) → publish status='error'."),
    ]
    elems.append(make_table(
        [[th("Step"), th("Action")]] + [[tdc(s), td(a)] for s, a in steps],
        [16*mm, 148*mm]
    ))
    elems.append(Spacer(1, 4*mm))

    # Unit conversions
    elems.append(Paragraph("Unit Conversions  (steps_per_mm = encoder_pulses_per_rev / screw_lead_mm_per_rev)", sH2))
    elems.append(make_table(
        [[th("ROS parameter"), th("Assembly unit"), th("Conversion formula")],
         [td("pos_mm  (mm)"),         td("position  (steps)"), td("steps = round(pos_mm × steps_per_mm)")],
         [td("spd_mms  (mm/s)"),      td("speed  (Hz)"),       td("hz = round(spd_mms × steps_per_mm)")],
         [td("acceleration_mms2  (mm/s²)"), td("accel_raw  (1=0.001 kHz/s)"), td("raw = round(accel_mms2 × steps_per_mm)  or 1 000 000 if 0")]],
        [50*mm, 38*mm, 76*mm]
    ))
    elems.append(Spacer(1, 3*mm))
    elems.append(Paragraph(
        "Example: DR28T with 1000 pulses/rev and 2.5 mm/rev lead → steps_per_mm = 400.  "
        "Moving 10 mm at 5 mm/s → position=4000 steps, speed=2000 Hz.", sNote))
    return elems


# ══════════════════════════════════════════════════════════════
# SECTION 3 – ROS2 NODES
# ══════════════════════════════════════════════════════════════
def section_ros():
    elems = [Paragraph("3  ROS2 Nodes", sH1)]

    # ── 3.1 Topic API (shared) ──
    elems.append(Paragraph("3.1  Topic API  (identical for both driver nodes)", sH2))
    elems.append(make_table(
        [[th("Topic  (relative)"), th("Type"), th("Dir"), th("Description")],
         [td("~/motor_cmd"),    td("std_msgs/Float32MultiArray"), tdc("Sub"),
          td("data[0]=pos (mm),  data[1]=speed (mm/s)")],
         [td("~/motor_home"),   td("std_msgs/Empty"),             tdc("Sub"),
          td("Trigger return-to-home")],
         [td("~/motor_status"), td("std_msgs/String"),            tdc("Pub"),
          td("idle | busy | running | error")]],
        [44*mm, 56*mm, 16*mm, 50*mm]
    ))
    elems.append(Spacer(1, 4*mm))

    # ── 3.2 USB node ──
    elems.append(Paragraph("3.2  motor_controller_node.py  —  USB Serial (Legacy)", sH2))
    elems.append(make_table(
        [[th("Parameter"),            th("Default"),       th("Description")],
         [td("com_port"),             tdc("/dev/ttyACM0"), td("Serial device path")],
         [td("baudrate"),             tdc("19200"),        td("Must match MEXE02 setting")],
         [td("max_position_mm"),      tdc("29.0"),         td("Soft upper limit (mm)")],
         [td("max_speed_mms"),        tdc("100.0"),        td("Hard speed cap (mm/s)")],
         [td("frame_delay"),          tdc("0.1"),          td("Inter-frame delay (s)")],
         [td("heartbeat_interval"),   tdc("0.2"),          td("Watchdog ping period (s)")]],
        [52*mm, 28*mm, 84*mm]
    ))
    elems.append(Spacer(1, 4*mm))

    # ── 3.3 EtherNet/IP node ──
    elems.append(Paragraph("3.3  ethernet_ip_motor_node.py  —  EtherNet/IP  (Production)", sH2))
    elems.append(Paragraph(
        "Requires:  pip install pycomm3", sCode))
    elems.append(Spacer(1, 2*mm))
    elems.append(make_table(
        [[th("Parameter"),          th("Default"),       th("Description")],
         [td("ip_address"),         tdc("192.168.1.2"),  td("Driver IP  (set via rotary switches)")],
         [td("motor_id"),           tdc("\"motor\""),    td("Label for log messages")],
         [td("steps_per_mm"),       tdc("100.0"),        td("!! CALIBRATE !! — encoder_pulses_per_rev / screw_lead_mm_per_rev")],
         [td("max_position_mm"),    tdc("29.0"),         td("Soft upper limit (mm)")],
         [td("min_speed_mms"),      tdc("0.1"),          td("Minimum commanded speed (mm/s)")],
         [td("max_speed_mms"),      tdc("100.0"),        td("Maximum commanded speed (mm/s)")],
         [td("acceleration_mms2"),  tdc("0.0"),          td("0 = use driver default  (1000 kHz/s)")],
         [td("deceleration_mms2"),  tdc("0.0"),          td("0 = use driver default  (1000 kHz/s)")],
         [td("poll_interval_s"),    tdc("0.05"),         td("Status polling period (s)")],
         [td("dcmd_rdy_timeout_s"), tdc("3.0"),          td("Max wait for DCMD-RDY before move")]],
        [52*mm, 28*mm, 84*mm]
    ))
    elems.append(Spacer(1, 3*mm))
    elems.append(Paragraph(
        "Tune steps_per_mm at runtime without rebuilding:", sBody))
    elems.append(Paragraph(
        "ros2 param set /motor_z/ethernet_ip_motor_node steps_per_mm 400.0", sCode))
    elems.append(Spacer(1, 5*mm))

    # ── 3.4 Rotary stub ──
    elems.append(Paragraph("3.4  rotary_motor_stub_node.py  —  Simulation Stub", sH2))
    elems.append(Paragraph(
        "Simulates rotary and conveyor motors so the full ROS2 graph can run "
        "without hardware.  Publishes idle/spinning/stepping status after configurable delays.", sBody))
    elems.append(make_table(
        [[th("Topic  (relative)"), th("Type"), th("Dir"), th("Description")],
         [td("~/motor_spin"),   td("std_msgs/Float32"), tdc("Sub"), td("Set target RPM")],
         [td("~/motor_stop"),   td("std_msgs/Empty"),   tdc("Sub"), td("Stop rotation")],
         [td("~/motor_step"),   td("std_msgs/Empty"),   tdc("Sub"), td("Advance one conveyor step")],
         [td("~/motor_status"), td("std_msgs/String"),  tdc("Pub"), td("idle | spinning | stepping")]],
        [40*mm, 46*mm, 16*mm, 62*mm]
    ))
    elems.append(Spacer(1, 5*mm))

    # ── 3.5 Harvest sequencer ──
    elems.append(Paragraph("3.5  harvest_sequence_node.py  —  Harvest Orchestrator", sH2))
    elems.append(Paragraph(
        "Coordinates all 5 axes through one yuzu peel cycle.  "
        "Receives optional yuzu measurements from the image team and adapts "
        "Z depth and blade advance accordingly.", sBody))
    elems.append(make_table(
        [[th("Topic"), th("Type"), th("Dir"), th("Description")],
         [td("/yuzu_detected"), td("std_msgs/Float32MultiArray"), tdc("Sub"),
          td("[x_off, y_off, long_mm, short_mm]  —  optional, improves depth")],
         [td("/harvest/start"),  td("std_msgs/Empty"),  tdc("Sub"), td("Start one peel cycle")],
         [td("/harvest/abort"),  td("std_msgs/Empty"),  tdc("Sub"), td("Emergency stop all motors")],
         [td("/harvest/status"), td("std_msgs/String"), tdc("Pub"),
          td("idle|feeding|lowering|spinning_up|peeling|retracting|spinning_down|raising|done|error|aborted")]],
        [38*mm, 48*mm, 14*mm, 64*mm]
    ))
    elems.append(Spacer(1, 3*mm))
    elems.append(Paragraph("Key harvest parameters (all tunable via ros2 param set):", sBody))
    elems.append(make_table(
        [[th("Parameter"),           th("Default"), th("Description")],
         [td("yuzu_rotation_rpm"),   tdc("225.0"),  td("Yuzu spindle speed  (200–250 rpm spec)")],
         [td("peeler_orbit_rpm"),    tdc("25.0"),   td("Peeler orbit speed  (20–30 rpm spec)")],
         [td("peel_duration_s"),     tdc("1.5"),    td("Peel arc time  (0.5 rev at 20 rpm)")],
         [td("z_lower_pos_mm"),      tdc("25.0"),   td("Z descent onto yuzu  (scales with long_axis_mm)")],
         [td("peeler_advance_mm"),   tdc("10.0"),   td("Blade push depth  (scales with short_axis_mm)")],
         [td("conveyor_settle_s"),   tdc("1.2"),    td("Wait after conveyor step")]],
        [52*mm, 28*mm, 84*mm]
    ))
    return elems


# ══════════════════════════════════════════════════════════════
# SECTION 4 – LAUNCH FILES
# ══════════════════════════════════════════════════════════════
def section_launch():
    elems = [Paragraph("4  Launch Files", sH1)]
    elems.append(Paragraph(
        "Two separate launch files — use the one that matches your hardware connection:", sBody))

    elems.append(make_table(
        [[th("File"), th("Command"), th("Linear driver used")],
         [td("motor_usb.launch.py"),
          td("ros2 launch motor_controller motor_usb.launch.py"),
          td("motor_controller_node  (USB serial)")],
         [td("motor_lan.launch.py"),
          td("ros2 launch motor_controller motor_lan.launch.py"),
          td("ethernet_ip_motor_node  (EtherNet/IP)")]],
        [42*mm, 78*mm, 44*mm]
    ))
    elems.append(Spacer(1, 4*mm))
    elems.append(Paragraph(
        "Both files launch the rotary stubs and harvest_sequence_node automatically.  "
        "Stubs are skipped for any namespace that already has a real driver configured.", sNote))
    elems.append(Spacer(1, 4*mm))

    elems.append(Paragraph("4.1  motor_usb.launch.py  — configure SERIAL_MOTORS list", sH2))
    elems.append(Paragraph(
        "SERIAL_MOTORS = [\n"
        "    { \"ns\": \"motor_z\",  \"motor_id\": \"motor_z\",  \"com_port\": \"/dev/ttyACM0\" },\n"
        "]", sCode))
    elems.append(Spacer(1, 4*mm))

    elems.append(Paragraph("4.2  motor_lan.launch.py  — configure ETHERNET_IP_MOTORS list", sH2))
    elems.append(Paragraph(
        "ETHERNET_IP_MOTORS = [\n"
        "    { \"ns\": \"motor_z\",  \"motor_id\": \"motor_z\",\n"
        "      \"ip\": \"192.168.1.2\",  \"steps_per_mm\": 400.0 },\n"
        "]", sCode))
    elems.append(Spacer(1, 3*mm))

    elems.append(Paragraph("Build and launch:", sH2))
    for cmd in [
        "cd ~/ros2_ws",
        "colcon build --packages-select motor_controller",
        "source install/setup.bash",
        "# USB method:",
        "ros2 launch motor_controller motor_usb.launch.py",
        "# LAN method:",
        "ros2 launch motor_controller motor_lan.launch.py",
    ]:
        elems.append(Paragraph(cmd, sCode))
    return elems


# ══════════════════════════════════════════════════════════════
# SECTION 5 – GUI
# ══════════════════════════════════════════════════════════════
def section_gui():
    elems = [Paragraph("5  GUI — motor_gui.py", sH1)]
    elems.append(Paragraph("Run from the motion_cmd directory:", sBody))
    elems.append(Paragraph("python motor_controller/motor_gui.py", sCode))
    elems.append(Spacer(1, 4*mm))

    elems.append(Paragraph(
        "Tabbed Tkinter GUI.  Each motor has its own tab; a Harvest tab "
        "triggers and monitors the full peel cycle.", sBody))
    elems.append(make_table(
        [[th("Element"),              th("Function")],
         [td("Per-motor tab"),        td("Status dot, position/speed inputs, Send Command, Go Home, log box")],
         [td("Status dot"),           td("Colour-coded live /motor_status feed")],
         [td("Position entry"),       td("0 – 29 mm; validated before publish")],
         [td("Speed entry"),          td("0.1 – 100.0 mm/s; validated before publish")],
         [td("Send Command"),         td("Publishes Float32MultiArray to ~/motor_cmd")],
         [td("Go Home"),              td("Confirmation dialog → publishes to ~/motor_home")],
         [td("Harvest tab — Start"),  td("Publishes Empty to /harvest/start")],
         [td("Harvest tab — Abort"),  td("Publishes Empty to /harvest/abort (emergency)")],
         [td("Harvest status bar"),   td("Shows /harvest/status live")]],
        [50*mm, 116*mm]
    ))
    elems.append(Spacer(1, 4*mm))
    elems.append(make_table(
        [[th("Colour"),        th("Status"),   th("Meaning")],
         [tdc("●  Green"),    tdc("idle"),     td("Ready, no active motion")],
         [tdc("●  Orange"),   tdc("busy"),     td("Command being transmitted")],
         [tdc("●  Blue"),     tdc("running"),  td("Motor in motion")],
         [tdc("●  Red"),      tdc("error"),    td("Command rejected or alarm")]],
        [35*mm, 30*mm, 101*mm],
        extra=[
            ("TEXTCOLOR",(0,1),(0,1), GREEN),
            ("TEXTCOLOR",(0,2),(0,2), ORANGE),
            ("TEXTCOLOR",(0,3),(0,3), BLUE),
            ("TEXTCOLOR",(0,4),(0,4), RED),
        ]
    ))
    elems.append(Spacer(1, 3*mm))
    elems.append(Paragraph(
        "Requires: rclpy, tkinter, std_msgs.  "
        "ROS2 spin runs in a daemon thread — closing the window shuts down cleanly.", sNote))
    return elems


# ══════════════════════════════════════════════════════════════
# SECTION 6 – SAFETY
# ══════════════════════════════════════════════════════════════
def section_safety():
    elems = [Paragraph("6  Safety & Operating Limits", sH1)]
    elems.append(make_table(
        [[th("Limit"),            th("Value"),       th("Enforced By")],
         [td("Max position"),     tdc("29.0 mm"),    td("Node parameter + GUI validation")],
         [td("Min position"),     tdc("0.0 mm"),     td("Node + GUI")],
         [td("Max speed"),        tdc("100.0 mm/s"), td("Node parameter + GUI validation")],
         [td("Min speed"),        tdc("0.1 mm/s"),   td("Node + GUI")],
         [td("Physical stroke"),  tdc("30 mm"),      td("Hardware — do not override max_position_mm above 29")]],
        [50*mm, 32*mm, 82*mm]
    ))
    elems.append(Spacer(1, 4*mm))
    elems.append(Paragraph("Warnings", sH2))
    for w in [
        "Always home the motor after power-on before sending position commands.",
        "Do NOT set max_position_mm above 29.0 mm — physical stroke is 30 mm; over-travel damages the cylinder.",
        "USB: ensure /dev/ttyACM0 is accessible:  sudo chmod 666 /dev/ttyACM0",
        "USB: close MEXE02 on Windows before running the ROS2 node — only one process can hold the COM port.",
        "LAN: calibrate steps_per_mm before the first move, or the motor will travel the wrong distance.",
        "LAN: set PC NIC to 192.168.1.x subnet before launch, otherwise connection will fail silently.",
        "Harvest abort (/harvest/abort) homes Z and advance axes immediately — ensure clear area before pressing.",
    ]:
        elems.append(Paragraph(f"▲  {w}", sWarn))
    return elems


# ══════════════════════════════════════════════════════════════
# SECTION 7 – TROUBLESHOOTING
# ══════════════════════════════════════════════════════════════
def section_trouble():
    elems = [Paragraph("7  Troubleshooting", sH1)]

    elems.append(Paragraph("7.1  USB Serial Issues", sH2))
    elems.append(make_table(
        [[th("Symptom"), th("Likely Cause"), th("Fix")],
         [td("Failed to open /dev/ttyACM0"),
          td("Permission denied or cable unplugged"),
          td("sudo chmod 666 /dev/ttyACM0")],
         [td("No motor response after frames sent"),
          td("Wrong baud/parity or motor not powered"),
          td("Confirm 24 V supply; check serial settings")],
         [td("FileNotFoundError: write25mm.html"),
          td("Node launched from wrong directory"),
          td("Run from motion_cmd directory")],
         [td("Motor stops mid-motion"),
          td("Heartbeat thread died (SerialException)"),
          td("Restart node; check USB cable")]],
        [52*mm, 52*mm, 60*mm]
    ))
    elems.append(Spacer(1, 5*mm))

    elems.append(Paragraph("7.2  EtherNet/IP (LAN) Issues", sH2))
    elems.append(make_table(
        [[th("Symptom"), th("Likely Cause"), th("Fix")],
         [td("EtherNet/IP connect failed"),
          td("Wrong IP / subnet mismatch / cable unplugged"),
          td("Ping driver IP; set PC NIC to 192.168.1.x")],
         [td("ModuleNotFoundError: pycomm3"),
          td("pycomm3 not installed"),
          td("pip install pycomm3")],
         [td("Timeout waiting for DCMD-RDY"),
          td("Driver not ready or alarm active"),
          td("Check PWR/ALM LED; clear alarm via ALM-RST")],
         [td("Motor moves wrong distance"),
          td("steps_per_mm not calibrated"),
          td("Measure actual travel; adjust steps_per_mm")],
         [td("SET-ERR after TRIG"),
          td("Op type / position out of range"),
          td("Check position limits; confirm op type = 1")],
         [td("EXE-ERR after TRIG"),
          td("Driver not in executable state"),
          td("Send ZHOME first; check driver enable signal")],
         [td("NS LED red/blinking"),
          td("EtherNet/IP communication error"),
          td("Check cable; verify IP address on driver")],
         [td("PWR/ALM LED blinking red"),
          td("Driver alarm active"),
          td("Read alarm code from /harvest/status or GUI; clear with ALM-RST")]],
        [46*mm, 54*mm, 64*mm]
    ))
    elems.append(Spacer(1, 5*mm))

    elems.append(Paragraph("7.3  General ROS2 Issues", sH2))
    elems.append(make_table(
        [[th("Symptom"), th("Fix")],
         [td("ModuleNotFoundError: rclpy"),
          td("source /opt/ros/humble/setup.bash")],
         [td("Node not found after colcon build"),
          td("source install/setup.bash && rebuild")],
         [td("GUI shows no status updates"),
          td("Confirm motor driver node is running:  ros2 node list")],
         [td("Position rejected (out of range)"),
          td("Send value between 0.0 and 29.0 mm")]],
        [70*mm, 94*mm]
    ))
    return elems


# ══════════════════════════════════════════════════════════════
# BUILD
# ══════════════════════════════════════════════════════════════
def build():
    doc = SimpleDocTemplate(
        OUTPUT,
        pagesize=A4,
        leftMargin=18*mm, rightMargin=18*mm,
        topMargin=18*mm,  bottomMargin=18*mm,
        title="DR28T AZD-KEP Motor Control Manual",
        author="Auto-generated",
        subject="Motor Control Quick Reference Rev 2.0"
    )

    story = []
    story += cover();          story.append(PageBreak())
    story += section_hw();     story.append(Spacer(1, 6*mm))
    story += section_comm();   story.append(PageBreak())
    story += section_ros();    story.append(PageBreak())
    story += section_launch(); story.append(Spacer(1, 6*mm))
    story += section_gui();    story.append(PageBreak())
    story += section_safety(); story.append(Spacer(1, 6*mm))
    story += section_trouble()

    doc.build(story)
    print(f"Created: {OUTPUT}")

if __name__ == "__main__":
    build()
