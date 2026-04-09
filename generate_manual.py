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

# ── Colour palette ───────────────────────────────────────────
DARK    = colors.HexColor("#1a1a2e")
ACCENT  = colors.HexColor("#0f3460")
BLUE    = colors.HexColor("#2196f3")
GREEN   = colors.HexColor("#4caf50")
ORANGE  = colors.HexColor("#ff9800")
RED     = colors.HexColor("#f44336")
LGREY   = colors.HexColor("#f5f5f5")
MGREY   = colors.HexColor("#e0e0e0")
WHITE   = colors.white

W, H = A4

# ── Styles ───────────────────────────────────────────────────
base = getSampleStyleSheet()

def S(name, parent="Normal", **kw):
    return ParagraphStyle(name, parent=base[parent], **kw)

sTitle   = S("sTitle",   "Title",   fontSize=26, textColor=WHITE,   alignment=TA_CENTER, spaceAfter=4)
sSubtitle= S("sSub",     "Normal",  fontSize=13, textColor=MGREY,   alignment=TA_CENTER, spaceAfter=2)
sH1      = S("sH1",      "Heading1",fontSize=14, textColor=WHITE,   spaceBefore=14, spaceAfter=6,
             backColor=ACCENT, borderPad=6)
sH2      = S("sH2",      "Heading2",fontSize=11, textColor=ACCENT,  spaceBefore=10, spaceAfter=4,
             borderPad=2)
sBody    = S("sBody",    "Normal",  fontSize=9,  leading=14,        spaceAfter=4)
sNote    = S("sNote",    "Normal",  fontSize=8,  textColor=colors.HexColor("#555555"),
             leftIndent=10, spaceAfter=4)
sCode    = S("sCode",    "Code",    fontSize=8,  backColor=LGREY,   leftIndent=10, rightIndent=10,
             borderPad=6, leading=12)
sWarn    = S("sWarn",    "Normal",  fontSize=9,  textColor=RED,     leftIndent=8, spaceAfter=4)
sTH      = S("sTH",      "Normal",  fontSize=9,  textColor=WHITE,   alignment=TA_CENTER)
sTD      = S("sTD",      "Normal",  fontSize=9,  alignment=TA_LEFT)
sTDc     = S("sTDc",     "Normal",  fontSize=9,  alignment=TA_CENTER)

# ── Table helpers ────────────────────────────────────────────
def th(text): return Paragraph(text, sTH)
def td(text): return Paragraph(str(text), sTD)
def tdc(text): return Paragraph(str(text), sTDc)

TSTYLE_BASE = [
    ("BACKGROUND", (0,0), (-1,0), ACCENT),
    ("ROWBACKGROUNDS", (0,1), (-1,-1), [WHITE, LGREY]),
    ("GRID",       (0,0), (-1,-1), 0.4, MGREY),
    ("TOPPADDING", (0,0), (-1,-1), 5),
    ("BOTTOMPADDING",(0,0),(-1,-1),5),
    ("LEFTPADDING",(0,0),(-1,-1), 7),
    ("RIGHTPADDING",(0,0),(-1,-1),7),
    ("VALIGN",     (0,0), (-1,-1), "MIDDLE"),
]

def make_table(data, col_widths, extra_style=None):
    style = list(TSTYLE_BASE)
    if extra_style:
        style.extend(extra_style)
    return Table(data, colWidths=col_widths, style=TableStyle(style))

# ── Cover page ───────────────────────────────────────────────
def cover():
    elems = []
    elems.append(Spacer(1, 30*mm))

    # Blue banner
    banner = Table(
        [[Paragraph("MOTOR CONTROL SYSTEM", sTitle)],
         [Paragraph("Quick Reference Manual", sSubtitle)],
         [Spacer(1, 4*mm)],
         [Paragraph("DR28T2.5B03-AZAKD  ×  AZD-KEP Driver", sSubtitle)]],
        colWidths=[160*mm],
        style=TableStyle([
            ("BACKGROUND",(0,0),(-1,-1), DARK),
            ("TOPPADDING",(0,0),(-1,-1), 8),
            ("BOTTOMPADDING",(0,0),(-1,-1), 8),
            ("LEFTPADDING",(0,0),(-1,-1), 14),
            ("RIGHTPADDING",(0,0),(-1,-1), 14),
        ])
    )
    elems.append(banner)
    elems.append(Spacer(1, 14*mm))

    # Meta table
    meta = make_table(
        [[th("Revision"), th("Platform"), th("Interface"), th("Date")],
         [tdc("1.0"), tdc("Ubuntu + ROS2"), tdc("USB / EtherNet/IP"), tdc("April 2026")]],
        [35*mm, 45*mm, 50*mm, 35*mm]
    )
    elems.append(meta)
    elems.append(Spacer(1, 10*mm))
    elems.append(Paragraph(
        "This manual covers hardware specifications, communication methods, "
        "ROS2 node parameters, topic API, and the GUI for the Oriental Motor "
        "DR28T compact electric cylinder paired with the AZD-KEP EtherNet/IP driver.",
        sBody))
    return elems

# ── Section 1 – Hardware Specifications ─────────────────────
def section_hw():
    elems = []
    elems.append(Paragraph("1  Hardware Specifications", sH1))

    elems.append(Paragraph("1.1  Motor — DR28T2.5B03-AZAKD", sH2))
    elems.append(make_table(
        [[th("Parameter"), th("Value"), th("Notes")],
         [td("Type"),           tdc("Compact Electric Cylinder (Ball Screw)"), td("")],
         [td("Stroke"),         tdc("30 mm"),         td("Soft limit set to 29 mm in software")],
         [td("Lead / Pitch"),   tdc("2.5 mm / rev"),  td("")],
         [td("Max Speed"),      tdc("100 mm/s"),       td("Hard cap in motor_controller_node.py")],
         [td("Max Thrust"),     tdc("20 N"),           td("")],
         [td("Repeat Accuracy"),tdc("±0.003 mm"),      td("Ball screw shaft tip")],
         [td("Min Travel"),     tdc("0.001 mm"),       td("")],
         [td("Encoder"),        tdc("Absolute (Mechanical Multi-turn)"), td("No battery required")],
         [td("Supply Voltage"), tdc("24 / 48 VDC"),   td("Driver dependent")]],
        [50*mm, 60*mm, 55*mm]
    ))
    elems.append(Spacer(1, 5*mm))

    elems.append(Paragraph("1.2  Driver — AZD-KEP", sH2))
    elems.append(make_table(
        [[th("Parameter"), th("Value")],
         [td("Series"),          tdc("AZ Series Closed Loop Stepper")],
         [td("Fieldbus"),        tdc("EtherNet/IP™  (CT16 conformant)")],
         [td("Network Speed"),   tdc("10 / 100 Mbps  (autonegotiation)")],
         [td("Control Power"),   tdc("24 VDC ±5%,  0.25 A")],
         [td("Pulse Input"),     tdc("2 pts  (Photocoupler, max 1 MHz)")],
         [td("Control I/O"),     tdc("6 inputs / 6 outputs  (Photocoupler)")],
         [td("USB Port"),        tdc("Mini-B  →  MEXE02 configuration (Windows only)")],
         [td("EDS File"),        tdc("Provided by Oriental Motor")]],
        [65*mm, 100*mm]
    ))
    elems.append(Spacer(1, 3*mm))
    elems.append(Paragraph(
        "Note: The AZD-KEP's USB port exposes a Virtual COM Port (Windows: COMx, "
        "Linux: /dev/ttyACM0). On Ubuntu/ROS2 this is used for MEXE02 frame replay "
        "since MEXE02 software is Windows-only.", sNote))
    return elems

# ── Section 2 – Communication ────────────────────────────────
def section_comm():
    elems = []
    elems.append(Paragraph("2  Communication Interfaces", sH1))

    elems.append(Paragraph("2.1  USB Serial (Current Method — Ubuntu/ROS2)", sH2))
    elems.append(Paragraph(
        "The motor is controlled by replaying binary MEXE02 protocol frames "
        "originally captured on Windows. Frames are extracted from HTML log files "
        "generated by MEXE02 and sent over the USB Virtual COM Port.", sBody))

    elems.append(make_table(
        [[th("Setting"), th("Value")],
         [td("Device"),      tdc("/dev/ttyACM0")],
         [td("Baud Rate"),   tdc("19200")],
         [td("Data Bits"),   tdc("8")],
         [td("Parity"),      tdc("Even")],
         [td("Stop Bits"),   tdc("1")],
         [td("Timeout"),     tdc("0.5 s")]],
        [65*mm, 100*mm]
    ))
    elems.append(Spacer(1, 4*mm))

    elems.append(Paragraph("Position & Speed Encoding", sH2))
    elems.append(Paragraph("All values are transmitted in <b>micrometres (μm)</b>, "
                            "little-endian 32-bit integers:", sBody))
    elems.append(make_table(
        [[th("Value"), th("Formula"), th("Baseline Hex (little-endian)"), th("Example")],
         [td("Position"), td("mm × 1000"), tdc("a8 61 00 00  (= 25 mm)"), td("15 mm → 3A 98 00 00")],
         [td("Speed"),    td("mm/s × 1000"), tdc("e8 03 00 00  (= 1 mm/s)"), td("3 mm/s → B8 0B 00 00")]],
        [30*mm, 35*mm, 65*mm, 40*mm]
    ))
    elems.append(Spacer(1, 3*mm))
    elems.append(Paragraph(
        "XOR Checksum: mask = XOR(baseline bytes) ⊕ XOR(target bytes). "
        "New checksums = old checksums ⊕ mask. "
        "Implemented in create_combined_html() in motor_controller_node.py.", sNote))

    elems.append(Spacer(1, 4*mm))
    elems.append(Paragraph("HTML Template Files Required", sH2))
    elems.append(make_table(
        [[th("File"), th("Purpose")],
         [td("write25mm.html"),   td("Baseline position command  (25 mm)")],
         [td("1mms.html"),        td("Baseline speed command  (1 mm/s)")],
         [td("2home.html"),       td("Homing sequence frames")],
         [td("start_run.html"),   td("Execute motion trigger")]],
        [60*mm, 105*mm]
    ))

    elems.append(Spacer(1, 5*mm))
    elems.append(Paragraph("2.2  EtherNet/IP (Recommended for Production)", sH2))
    elems.append(Paragraph(
        "The AZD-KEP's primary interface is EtherNet/IP (CIP). "
        "Implicit messaging exchanges 40-byte command and 56-byte status "
        "assemblies every 10 ms over UDP.", sBody))
    elems.append(make_table(
        [[th("Direction"), th("Instance"), th("Size"), th("Key Contents")],
         [td("Scanner → Driver  (command)"), tdc("0x65  (101)"), tdc("40 bytes"),
          td("Control word, target position (μm), target speed (μm/s)")],
         [td("Driver → Scanner  (status)"),  tdc("0x64  (100)"), tdc("56 bytes"),
          td("Status word, current position (μm), alarm code")]],
        [52*mm, 30*mm, 22*mm, 64*mm]
    ))
    elems.append(Spacer(1, 3*mm))
    elems.append(Paragraph(
        "Setup: assign IP via driver rotary switches → connect Ethernet → "
        "ForwardOpen with RPI = 10,000 μs, Point-to-Point, Header32Bit on O→T. "
        "Python libraries: cpppo or pycomm3 CIPDriver.", sNote))
    return elems

# ── Section 3 – ROS2 Node ────────────────────────────────────
def section_ros():
    elems = []
    elems.append(Paragraph("3  ROS2 Node — motor_controller_node.py", sH1))

    elems.append(Paragraph("3.1  Topics", sH2))
    elems.append(make_table(
        [[th("Topic"), th("Type"), th("Direction"), th("Description")],
         [td("/motor_cmd"),    td("std_msgs/Float32MultiArray"), tdc("Subscribe"),
          td("data[0] = position (mm),  data[1] = speed (mm/s)")],
         [td("/motor_home"),   td("std_msgs/Empty"),             tdc("Subscribe"),
          td("Trigger homing sequence")],
         [td("/motor_status"), td("std_msgs/String"),            tdc("Publish"),
          td("idle | busy | running | error")]],
        [42*mm, 58*mm, 25*mm, 44*mm]
    ))
    elems.append(Spacer(1, 5*mm))

    elems.append(Paragraph("3.2  Parameters", sH2))
    elems.append(make_table(
        [[th("Parameter"), th("Default"), th("Description")],
         [td("com_port"),              tdc("/dev/ttyACM0"), td("Serial device path")],
         [td("baudrate"),              tdc("19200"),        td("Serial baud rate")],
         [td("serial_timeout"),        tdc("0.5"),          td("Read timeout (s)")],
         [td("serial_write_timeout"),  tdc("0.5"),          td("Write timeout (s)")],
         [td("max_position_mm"),       tdc("29.0"),         td("Position upper limit (mm).  Physical stroke = 30 mm")],
         [td("max_speed_mms"),         tdc("100.0"),        td("Speed upper limit (mm/s).  Datasheet rated max")],
         [td("frame_delay"),           tdc("0.1"),          td("Inter-frame delay (s)")],
         [td("heartbeat_interval"),    tdc("0.2"),          td("Watchdog ping period (s)")]],
        [52*mm, 28*mm, 86*mm]
    ))
    elems.append(Spacer(1, 5*mm))

    elems.append(Paragraph("3.3  Motor Status Values", sH2))
    elems.append(make_table(
        [[th("Status"), th("Meaning")],
         [td("idle"),    td("Node started or shut down.  No active command.")],
         [td("busy"),    td("Transmitting serial frames to the motor.")],
         [td("running"), td("Heartbeat active — motor is executing motion.")],
         [td("error"),   td("Missing template file or invalid command received.")]],
        [30*mm, 136*mm],
        extra_style=[
            ("TEXTCOLOR", (0,1), (0,1), colors.HexColor("#4caf50")),
            ("TEXTCOLOR", (0,2), (0,2), colors.HexColor("#ff9800")),
            ("TEXTCOLOR", (0,3), (0,3), colors.HexColor("#2196f3")),
            ("TEXTCOLOR", (0,4), (0,4), colors.HexColor("#f44336")),
        ]
    ))
    elems.append(Spacer(1, 5*mm))

    elems.append(Paragraph("3.4  Quick Commands", sH2))
    cmds = [
        ("Run node (default port)",
         "python motor_controller_node.py"),
        ("Run with custom port",
         "ros2 run <pkg> motor_controller_node --ros-args -p com_port:=/dev/ttyUSB0"),
        ("Send motion command",
         'ros2 topic pub /motor_cmd std_msgs/msg/Float32MultiArray "{data: [15.0, 3.0]}" --once'),
        ("Trigger homing",
         'ros2 topic pub /motor_home std_msgs/msg/Empty "{}" --once'),
        ("Monitor status",
         "ros2 topic echo /motor_status"),
    ]
    for label, cmd in cmds:
        elems.append(Paragraph(f"<b>{label}</b>", sBody))
        elems.append(Paragraph(cmd, sCode))
        elems.append(Spacer(1, 2*mm))
    return elems

# ── Section 4 – GUI ──────────────────────────────────────────
def section_gui():
    elems = []
    elems.append(Paragraph("4  GUI — motor_gui.py", sH1))
    elems.append(Paragraph("Run from the motion_cmd directory:", sBody))
    elems.append(Paragraph("python motor_gui.py", sCode))
    elems.append(Spacer(1, 4*mm))

    elems.append(make_table(
        [[th("Element"), th("Function")],
         [td("Status dot + label"),  td("Live colour-coded /motor_status feed")],
         [td("Position entry"),      td("Input 0 – 29 mm.  Validated before publish.")],
         [td("Speed entry"),         td("Input 0.1 – 100.0 mm/s.  Validated before publish.")],
         [td("Send Command button"), td("Publishes position + speed to /motor_cmd")],
         [td("Go Home button"),      td("Confirmation dialog → publishes to /motor_home")],
         [td("Log box"),             td("Shows all outgoing publishes and incoming status")]],
        [50*mm, 116*mm]
    ))
    elems.append(Spacer(1, 4*mm))

    elems.append(Paragraph("Status indicator colours:", sH2))
    elems.append(make_table(
        [[th("Colour"), th("Status"), th("Meaning")],
         [tdc("●  Green"),  tdc("idle"),    td("Motor ready, no active motion")],
         [tdc("●  Orange"), tdc("busy"),    td("Frames being sent over serial")],
         [tdc("●  Blue"),   tdc("running"), td("Heartbeat active, motor moving")],
         [tdc("●  Red"),    tdc("error"),   td("Command rejected or file missing")]],
        [35*mm, 30*mm, 101*mm],
        extra_style=[
            ("TEXTCOLOR", (0,1), (0,1), GREEN),
            ("TEXTCOLOR", (0,2), (0,2), ORANGE),
            ("TEXTCOLOR", (0,3), (0,3), BLUE),
            ("TEXTCOLOR", (0,4), (0,4), RED),
        ]
    ))
    elems.append(Spacer(1, 3*mm))
    elems.append(Paragraph(
        "Requires: rclpy, tkinter, std_msgs.  "
        "ROS2 spin runs in a daemon thread — closing the window shuts down the node cleanly.", sNote))
    return elems

# ── Section 5 – Safety ───────────────────────────────────────
def section_safety():
    elems = []
    elems.append(Paragraph("5  Safety & Operating Limits", sH1))
    elems.append(make_table(
        [[th("Limit"), th("Value"), th("Enforced By")],
         [td("Max position"),     tdc("29.0 mm"),    td("motor_controller_node (parameter) + GUI validation")],
         [td("Min position"),     tdc("0.0 mm"),     td("motor_controller_node + GUI")],
         [td("Max speed"),        tdc("100.0 mm/s"), td("motor_controller_node (parameter) + GUI validation")],
         [td("Min speed"),        tdc("> 0.1 mm/s"), td("motor_controller_node + GUI")],
         [td("Physical stroke"),  tdc("30 mm"),      td("Hardware — do not override max_position_mm above 29")]],
        [55*mm, 35*mm, 76*mm]
    ))
    elems.append(Spacer(1, 4*mm))
    elems.append(Paragraph("Important Warnings", sH2))
    warnings = [
        "Always home the motor after power-on before sending position commands.",
        "Do not set max_position_mm above 29.0 mm — the physical stroke is 30 mm and over-travel will damage the cylinder.",
        "Ensure /dev/ttyACM0 permissions are correct on Ubuntu:  sudo chmod 666 /dev/ttyACM0",
        "The heartbeat (watchdog) must be running at all times.  If the node crashes, the motor will stop receiving pings and may fault.",
        "Only one process should hold /dev/ttyACM0 at a time.  Close MEXE02 (Windows) before running the ROS2 node.",
    ]
    for w in warnings:
        elems.append(Paragraph(f"▲  {w}", sWarn))
    return elems

# ── Section 6 – Troubleshooting ──────────────────────────────
def section_trouble():
    elems = []
    elems.append(Paragraph("6  Troubleshooting", sH1))
    elems.append(make_table(
        [[th("Symptom"), th("Likely Cause"), th("Fix")],
         [td("ModuleNotFoundError: rclpy"),
          td("ROS2 environment not sourced"),
          td("source /opt/ros/humble/setup.bash")],
         [td("Failed to open /dev/ttyACM0"),
          td("Permission denied or cable unplugged"),
          td("sudo chmod 666 /dev/ttyACM0")],
         [td("No response after sending frames"),
          td("Wrong baud / parity or motor not powered"),
          td("Check 24 V supply; verify serial settings")],
         [td("Position rejected (out of range)"),
          td("Value exceeds max_position_mm (29.0)"),
          td("Send value between 0.0 and 29.0 mm")],
         [td("Speed rejected"),
          td("Value exceeds 100.0 mm/s or ≤ 0"),
          td("Send value between 0.1 and 100.0 mm/s")],
         [td("Motor stops mid-motion"),
          td("Heartbeat thread died (SerialException)"),
          td("Restart node; check USB connection")],
         [td("GUI shows no status updates"),
          td("/motor_status not published"),
          td("Confirm motor_controller_node is running")],
         [td("FileNotFoundError: write25mm.html"),
          td("Node launched from wrong directory"),
          td("cd motion_cmd before running node")]],
        [52*mm, 52*mm, 62*mm]
    ))
    return elems

# ── Build PDF ────────────────────────────────────────────────
def build():
    doc = SimpleDocTemplate(
        OUTPUT,
        pagesize=A4,
        leftMargin=18*mm, rightMargin=18*mm,
        topMargin=18*mm, bottomMargin=18*mm,
        title="DR28T AZD-KEP Motor Control Manual",
        author="Auto-generated",
        subject="Motor Control Quick Reference"
    )

    story = []
    story += cover()
    story.append(PageBreak())
    story += section_hw()
    story.append(Spacer(1, 6*mm))
    story += section_comm()
    story.append(PageBreak())
    story += section_ros()
    story.append(Spacer(1, 6*mm))
    story += section_gui()
    story.append(PageBreak())
    story += section_safety()
    story.append(Spacer(1, 6*mm))
    story += section_trouble()

    doc.build(story)
    print(f"Created: {OUTPUT}")

if __name__ == "__main__":
    build()
