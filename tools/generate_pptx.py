#!/usr/bin/env python3
"""Script to generate a professional PowerPoint presentation (.pptx)
for the NIDAR AirMouse autonomous drone progress review.

Requires: pip install python-pptx
Output: docs/NIDAR_AirMouse_Progress_Presentation.pptx
"""
import sys
from pathlib import Path

try:
    from pptx import Presentation
    from pptx.util import Inches, Pt
    from pptx.dml.color import RGBColor
    from pptx.enum.text import PP_ALIGN
    from pptx.enum.shapes import MSO_SHAPE
except ImportError:
    print("Error: python-pptx is not installed.")
    print("Install it with: pip install python-pptx")
    sys.exit(1)


def create_deck(output_path: str = "docs/NIDAR_AirMouse_Progress_Presentation.pptx"):
    prs = Presentation()
    prs.slide_width = Inches(13.333)
    prs.slide_height = Inches(7.5)

    blank_layout = prs.slide_layouts[6]

    # Theme Colors
    DARK_BG = RGBColor(11, 15, 25)
    CARD_BG = RGBColor(17, 24, 39)
    TEXT_LIGHT = RGBColor(243, 244, 246)
    TEXT_MUTED = RGBColor(156, 163, 175)
    ACCENT_BLUE = RGBColor(59, 130, 246)
    ACCENT_EMERALD = RGBColor(16, 185, 129)
    ACCENT_RED = RGBColor(239, 68, 68)

    def set_bg(slide):
        bg = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, Inches(13.333), Inches(7.5))
        bg.fill.solid()
        bg.fill.fore_color.rgb = DARK_BG
        bg.line.fill.background()

    def add_header(slide, tag: str, title: str, subtitle: str = ""):
        txBox = slide.shapes.add_textbox(Inches(0.8), Inches(0.5), Inches(11.7), Inches(1.2))
        tf = txBox.text_frame
        tf.word_wrap = True
        tf.margin_left = tf.margin_top = tf.margin_right = tf.margin_bottom = 0

        p_tag = tf.paragraphs[0]
        p_tag.text = tag.upper()
        p_tag.font.size = Pt(11)
        p_tag.font.bold = True
        p_tag.font.color.rgb = ACCENT_BLUE

        p_title = tf.add_paragraph()
        p_title.text = title
        p_title.font.size = Pt(26)
        p_title.font.bold = True
        p_title.font.color.rgb = TEXT_LIGHT

        if subtitle:
            p_sub = tf.add_paragraph()
            p_sub.text = subtitle
            p_sub.font.size = Pt(13)
            p_sub.font.color.rgb = TEXT_MUTED

    # Slide 1: Title
    s1 = prs.slides.add_slide(blank_layout)
    set_bg(s1)
    txBox = s1.shapes.add_textbox(Inches(1.0), Inches(1.8), Inches(11.3), Inches(3.5))
    tf = txBox.text_frame
    tf.word_wrap = True

    p0 = tf.paragraphs[0]
    p0.text = "AUTONOMOUS SEARCH & RESCUE DRONE"
    p0.font.size = Pt(14)
    p0.font.bold = True
    p0.font.color.rgb = ACCENT_BLUE

    p1 = tf.add_paragraph()
    p1.text = "NIDAR AirMouse: Engineering Progress & Benchmark Review"
    p1.font.size = Pt(38)
    p1.font.bold = True
    p1.font.color.rgb = TEXT_LIGHT

    p2 = tf.add_paragraph()
    p2.text = "Multi-Rate EKF SLAM, 2D LiDAR Scan Matching, A* Frontier Exploration, and YOLOv8 Vision"
    p2.font.size = Pt(16)
    p2.font.color.rgb = TEXT_MUTED

    # Slide 2: Unified Hardware
    s2 = prs.slides.add_slide(blank_layout)
    set_bg(s2)
    add_header(s2, "Hardware Specification", "Unified Multi-Sensor Aerial Platform", "Co-located complementary sensing modalities for surveillance and GPS-denied SLAM")
    txBox = s2.shapes.add_textbox(Inches(0.8), Inches(2.0), Inches(11.7), Inches(4.5))
    tf = txBox.text_frame
    tf.word_wrap = True

    sensors = [
        ("360° 2D LiDAR", "RPLIDAR C1 / LD06 (12m range, 8Hz) for planar SLAM and LM-ICP point-to-grid scan matching."),
        ("4–5 Ultrasonic Sonars", "HC-SR04 (Front, Left, Right, Rear) @ 20Hz for tight close-range (0.05-2.0m) reactive obstacle avoidance."),
        ("Downward Laser ToF", "VL53L1X laser rangefinder @ 30-50Hz for precision millimetric ground tracking and altitude lock (1.2m)."),
        ("Front RGB Camera", "Pi Cam v3 (70° FoV) @ 20Hz for live operator surveillance, YOLOv8 victim recognition, and 3D ray projection."),
        ("6-DOF IMU", "ICM-42688-P / MPU6050 @ 100Hz on flight controller for high-rate kinematic state propagation in EKF."),
        ("Hardware Abstraction Layer", "core/hal.py (SensorHub & FlightInterface) decouples autonomy logic from simulation and hardware."),
    ]
    for name, desc in sensors:
        p = tf.add_paragraph()
        p.text = f"• {name}: {desc}"
        p.font.size = Pt(14)
        p.font.color.rgb = TEXT_LIGHT

    # Slide 3: Multi-rate software
    s3 = prs.slides.add_slide(blank_layout)
    set_bg(s3)
    add_header(s3, "Autonomy Stack", "Multi-Rate Autonomous Software Pipeline", "Hierarchical execution loops balancing stabilization with heavy perception")
    txBox = s3.shapes.add_textbox(Inches(0.8), Inches(2.0), Inches(11.7), Inches(4.5))
    tf = txBox.text_frame
    tf.word_wrap = True
    rates = [
        ("[100 Hz] Inertial Kinematics & EKF Prediction", "15-state propagation (x, y, z, vx, vy, vz, rpy, accel/gyro bias) via quadcopter kinematics."),
        ("[20 Hz]  ToF Altitude Lock & Reactive Repulsion", "Laser altitude gating and artificial potential field repulsion from ultrasonic proximity."),
        ("[8 Hz]   2D LiDAR Scan Matching & SLAM Mapping", "Levenberg-Marquardt ICP pose corrections fused into EKF; Bayesian log-odds grid updates."),
        ("[2-0.5 Hz] Global Exploration & Path Planning", "Frontier detector clusters unknown boundary cells; 8-connected grid A* plans collision-free paths."),
        ("[0.5-1 Hz] Monocular YOLOv8 Target Perception", "Person detection back-projected to world coordinates with temporal confirmation (N>=3)."),
    ]
    for name, desc in rates:
        p = tf.add_paragraph()
        p.text = f"• {name}: {desc}"
        p.font.size = Pt(14)
        p.font.color.rgb = TEXT_LIGHT

    # Slide 4: Results
    s4 = prs.slides.add_slide(blank_layout)
    set_bg(s4)
    add_header(s4, "Empirical Verification", "Quantitative Flight Benchmark Comparison", "Progress from open-loop baseline to the full LiDAR SLAM autonomy stack")
    txBox = s4.shapes.add_textbox(Inches(0.8), Inches(2.0), Inches(11.7), Inches(4.5))
    tf = txBox.text_frame
    tf.word_wrap = True
    results = [
        ("Exploration Coverage", "Surged from 10.8% in baseline up to 98.7% on procedural layouts (85.0% in tight mazes)."),
        ("Collision Events", "Plummeted by 99.8% (from 547.8 contacts down to 0–2 events) via 360° LiDAR quadrant synthesis."),
        ("State Estimation Drift", "Slashed from 12.54m open-loop dead reckoning down to 0.60m–2.44m with 8Hz scan matching."),
        ("Survivor Detection", "100% precision (0.0 false positives across 10 benchmark layouts) via grazing-ray cutoff (<=4.5m)."),
        ("Takeoff Success", "100% (10/10) flawless vertical ascents to target 1.2m hover ceiling across all trials."),
    ]
    for name, desc in results:
        p = tf.add_paragraph()
        p.text = f"• {name}: {desc}"
        p.font.size = Pt(15)
        p.font.color.rgb = TEXT_LIGHT

    # Slide 5: Roadmap
    s5 = prs.slides.add_slide(blank_layout)
    set_bg(s5)
    add_header(s5, "Active Roadmap", "Phase D-III: Keyframe Memory & Home Relocalisation", "Solving residual heading drift over 80+ meter missions")
    txBox = s5.shapes.add_textbox(Inches(0.8), Inches(2.0), Inches(11.7), Inches(4.5))
    tf = txBox.text_frame
    tf.word_wrap = True
    roadmap = [
        ("The Challenge", "Open-loop scan matching accumulates 1°-3° angular drift over long flights, rotating the perceived origin."),
        ("Task D8-D9 (MapMemory)", "Implement keyframe storage (timestamped poses, downsampled scans, detected victim coordinates)."),
        ("Task D10-D11 (Loop Closure)", "Candidate generation and Pose Graph Optimization (PGO) to relax accumulated drift."),
        ("Task D12 (Home Relocalisation)", "Match live scan against takeoff keyframe when within 3m, pulling the drone onto the pad (<0.5m)."),
        ("Hardware Transition", "Deploying via airmouse_node.py to companion computer (Pi 5 / Jetson) bridging to PX4/ArduPilot."),
    ]
    for name, desc in roadmap:
        p = tf.add_paragraph()
        p.text = f"• {name}: {desc}"
        p.font.size = Pt(15)
        p.font.color.rgb = TEXT_LIGHT

    output_file = Path(output_path)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    prs.save(str(output_file))
    print(f"Presentation successfully generated at: {output_file.resolve()}")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "docs/NIDAR_AirMouse_Progress_Presentation.pptx"
    create_deck(out)

