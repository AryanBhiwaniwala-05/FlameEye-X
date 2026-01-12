FlameEye-X 🔥🤖
Predictive Smart Safety Bot for Gas & Fire Prevention
ESP32 • Multi-Sensor Fusion • Physical AI

🏆 IIT Bombay Techfest – Safetronics Theme
🥇 Theme Rank: 1st | 🥉 Overall Rank: 3rd

Overview
FlameEye-X is an ESP32-based smart safety system that predicts and prevents kitchen fire hazards.
It fuses gas concentration, temperature rise, and flame detection to estimate risk and autonomously act before ignition escalates.

Key Features
Predictive risk estimation (probability-based, not fixed thresholds)
Gas + temperature + flame multi-sensor fusion
Autonomous mitigation: gas valve control, ventilation, alarms
Real-time Wi-Fi alerts via Blynk IoT
Adaptive baseline learning to reduce false alarms
Typical end-to-end response time < 10 seconds
System Working
Sensors continuously sample the environment
Data is normalized and evaluated using a risk probability model
System states:
Safe: monitoring only
Pre-emptive: ventilation and partial control
Emergency: gas shutoff, alarm, mobile alert
Hardware Used
ESP32 Dev Module
MQ-2 Gas Sensor
DS18B20 Temperature Sensor
IR Flame Sensor
Servo Motor (Gas Valve Control)
Relay + Exhaust Fan
Buzzer, LEDs, LCD
(Exact pin mapping is available in the documentation.)

Getting Started
1️⃣ Firmware Setup
Use PlatformIO (recommended) or Arduino IDE with ESP32 support and required libraries.

2️⃣ Blynk & Wi-Fi Configuration (Required)
This project uses Blynk IoT for alerts and monitoring.

You must:

Create your own Blynk template in the Blynk Console
Generate:
Template ID
Template Name
Auth Token
Replace the clearly marked placeholders in the code:

#define BLYNK_TEMPLATE_ID   "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "YOUR_TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN    "YOUR_AUTH_TOKEN"

char ssid[] = "YOUR_WIFI_NAME";
char pass[] = "YOUR_WIFI_PASSWORD";
Safety Notice ⚠️:

This project is a prototype intended strictly for academic, research, and demonstration purposes.It is NOT certified for real kitchen or commercial deployment Do NOT rely on this system for life-critical safety, Real-world use requires:

1)Certified gas shutoff valves

2)Industrial-grade sensors

3)Electrical and fire-safety compliance

The user is fully responsible for safe deployment and usage.
