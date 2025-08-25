# tm4c-freertos-sensor-fusion-demo
A simple train control system demo for ARM Cortex-M4, showcasing advanced FreeRTOS concepts: inter-task synchronization (Task Notifications) and thread-safe resource management (Mutex).

# 🚆 TM4C-FreeRTOS Train Control System

This repository contains the source code for a **real-time embedded system** developed on the **Tiva C TM4C1294XL** platform.  

What began as a simple *train control* concept evolved into a **practical case study** on solving fundamental challenges in multi-threaded embedded environments using **FreeRTOS**.  

The **primary goal** of this project is not just to read from sensors, but to demonstrate a **robust and professional software architecture** that correctly handles resource sharing, task synchronization, and hardware faults.

---

## 📖 Project Overview  

The system utilizes two independent sensors:  
- **Light Dependent Resistor (LDR)** → Detects ambient light (simulates tunnels).  
- **HC-SR04 Ultrasonic Sensor** → Measures distance to detect obstacles.  

Each sensor is managed by its own **dedicated FreeRTOS task**.  
Tasks run **concurrently**, providing:  
- Visual feedback via LEDs,  
- Real-time console output via UART,  
- Predictable and fault-tolerant execution flow.  

The **true complexity** and **educational value** of this project lie in the **architectural decisions** made to ensure the system is **stable, deterministic, and reliable**.  

---

## 🧩 Architectural Deep Dive & Concepts Demonstrated  

This project is a practical guide to **common RTOS challenges** and their solutions.  

### 1. Resource Management → Mutex to Solve Race Conditions  
- **Problem**: Both tasks attempted to write to the UART simultaneously → garbled, interleaved, unreadable output.  
- **Solution**: A **FreeRTOS Mutex** was introduced.  
  - Tasks must *lock* the mutex before writing.  
  - If UART is busy, tasks block without wasting CPU cycles.  
- **Outcome**: All UART messages are now atomic and **thread-safe**.  

---

### 2. Task Synchronization → Deterministic Execution with Notifications  
- **Problem**: Even with mutex protection, task order was random (e.g., LDR twice before Distance).  
- **Solution**: **Task Notifications** used as lightweight binary signals.  
  - Tasks follow a **ping-pong** scheme:  
    - `TaskLDR` notifies → `TaskDistance`  
    - `TaskDistance` notifies → `TaskLDR`  
- **Outcome**: Tasks execute in a strict alternating order:
  LDR → Distance → LDR → Distance → ...
Predictable and perfectly synchronized.  

---

### 3. System Robustness → Defensive Programming in Drivers  
- **Problem**: A faulty sensor (e.g., loose wire on HC-SR04) caused infinite loops, freezing the system.  
- **Solution**: Drivers were rewritten with **timeouts**.  
- ADC read (LDR) returns `0xFFFFFFFF` on timeout.  
- Distance read (HC-SR04) returns negative values (`-1`, `-2`) on timeout.  
- **Outcome**:  
- Errors are reported in console instead of freezing.  
- The system **remains alive** and continues notifying other tasks.  
- Faults can be recovered automatically when hardware returns to normal.  

---

## 🛠️ Hardware Requirements  (TM4C1294XL)

- **Microcontroller**: TI Tiva C Series TM4C1294XL LaunchPad  
- **Sensors**:  
- LDR with 10kΩ pull-down resistor  
- HC-SR04 ultrasonic distance sensor  
- **LEDs**:  
- PM3 → Tunnel indicator  
- PN1 → Proximity warning  
- PN0 → Heartbeat (system health)  
- **Other**: Jumper wires, breadboard  

---

## 💻 Software Requirements  

- **IDE**: Code Composer Studio (CCS) v8.0 or later  
- **SDK**: TivaWare for C Series (tested with v2.2.0.295)  
- **RTOS**: FreeRTOS (bundled in `third_party/FreeRTOS/`)  
- **Serial Terminal**: PuTTY, TeraTerm, or CCS terminal  
- **Settings**: `115200 baud, 8 data bits, 1 stop bit, no parity`  

---

## 🔌 Hardware Setup  

| Component      | Pin (Component) | Pin (TM4C1294XL) | Description |
|----------------|-----------------|-----------------|-------------|
| **HC-SR04**    | VCC             | +5V             | Power |
|                | GND             | GND             | Ground |
|                | TRIG            | PL5             | Trigger pulse |
|                | ECHO            | PL4             | Echo input |
| **LDR**        | Pin 1           | PE3 (AIN0)      | ADC input |
|                | Pin 2           | +3.3V           | Power |
| **Resistor**   | 10kΩ            | PE3–GND         | Pull-down |
| **LEDs**       | Tunnel LED      | PM3             | Tunnel detection |
|                | Warning LED     | PN1             | Proximity warning |
|                | Heartbeat LED   | PN0             | Scheduler alive |

⚠️ LEDs should be connected via **330Ω resistors**.  

---

## ⚙️ Step-by-Step Setup Guide  

1. **Clone Repository**  
   ```bash
   git clone <git@github.com:cetinss/tm4c-freertos-sensor-fusion-demo.git>
   ```

2. **Open Code Composer Studio (CCS)**  

3. **Import Project**  
   - Go to `File → Import → CCS Projects`  
   - Browse to the cloned repository folder  
   - The project will appear, select it, and click **Finish**  

4. **Build the Project**  
   - Right-click on the project in Project Explorer and select **Build Project**  
   - Or click the 🔨 (build) icon  

5. **Run the Project**  
   - Connect the TM4C1294XL board to your PC via USB (debug port)  
   - Click the 🐞 (Debug) button  
   - Once halted at `main()`, click ▶ (Resume) to run  

6. **Observe the Output**  
   - Open a UART terminal (PuTTY, Tera Term, or CCS Terminal)  
   - Set to **115200 baud, 8-N-1**  
   - Watch the real-time sensor data stream  
   - Verify the **heartbeat LED (PN0)** is blinking to confirm scheduler activity  

---

## 📟 Example Output  

```
Train Control System Initialized
--------------------------------
LDR Value: 95
Distance: 38.11 cm
LDR Value: 93
Distance: 14.53 cm
LDR Value: 88
...
```
---

## 📂 Code Structure  

- **main.c** → Initializes hardware, creates tasks, handles RTOS hooks  
- **train_system.c / train_system.h** → Hardware drivers (GPIO, ADC, Timer, UART)  
- **FreeRTOSConfig.h** → FreeRTOS kernel configuration (clock, heap, priorities)  
- **third_party/FreeRTOS/** → Unmodified FreeRTOS source code  

---

## 🔒 Core Concepts in Action  

- **Mutex (Semaphore)**  
  - Protects UART peripheral  
  - Prevents garbled/overlapping serial output  

- **Task Notifications**  
  - Deterministic synchronization ("ping-pong" execution order)  
  - Ensures LDR → Distance → LDR … sequence  

- **Defensive Timeouts**  
  - ADC and ultrasonic drivers include loop timeouts  
  - Prevents infinite blocking on hardware faults  

- **Idle Hook with Heartbeat LED**  
  - Visual system health monitor  
  - Confirms scheduler is always running  

- **Preemption & Scheduling**  
  - FreeRTOS manages multiple concurrent tasks efficientl
