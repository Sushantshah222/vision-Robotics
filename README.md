
# Vision-Robotics 🖐️🤖💡

Hand-gesture controlled LED/Relay project using **OpenCV**, **MediaPipe**, and **Arduino Uno**.  
This project lets you control an LED (or a relay module) simply by moving your **index finger** in front of a webcam.

---

## 📂 Project Structure



---

## ⚡ Features
- **Virtual Button Mode (pressing.py):**
  - Two on-screen buttons: **ON** (left) and **OFF** (right)
  - Hover with index fingertip + “poke” motion to press
  - LED/Relay toggles just like a real switch
- **Cross-platform:** Works on macOS, Linux, and Windows
- **Arduino Integration:** Controls an LED or relay via serial (USB)

---

## 🛠️ Requirements

- Python **3.10+**
- Arduino Uno (or compatible board)
- USB cable
- LED + resistor (or relay module)

Python libraries:
```bash
pip install opencv-python mediapipe pyserial
```


## ▶️ Running the Code

1. Plug Arduino into your computer and upload Arduino code into it.
2. Close arduino IDE and shift towards Python Code.
3. Find your serial port:  

```bash
   ls /dev/cu.*    # macOS
   ls /dev/ttyUSB* # Linux
```
Example: /dev/tty.usbserial-1130

4. Update SERIAL_PORT in the script (e.g., pressing.py).
5. Run:

```bash
   uv run pressing.py
```
6. Use your finger gestures or press the virtual buttons on screen!

![2](https://github.com/user-attachments/assets/41f03ecd-d50e-49d5-9044-535c9afc023d)
