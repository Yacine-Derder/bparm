import time
import threading
import tkinter as tk
from inputs import get_gamepad, UnpluggedError
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import os
import signal

JOYSTICK_SCALE = 0.02  # Constant increment per frame

class SliderGUI(Node):
    def __init__(self, use_gui=True):
        super().__init__('slider_gui_node')
        self.use_gui = use_gui
        self.sliders = {}
        self.gamepad_warning_issued = False
        self.is_running = True
        self.is_gripped = False
        self.is_on = False

        self.layout = [
            ("Platform Yaw", (-2.356, 2.356)),
            ("Shoulder Pitch", (-1.9, 1.9)),
            ("Shoulder Yaw", (-2.356, 2.356)),
            ("Elbow Pitch", (-2.356, 2.356)),
            ("Wrist Yaw", (-2.356, 2.356)),
            ("Wrist Pitch", (-2.356, 2.356)),
        ]

        self.joystick_flags = {
            f"{label} {sign}": False
            for label, _ in self.layout
            for sign in ("+", "-")
        }

        self.slider_values = {}
        for label, (min_val, max_val) in self.layout:
            self.slider_values[label] = 0.0

        self.publisher_ = self.create_publisher(String, 'joint_values', 10)
        self.flag_publisher_ = self.create_publisher(String, 'control_flags', 10)

        threading.Thread(target=self.poll_gamepad_thread, daemon=True).start()
        threading.Thread(target=self.publish_loop, daemon=True).start()

        if self.use_gui:
            self.init_gui()
            self.root.after(50, self.apply_joystick_loop)
        else:
            threading.Thread(target=self.headless_joystick_loop, daemon=True).start()

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("Slider Control")

        row = 0
        for label_text, (range_min, range_max) in self.layout:
            label = tk.Label(self.root, text=f"{label_text} [{range_min}, {range_max}]", font=("Arial", 12))
            label.grid(row=row, column=0, sticky="w", padx=10, pady=5)

            slider = tk.Scale(
                self.root,
                from_=range_min,
                to=range_max,
                orient=tk.HORIZONTAL,
                length=600,
                resolution=0.001,
                font=("Arial", 10)
            )
            slider.set(0.0)
            slider.grid(row=row, column=1, padx=10, pady=5, sticky="w")
            self.sliders[label_text] = slider

            # ✅ Bind each slider individually here
            slider.bind("<B1-Motion>", lambda e, lbl=label_text: self.on_slider_change(lbl))
            slider.bind("<ButtonRelease-1>", lambda e, lbl=label_text: self.on_slider_change(lbl))

            row += 1

        button_frame = tk.Frame(self.root)
        button_frame.grid(row=row, column=1, pady=15, sticky="e")

        reset_btn = tk.Button(button_frame, text="Reset All", command=self.reset_all, font=("Arial", 12))
        reset_btn.pack(side=tk.LEFT, padx=10)

        quit_btn = tk.Button(button_frame, text="Quit", command=self.safe_exit, font=("Arial", 12))
        quit_btn.pack(side=tk.LEFT)

        toggle_frame = tk.Frame(self.root)
        toggle_frame.grid(row=row + 1, column=0, columnspan=2, pady=10)

        self.release_btn = tk.Button(toggle_frame, text="Release", width=10, command=self.set_toggle_release, font=("Arial", 12))
        self.release_btn.pack(side=tk.LEFT, padx=10)

        self.catch_btn = tk.Button(toggle_frame, text="Catch", width=10, command=self.set_toggle_catch, font=("Arial", 12))
        self.catch_btn.pack(side=tk.LEFT, padx=10)

        self.on_off_btn = tk.Button(toggle_frame, text="Turn On", width=10, command=self.toggle_on_off, font=("Arial", 12))
        self.on_off_btn.pack(side=tk.LEFT, padx=10)

        slider.bind("<B1-Motion>", lambda e, lbl=label_text: self.on_slider_change(lbl))
        slider.bind("<ButtonRelease-1>", lambda e, lbl=label_text: self.on_slider_change(lbl))

        self.update_toggle_button_styles()
        self.update_on_off_button_style()

    def on_slider_change(self, label):
        if label in self.sliders:
            self.slider_values[label] = self.sliders[label].get()

    def update_toggle_button_styles(self):
        if not self.use_gui:
            return
        self.catch_btn.config(relief=tk.SUNKEN if self.is_gripped else tk.RAISED)
        self.release_btn.config(relief=tk.RAISED if self.is_gripped else tk.SUNKEN)

    def update_on_off_button_style(self):
        if self.use_gui:
            self.on_off_btn.config(text="Turn Off" if self.is_on else "Turn On")

    def toggle_on_off(self):
        self.is_on = not self.is_on
        self.get_logger().info(f"System turned {'ON' if self.is_on else 'OFF'}")
        self.update_on_off_button_style()

    def set_toggle_catch(self):
        self.is_gripped = True
        self.get_logger().info("Toggle set to Catch")
        self.update_toggle_button_styles()

    def set_toggle_release(self):
        self.is_gripped = False
        self.get_logger().info("Toggle set to Release")
        self.update_toggle_button_styles()

    def poll_gamepad_thread(self):
        while rclpy.ok():
            try:
                events = get_gamepad()
                if self.gamepad_warning_issued:
                    self.get_logger().info("Gamepad connected")
                    self.gamepad_warning_issued = False
                for e in events:
                    if e.ev_type == "Absolute":
                        val = e.state
                        if e.code == "ABS_X":
                            self.joystick_flags["Platform Yaw +"] = val > 16384
                            self.joystick_flags["Platform Yaw -"] = val < -16384
                        elif e.code == "ABS_Y":
                            self.joystick_flags["Shoulder Pitch -"] = val > 16384
                            self.joystick_flags["Shoulder Pitch +"] = val < -16384
                        elif e.code == "ABS_HAT0X":
                            self.joystick_flags["Shoulder Yaw +"] = val > 0
                            self.joystick_flags["Shoulder Yaw -"] = val < 0
                        elif e.code == "ABS_RX":
                            self.joystick_flags["Elbow Pitch +"] = val > 16384
                            self.joystick_flags["Elbow Pitch -"] = val < -16384
                        elif e.code == "ABS_RY":
                            self.joystick_flags["Wrist Yaw -"] = val > 16384
                            self.joystick_flags["Wrist Yaw +"] = val < -16384
                        elif e.code == "ABS_HAT0Y":
                            self.joystick_flags["Wrist Pitch -"] = val > 0
                            self.joystick_flags["Wrist Pitch +"] = val < 0
                    elif e.ev_type == "Key":
                        if e.code == "BTN_START" and e.state == 1:
                            self.get_logger().info("MENU (START) pressed → Resetting sliders")
                            self.reset_all()
                        elif e.code == "BTN_MODE" and e.state == 1:
                            self.get_logger().info("HOME button pressed → Exiting")
                            self.safe_exit()
                        elif e.code == "BTN_TL" and e.state == 1:
                            self.set_toggle_release()
                        elif e.code == "BTN_TR" and e.state == 1:
                            self.set_toggle_catch()
                        elif e.code == "BTN_SELECT" and e.state == 1:
                            self.toggle_on_off()
            except Exception:
                if not self.gamepad_warning_issued:
                    self.get_logger().warn("Gamepad not found or unplugged")
                    self.gamepad_warning_issued = True
            time.sleep(0.01)

    def headless_joystick_loop(self):
        while self.is_running and rclpy.ok():
            self.apply_joystick_loop()
            time.sleep(0.05)  # 50 ms

    def apply_joystick_loop(self):
        for label, _ in self.layout:
            if self.joystick_flags.get(f"{label} +"):
                self.integrate_slider(label, JOYSTICK_SCALE)
            if self.joystick_flags.get(f"{label} -"):
                self.integrate_slider(label, -JOYSTICK_SCALE)

        if self.use_gui:
            self.root.after(50, self.apply_joystick_loop)

    def integrate_slider(self, label, delta):
        # Always update slider_values dict
        min_val, max_val = dict(self.layout)[label]
        new_val = max(min(self.slider_values[label] + delta, max_val), min_val)
        self.slider_values[label] = new_val

        # Also update GUI slider if using GUI
        if self.use_gui and label in self.sliders:
            self.sliders[label].set(new_val)

    def reset_all(self):
        for label, _ in self.layout:
            self.slider_values[label] = 0.0
            if self.use_gui and label in self.sliders:
                self.sliders[label].set(0.0)

    def safe_exit(self):
        if hasattr(self, 'publish_timer') and self.publish_timer is not None:
            self.publish_timer.cancel()
        try:
            self.is_running = False
            self.is_on = False
            self.publish_slider_values()
            if self.use_gui and self.root.winfo_exists():
                self.root.destroy()
            rclpy.shutdown()

            # Terminate the whole launch system
            os.killpg(os.getpgid(os.getpid()), signal.SIGTERM)
        except:
            pass

    def publish_slider_values(self):
        joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        try:
            joint_dict = {joint_names[i]: self.slider_values[label] for i, (label, _) in enumerate(self.layout)}
            joint_msg = String()
            joint_msg.data = json.dumps(joint_dict)
            self.publisher_.publish(joint_msg)

            flags_dict = {
                "is_gripped": self.is_gripped,
                "is_on": self.is_on
            }
            flag_msg = String()
            flag_msg.data = json.dumps(flags_dict)
            self.flag_publisher_.publish(flag_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish: {e}")

    def publish_loop(self):
        while self.is_running and rclpy.ok():
            self.publish_slider_values()
            time.sleep(0.05)

    @property
    def layout_dict(self):
        return dict(self.layout)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('slider_gui_arg_parser')

    use_gui = node.declare_parameter('use_gui', True).get_parameter_value().bool_value
    node.destroy_node()

    app = SliderGUI(use_gui=use_gui)
    try:
        if use_gui:
            app.root.mainloop()
        else:
            while rclpy.ok() and app.is_running:
                time.sleep(0.1)
    except KeyboardInterrupt:
        app.get_logger().info("KeyboardInterrupt received. Exiting...")
        app.safe_exit()
    except Exception as e:
        app.get_logger().error(f"Exception occurred: {e}")
        app.safe_exit()
    finally:
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
