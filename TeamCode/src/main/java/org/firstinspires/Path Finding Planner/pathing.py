import tkinter as tk
from tkinter import simpledialog, messagebox, filedialog
import json, math, time, bisect

# ---------- Data Structures ----------
class Waypoint:
    def __init__(self, x, y):
        self.x = x      # in mm
        self.y = y      # in mm
        self.actions = []   # generic actions (e.g., autoIntakeAndDepositPiece)
        self.rotate_deg = None  # if set, rotation (in degrees) to execute at this checkpoint

# ---------- Field Canvas with Enhanced Simulation ----------
class FieldCanvas(tk.Canvas):
    def __init__(self, master, field_width, field_height, scale, *args, **kwargs):
        """
        field_width, field_height: dimensions in mm.
        scale: pixels per mm.
        """
        super().__init__(master, *args, **kwargs)
        self.field_width = field_width
        self.field_height = field_height
        self.scale = scale
        self.mode = "Path"            # "Path" or "Overlay"
        self.waypoints = []           # List of Waypoint objects
        self.custom_overlays = []     # List of custom overlay shapes
        self.overlay_template = "None"  # "None" or "Standard FTC"
        self.overlay_start = None     # For drawing custom overlays
        # Robot dimensions for drawing/simulation:
        self.robot_width = 0
        self.robot_height = 0
        # Simulation state:
        self.sim_points = []          # List of (x, y, angle) simulation points
        self.sim_time_profile = []    # Corresponding cumulative times (seconds)
        self.sim_total_time = 0
        self.sim_start_time = 0
        self.simulation_running = False
        self.current_simulated_point = None
        # Bind mouse events:
        self.bind("<Button-1>", self.on_left_click)
        self.bind("<Button-3>", self.on_right_click)
        self.draw_field()

    def draw_field(self):
        self.delete("all")
        # Draw field boundary:
        self.create_rectangle(0, 0, self.field_width * self.scale, self.field_height * self.scale,
                              outline="black", width=2)
        # Center line (example FTC marking):
        self.create_line(0, self.field_height * self.scale/2,
                         self.field_width * self.scale, self.field_height * self.scale/2,
                         fill="red", dash=(4,2))
        # Draw overlay template if selected:
        if self.overlay_template == "Standard FTC":
            self.draw_standard_overlay()
        # Draw any custom overlays:
        self.draw_custom_overlays()
        # Draw path (waypoints and connecting line):
        if self.waypoints:
            for wp in self.waypoints:
                self.draw_waypoint(wp)
            if len(self.waypoints) > 1:
                self.draw_path()
        # If simulation is running, draw the robot:
        if self.simulation_running and self.sim_points:
            self.draw_simulated_robot()

    def draw_standard_overlay(self):
        # Example overlay: alliance zones and scoring areas
        self.create_rectangle(0, 0, self.field_width * self.scale, self.field_height * self.scale/2,
                              outline="blue", width=2, dash=(2,2))
        self.create_rectangle(0, self.field_height * self.scale/2,
                              self.field_width * self.scale, self.field_height * self.scale,
                              outline="green", width=2, dash=(2,2))
        center_top = (self.field_width * self.scale/2, self.field_height * self.scale/4)
        center_bot = (self.field_width * self.scale/2, 3 * self.field_height * self.scale/4)
        r = 30
        self.create_oval(center_top[0]-r, center_top[1]-r, center_top[0]+r, center_top[1]+r,
                         outline="purple", width=2)
        self.create_oval(center_bot[0]-r, center_bot[1]-r, center_bot[0]+r, center_bot[1]+r,
                         outline="purple", width=2)

    def draw_custom_overlays(self):
        for overlay in self.custom_overlays:
            if overlay["type"] == "line":
                start = overlay["start"]
                end = overlay["end"]
                self.create_line(start[0]*self.scale, start[1]*self.scale,
                                 end[0]*self.scale, end[1]*self.scale,
                                 fill="brown", width=2, dash=(3,3))

    def draw_waypoint(self, wp):
        r = 5
        x = wp.x * self.scale
        y = wp.y * self.scale
        self.create_oval(x-r, y-r, x+r, y+r, fill="blue")
        # Mark if actions exist:
        if wp.actions or wp.rotate_deg is not None:
            self.create_text(x, y-10, text="*", fill="darkgreen", font=("Arial", 10, "bold"))

    def draw_path(self):
        pts = []
        for wp in self.waypoints:
            pts.extend([wp.x*self.scale, wp.y*self.scale])
        self.create_line(pts, fill="green", width=2, smooth=True)

    def on_left_click(self, event):
        x_mm = event.x/self.scale
        y_mm = event.y/self.scale
        if self.mode == "Path":
            new_wp = Waypoint(x_mm, y_mm)
            self.waypoints.append(new_wp)
        elif self.mode == "Overlay":
            if self.overlay_start is None:
                self.overlay_start = (x_mm, y_mm)
            else:
                overlay = {"type": "line", "start": self.overlay_start, "end": (x_mm, y_mm)}
                self.custom_overlays.append(overlay)
                self.overlay_start = None
        self.draw_field()

    def on_right_click(self, event):
        # In "Path" mode, right-click near a waypoint shows a context menu
        if self.mode == "Path":
            x_mm = event.x/self.scale
            y_mm = event.y/self.scale
            for wp in self.waypoints:
                if abs(wp.x*self.scale - event.x) < 10 and abs(wp.y*self.scale - event.y) < 10:
                    self.show_context_menu(event, wp)
                    break

    def show_context_menu(self, event, waypoint):
        menu = tk.Menu(self, tearoff=0)
        menu.add_command(label="Add Action", command=lambda: self.add_action_to_waypoint(waypoint))
        menu.add_command(label="Rotate...", command=lambda: self.add_rotate_to_waypoint(waypoint))
        menu.post(event.x_root, event.y_root)

    def add_action_to_waypoint(self, waypoint):
        action = simpledialog.askstring("Add Action", "Enter action name (e.g., autoIntakeAndDepositPiece):")
        if action:
            param = simpledialog.askstring("Action Parameter", "Enter parameter (if any):")
            waypoint.actions.append({"name": action, "value": param})
            self.draw_field()

    def add_rotate_to_waypoint(self, waypoint):
        deg_str = simpledialog.askstring("Rotate", "Enter rotation (in degrees):")
        try:
            deg = float(deg_str)
            waypoint.rotate_deg = deg  # store as a rotation command (in degrees)
            self.draw_field()
        except (ValueError, TypeError):
            messagebox.showerror("Error", "Please enter a valid number of degrees.")

    def export_path(self):
        path_data = {
            "fieldDimensions": {"width": self.field_width, "height": self.field_height},
            "robotDimensions": {"width": self.robot_width, "height": self.robot_height},
            "waypoints": []
        }
        for wp in self.waypoints:
            data = {"x": wp.x, "y": wp.y, "actions": wp.actions}
            if wp.rotate_deg is not None:
                data["rotate_deg"] = wp.rotate_deg
            path_data["waypoints"].append(data)
        if self.custom_overlays:
            path_data["customOverlays"] = self.custom_overlays
        return json.dumps(path_data, indent=4)

    # ----- Simulation Physics Model -----
    def compute_simulation_profile(self, sim_params):
        """
        Build a simulation profile that includes linear segments and inserted turning segments.
        sim_params should include:
           robot_weight, wheel_diameter, motor_torque, motor_rpm, gear_ratio,
           friction_coefficient, turning_friction_coefficient.
        Returns a tuple (points, time_profile, total_time).
        """
        points = []
        time_profile = [0]
        total_time = 0

        # For each segment between waypoints, simulate linear movement.
        for i in range(len(self.waypoints)-1):
            start_wp = self.waypoints[i]
            end_wp = self.waypoints[i+1]
            seg_pts, seg_times = self.simulate_linear_segment(start_wp, end_wp, sim_params)
            # Append, avoiding duplicate of the first point (except for the first segment)
            if i > 0:
                seg_pts = seg_pts[1:]
                seg_times = seg_times[1:]
            # Append linear segment:
            points.extend(seg_pts)
            # Adjust times:
            seg_times = [t + total_time for t in seg_times]
            time_profile.extend(seg_times[1:])
            total_time = time_profile[-1]
            # If the end waypoint has a rotate command, insert a turning segment.
            if end_wp.rotate_deg is not None:
                turn_pts, turn_times = self.simulate_turn_segment(end_wp, end_wp.rotate_deg, sim_params)
                # Append turning segment (skip duplicate point)
                turn_pts = turn_pts[1:]
                turn_times = [t + total_time for t in turn_times[1:]]
                points.extend(turn_pts)
                time_profile.extend(turn_times)
                total_time = time_profile[-1]
        return points, time_profile, total_time

    def simulate_linear_segment(self, start_wp, end_wp, sim_params):
        """
        Simulate a linear (drive) segment using a trapezoidal velocity profile.
        Returns (points, times) where points is a list of (x, y, angle) and times in seconds.
        """
        # Extract simulation parameters:
        mass = sim_params["robot_weight"]       # kg
        wheel_diam = sim_params["wheel_diameter"] # mm
        motor_torque = sim_params["motor_torque"] # Nm
        motor_rpm = sim_params["motor_rpm"]
        gear_ratio = sim_params["gear_ratio"]
        mu = sim_params["friction_coefficient"]
        # Compute constants:
        wheel_radius = (wheel_diam/2)/1000       # in meters
        wheel_circ = math.pi * wheel_diam         # mm
        v_no_load = (motor_rpm/60)*wheel_circ*gear_ratio  # mm/s
        F_motor_max = motor_torque * gear_ratio / wheel_radius  # Newtons
        F_friction = mu * mass * 9.81             # Newtons
        a_max = max(0, (F_motor_max - F_friction)/mass)*1000  # mm/s²
        # Determine straight-line distance and angle:
        dx = end_wp.x - start_wp.x
        dy = end_wp.y - start_wp.y
        distance = math.hypot(dx, dy)
        seg_angle = math.atan2(dy, dx)
        # Use a trapezoidal profile:
        t_accel = v_no_load / a_max if a_max > 0 else 0
        d_accel = 0.5 * a_max * t_accel**2
        if 2*d_accel > distance:
            # Triangle profile (never reach v_no_load)
            t_accel = math.sqrt(distance / a_max)
            t_total = 2 * t_accel
            v_peak = a_max * t_accel
        else:
            d_const = distance - 2*d_accel
            t_const = d_const / v_no_load
            t_total = 2*t_accel + t_const
            v_peak = v_no_load
        # Sample the segment:
        dt = 0.05  # time step (s)
        num_steps = int(t_total/dt) + 1
        pts = []
        times = []
        for i in range(num_steps):
            t = i * dt
            if t < t_accel:
                # Acceleration phase:
                s = 0.5 * a_max * t**2
            elif t < t_total - t_accel:
                s = d_accel + v_peak*(t - t_accel)
            else:
                t_dec = t - (t_total - t_accel)
                s = distance - 0.5 * a_max * (t_accel - t_dec)**2
            # Compute point along line:
            frac = s/distance if distance > 0 else 0
            x = start_wp.x + dx*frac
            y = start_wp.y + dy*frac
            pts.append((x, y, seg_angle))
            times.append(t)
        # Ensure last point is exactly the endpoint:
        pts[-1] = (end_wp.x, end_wp.y, seg_angle)
        times[-1] = t_total
        return pts, times

    def simulate_turn_segment(self, waypoint, rotate_deg, sim_params):
        """
        Simulate a turning segment at a waypoint.
        rotate_deg is the commanded rotation (in degrees) relative to current orientation.
        Returns (points, times) where points are (x, y, angle). The position remains fixed.
        """
        # For turning dynamics, extract parameters:
        mass = sim_params["robot_weight"]
        motor_torque = sim_params["motor_torque"]
        motor_rpm = sim_params["motor_rpm"]
        gear_ratio = sim_params["gear_ratio"]
        mu_turn = sim_params["turning_friction_coefficient"]
        # Estimate robot moment of inertia (rectangular plate):
        width = self.robot_width  # mm
        height = self.robot_height
        I = (mass/12)*(((width)/1000)**2+((height)/1000)**2)  # kg·m²
        T_motor_max = motor_torque * gear_ratio  # Nm
        T_friction = mu_turn * mass * 9.81 * (width/2000)  # approximate friction torque
        net_torque = max(0, T_motor_max - T_friction)
        a_max_turn = net_torque / I if I > 0 else 0  # rad/s²
        if a_max_turn == 0:
            a_max_turn = 0.1  # fallback
        # Determine turning angle in radians:
        delta = math.radians(rotate_deg)
        delta = delta if delta >= 0 else -abs(delta)  # preserve sign
        delta_abs = abs(delta)
        # Compute time for a symmetric (bang-bang) profile:
        t_turn = 2 * math.sqrt(delta_abs / a_max_turn)
        dt = 0.05
        num_steps = int(t_turn/dt) + 1
        pts = []
        times = []
        # Assume current orientation is the angle from the linear segment at this waypoint.
        # (For simplicity, we assume the turning starts from that angle.)
        current_angle = 0
        if self.waypoints:
            # Use the last computed linear segment angle if available.
            current_angle = math.atan2(0,1)  # default 0 if not available
        # Here, we let the turning start at the waypoint's current orientation.
        for i in range(num_steps):
            t = i * dt
            if t < t_turn/2:
                ang = 0.5 * a_max_turn * t**2
            else:
                t_dec = t - t_turn/2
                ang_peak = 0.5 * a_max_turn * (t_turn/2)**2
                ang = ang_peak + a_max_turn*(t_turn/2)*t_dec - 0.5*a_max_turn*t_dec**2
            ang = ang if delta >= 0 else -ang
            pts.append((waypoint.x, waypoint.y, current_angle + ang))
            times.append(t)
        # Ensure final orientation exactly equals commanded rotation:
        pts[-1] = (waypoint.x, waypoint.y, current_angle + delta)
        times[-1] = t_turn
        return pts, times

    def start_simulation(self, sim_params):
        profile = self.compute_simulation_profile(sim_params)
        if not profile:
            messagebox.showerror("Error", "Simulation profile could not be computed. Ensure at least two waypoints exist.")
            return
        self.sim_points, self.sim_time_profile, self.sim_total_time = profile
        self.sim_start_time = time.time()
        self.simulation_running = True
        self.animate_simulation()

    def animate_simulation(self):
        if not self.simulation_running:
            return
        elapsed = time.time() - self.sim_start_time
        if elapsed > self.sim_total_time:
            self.simulation_running = False
            self.draw_field()
            return
        # Find index corresponding to elapsed time
        i = bisect.bisect_left(self.sim_time_profile, elapsed)
        if i >= len(self.sim_time_profile):
            i = len(self.sim_time_profile) - 1
        # Interpolate between simulation points if needed:
        if i == 0:
            current_pt = self.sim_points[0]
        else:
            t0 = self.sim_time_profile[i-1]
            t1 = self.sim_time_profile[i]
            frac = (elapsed - t0) / (t1 - t0) if t1 > t0 else 0
            p0 = self.sim_points[i-1]
            p1 = self.sim_points[i]
            x = p0[0] + (p1[0]-p0[0])*frac
            y = p0[1] + (p1[1]-p0[1])*frac
            angle = p0[2] + (p1[2]-p0[2])*frac
            current_pt = (x, y, angle)
        self.current_simulated_point = current_pt
        self.draw_field()
        self.after(20, self.animate_simulation)

    def draw_simulated_robot(self):
        if not self.current_simulated_point:
            return
        x_mm, y_mm, angle = self.current_simulated_point
        x = x_mm * self.scale
        y = y_mm * self.scale
        w = self.robot_width * self.scale
        h = self.robot_height * self.scale
        # Compute rectangle corners (rotated)
        corners = [(-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2)]
        rotated = []
        for cx, cy in corners:
            rx = cx * math.cos(angle) - cy * math.sin(angle)
            ry = cx * math.sin(angle) + cy * math.cos(angle)
            rotated.append((x+rx, y+ry))
        pts = []
        for corner in rotated:
            pts.extend(corner)
        self.create_polygon(pts, outline="orange", fill="", width=2)
        # Forward arrow:
        arrow_length = 20
        fx = x + arrow_length * math.cos(angle)
        fy = y + arrow_length * math.sin(angle)
        self.create_line(x, y, fx, fy, fill="red", width=2, arrow=tk.LAST)

# ---------- Main Application ----------
class FTCPathPlannerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("FTC Autonomous Path Planner")
        self.field_width = 3600   # mm
        self.field_height = 3600  # mm
        self.scale = 0.1          # pixels per mm (e.g., 360x360 canvas)
        self.canvas = FieldCanvas(self, self.field_width, self.field_height, self.scale,
                                  width=self.field_width*self.scale, height=self.field_height*self.scale, bg="white")
        self.canvas.pack(side=tk.LEFT)
        control_frame = tk.Frame(self)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)
        # Robot dimensions:
        tk.Label(control_frame, text="Robot Dimensions (mm):").pack(anchor="w")
        tk.Label(control_frame, text="Width:").pack(anchor="w")
        self.robot_width_entry = tk.Entry(control_frame)
        self.robot_width_entry.pack(anchor="w")
        tk.Label(control_frame, text="Height:").pack(anchor="w")
        self.robot_height_entry = tk.Entry(control_frame)
        self.robot_height_entry.pack(anchor="w")
        self.robot_width_entry.insert(0, "350")
        self.robot_height_entry.insert(0, "400")
        tk.Button(control_frame, text="Update Robot Dimensions", command=self.update_robot_dimensions).pack(pady=5, anchor="w")
        # Drawing mode:
        tk.Label(control_frame, text="Drawing Mode:").pack(anchor="w")
        self.mode_var = tk.StringVar(value="Path")
        mode_menu = tk.OptionMenu(control_frame, self.mode_var, "Path", "Overlay", command=self.change_mode)
        mode_menu.pack(anchor="w", pady=5)
        # Overlay template:
        tk.Label(control_frame, text="Field Overlay Template:").pack(anchor="w")
        self.overlay_var = tk.StringVar(value="None")
        overlay_menu = tk.OptionMenu(control_frame, self.overlay_var, "None", "Standard FTC", command=self.update_overlay)
        overlay_menu.pack(anchor="w", pady=5)
        # Custom overlay save/load:
        tk.Button(control_frame, text="Save Custom Overlay", command=self.save_custom_overlay).pack(pady=5, anchor="w")
        tk.Button(control_frame, text="Load Custom Overlay", command=self.load_custom_overlay).pack(pady=5, anchor="w")
        # Simulation Parameters:
        sim_frame = tk.LabelFrame(control_frame, text="Simulation Parameters", padx=5, pady=5)
        sim_frame.pack(fill="x", pady=10)
        tk.Label(sim_frame, text="Robot Weight (kg):").grid(row=0, column=0, sticky="w")
        self.weight_entry = tk.Entry(sim_frame)
        self.weight_entry.grid(row=0, column=1)
        tk.Label(sim_frame, text="Wheel Diameter (mm):").grid(row=1, column=0, sticky="w")
        self.wheel_entry = tk.Entry(sim_frame)
        self.wheel_entry.grid(row=1, column=1)
        tk.Label(sim_frame, text="Motor Torque (Nm):").grid(row=2, column=0, sticky="w")
        self.torque_entry = tk.Entry(sim_frame)
        self.torque_entry.grid(row=2, column=1)
        tk.Label(sim_frame, text="Motor RPM:").grid(row=3, column=0, sticky="w")
        self.rpm_entry = tk.Entry(sim_frame)
        self.rpm_entry.grid(row=3, column=1)
        tk.Label(sim_frame, text="Gear Ratio:").grid(row=4, column=0, sticky="w")
        self.gear_entry = tk.Entry(sim_frame)
        self.gear_entry.grid(row=4, column=1)
        tk.Label(sim_frame, text="Friction Coefficient:").grid(row=5, column=0, sticky="w")
        self.friction_entry = tk.Entry(sim_frame)
        self.friction_entry.grid(row=5, column=1)
        tk.Label(sim_frame, text="Turning Friction Coefficient:").grid(row=6, column=0, sticky="w")
        self.turn_friction_entry = tk.Entry(sim_frame)
        self.turn_friction_entry.grid(row=6, column=1)
        # Defaults:
        self.weight_entry.insert(0, "20")
        self.wheel_entry.insert(0, "100")
        self.torque_entry.insert(0, "0.5")
        self.rpm_entry.insert(0, "300")
        self.gear_entry.insert(0, "1")
        self.friction_entry.insert(0, "0.05")
        self.turn_friction_entry.insert(0, "0.2")
        tk.Button(sim_frame, text="Simulate Path (Physics)", command=self.simulate_path).grid(row=7, column=0, columnspan=2, pady=5)
        # Export and Clear:
        tk.Button(control_frame, text="Export Path as JSON", command=self.export_json).pack(pady=5, anchor="w")
        tk.Button(control_frame, text="Clear Path", command=self.clear_path).pack(pady=5, anchor="w")

    def update_robot_dimensions(self):
        try:
            width = float(self.robot_width_entry.get())
            height = float(self.robot_height_entry.get())
            self.canvas.robot_width = width
            self.canvas.robot_height = height
            messagebox.showinfo("Info", f"Robot dimensions updated: {width} mm × {height} mm")
            self.canvas.draw_field()
        except ValueError:
            messagebox.showerror("Error", "Enter valid numeric robot dimensions.")

    def change_mode(self, value):
        self.canvas.mode = value
        self.canvas.overlay_start = None
        self.canvas.draw_field()

    def update_overlay(self, value):
        self.canvas.overlay_template = value
        self.canvas.draw_field()

    def save_custom_overlay(self):
        file_path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON files", "*.json")])
        if file_path:
            with open(file_path, "w") as f:
                json.dump(self.canvas.custom_overlays, f, indent=4)
            messagebox.showinfo("Saved", f"Custom overlay saved to {file_path}")

    def load_custom_overlay(self):
        file_path = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")])
        if file_path:
            with open(file_path, "r") as f:
                self.canvas.custom_overlays = json.load(f)
            messagebox.showinfo("Loaded", f"Custom overlay loaded from {file_path}")
            self.canvas.draw_field()

    def simulate_path(self):
        try:
            sim_params = {
                "robot_weight": float(self.weight_entry.get()),
                "wheel_diameter": float(self.wheel_entry.get()),
                "motor_torque": float(self.torque_entry.get()),
                "motor_rpm": float(self.rpm_entry.get()),
                "gear_ratio": float(self.gear_entry.get()),
                "friction_coefficient": float(self.friction_entry.get()),
                "turning_friction_coefficient": float(self.turn_friction_entry.get())
            }
        except ValueError:
            messagebox.showerror("Error", "Enter valid numeric simulation parameters.")
            return
        self.update_robot_dimensions()
        self.canvas.start_simulation(sim_params)

    def export_json(self):
        json_data = self.canvas.export_path()
        file_path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON files", "*.json")])
        if file_path:
            with open(file_path, "w") as f:
                f.write(json_data)
            messagebox.showinfo("Exported", f"Path exported to {file_path}")

    def clear_path(self):
        self.canvas.waypoints = []
        self.canvas.custom_overlays = []
        self.canvas.draw_field()

if __name__ == "__main__":
    app = FTCPathPlannerApp()
    app.mainloop()
