import tkinter as tk
import math


class RobotTrajectoryGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Trajectory Planner - Circle/Spline Fit")

        # ===== CONFIG =====
        self.W = 900
        self.H = 700
        self.scale = 120.0  # 1m = 120px
        self.grid_size = 0.5  # meter

        # ===== CANVAS =====
        self.canvas = tk.Canvas(root, width=self.W, height=self.H, bg="white")
        self.canvas.pack()

        self.points = []
        self.last = None

        self.draw_grid()
        self.draw_axes()

        self.canvas.bind("<B1-Motion>", self.on_draw)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Motion>", self.show_mouse)

        tk.Button(root, text="EXPORT WAYPOINTS", command=self.export).pack(pady=10)
        tk.Button(root, text="CLEAR", command=self.clear).pack(pady=5)

        self.label = tk.Label(root, text="x: 0.00 m, y: 0.00 m")
        self.label.pack()

    # =========================
    # GRID
    # =========================
    def draw_grid(self):
        step = self.grid_size * self.scale

        for x in range(0, self.W, int(step)):
            self.canvas.create_line(x, 0, x, self.H, fill="#eeeeee")

        for y in range(0, self.H, int(step)):
            self.canvas.create_line(0, y, self.W, y, fill="#eeeeee")

    def draw_axes(self):
        self.canvas.create_line(self.W/2, 0, self.W/2, self.H, fill="black")
        self.canvas.create_line(0, self.H/2, self.W, self.H/2, fill="black")

    # =========================
    # DRAW
    # =========================
    def on_draw(self, e):
        x, y = e.x, e.y

        if self.last:
            self.canvas.create_line(self.last[0], self.last[1], x, y, fill="blue", width=2)

        self.points.append((x, y))
        self.last = (x, y)

    def on_release(self, e):
        self.last = None

    # =========================
    # MOUSE DISPLAY
    # =========================
    def show_mouse(self, e):
        x = (e.x - self.W/2) / self.scale
        y = (self.H/2 - e.y) / self.scale
        self.label.config(text=f"x: {x:.2f} m, y: {y:.2f} m")

    # =========================
    # CONVERT
    # =========================
    def to_meter(self, pts):
        out = []
        for x, y in pts:
            mx = (x - self.W/2) / self.scale
            my = (self.H/2 - y) / self.scale
            out.append((mx, my))
        return out

    def to_pixel(self, pts):
        out = []
        for x, y in pts:
            px = x * self.scale + self.W/2
            py = self.H/2 - y * self.scale
            out.extend([px, py])
        return out

    # =========================
    # CIRCLE FIT (Kasa)
    # =========================
    def fit_circle(self, pts):
        x = [p[0] for p in pts]
        y = [p[1] for p in pts]

        x_m = sum(x) / len(x)
        y_m = sum(y) / len(y)

        u = [xi - x_m for xi in x]
        v = [yi - y_m for yi in y]

        Suu = sum(ui**2 for ui in u)
        Svv = sum(vi**2 for vi in v)
        Suv = sum(ui * vi for ui, vi in zip(u, v))
        Suuu = sum(ui**3 for ui in u)
        Svvv = sum(vi**3 for vi in v)
        Suvv = sum(ui * vi**2 for ui, vi in zip(u, v))
        Svuu = sum(vi * ui**2 for ui, vi in zip(u, v))

        A = [[Suu, Suv],
             [Suv, Svv]]

        B = [0.5 * (Suuu + Suvv),
             0.5 * (Svvv + Svuu)]

        det = A[0][0]*A[1][1] - A[0][1]*A[1][0]
        if abs(det) < 1e-12:
            return None

        inv = [
            [A[1][1]/det, -A[0][1]/det],
            [-A[1][0]/det, A[0][0]/det]
        ]

        uc = inv[0][0]*B[0] + inv[0][1]*B[1]
        vc = inv[1][0]*B[0] + inv[1][1]*B[1]

        cx = uc + x_m
        cy = vc + y_m
        r = math.sqrt(uc**2 + vc**2 + (Suu + Svv)/len(x))

        return cx, cy, r

    # =========================
    # SPLINE (Catmull-Rom)
    # =========================
    def spline(self, pts, steps=15):
        if len(pts) < 4:
            return pts

        res = []

        def P(i):
            if i < 0:
                return pts[0]
            if i >= len(pts):
                return pts[-1]
            return pts[i]

        for i in range(len(pts)-1):
            p0, p1, p2, p3 = P(i-1), P(i), P(i+1), P(i+2)

            for t in range(steps):
                t /= steps

                x = 0.5 * (
                    (2*p1[0]) +
                    (-p0[0] + p2[0]) * t +
                    (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t*t +
                    (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t*t*t
                )

                y = 0.5 * (
                    (2*p1[1]) +
                    (-p0[1] + p2[1]) * t +
                    (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t*t +
                    (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t*t*t
                )

                res.append((x, y))

        return res

    # =========================
    # RESAMPLE (robot-friendly)
    # =========================
    def resample(self, pts, step=0.1):
        if len(pts) < 2:
            return pts

        res = [pts[0]]
        acc = 0.0

        for i in range(1, len(pts)):
            x1, y1 = pts[i-1]
            x2, y2 = pts[i]

            dx, dy = x2-x1, y2-y1
            dist = math.hypot(dx, dy)

            if dist == 0:
                continue

            while acc + dist >= step:
                t = (step - acc) / dist
                nx = x1 + t*dx
                ny = y1 + t*dy
                res.append((nx, ny))

                x1, y1 = nx, ny
                dx, dy = x2-x1, y2-y1
                dist = math.hypot(dx, dy)
                acc = 0

            acc += dist

        return res

    # =========================
    # EXPORT MAIN LOGIC
    # =========================
    def export(self):
        pts = self.to_meter(self.points)

        if len(pts) < 5:
            print("Not enough points")
            return

        circle = self.fit_circle(pts)

        if circle:
            cx, cy, r = circle

            err = sum(
                abs(math.hypot(x-cx, y-cy) - r)
                for x, y in pts
            ) / len(pts)

            print("Circle error:", err)

            if err < 0.15:
                print("\n>>> CIRCLE DETECTED")

                smooth = []
                for i in range(120):
                    t = 2 * math.pi * i / 120
                    x = cx + r * math.cos(t)
                    y = cy + r * math.sin(t)
                    smooth.append((x, y))
            else:
                print("\n>>> SPLINE MODE")
                smooth = self.spline(pts, steps=20)
                smooth = self.resample(smooth, step=0.1)
        else:
            smooth = self.spline(pts, steps=20)
            smooth = self.resample(smooth, step=0.1)

        print("\n=== FINAL WAYPOINTS (m) ===")
        for p in smooth:
            print(p)

        self.canvas.create_line(self.to_pixel(smooth), fill="red", width=3)

    # =========================
    def clear(self):
        self.canvas.delete("all")
        self.points = []
        self.last = None
        self.draw_grid()
        self.draw_axes()


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotTrajectoryGUI(root)
    root.mainloop()