#!/usr/bin/env python3
"""
Ball-on-Plate Path Creator
Combines drawing interface and Bézier curve fitting into one program.
"""

import tkinter as tk
import csv
import numpy as np
from scipy.optimize import least_squares
from tkinter import simpledialog

# ==================== DRAWING INTERFACE ====================

class PathDrawer:
    def __init__(self, root, size=500):
        self.root = root
        self.root.title("Draw a Path for Ball-on-Plate")
        self.size = size
        
        # Get coordinate range from user
        self.coord_range = simpledialog.askfloat(
            "Coordinate Range", 
            "Enter coordinate range (e.g., 0.15 for ±15cm):",
            initialvalue=0.15
        )
        
        if self.coord_range is None:
            self.coord_range = 0.15
        
        # Canvas for drawing
        self.canvas = tk.Canvas(root, bg="white", width=size, height=size)
        self.canvas.pack(fill="both", expand=True)
        
        # Instructions
        instructions = tk.Label(
            root, 
            text="Draw your path with the mouse. Release to finish.",
            font=("Arial", 12)
        )
        instructions.pack()
        
        # Store stroke points
        self.points = []
        self.last_x, self.last_y = None, None
        
        # Bind mouse events
        self.canvas.bind("<Button-1>", self.start_stroke)
        self.canvas.bind("<B1-Motion>", self.draw_stroke)
        self.canvas.bind("<ButtonRelease-1>", self.end_stroke)
    
    def normalize_coordinate(self, coordinate):
        """Convert canvas coordinate to real-world coordinate"""
        num = coordinate / self.size  # 0 to 1
        num = num * 2 - 1             # -1 to 1
        num *= self.coord_range       # scale to range
        return round(num, 8)
    
    def start_stroke(self, event):
        x = self.normalize_coordinate(event.x)
        y = -self.normalize_coordinate(event.y)  # Flip y-axis
        self.points.append((x, y))
        self.last_x, self.last_y = event.x, event.y
    
    def draw_stroke(self, event):
        # Draw line on canvas
        self.canvas.create_line(
            self.last_x, self.last_y, 
            event.x, event.y, 
            fill="black", width=3
        )
        
        # Store normalized point
        x = self.normalize_coordinate(event.x)
        y = -self.normalize_coordinate(event.y)  # Flip y-axis
        self.points.append((x, y))
        
        self.last_x, self.last_y = event.x, event.y
    
    def end_stroke(self, event):
        if len(self.points) < 10:
            print("Path too short! Please draw a longer path.")
            self.points.clear()
            return
        
        print(f"\nPath drawn with {len(self.points)} points")
        print(f"Coordinate range: ±{self.coord_range}")
        self.root.destroy()
    
    def get_points(self):
        return np.array(self.points)


# ==================== BÉZIER CURVE FITTING ====================

def bernstein_poly(i, n, t):
    """Bernstein polynomial basis function"""
    from math import comb
    return comb(n, i) * (t**i) * ((1 - t) ** (n - i))


def bezier_curve(control_points, num_points=50):
    """Evaluate Bézier curve at num_points along its length"""
    n = len(control_points) - 1
    t = np.linspace(0, 1, num_points)
    curve = np.zeros((num_points, 2))
    for i, p in enumerate(control_points):
        curve += np.outer([bernstein_poly(i, n, ti) for ti in t], p)
    return curve


def fit_bezier(points, degree=3, num_curve_points=200):
    """Fit a single Bézier curve to points using least squares"""
    if len(points) < degree + 1:
        degree = len(points) - 1
    
    # Parameterization by cumulative arc length
    arc_lengths = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
    t_values = np.insert(np.cumsum(arc_lengths), 0, 0)
    t_values /= t_values[-1]
    
    # Initial guess: linear interpolation
    init_ctrl = np.linspace(points[0], points[-1], degree + 1)
    
    def residuals(flat_ctrl):
        ctrl = flat_ctrl.reshape(-1, 2)
        approx = np.zeros_like(points)
        for j, t in enumerate(t_values):
            B = np.array([bernstein_poly(i, degree, t) for i in range(degree+1)])
            approx[j] = B @ ctrl
        return (approx - points).ravel()
    
    res = least_squares(residuals, init_ctrl.ravel())
    ctrl = res.x.reshape(-1, 2)
    curve = bezier_curve(ctrl, num_curve_points)
    
    # Compute maximum error
    diffs = []
    for j, t in enumerate(t_values):
        B = np.array([bernstein_poly(i, degree, t) for i in range(degree+1)])
        approx = B @ ctrl
        diffs.append(np.linalg.norm(approx - points[j]))
    max_err = max(diffs)
    
    return ctrl, curve, max_err


def fit_poly_bezier(points, degree=3, max_error=0.000005, 
                   num_curve_points=200, points_per_curve=100):
    """
    Fit multiple connected Bézier curves to points.
    Recursively splits until error < max_error.
    """
    def recursive_fit(pts):
        ctrl, curve, err = fit_bezier(pts, degree, num_curve_points)
        
        # If error too high and enough points, split
        if err > max_error and len(pts) > points_per_curve * degree + 1:
            mid = len(pts) // 2
            left = recursive_fit(pts[:mid+1])
            right = recursive_fit(pts[mid:])
            
            # Enforce connectivity
            if not np.allclose(left[-1][1][-1], right[0][1][0]):
                right[0] = (
                    np.vstack([left[-1][1][-1], right[0][0][1:]]),
                    bezier_curve(np.vstack([left[-1][1][-1], right[0][0][1:]]), num_curve_points)
                )
            return left + right
        else:
            return [(ctrl, curve)]
    
    return recursive_fit(points)


def export_bezier_to_csv(curve_data, bezier_filename="bezier_points.csv"):
    """Export Bézier control points to CSV"""
    with open(bezier_filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["x", "y"])
        
        for ctrl_points, _ in curve_data:
            for x, y in ctrl_points:
                writer.writerow([x, y])
    
    print(f"Exported {len(curve_data)} Bézier curve(s) to {bezier_filename}")


def export_raw_points(points, filename="points.csv"):
    """Export raw drawn points to CSV"""
    with open(filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["x", "y"])
        for x, y in points:
            writer.writerow([x, y])
    
    print(f"Exported {len(points)} raw points to {filename}")


# ==================== MAIN ====================

def main():
    print("=" * 50)
    print("Ball-on-Plate Path Creator")
    print("=" * 50)
    
    # Step 1: Draw path
    print("\n[1/3] Drawing Interface")
    print("Draw your desired path with the mouse...")
    
    root = tk.Tk()
    drawer = PathDrawer(root, size=600)
    root.mainloop()
    
    points = drawer.get_points()
    
    if len(points) == 0:
        print("No path drawn. Exiting.")
        return
    
    # Step 2: Fit Bézier curves
    print("\n[2/3] Fitting Bézier Curves")
    print("Processing path...")
    
    curve_data = fit_poly_bezier(points)
    
    print(f"Generated {len(curve_data)} Bézier curve segment(s)")
    
    # Step 3: Export
    print("\n[3/3] Exporting")
    export_raw_points(points, "points.csv")
    export_bezier_to_csv(curve_data, "bezier_points.csv")
    
    print("\n" + "=" * 50)
    print("✓ Path creation complete!")
    print("=" * 50)
    print("\nGenerated files:")
    print("  • bezier_points.csv  (control points for controller)")
    print("  • points.csv         (raw drawn points)")
    print("\nThe controller will automatically detect and load the new path.")


if __name__ == "__main__":
    main()