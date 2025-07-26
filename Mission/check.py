import matplotlib.pyplot as plt
from path import generate_waypoints
from mission import geofence, x_divisions, y_divisions

modules = ["log.py", "detection.py", "align.py", "path.py", "basic.py", "mission.py"]

def plot_serpentine_path(grid, geofence):
    """Plot the serpentine path using Matplotlib."""
    print("Serpentine path started")
    fig, ax = plt.subplots()
    
    x_vals, y_vals = [], []
    for i, row in enumerate(grid):
        if i % 2 == 0:
            # Move from bottom to top
            x_vals.extend([lat for lat, lon in row])
            y_vals.extend([lon for lat, lon in row])
        else:
            # Move from top to bottom
            x_vals.extend([lat for lat, lon in reversed(row)])
            y_vals.extend([lon for lat, lon in reversed(row)])
    
    # Plot serpentine path
    ax.plot(x_vals, y_vals, marker="o", linestyle="-", color="blue", markersize=4, label="Serpentine Path")
    # ax.plot(x, y, marker="o", linestyle="-", color="green", markersize=4)
    # Plot geofence points
    geo_x = [lat for lat, lon in geofence] + [geofence[0][0]]  # Closing the loop
    geo_y = [lon for lat, lon in geofence] + [geofence[0][1]]
    ax.plot(geo_x, geo_y, marker="s", linestyle="--", color="red", markersize=6, label="Geofence")
    
    # Set actual coordinates as tick labels with rotation for readability
    ax.set_xticks([lat for lat, lon in geofence])
    ax.set_yticks([lon for lat, lon in geofence])
    ax.set_xticklabels([f"{lat:.6f}" for lat, lon in geofence], rotation=45, ha="right")
    ax.set_yticklabels([f"{lon:.6f}" for lat, lon in geofence])
    
    ax.set_xlabel("Latitude")
    ax.set_ylabel("Longitude")
    ax.set_title("Serpentine Path Grid with Geofence")
    ax.legend()
    ax.grid(True)
    
    plt.show()

def check():
    for module in modules:
        with open(module, "r") as f:
            code = f.read()

        try:
            compile(code, module, "exec")
            print(f"{module} Syntax is good.")
        except SyntaxError as e:
            print(f"Syntax Error in {module}:", e)
            continue
    
    grid = generate_waypoints(geofence, x_divisions, y_divisions)
    plot_serpentine_path(grid, geofence)

if __name__ == "__main__":
    check()
