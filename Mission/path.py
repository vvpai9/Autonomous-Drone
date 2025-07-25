import logging
import asyncio

# Generate a grid of waypoints over the geofenced area
def generate_waypoints(geofence, x_divisions, y_divisions):
    """Generate a search grid based on ordered geofence coordinates."""
    print("Generating grid...")
    
    bottom_left, bottom_right, top_right, top_left = geofence
    lat_start, lon_start = bottom_left
    lat_end, lon_end = top_right

    lat_step = (lat_end - lat_start) / y_divisions
    lon_step = (lon_end - lon_start) / x_divisions
    print(int(lat_step * 1e7))

    grid = []
    for i in range(y_divisions + 1):
        row = []
        for j in range(x_divisions + 1):
            lat = lat_start + i * lat_step
            lon = lon_start + j * lon_step
            row.append((lat, lon))
        grid.append(row)
    
    return grid

# Traverse the waypoints in a boustrophedon (zig-zag) pattern
async def boustrophedon_path(drone, wp, altitude):
    for i, row in enumerate(wp):
        path = row if i % 2 == 0 else reversed(row)
        for point in path:
            lat, lon = point
            await drone.go_to_location(lat, lon, altitude)
            await asyncio.sleep(0.1)