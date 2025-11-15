#!/usr/bin/env python3
"""
Generate AprilTag images for use in Gazebo simulation and real-world
"""
import numpy as np
from PIL import Image, ImageDraw

def generate_apriltag_image(tag_id=0, size_px=512, border_px=64):
    """
    Generate a simple AprilTag pattern
    This is a simplified version - real tags have specific bit patterns
    For actual detection, use pre-generated tags from AprilTag library
    """
    # Create white background with black border
    img = Image.new('L', (size_px, size_px), 255)
    draw = ImageDraw.Draw(img)

    # Black outer border
    draw.rectangle([0, 0, size_px-1, size_px-1], fill=0)

    # White inner square
    draw.rectangle([border_px, border_px, size_px-border_px-1, size_px-border_px-1], fill=255)

    # Draw grid pattern (simplified tag36h11 pattern)
    # This is NOT a real AprilTag - just for visualization
    # For real detection, download actual tags from: https://github.com/AprilRobotics/apriltag-imgs

    grid_size = 6  # tag36h11 uses 6x6 grid
    cell_size = (size_px - 2*border_px) // grid_size

    # Example pattern for tag ID 0 (simplified)
    # Real patterns are in the AprilTag library
    pattern = np.array([
        [1, 1, 0, 0, 1, 1],
        [1, 0, 1, 1, 0, 1],
        [0, 1, 0, 0, 1, 0],
        [0, 1, 0, 0, 1, 0],
        [1, 0, 1, 1, 0, 1],
        [1, 1, 0, 0, 1, 1],
    ])

    for i in range(grid_size):
        for j in range(grid_size):
            if pattern[i][j] == 0:  # Black cell
                x1 = border_px + j * cell_size
                y1 = border_px + i * cell_size
                x2 = x1 + cell_size - 1
                y2 = y1 + cell_size - 1
                draw.rectangle([x1, y1, x2, y2], fill=0)

    return img

if __name__ == '__main__':
    # Generate tag for simulation
    tag = generate_apriltag_image(tag_id=0, size_px=1024, border_px=128)
    output_path = '/home/jay/Git Sandbox/Jetson Cube Orange Outdoor Rover/ros2_ws/src/jetson_rover_sim/models/apriltag_stand/apriltag_0.png'
    tag.save(output_path)
    print(f'AprilTag saved to: {output_path}')
    print('')
    print('NOTE: This is a simplified tag for visualization.')
    print('For real AprilTag detection, download actual tag images from:')
    print('  https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11')
    print('')
    print('To use in simulation:')
    print('  1. Download tag36_11_00000.png from above link')
    print('  2. Replace apriltag_0.png with the real tag image')
    print('  3. Or use this simplified version for basic testing')
