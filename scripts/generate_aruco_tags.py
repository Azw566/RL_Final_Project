#!/usr/bin/env python3
"""
generate_aruco_tags.py — Generates ArUco marker PNG images for use in Gazebo.

Generates PNG files for IDs 0..N-1 from the DICT_4X4_50 dictionary.
Output: maze_ws/src/maze_gazebo/models/aruco_tags/tagN.png

Usage:
    python3 scripts/generate_aruco_tags.py [--count N] [--size PIXELS]
"""

import argparse
import os
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    print("ERROR: opencv-python not found. Install with: pip3 install opencv-python")
    sys.exit(1)


def generate_tags(count: int, pixel_size: int, output_dir: str):
    os.makedirs(output_dir, exist_ok=True)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    for marker_id in range(count):
        # Generate the marker image
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, pixel_size)

        # Add a white border (makes detection more robust)
        border_size = pixel_size // 8
        marker_bordered = cv2.copyMakeBorder(
            marker_img,
            border_size, border_size, border_size, border_size,
            cv2.BORDER_CONSTANT,
            value=255
        )

        out_path = os.path.join(output_dir, f'tag{marker_id}.png')
        cv2.imwrite(out_path, marker_bordered)
        print(f'Generated: {out_path}  ({marker_bordered.shape[1]}×{marker_bordered.shape[0]}px)')

    print(f'\nGenerated {count} ArUco tags in: {output_dir}')
    print('Place these PNG files in maze_ws/src/maze_gazebo/models/aruco_tags/')
    print('and reference them via model://aruco_tags/tagN.png in maze.sdf')


def main():
    parser = argparse.ArgumentParser(description='Generate ArUco marker PNGs for Gazebo')
    parser.add_argument('--count', type=int, default=5, help='Number of markers (default: 5)')
    parser.add_argument('--size', type=int, default=512,
                        help='Pixel size of marker image (default: 512)')
    parser.add_argument('--output', type=str,
                        default=os.path.join(
                            os.path.dirname(__file__),
                            '..', 'src', 'maze_gazebo', 'models', 'aruco_tags'
                        ),
                        help='Output directory')
    args = parser.parse_args()

    generate_tags(args.count, args.size, os.path.abspath(args.output))


if __name__ == '__main__':
    main()
