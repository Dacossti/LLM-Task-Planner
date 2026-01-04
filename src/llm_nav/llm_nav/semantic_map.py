# llm_nav/semantic_map.py

import os
import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid
import cv2

def load_map(map_path):
    """
    Load the map from the .pgm file and .yaml metadata
    """
    map_yaml = map_path + ".yaml"
    with open(map_yaml, 'r') as f:
        map_info = yaml.safe_load(f)

    image_path = map_info['image']
    image_full_path = os.path.join(map_path, image_path)
    img = cv2.imread(image_full_path, cv2.IMREAD_GRAYSCALE)

    return img, map_info

def generate_room_from_map(img):
    """
    This is a simple method to detect rooms from the map image.
    It can be extended with more sophisticated room detection techniques.
    """
    room_list = []

    # Simple threshold to define free space and obstacles
    _, binary_map = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY_INV)

    # Use contours to detect rooms
    contours, _ = cv2.findContours(binary_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        room_list.append((x, y, w, h))

    return room_list

def get_semantic_map(map_path):
    """
    Given the path to the map files (.pgm + .yaml), returns the rooms (detected areas).
    """
    img, map_info = load_map(map_path)
    rooms = generate_room_from_map(img)
    
    return rooms
