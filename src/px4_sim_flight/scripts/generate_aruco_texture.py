#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import numpy as np
import os

def generate_marker(dictionary_id=cv2.aruco.DICT_6X6_250, marker_id=0, size=200):
    dictionary = aruco.getPredefinedDictionary(dictionary_id)
    img = aruco.generateImageMarker(dictionary, marker_id, size)
    border = 40
    out = 255 * np.ones((size + 2*border, size + 2*border), dtype=np.uint8)
    out[border:border+size, border:border+size] = img
    return out

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    texture_dir = os.path.join(script_dir, "../models/aruco_6x6_100mm/materials/textures")
    os.makedirs(texture_dir, exist_ok=True)
    
    marker_img = generate_marker(marker_id=0, size=400)
    out_path = os.path.join(texture_dir, "aruco_6x6_100mm.png")
    cv2.imwrite(out_path, marker_img)
    print(f"[OK] Generated ArUco marker: {out_path}")
