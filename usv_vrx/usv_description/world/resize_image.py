#!/usr/bin/env python3

from PIL import Image

# Open an image file
with Image.open("blank_map_with_obstacle.pgm") as img:
    # Resize the image
    img_resized = img.resize((6000, 6000), Image.LANCZOS)  # Replace 1024, 1024 with your desired size

    # Save the resized image
    img_resized.save("blank_map_with_obstacle_resized.pgm")

