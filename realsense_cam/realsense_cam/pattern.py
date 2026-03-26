# Copyright 2024 nathan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from PIL import Image


def extract_emitter_pattern(input_image_path, output_image_path, threshold):
    """
    Isolate bright patterns from an image based on a brightness threshold.

    Saves them to a new PNG with a transparent background.

    Args:
        input_image_path (str): The path to the source image.
        output_image_path (str): The path to save the output PNG file.
        threshold (int): A value from 0-255. Pixels brighter than this will be kept.
    """
    try:
        # Open the original image
        img = Image.open(input_image_path)
    except FileNotFoundError:
        print(f"Error: The file '{input_image_path}' was not found.")
        return

    # Resize the image to the desired output dimensions (1024x1024)
    img = img.resize((1024, 1024), Image.Resampling.LANCZOS)

    # Convert the resized image to grayscale for accurate brightness comparison
    grayscale_img = img.convert('L')

    # Create a new image for the output with an alpha channel (for transparency)
    output_img = Image.new('RGBA', grayscale_img.size, (0, 0, 0, 0))

    # Get access to the pixel data
    grayscale_pixels = grayscale_img.load()
    output_pixels = output_img.load()

    width, height = grayscale_img.size

    # Iterate over every pixel in the image
    for x in range(width):
        for y in range(height):
            # Get the brightness of the current pixel
            brightness = grayscale_pixels[x, y]

            # If the pixel is brighter than the threshold, make it visible
            if brightness > threshold:
                # Set the output pixel to white and fully opaque
                output_pixels[x, y] = (255, 255, 255, 255)
            # Otherwise, the pixel remains transparent (from initial creation)

    # Save the final image as a PNG to preserve transparency
    output_img.save(output_image_path)
    print(f"Pattern extracted and saved to '{output_image_path}' "
          f"with size {output_img.size}")


if __name__ == '__main__':
    # --- Configuration ---
    INPUT_IMAGE = '/home/nathan/clearpath_dev/src/realsense_cam/pattern/d455.png'
    OUTPUT_IMAGE = '/home/nathan/clearpath_dev/src/realsense_cam/pattern/extracted_realsense_pattern.png'

    # Adjust this value if needed. Higher values are more strict,
    # selecting only the brightest spots. A good starting point is 120.
    BRIGHTNESS_THRESHOLD = 120

    extract_emitter_pattern(INPUT_IMAGE, OUTPUT_IMAGE, BRIGHTNESS_THRESHOLD)
