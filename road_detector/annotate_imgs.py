import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import argparse
import os
from __utils__ import *

parser = argparse.ArgumentParser()
parser.add_argument(
    "--path",
    dest="path",
    type=str,
    help="Path to image or folder containing images to annotate")
parser.add_argument("--results",
                    dest="results",
                    type=str,
                    help="Path to folder to save annotation tensors to")
args = parser.parse_args()

matplotlib.use('TkAgg')


def annotate_img(img_path, dims, use_patch_size=True, num_points=8):
    """
    Opens image for annotation. 
    
    Displays the image in an external window
    and allows user to click points to select a polygon bounding the area
    of interest. Then, breaks the image into (num_h, num_n) grid cells and saves
    a binary mask tensor of that size with value 1 when the corresponding
    grid cell's center is within the bounding polygon.

    Args:
        img_path: str path to image to annotate
        dims: tuple of size 2 containing [H, W]
        use_patch_size: bool stating whether to treat [H, W]
            as the dimensions of the patch (default) or as the number of patches
            to break the image into. If false, [num_h, num_w] = [H, W]
        num_points: int number of points annotator can select to specify bounding
            polygon for area of interest. Default = 8
    
    Returns:
        np.ndarray binary mask of size [num_h, num_w] specifying 
            grid cells in area of interest
        
    """
    # Open image
    img_t = open_img_as_array(img_path)

    # Compute grid cell dimensions and grid dimensions
    if use_patch_size:
        # If true, grid cell dimensions are in dims, [num_h, num_w] is computed
        # by dividing image dimensions by cell dimensions
        patch_h, patch_w = dims
        num_h, num_w = img_t.shape[1] // patch_h, img_t.shape[2] // patch_w
    else:
        # Else, dims specify number of grid cells, so need to compute cell dimensions
        num_h, num_w = dims
        patch_h, patch_w = img_t.shape[1] // num_h, img_t.shape[2] // num_w

    # Show image to annotate
    img_np = img_t.transpose([1, 2, 0])
    plt.imshow(img_np)

    # Select points on polygon bounding area of interest
    pts = np.array(plt.ginput(num_points))

    # Draw and fill bounding polygon
    x, y = pts.T[0], pts.T[1]
    plt.fill(x, y, alpha=.15)

    # Compute which grid cell centers are in bounding polygon
    inside_pts = []

    # Initialize binary mask
    mask = np.zeros([num_h, num_w])
    poly = Polygon(pts)

    for i in range(num_h):
        for j in range(num_w):
            y, x = i * patch_h + patch_h / 2., j * patch_w + patch_w / 2.

            pt = Point(x, y)
            if poly.contains(pt):
                inside_pts.append((x, y))
                mask[i, j] = 1

    inside_pts = np.array(inside_pts).T

    # Plot all the grid cell centers inside the bounding polygon
    plt.scatter(inside_pts[0],
                inside_pts[1],
                c=[[.5, 0, .5, .4]],
                s=15,
                marker="+")
    plt.show()

    return mask


if __name__ == "__main__":
    num_points = 10

    # Make sure path exists
    assert os.path.isfile(args.path) or os.path.isdir(args.path)
    # Make sure results exists and is a folder
    assert os.path.isdir(args.results)

    if os.path.isfile(args.path):
        mask = annotate_img(args.path, (patch_H, patch_W), num_points=num_points)
        # get filename without extension
        file_name = os.path.split(args.path).split(".")[0]
        result_path = os.path.join(args.results, file_name + ".npy")
        np.save(result_path, mask)
    else:
        for file in os.listdir(args.path):
            file_path = os.path.join(args.path, file)
            assert os.path.isfile(file_path)
            mask = annotate_img(file_path, (patch_H, patch_W), num_points=num_points)
            file_name = file.split(".")[0]
            result_path = os.path.join(args.results, file_name + ".npy")
            np.save(result_path, mask)
