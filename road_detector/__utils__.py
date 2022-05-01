from PIL import Image
from matplotlib import pyplot as plt
import numpy as np

# TODO: Modify the path to model weights
path_to_model_weights = "/path/to/model/weights.pth"

def open_img_as_array(img_path):
    """ 
    Opens image at specified path as torch.tensor.
    
    Args:
        img_path: str path to image to open
    
    Returns:
        np.array representation of the image of size [3, h, w]
    """
    return np.array(Image.open(img_path)).transpose([2, 0, 1])


def split_img(img, dims, use_patch_size=True):
    """
    Splits image into evenly-sized patches
    
    Args:
        img: torch.tensor image of size [channels=3, h, w]
        dims: tuple of size 2 containing [H, W]
        use_patch_size: bool stating whether to treat [H, W]
            as the dimensions of the patch (default) or as the number of patches
            to break the image into
    
    Returns:
        torch.tensor of size [num_h, num_w, 3, h // num_h, w // num_w]
            where the first two indices represent the patch index. 
            (0, 0) would be the upper left patch
    """
    if use_patch_size:
        patch_h, patch_w = dims
    else:
        num_h, num_w = dims
        patch_h, patch_w = img.shape[1] // num_h, img.shape[2] // num_w
    patches = img.data.unfold(0, 3,
                              3).unfold(1, patch_h,
                                        patch_h).unfold(2, patch_w, patch_w)
    return patches[0]


def visualize(patches):
    """Imshow for Tensor."""
    num_h, num_w = patches.shape[0], patches.shape[1]
    fig = plt.figure(figsize=(num_h, num_w))
    for i in range(num_h):
        for j in range(num_w):
            inp = patches[i][j]
            inp = np.array(inp)

            ax = fig.add_subplot(num_h,
                                 num_w, ((i * num_w) + j) + 1,
                                 xticks=[],
                                 yticks=[])
            plt.imshow(inp)