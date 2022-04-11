import matplotlib.pyplot as plt
import numpy as np

def visualize_image(image, mask):
    img_H, img_W = image.shape[0], image.shape[1]
    plt.imshow((image*255).astype(np.uint8))
    vis_mask = np.zeros([img_H, img_W], np.uint16)
    for i in range(mask.shape[0]):
        for j in range(mask.shape[1]):
            if mask[i,j] == 1:
                vis_mask[i*H:(i+1)*H, j*W:(j+1)*W] = 1
    plt.imshow(vis_mask, alpha = 0.25)
    plt.show()


loaded_image = np.loadtxt("np_img.txt")
np_arr = loaded_image.reshape(376, 672, 3)
mask_arr = np.loadtxt("mask.txt")

H = 20
W = 20

visualize_image(np_arr, mask_arr)

