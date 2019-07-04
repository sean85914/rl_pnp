from trainer import Trainer
import cv2
import numpy as np

trainer = Trainer(1.0, 1.0, 0.5, False, True)
color_img = cv2.imread("images/color/color_000000.jpg")
depth_img = cv2.imread("images/depth/depth_000000.png", -1)

suck_predictions, grasp_predictions, state_feat = trainer.forward(color_img, depth_img, True)

## Ploting
tmp = suck_predictions.transpose(1, 2, 0) # H, W, C
tmp -= np.min(tmp)
plot_suck = tmp / np.max(tmp)
plot_suck = (plot_suck*255).astype(np.uint8)
plot_suck_color = cv2.applyColorMap(plot_suck, cv2.COLORMAP_JET)
cv2.imshow("suck", plot_suck_color)

for rotate_idx in range(len(grasp_predictions)):
    tmp = grasp_predictions[rotate_idx]
    tmp -= np.min(tmp)
    window_name = "grasp_{}".format(rotate_idx)
    plot_grasp = tmp / np.max(tmp)
    plot_grasp = (plot_grasp*255).astype(np.uint8)
    plot_grasp_color = cv2.applyColorMap(plot_grasp, cv2.COLORMAP_JET)
    cv2.imshow(window_name, plot_grasp_color)
cv2.waitKey(0)

del suck_predictions, grasp_predictions, state_feat
