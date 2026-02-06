
Emergency Exit Signs v2 - v8 Augmentation Dataset 1024x1024 (YOLOv8)
==============================

This dataset was exported via roboflow.com on September 24, 2025 at 6:58 PM GMT

Roboflow is an end-to-end computer vision platform that helps you
* collaborate with your team on computer vision projects
* collect & organize images
* understand and search unstructured image data
* annotate, and create datasets
* export, train, and deploy computer vision models
* use active learning to improve your dataset over time

For state of the art Computer Vision training notebooks you can use with this dataset,
visit https://github.com/roboflow/notebooks

To find over 100k other datasets and pre-trained models, visit https://universe.roboflow.com

The dataset includes 3330 images.
Types-of-Exits are annotated in YOLOv8 Oriented Object Detection format.

The following pre-processing was applied to each image:
* Resize to 1024x1024 (Stretch)

The following augmentation was applied to create 4 versions of each source image:
* Randomly crop between 5 and 15 percent of the image
* Random shear of between -10° to +10° horizontally and -10° to +10° vertically
* Random brigthness adjustment of between -30 and +30 percent
* Random exposure adjustment of between -30 and +30 percent
* Random Gaussian blur of between 0 and 1 pixels
* Salt and pepper noise was applied to 1.01 percent of pixels


