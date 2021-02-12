# **Home Service Robot Project** 
---

Pongrut Palarpong  
February 12, 2021

---


The goals / steps of this project are the following:

* The goal is to write software for simulating a full home service robot capable of navigating to pick up and deliver virtual objects on the ROS platform.




---
### Overview
The overview of the home service robot simulation consists of 3 main components.
1. The simulated World in gazebo simulator 3D world created in gazebo building tools as shown in Fig.1 left.
2. The simulated Robot in the gazebo develops with Unified Robot Description Format (URDF), an XML format for representing a robot model.
3. The static 2D map of the world created from manual navigation with SLAM, as shown in Fig.1 right.


There are multiple service nodes with specific functions in the background, with each service nodes communicate with each other by passing messages communicating through the central ROS Master. Gazabo Simulator will receive location and pose change information of the robot from ROS Master and be displayed in the simulation world.

![world_robot_map](./images/world_robot_map.jpg)
Figure 1. Left:  The simulated 3D World. Middle: The simulated Robot in the gazebo. Right: 2D Map (image file with 0.0500 meter/pixel resolution, defined in pongrut_map.yaml).


### Robot with URDF & ROS TF
![all_frames](./images/frames.png)
![tf](./images/robot_tf_urdf.jpg)

![SLAM_mapping](./images/home_service_SLAM.jpg)




The use of HOG features and a linear SVM is well-known since 2005. Very recently high-speed neural network based object detectors have emerged which allow object detection faster than real-time. 

However, the most popular method of doing object detection nowadays is to use The YOLO is a very effective way for Real-time Object Detection, with only a small amount of code, which yields impressive results.

### Histogram of Oriented Gradients (HOG) vs. Convolutional Neural Network (CNN)

The histogram of gradients (HOG) is a descriptor feature. The HOG algorithm will check every pixel about how much darker the surrounding pixels are and then specify the direction that pixel is getting darker, then counts the occurrences of gradient orientation in localized portions of an image. The HOG result is features that use in support vector machine for the classification task.


![HOG](./figures/HOG.jpg)<br/>
<p align="center">HOG Features Visualization</p>
<br/>

![car_color_hist](./figures/car_color_hist.jpg)
![notcar_color_hist](./figures/notcar_color_hist.jpg)
<p align="center">Color Histogram Features Visualization</p>

YOLO Real-Time Object Detection apply convolutional neural network architecture to classify an object. CNN architecture suitable for image classification because the image is indeed 2D width and height. CNN can do convolution operation by sweeping the relationship between each part of the image and creating essential filters. This convolution operation makes it easy for CNN to detect objects in multiple locations, difference lightings, or even just some part of objects in an image.

![CNN](https://www.mathworks.com/content/mathworks/www/en/discovery/deep-learning/jcr:content/mainParsys/band_2123350969_copy_1983242569/mainParsys/columns_1635259577/1/image_2128876021_cop.adapt.full.high.svg/1508444613873.svg)
Example: A network with many convolutional layers

### My Vehicle Detection Model.
Model Cfg : yolo.cfg<br/>
Model weights: olo.weights<br/>
Model has a coco model name, loading coco labels.

|Source| Train? | Layer description                | Output size     |
|:----:|:------:|:--------------------------------:|:---------------:|
|      |        | input                            | (?, 608, 608, 3)|
| Load |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 608, 608, 32)|
| Load  |  Yep!  | maxp 2x2p0_2                     | (?, 304, 304, 32)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 304, 304, 64)|
| Load  |  Yep!  | maxp 2x2p0_2                     | (?, 152, 152, 64)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 152, 152, 128)|
| Load  |  Yep!  | conv 1x1p0_1  +bnorm  leaky      | (?, 152, 152, 64)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 152, 152, 128)|
| Load  |  Yep!  | maxp 2x2p0_2                     | (?, 76, 76, 128)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 76, 76, 256)|
| Load  |  Yep!  | conv 1x1p0_1  +bnorm  leaky      | (?, 76, 76, 128)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 76, 76, 256)|
| Load  |  Yep!  | maxp 2x2p0_2                     | (?, 38, 38, 256)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 38, 38, 512)|
| Load  |  Yep!  | conv 1x1p0_1  +bnorm  leaky      | (?, 38, 38, 256)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 38, 38, 512)|
| Load  |  Yep!  | conv 1x1p0_1  +bnorm  leaky      | (?, 38, 38, 256)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 38, 38, 512)|
| Load  |  Yep!  | maxp 2x2p0_2                     | (?, 19, 19, 512)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 19, 19, 1024)|
| Load  |  Yep!  | conv 1x1p0_1  +bnorm  leaky      | (?, 19, 19, 512)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 19, 19, 1024)|
| Load  |  Yep!  | conv 1x1p0_1  +bnorm  leaky      | (?, 19, 19, 512)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 19, 19, 1024)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 19, 19, 1024)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 19, 19, 1024)|
| Load  |  Yep!  | concat [16]                      | (?, 38, 38, 512)|
| Load  |  Yep!  | conv 1x1p0_1  +bnorm  leaky      | (?, 38, 38, 64)|
| Load  |  Yep!  | local flatten 2x2                | (?, 19, 19, 256)|
| Load  |  Yep!  | concat [27, 24]                  | (?, 19, 19, 1280)|
| Load  |  Yep!  | conv 3x3p1_1  +bnorm  leaky      | (?, 19, 19, 1024)|
| Load  |  Yep!  | conv 1x1p0_1    linear           | (?, 19, 19, 425)|

Running entirely on CPU
Finished in 37.28185439109802s


### Sliding Window Search vs. You only look once 
The HOG Sub-sampling to classify vehicle, we have to extract hog features once, for each of a small set of predetermined window sizes (defined by a scale argument), and then can be sub-sampled to get all of its overlaying windows. Each window defined by a scaling factor that impacts the window size. 
![windows_search](./figures/windows_search.jpg)

YOLO looks at the image just once and divides up the image into a grid of 13 by 13 cells, each of these cells is responsible for predicting five bounding boxes. A bounding box describes the rectangle that encloses an object, and it also outputs a confidence score that tells us how confident it is that the predicted bounding box encloses some object.

![yolo](https://statsbot.co/blog/wp-content/uploads/2017/10/b9213-1psfl5og1c9hikxlmijv8-q-e1516381607519-752x223-2-1024x304.png)


The code for YOLO object detection is contained in the Step 6: of the IPython notebook.
Here is an example of vehicle detection code.
 
```
image = mpimg.imread('./test_image.jpg')
results = vehicle_detector.detect_vehicle(image)
plt.figure(figsize=(16, 9))
plt.imshow(image)
plt.show() 
```
Here is an example result image:
![yolo_result](./figures/yolo_result.jpg)


References:<br/>
- [Gazebo: Totorial Build a world](http://gazebosim.org/tutorials?tut=build_world)<br/>
- [You Only Look Once: Unified, Real-Time Object Detection](https://arxiv.org/abs/1506.02640)<br/>
- [YOLO — You only look once, real time object detection explained](https://towardsdatascience.com/yolo-you-only-look-once-real-time-object-detection-explained-492dc9230006)<br/>
- [Real-time object detection with YOLO](http://machinethink.net/blog/object-detection-with-yolo/)

### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result](./project_video_detect_out.mp4)<br/>
[![](http://img.youtube.com/vi/CFzyrBdf2qQ/0.jpg)](http://www.youtube.com/watch?v=CFzyrBdf2qQ "Advanced Lane Finding & Vehicle Detection with YOLO")



---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

In this project, I used the Yolov1 model for the Vehicle Detection. YOLO can define the bounding box is more robust than HOG & SVM. However, in some situation pre-trained models confuse in predicting object class such as car and truck, especially a small object image. 

In high accuracy classify case, it may require using the more accurate model such as YOLOv3, including the use of image segmentation and image upscaling techniques before detection.

