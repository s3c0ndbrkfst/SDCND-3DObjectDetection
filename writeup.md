# Writeup: Mid-Term

Below are 10 examples of vehicles with varying degrees of visibility in the point-cloud for `frames = [0, 200]` of `data_filename = 'training_segment-10963653239323173269_1924_000_1944_000_with_camera_labels.tfrecord'`.

| PCL Output      |               |
| :-------------: | :-----------: |
| ![png](/img_writeup/img_1.png)          |     ![png](/img_writeup/img_2.png)     |
| ![png](/img_writeup/img_3.png)          |     ![png](/img_writeup/img_4.png)     |
| ![png](/img_writeup/img_5.png)          |     ![png](/img_writeup/img_6.png)     |
| ![png](/img_writeup/img_7.png)          |     ![png](/img_writeup/img_8.png)     |
| ![png](/img_writeup/img_9.png)          |     ![png](/img_writeup/img_10.png)     |

It is clear from these images that the most stable and consistent features are front bumpers, rear bumpers, and side view mirrors. When visually inspecting the images, it is obvious that a car is being detected by the clear shape of the front or rear bumper depending on the orientation of the vehicle.

In the associated range image taken at frame 100, you can see the clear definition of the front and rear bumpers of the vehicles. Additionally, it is very clear to see tail lights and even license plates in several of the vehicles. This is precise detail is not as clear in the point-cloud images.

![png](/img_writeup/range_img.png)