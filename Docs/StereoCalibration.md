# What is Stereo Calibration?
Stereo Calibration is the initialization process for a pair of cameras viewing the same object from different angles. Similarly to a pair of human eyes, a system taking in 2 images needs to calculate the translation matrix so that it knows how to calculate feature points in a virtual 3D space.

## Wait, a Translation Matrix? Feature Points?
Yes, feature points are coordinates within an image that an represent "points of interest," such as sharp changes in pixel colors or corners of objects. How these points are determined is dependent on the algorithm you choose but generally, you want as many points as possible, since they give the computer a better starting place for the calibration process.

Now we've got a list of feature points but they only exist on a 2D plane. For all each camera knows, it may as well be looking at a flat poster rather than a real object. To solve for depth, a second camera is needed, which is why feature points need to be identified and matched for both camera images. It then attempts to figure out how those points are related. In other words, the computer must find a mathematical line on which Camera A's point lies which Camera B's msut reside on.

## Ok, how do I make an Essential Matrix?

There are 2 core components to an Essential Matrix: translation and rotation. Translation is best described as movement in 3D space, using an X, Y and Z scale. Rotation is, somewhat predictably, how much these points are rotated around those 3 axis. Combined, they form a 3x3 Matrix. Lucky for us, there are algorithms in several libraries to take care of that for us but the one in the Calibration POC uses Eigen's JacobiSVD library to calculate something called the Essential Matrix, from which the relative rotation R and translation direction t can be obtained. There are several linear algorithms, some needing 8 feature points like ours, some taking 7 points and some even useing just 5. But the rule of thumb is that the more points the algorithm uses, the better.

## Anything else to know?
Yes, when working with real camera equipment, you also need to take into account 3 additional factors: the Focal Length and the Principal Point. The focal length determines how distance of the lense, basically determining how quickly things get larger or smaller as they pull toward or away from the camera. The Principal Point is how off-center the image will be, due to micro-defects in the camera hardware. Lastly we need to know how a camera's image is distorted. You may have noticed in some camera feeds (like Go-Pro videos) have a fish-eye distortion. That's something all cameras have, and outside of this POC, we'll need to correct for that. However, we're working with digitally rendered images so we can skip that part for now. Once we have those p`ieces of information, we can turn our 2D pixel coordiantes into 2D camera coordinates, which we can apply to our Essential Matrix.

So our process right now is to first use an algorithm to find as many matching feature points as possible across both cameras. Then, convert those pixel coordinates into camera coordinates using the focal length and center points. Next, we find the Essential Matrix using the normalized point correspondences via the algorithms I mentioned earlier. 

Finally, we do a quick check to see how accurate our matrix is, which is why we needed to find as many matching points as possible. The more of them match, the better we know our calculations were. (Of course, in our POC the feature points are hard-coded, so we don't have to worry about that too much.)

## Ok, so now we have a Essential Matrix. But how do we get 3D Points from this?
At this point, we can take any point on one camera feed and find its matching point on the other. But we still need to find those points on the Z axis. To do this, we use our existing information to find the positions of the cameras themselves, use them as our "anchors" in 3D, then draw a vector between the two camera centers, from which we can triangulate the depth of each feature point. 

## That's a lot of work just to find a set of points!
Indeed, when written like the POC, there's a lot of complicated math that must be explicitly done. But luckily, we don't have to worry about that once we understand the theory. We can simply use the OpenCV library, which drastically cuts down on the amount of work we have to do. This POC's job was simply to teach how the sausage is made. For this use case, we've simply used 2 perspectives of a 3D cube, explicitly marked each feature point, then run the numbers to find the 3D coordinates of each corner.

## Ok, so what's with the output of the 3D points?
As you might notice, the 3D points provided by the POC aren't perfect. There will always be a margin of error. This is for 2 reasons. First, if the feature points are on the same or similar planes, the essential matrix becomes less accurate. This is because we aren't giving the computer nearly as much info about how point location changes with depth. The second reason is simply that it's impossible to create a perfect mathematical transformation that will apply to all points. It simply needs to be as accurate as possible. How good is good enough depends on your project requirements. 

That's all, I hope you learned something!