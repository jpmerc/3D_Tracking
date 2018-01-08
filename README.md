# 3D_Tracking

I used ar_track_alvar ROS package to track individual tags (http://wiki.ros.org/ar_track_alvar). I added a ROS message to the package to publish a point cloud of the 3D corners of detected tags. Instead of using the standard solvePnP function of OpenCV to minimize the reprojection error, I applied RANSAC on the 3D corners and their corresponding real 3D coordinates to find the 3D transformation from the camera to the real tag coordinate system.

- The code was not meant to be shared, but since I've had many requests from the youtube video  (https://youtu.be/SdBDATclhXM), I've decided it would be simpler just to make it available here. 

- To make the code work, you will have to change the hardcoded tag positions (again, sorry for the not user friendly code!!).



<a href="http://www.youtube.com/watch?feature=player_embedded&v=SdBDATclhXM
" target="_blank"><img src="http://i3.ytimg.com/vi/SdBDATclhXM/maxresdefault.jpg" 
alt="Youtube Video" width="480" height="270" border="10" /></a>
