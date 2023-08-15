```mkdir -p ~/ros_catkin_ws/external_src<br>cd ~/ros_catkin_ws/external_src```
```wget <a href="http://sourceforge.net/projects/assimp/files/assi" rel="nofollow">http://sourceforge.net/projects/assimp/files</a>/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip```
```unzip assimp-3.1.1_no_test_models.zip```
```cd assimp-3.1.1```
```cmake .```
```make```
```sudo make install```