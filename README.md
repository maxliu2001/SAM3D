# SAM Service 

## Demo Video
[SAM Service Demo](https://drive.google.com/file/d/1pzJgu_1q0TXr2X5bVeldBZIeMFzdJczH/view?usp=sharing)

## Service Definition

### Request
sensor_msgs/Image image <br>
geometry_msgs/Point[] points <br>
int32[] point_masks <br>
int32[] input_box <br>
bool multimask <br>
bool manual <br>
sensor_msgs/Image mask_input <br>

### Response
sensor_msgs/Image[] masks <br>
float32[] scores <br>
sensor_msgs/Image[] logits <br>

## Build the project
Run this in the root directory:
```
catkin_make
```

## Running the service 
In a fresh terminal, run the following:
```
source devel/setup.bash
```
then run:
```
rosrun sam3d sam_node.py
```

## Running the test client
In a fresh terminal, run the following:
```
source devel/setup.bash
```
then run:
```
rosrun sam3d sam_test_client.py
```


