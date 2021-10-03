# obstacle_avoidance
obstacle_avoidance

```bash
state = 0 right obstacle
state = 1 left obstracle
state = 2 no obstracle detection
```

lidarで観測する前方180度の内、最も短い距離(閾値は調整)が閾値以下なら/cmd_vel_obstacle_avoidanceを0.0、stateを衝突回避としてpublish  
以下の手順で衝突回避の試験を行う事が出来る
    
```bash
sudo chmod 666 /dev/ttyUSB0
source devel/setup.bash
roslaunch supervisor supervisor.launch 
```

[supervisor](https://github.com/TSUKUBA-CHALLENGE/supervisor)
