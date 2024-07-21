# bt_sample

## インストール & ビルド
```
mkdir -p ros2_ws/src && cd ~/ros2_ws/src
```
srcにbt_sampleを配置
```
cd ~/ros2_ws
colcon build --packages-select bt_sample
```

## sample_no01について
ターミナル1
```
ros2 run bt_sample publisher_node
```

ターミナル2
```
ros2 run bt_sample sample_no01
```

## sample_no02について
ターミナル1
```
ros2 run bt_sample publisher_node
```

ターミナル2
```
ros2 run bt_sample sample_no02
```

## sample_no03について
ターミナル1
```
ros2 run bt_sample publisher_node
```

ターミナル2
```
ros2 run bt_sample sample_no03
```