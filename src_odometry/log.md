ROS 2 시간 체계

https://answers.ros.org/question/287946/ros-2-time-handling/

* #include <ros/time.h> => #include <rclcpp/rclcpp.hpp>
* const ros::Time &time => const rclcpp::Time &time
* const ros::Duration &period => const rclcpp::Duration &period
* Time duration
```C++
const double dt = (time - timestamp_).toSec();
if (dt < 0.0001)
```

```C++
const double dt = (time - timestamp_).seconds();
if (dt < 0.0001)
```

단순하게 생각하자.

* joint_state & FloatMultiArray 받아서 speed, angle joint 값을 얻는다.
* odom계산해 주고 
* tf + odom topic을 publish 하자.
=> 이거 부터 하고 그 외의 기교를 넣자고

ackermann_steering_controller에서 가져와야 하는 것들 

* init
* update
* starting
* brake
* setOdomPubFields
=> 이정도?

odometry에서 사용되는 api
* init
* setVelocityRollingWindowSize
* setWheelParams
* udpate -> updateOpenLoop / udpate -> update
* getHeading / getX / getY / getLinear / getAngular

주의
항상 이 순서가 아니다.

- left_front_wheel_joint
- left_rear_wheel_joint
- right_front_wheel_joint
- right_rear_wheel_joint
- left_steering_hinge_joint
- right_steering_hinge_joint

Duration API 관련 - https://github.com/cra-ros-pkg/robot_localization/blob/b5345749e1449a5ec3536d3c8cc80f1de65ae9f6/src/navsat_transform.cpp#L196

ROS 1 - ros::Duration이 double도 받는다.
```
periodicUpdateTimer_ = nh.createTimer(ros::Duration(1./frequency), &NavSatTransform::periodicUpdate, this);
```

ROS 2 - rclcpp::Duration은 int seconds, int nanosecond만 받음, 따라서 아래와 같이 timer를 만들어줌
```
auto interval = std::chrono::duration<double>(1.0 / frequency);
timer_ = this->create_wall_timer(interval, std::bind(&NavSatTransform::transformCallback, this));
```

문제
현재 odom은 다음 두개의 값을 받음
```
double wheel_pos  = rear_wheel_joint_.getPosition();
double steer_pos = front_steer_joint_.getPosition();
```
gazebo에서는 뒷바퀴의 양쪽 회전각이 다르다.
/steering_angle_middle 처럼, /throttling_angle_middle을 만들어서 publish 해보자.

steering_angle_middle은 되지만, throttling_angle_middle은 안된다.
throttling_vel만 다룰 수 있음

!!결론!! => 양쪽 뒷바퀴 pose를 평균내서 사용하자. (linear 관계라고 가정하면 성립함)


주기적으로 0이 나온다. => 예외처리해줌
```
139.732 160.031 149.881
139.732 160.031 149.881
139.732 160.031 149.881
139.732 160.031 149.881
139.732 160.031 149.881
139.732 160.031 149.881
0 0 0
```

createQuaternionMsgFromYaw => 직접 만들어 쓰기
https://answers.ros.org/question/364561/tfcreatequaternionfromyaw-equivalent-in-ros2/

한바퀴 아직 안돌았는데 이미 한바퀴 넘어 있음
open loop 써도 그러네

=> controller가 잘못되었나 보다 ㅅㅂ...
수치가 잘못되었는지 살펴보자.

![image](https://user-images.githubusercontent.com/12381733/158050479-0305fa59-55b7-4563-95cf-3d66a2d3a24e.png)

샤시가 뒤쪽이었다.

![image](https://user-images.githubusercontent.com/12381733/158050513-d010aa3f-0a4c-4267-9c5e-0d4b72774f5e.png)

right_rear_wheel로는 바퀴 사이 거리 유추 불가

```
$ ros2 run tf2_ros tf2_echo chassis right_front_wheel
[INFO] [1647158272.592267167] [tf2_echo]: Waiting for transform chassis ->  right_front_wheel: Invalid frame ID "chassis" passed to canTransform argument target_frame - frame does not exist
At time 1647158273.536024634
- Translation: [0.325, -0.100, 0.000]
- Rotation: in Quaternion [-0.500, -0.500, 0.500, -0.500]
```

이건 이 둘 사이 거리니까, 실제 car_wheel_threat는 2배가 되어야지 ㅇㅇ 맞네 ㅠ
왼쪽 오른쪽 사이 거리 확인 

```xml
<joint name="left_steering_hinge_joint" type="revolute">
    <origin xyz="0.325 0.0 0" rpy="0 1.5708 0" />
    <parent link="chassis" />
```

![image](https://user-images.githubusercontent.com/12381733/158051253-99f378b6-4cd9-40f1-a3b4-d773c624a084.png)

완전 가운데가 아니네, 바퀴 두께가 있었음 (0.045)
```
<xacro:macro name="left_wheels_collision_geometry">
    <origin xyz="0 0 -0.0225" rpy="0 0 0" />
    <geometry>
        <cylinder length="0.045" radius="0.05" />
    </geometry>
</xacro:macro>
```
그렇다면, 실제 오른쪽/왼쪽 바퀴 사이 거리는 (0.1 + 0.0225) * 2 = 0.245
=> 여전히 빠르다. 

컨트롤러를 바꾸자. => Bycicle Model로!
```

```

rqt_plot을 통해 수치 비교를 해보자. (odom을 가지고)
에러 => rqt_plot 했는데, 시간축이 안간다.

![image](https://user-images.githubusercontent.com/12381733/158099629-8f8413ac-967e-4bf0-be5d-447b24357b25.png)

실제 topic stamp 업데이트가 안되고 있었음
```c++
odom->header.frame_id = odom_frame_id_;
odom->child_frame_id = base_frame_id_;
odom->header.stamp = last_state_publish_time_;
```

선속도 0.3 각속도 0.5여서 반지름 0.6짜리 원을 그려야 하는 상황
- open loop 사용시
![image](https://user-images.githubusercontent.com/12381733/158100684-4cbbfdec-9446-434a-ab48-1444218678ff.png)
rqt_plot 결과 odom은 맞다. => 그렇다면, 컨트롤러가 잘못 된 것!

- closed loop 사용 시
지름 0.75짜리가 나왔다.
![image](https://user-images.githubusercontent.com/12381733/158101170-aef02955-0ff4-4529-9a15-40d638f46c99.png)

=> 일단 open loop로 두고, 여기에 컨트롤러를 맞춰보자.

gazebo 절대 좌표도 같이 tracking하기
```
ros2 launch src_gazebo src_gazebo.launch.py
ros2 run src_gazebo_controller odom_utility_tools
ros2 run src_odometry src_odometry_gazebo
rqt
```

![image](https://user-images.githubusercontent.com/12381733/158103127-9dc1d328-60ac-42e6-a658-d735600dba9f.png)

지금 1.2가 나와야하는데
아래를 보면 실제 gt odom은 1.5 좀 적게 나온다. 
=> 컨트롤러 바꿔보자. (w를 키우거나, v를 줄이거나)

throttle값 통일
```
self.throttling_msg.data = [
    wheel_turnig_speed_com,
    wheel_turnig_speed_com,
    wheel_turnig_speed_com,
    wheel_turnig_speed_com,
]
```

지름 더 커짐, 약 1.7
![image](https://user-images.githubusercontent.com/12381733/158104023-6046f158-a62d-4770-9c23-1050e2030f5a.png)
회전각 변경
```
# alfa_right_front_wheel = math.atan(self.L / turning_radius_right_front_wheel)
alfa_right_front_wheel = math.atan( self.L / turning_radius_middle )

# alfa_left_front_wheel = math.atan(self.L / turning_radius_left_front_wheel)
alfa_left_front_wheel = math.atan( self.L / turning_radius_middle )
```
![image](https://user-images.githubusercontent.com/12381733/158104023-6046f158-a62d-4770-9c23-1050e2030f5a.png)
살짝 작아짐... 하지만 여전이 1.5근방

새로운 방식 써보자.
```
alfa_right_front_wheel = (omega_base_link * self.L ) / vel_base_link
```
![image](https://user-images.githubusercontent.com/12381733/158104849-617bac3a-6a5d-4805-9d72-4e172d57a5e3.png)
좀 더 작아졌지만 1.25 넘어버리는 상황

wheel speed를 순수 속도와 반지름으로만 사용하도록 수정
```
raw_wheel_speed = vel_base_link / self.wheel_radius
self.throttling_msg.data = [
    raw_wheel_speed,
    raw_wheel_speed,
    raw_wheel_speed,
    raw_wheel_speed,
]
```
아직도 1.25 넘는다.
![image](https://user-images.githubusercontent.com/12381733/158105482-7a9035e8-012b-478d-968f-adb35e760405.png)

angle에 atan값을 먹여보았다.
```
alfa_left_front_wheel = math.atan((omega_base_link * self.L ) / vel_base_link)
```
=> 더커진다 ㅠ

![image](https://user-images.githubusercontent.com/12381733/158106036-a71dc337-acd2-4947-92e2-7136394b6328.png)

손으로 직접 써본 결과, asin이 가장 유사했음
```
alfa_left_front_wheel = math.asin((omega_base_link * self.L ) / vel_base_link)
```

그럼, 이제 odom으로 가자. 일단 실행 
![image](https://user-images.githubusercontent.com/12381733/158144998-3d0ba615-eca6-4a5b-b7dd-62044b9af92e.png)
(갈길이 멀다 ㅠㅠ)

각속도 구하는 방식은 mit racecar와 동일하다.
```c++
    const double angular = tan(front_steer_pos) * linear / wheel_separation_h_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
```

일단 openloop부터 맞춰보자.

![image](https://user-images.githubusercontent.com/12381733/158158664-ef91c528-882f-429c-a9cc-9726621a47e4.png)

변위는 얼추 맞아보이는데, 타임 싱크가 다르다.
open_loop에 기체에 대한 정보는 전혀없음

왜 이런 시간 차이가 발생할까?

현재의 open_loop는 이러한 구조이다.
```
void cmd_vel_sub(const Twist::SharedPtr msg){
    linear_x = msg->linear.x;
    angular_z = msg->angular.z;
}

void odom_update(const rclcpp::Time &time){
    // COMPUTE AND PUBLISH ODOMETRY
    // TODO : open_loop implement & comparison
    if (open_loop_){
        odometry_.updateOpenLoop(linear_x, angular_z, time);
    }
```
문제될 건 없는데...

시간 문제일까? odom topic time과 odom 계산 time을 일치시켜봄
여전히 time 싱크가 안맞네

그냥 x 축만 와리가리 치면 아주 잘 감

![image](https://user-images.githubusercontent.com/12381733/158161500-6161f316-e579-4398-936f-687da1d85b0b.png)

원운동 살짝 해보면 바로 어긋남 => 결국 이 어긋남 때문에 주기가 밀려서 시간이 안맞게 되는 듯

그럼, closed loop는?

앞뒤 와리가리 후 가만히 있어본 결과
```
ros2 topic echo /ground_truth_x => data: 1.113367149944375
ros2 topic echo odom / x => 1.1913710991130013
Rear Wheel Pose : 22.266580
```
이렇게나 차이가 발생한다.
22.265 * 0.05 = 1.11325 이므로, odom 단에서 문제임, 찍어보자. 

```c++
const double rear_wheel_est_vel = rear_wheel_cur_pos - rear_wheel_old_pos_;

std::cout << "rear_wheel_pos : " << rear_wheel_pos << " / " << 
    "front_steer_pos" << front_steer_pos << " / " << 
    "rear_wheel_est_vel : " << rear_wheel_est_vel << std::endl;

### 결과
rear_wheel_pos : 75.1827 / front_steer_pos-0 / rear_wheel_est_vel : 0.00599996
rear_wheel_pos : 75.3027 / front_steer_pos-0 / rear_wheel_est_vel : 0.00599997
rear_wheel_pos : 75.3627 / front_steer_pos-0 / rear_wheel_est_vel : 0.00299999
rear_wheel_pos : 75.4827 / front_steer_pos-0 / rear_wheel_est_vel : 0.00599998
rear_wheel_pos : 75.6027 / front_steer_pos-0 / rear_wheel_est_vel : 0.00599998
rear_wheel_pos : 75.7227 / front_steer_pos-0 / rear_wheel_est_vel : 0.00599999
rear_wheel_pos : 75.7827 / front_steer_pos-0 / rear_wheel_est_vel : 0.003
rear_wheel_pos : 75.9027 / front_steer_pos-0 / rear_wheel_est_vel : 0.006
rear_wheel_pos : 76.0227 / front_steer_pos-0 / rear_wheel_est_vel : 0.00600001
rear_wheel_pos : 76.1427 / front_steer_pos-0 / rear_wheel_est_vel : 0.00600002
rear_wheel_pos : 76.2627 / front_steer_pos-0 / rear_wheel_est_vel : 0.00600003
rear_wheel_pos : 76.3227 / front_steer_pos-0 / rear_wheel_est_vel : 0.00300002
rear_wheel_pos : 76.4427 / front_steer_pos-0 / rear_wheel_est_vel : 0.00600005
rear_wheel_pos : 76.5627 / front_steer_pos-0 / rear_wheel_est_vel : 0.00600007
rear_wheel_pos : 76.6227 / front_steer_pos-0 / rear_wheel_est_vel : 0.00300003

0.1로 고정한 결과 
rear_wheel_pos : 29.9224 / front_steer_pos-0 / rear_wheel_est_vel : 0.00300062
rear_wheel_pos : 29.9624 / front_steer_pos-0 / rear_wheel_est_vel : 0.00199964
rear_wheel_pos : 29.9824 / front_steer_pos-0 / rear_wheel_est_vel : 0.000999845
rear_wheel_pos : 30.0424 / front_steer_pos-0 / rear_wheel_est_vel : 0.00299956
rear_wheel_pos : 30.0624 / front_steer_pos-0 / rear_wheel_est_vel : 0.000999869
rear_wheel_pos : 30.1024 / front_steer_pos-0 / rear_wheel_est_vel : 0.00199977
```
=> 이렇게 뒤죽박죽인 걸 그냥 썼다고?
=> rear_wheel_pos가 일정하게 증가하지 않는다.

dt로 나누어준 결과..
```
rear_wheel_pos : 3.9426 / front_steer_pos-0 / rear_wheel_est_vel : 0.100047
rear_wheel_pos : 4.0026 / front_steer_pos-0 / rear_wheel_est_vel : 0.130424
rear_wheel_pos : 4.0426 / front_steer_pos-0 / rear_wheel_est_vel : 0.117895
rear_wheel_pos : 4.0626 / front_steer_pos-0 / rear_wheel_est_vel : 0.0483452
rear_wheel_pos : 4.1026 / front_steer_pos-0 / rear_wheel_est_vel : 0.103535
rear_wheel_pos : 4.1226 / front_steer_pos-0 / rear_wheel_est_vel : 0.0466102
rear_wheel_pos : 4.18261 / front_steer_pos-0 / rear_wheel_est_vel : 0.161761
rear_wheel_pos : 4.20261 / front_steer_pos-0 / rear_wheel_est_vel : 0.0497566
rear_wheel_pos : 4.26261 / front_steer_pos-0 / rear_wheel_est_vel : 0.150726
rear_wheel_pos : 4.30261 / front_steer_pos-0 / rear_wheel_est_vel : 0.100022
rear_wheel_pos : 4.32261 / front_steer_pos-0 / rear_wheel_est_vel : 0.0498503
rear_wheel_pos : 4.38262 / front_steer_pos-0 / rear_wheel_est_vel : 0.150567
rear_wheel_pos : 4.40262 / front_steer_pos-0 / rear_wheel_est_vel : 0.0497586
rear_wheel_pos : 4.44262 / front_steer_pos-0 / rear_wheel_est_vel : 0.100079
rear_wheel_pos : 4.48262 / front_steer_pos-0 / rear_wheel_est_vel : 0.0998396
```
plot 결과도 갑자기 널뛰기 한다.


참고로 wheel_separation_h_는 앞뒤 바퀴 사이 거리이다.

자유롭게 돌아다녀본 결과 (제자리 돌아오기)

<img width="1438" alt="image" src="https://user-images.githubusercontent.com/12381733/158166262-6f531e4f-a56b-499c-94dd-0f2cccf6f260.png">

y 쪽이 좀 이상한데? => 이거 좀 확인해보자.

angle 관련해서 뭔가 잘못된 듯 - 가되어야 하는데 + 가 됨
![image](https://user-images.githubusercontent.com/12381733/158322854-bf35303e-5434-49fd-974d-5b3a6208385e.png)

엄밀히 말해서, linear와 angular는 속도가 아니다.
각도 차이, 위치 차이이다.

```c++
/// Compute linear and angular diff:
const double linear  = rear_wheel_est_vel;//(right_wheel_est_vel + left_wheel_est_vel) * 0.5;
//const double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation_w_;
const double angular = tan(front_steer_pos) * linear / wheel_separation_h_;\
```

다시 시작해보자.
https://medium.com/@waleedmansoor/how-i-built-ros-odometry-for-differential-drive-vehicle-without-encoder-c9f73fe63d87

ㄱ 자 주행 중 angle이 틀어지는 것 잡아야 한다.
아마 오차가 누적되어서 그런 것이 아닐까...

차이는 보통 이정도 난다.

```
header:
  stamp:
    sec: 1647782566
    nanosec: 719725250
  frame_id: ''
name:
- left_front_wheel_joint
- left_rear_wheel_joint
- right_front_wheel_joint
- right_rear_wheel_joint
- left_steering_hinge_joint
- right_steering_hinge_joint
position:
- 56.95777640412754
- 55.261805380446454
- 65.56167399551187
- 63.936011147333474
- 0.262864251819253
- 0.26294262204133556
```

소수점 아래 6자리에서 반올림한 수로 해보자
 

앞뒤 테스트는 완료, 잘 된다.

angle test를 경험적으로 해보자.

racecar는 이렇게 조향된다. 사실 ackermann이 아닌 것이지

![image](https://user-images.githubusercontent.com/12381733/159205806-4422c7ac-6961-4533-b1f3-3fbe85131b76.png)

회전 중심을 바퀴 중심으로 바꾸고 joint 위치를 수정하였다.

![image](https://user-images.githubusercontent.com/12381733/159218684-56776ddb-1514-46a1-ab0f-180407fafd5a.png)

R_real과 R_odom 비교 결과 1.1 정도의 차이가 있었다.
wheel_separation_h_multiplier_를 1.1로 변경하자.
=> 오키! 이정도면 합격!!

gazebo odom은 이제 그만~


# TODO

* [] realtime stuffs
    - https://docs.ros.org/en/foxy/Tutorials/Real-Time-Programming.html#id4
    - http://control.ros.org/

```
ros2 launch src_gazebo src_gazebo.launch.py

ros2 run src_gazebo_controller odom_utility_tools

ros2 run src_odometry src_odometry_gazebo
# or / 대신 launch 하면 std::cout가 안나옴
ros2 launch src_odometry src_odometry.launch.py

rqt

ros2 run src_odometry src_odometry_real

```

Odom blinking => Invalid tf tree 

```
ros2 run tf2_tools view_frames.py
```

```
docker run -it --rm --name micro-ros-foxy --net=host -v /dev:/dev --privileged tge1375/sw-micro-ros:0.0.4
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy4.0

ros2 launch src_demo joystick_control_foxy.launch.py

ros2 run mw_ahrsv1_ros2 mw_ahrsv1_ros2
ros2 run src_odometry src_odom

# Integrated version
ros2 launch src_odometry src_bringup.launch.py

ros2 launch src_odometry view_imu.launch.py 
```

기존 tan 방식 버리고 imu heading을 사용하는 odom을 만들었음
이게 제일 잘된다!
```
ros2 launch src_gazebo src_gazebo_empty_world.launch.py
ros2 launch src_odometry src_odometry_gazebo.launch.py
ros2 run src_gazebo_controller odom_utility_tools
rqt # 4개
```

![image](https://user-images.githubusercontent.com/92073907/161692652-83be20f4-f5a6-409b-96ee-aa5034950f2a.png)

```
ros2 launch src_gazebo src_gazebo_world.launch.py

```

rviz에서 odom => lidar가 안나오는 문제 발생

시간 문제다. gazebo 시간으로 바꿔야 함
계산시는 몰라도 publish 할 시는 시간 맞춰주자

/clock topic sub

/odom 변경
/tf 변경

이제 odom도 다 합쳐두었다.

```
ros2 launch src_gazebo src_gazebo_racecourse.launch.py
ros2 launch src_gazebo src_gazebo_empty_world.launch.py
```

Lidar와 Sensor Fusion

```

```

# Refactoring

gazebo odom 통합

```
docker run -it --rm --name micro-ros-foxy --net=host -v /dev:/dev --privileged tge1375/sw-micro-ros:0.0.4
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy4.0

ros2 launch src_demo joystick_control_foxy.launch.py    
ros2 launch src_odometry src_bringup.launch.py
```

```
ros2 launch cmd_to_src cmd_to_src.launch.py
ros2 launch src_odometry src_bringup.launch.py
```

부호 변경 시 진동 발생

