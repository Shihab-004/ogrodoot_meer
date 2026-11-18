# 🤖 MEER Real Rover — Operator Manual (Pi 5 + 2× ESP, USB cable)

## 🧠 আর্কিটেকচার (এক নজরে)

```
Nav2 / mission runner ──/cmd_vel──► esp32_bridge.py (Pi) ──USB cable "x,y\n"──► Motor ESP (NodeMCU) ──► মোটর
                                        ▲
Encoder+GPS ESP ──USB cable "L,R" ticks─┘──► /odom + TF (odom→base_footprint) ──► Nav2
LiDAR ──USB──► /lidar/scan ──► Nav2 costmap
```

* Pi মোবাইল হটস্পটে থাকলেও সমস্যা নেই — মোটর কমান্ড **USB ক্যাবল দিয়ে** যায়, WiFi লাগে না।
* মোটর ESP-এর প্রোটোকল: `"x,y"` (x = turn, y = drive, -100..100), 115200 baud। ১.৫ সেকেন্ড কমান্ড না এলে মোটর নিজে থেকে থামে।
* এনকোডার ডেটা না এলে bridge নিজে থেকে `/cmd_vel` থেকে open-loop odom বানায়, যাতে Nav2 আটকে না যায়।

---

## 🔧 A. এককালীন সেটআপ (একবারই করতে হবে)

### A1. মোটর ESP (NodeMCU)-তে আপডেট করা firmware আপলোড করুন — **ল্যাপটপ থেকে**

Firmware: `manual_rover/meer_2.0_ruet_rover/rover_esp` (আপনার গেমপ্যাড কোডই, শুধু USB Serial যোগ করা)।
* এখন একই `"x,y"` কমান্ড **WiFi UDP (গেমপ্যাড)** এবং **USB cable (Pi)** — দুই দিক থেকেই নেয়।
* WiFi না পেলে আর আটকে থাকে না, ১০ সেকেন্ড পর USB Serial মোডে চলে।

1. NodeMCU-টি USB দিয়ে **ল্যাপটপে** লাগান।
2. আপলোড করুন:
   ```bash
   cd ~/manual_rover/meer_2.0_ruet_rover/rover_esp && pio run -t upload
   ```
3. চেক: `pio device monitor` → `USB Serial control ready` লেখা আসবে। এরপর ESP-টি আবার Pi-এর USB-তে লাগান।

> এনকোডার/GPS ESP (`gps_esp`)-তে কিছু আপলোড করতে হবে না।

### A2. Pi-তে GitHub SSH key (একবার)

```bash
ssh rover@<PI_IP>
ssh-keygen -t ed25519 -C "rover-pi" -N "" -f ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub
```
যে লাইনটি দেখাবে সেটি GitHub → repo `ogrodoot_meer` → **Settings → Deploy keys → Add deploy key**-এ পেস্ট করুন (read-only যথেষ্ট)। তারপর:

```bash
ssh -T git@github.com          # "successfully authenticated" এলে ঠিক আছে
```

Pi-তে যদি আগে থেকে git clone **না থাকে**:
```bash
cd ~ && git clone git@github.com:Shihab-004/ogrodoot_meer.git
```
যদি আগে scp দিয়ে কপি করা ফোল্ডার থাকে, সেটির নাম বদলে (`mv ~/ogrodoot_meer ~/ogrodoot_meer_old`) তারপর clone করুন।

### A3. Pi-তে প্রয়োজনীয় প্যাকেজ (একবার)

```bash
sudo apt update && sudo apt install -y python3-serial ros-jazzy-teleop-twist-keyboard ros-jazzy-v4l2-camera
sudo usermod -aG dialout,video $USER     # তারপর একবার reboot
```
LiDAR ড্রাইভার (RPLIDAR/Slamtec) workspace-এ clone করুন:
```bash
cd ~/ogrodoot_meer/src && git clone https://github.com/Slamtec/sllidar_ros2.git
```
> প্যাকেজ না থাকলে launch আর crash করে না — শুধু `WARNING: package ... not installed -> skipping` দেখায়। এই লাইন দেখলে বুঝবেন ওই সেন্সর চালু হয়নি।
> LiDAR baud: A1/A2 = 115200 (ডিফল্ট), C1 = `lidar_baud:=460800`, A3/S1 = `lidar_baud:=256000`।

### A4. USB পোর্টের স্থায়ী নাম (udev) — একবার

Pi-তে প্রথমে একবার build করুন (B2 দেখুন), তারপর:
```bash
ros2 run meer_hardware find_rover_ports.sh
```
একটি করে ডিভাইস খুলে আবার চালালে বোঝা যাবে কোন `KERNELS` কোন ডিভাইসের। সেই মানগুলো `src/meer_hardware/udev/99-meer-rover.rules`-এ বসিয়ে:
```bash
sudo cp ~/ogrodoot_meer/src/meer_hardware/udev/99-meer-rover.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/rover_motor /dev/rover_encoder /dev/rplidar
```
> ⚠️ প্রতিটি ডিভাইস সবসময় **একই USB সকেটে** লাগাবেন।
> udev না করলে launch-এ পোর্ট হাতে দিতে হবে: `motor_port:=/dev/ttyUSB0 encoder_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB2`

---

## 🔄 B. প্রতিবার কোড আপডেট হলে

### B1. ল্যাপটপে (push)
```bash
cd ~/autonomous_meer && git add -A && git commit -m "update" && git push
```

### B2. Pi-তে (pull + build)
```bash
ssh rover@<PI_IP>
cd ~/ogrodoot_meer && git pull
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🧪 C. মিশনের আগে হার্ডওয়্যার টেস্ট (চাকা মাটি থেকে উপরে!)

### C1. ROS ছাড়া সরাসরি মোটর টেস্ট
```bash
ros2 run meer_hardware test_motor.py serial /dev/rover_motor 0 30 2
```
২ সেকেন্ড ধীরে সামনে ঘুরে থামলে **Pi → ESP লিঙ্ক ঠিক আছে**। টার্ন টেস্ট: `... 30 0 2`।

### C2. এনকোডার/GPS ESP কী পাঠাচ্ছে দেখুন
```bash
python3 -m serial.tools.miniterm /dev/rover_encoder 115200
```
(বের হতে `Ctrl+]`)। bridge এমন লাইন বোঝে: `123,456` / `ENC:123,456` / `E 123 456` (বাম, ডান টিক)। অন্য ফরম্যাট হলে bridge log-এ `Unrecognised line` দেখাবে — তখন সেই লাইনের নমুনা দিয়ে parser আপডেট করতে হবে।

### C3. ROS দিয়ে টেলিঅপ টেস্ট
Terminal 1:
```bash
ros2 launch meer_bringup real.launch.py launch_nav:=false launch_lidar:=true
```
Terminal 2:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
* উল্টো দিকে চললে: `invert_left_motor`/`invert_right_motor` (bridge param)।
* ছোট কমান্ডে না নড়লে `min_cmd_percent:=30`, বেশি জোরে চললে `max_wheel_speed:=1.5` (100%-এ রোভারের আসল গতি m/s মেপে বসান)।

---

## 🚀 D. অটোনোমাস মিশন

ক্লিনআপ:
```bash
pkill -9 -f "nav2|ros2|tree_vitality|autonomous_mission|lidar|esp32_bridge" || true
```

**Terminal 1 (Pi):** হার্ডওয়্যার + LiDAR + Nav2
```bash
cd ~/ogrodoot_meer && source install/setup.bash
ros2 launch meer_bringup real.launch.py odom_source:=cmd_vel
```
> এনকোডার ESP-এর ফরম্যাট নিশ্চিত না হওয়া পর্যন্ত `odom_source:=cmd_vel` দিন (ভুল ডেটা থেকে odom নষ্ট হবে না)। LiDAR + ক্যামেরা ডিফল্টে চালু হয়।

**Terminal 2 (Pi):** Vision
```bash
cd ~/ogrodoot_meer && source install/setup.bash
ros2 launch rover_vision tree_vitality.launch.py use_sim_time:=false
```

**Terminal 3 (Pi):** মিশন শুরু (**`use_sim_time:=false` অবশ্যই দিতে হবে**)
```bash
cd ~/ogrodoot_meer && source install/setup.bash
ros2 run meer_mission_control autonomous_mission_runner.py --ros-args -p use_sim_time:=false -p tree_x:=18.5 -p tree_y:=3.2 -p pano_x:=28.0 -p pano_y:=-5.0
```

**Terminal 4 (ল্যাপটপ, একই হটস্পটে):** RViz2
```bash
export ROS_DOMAIN_ID=42   # Pi-তেও একই ROS_DOMAIN_ID থাকতে হবে
rviz2
```

### দ্রুত চেক
```bash
ros2 topic hz /lidar/scan
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel
```

---

## 📂 E. সাবমিশন ফাইল
মিশন রানার ফাইল রাখে `~/autonomous_meer/log/mission_submissions/` (Pi-তে)।
```bash
scp -r rover@<PI_IP>:~/autonomous_meer/log/mission_submissions ~/Desktop/
```

## 🛑 F. জরুরি স্টপ
* Terminal 3-এ `Ctrl+C` → `/cmd_vel` বন্ধ → bridge ০.৫ সেকেন্ডে `0,0` পাঠায়; Pi/ক্যাবল খুলে গেলেও ESP ১.৫ সেকেন্ডে নিজে থামে।
* হার্ডওয়্যার কিল-সুইচ সবসময় হাতের কাছে রাখুন।
* বন্ধ করতে: `sudo shutdown now`
