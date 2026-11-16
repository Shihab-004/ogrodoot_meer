

```markdown
# 🤖 MEER Real Rover — Operator Manual & Terminal Run Guide

এই গাইডটি ফলো করে ল্যাপটপ থেকে **Raspberry Pi (`rovershihab`)** কন্ট্রোল করে রিয়েল রোভারে সম্পূর্ণ **Autonomous Navigation Mission** পরিচালনা করতে পারবেন।

---

## 📶 ০. প্রি-স্টার্ট: নেটওয়ার্ক ও SSH কানেকশন (সব টার্মিনালের জন্য)

1. মোবাইলের **Hotspot** অন করুন এবং ল্যাপটপটি সেই Hotspot-এ কানেক্ট করুন।
2. Raspberry Pi-তে পাওয়ার দিন এবং ১-২ মিনিট অপেক্ষা করুন।
3. মোবাইলের Hotspot-এর *Connected Devices* লিস্ট থেকে `rovershihab`-এর **IP Address** দেখে নিন (যেমন: `10.138.212.18`)।
4. ল্যাপটপে যেকোনো টার্মিনাল খুলে Pi-এর ভেতরে ঢুকুন:

```bash
ssh rover@10.138.212.18

```

*(নোট: প্রতিবার নতুন টার্মিনাল ট্যাব ওপেন করে প্রথমে এই `ssh` কমান্ড দিয়ে Pi-এর ভেতরে ঢুকে নিচের কমান্ডগুলো চালাবেন।)*

---

## 🧹 ১. প্রি-মিশন ক্লিনআপ (সব পুরোনো প্রসেস বন্ধ করা)

মিশন শুরু করার আগে অথবা কোনো নোড আটকে গেলে Pi-এর টার্মিনালে নিচের কমান্ডটি চালান:

```bash
pkill -9 -f "nav2|ros2|tree_vitality|autonomous_mission|lidar|motor" || true

```

---

## ⚙️ ২. `colcon build` কখন লাগবে?

কোড বা কনফিগ ফাইলে কোনো পরিবর্তন আনলে তবেই একবার বিল্ড দেবেন:

```bash
cd ~/ogrodoot_meer
colcon build --symlink-install
source install/setup.bash

```

---

## 🖥️ ৩. ধাপে ধাপে রিয়েল রোভার মিশন পরিচালনা (৪টি টার্মিনাল গাইড)

ল্যাপটপে **৪টি আলাদা টার্মিনাল ট্যাব** ওপেন করুন (প্রথম ৩টি SSH দিয়ে Pi-তে, এবং ৪ নম্বরটি আপনার ল্যাপটপে):

---

### 🟢 Terminal 1 (Pi via SSH): Hardware, LiDAR, Odometry & Nav2

*(এটি রিয়েল রোভারের বডি URDF, LiDAR, এনকোডার/মোটর ড্রাইভার এবং Nav2 নেভিগেশন চালু করবে)*

```bash

cd ~/ogrodoot_meer
source install/setup.bash
ros2 launch meer_bringup real.launch.py

```

> ⚠️ **গুরুত্বপূর্ণ নোট:** ল্যাবে গিয়ে আমাদের `meer_bringup/launch/` ফোল্ডারে `real.launch.py` ফাইলটি তৈরি করে নিতে হবে (যাতে `use_sim_time:=false` দিয়ে LiDAR, Motor/Encoder এবং Nav2 একসাথে চালু হয়)।

---

### 🟢 Terminal 2 (Pi via SSH): Tree Vitality AI Vision Subsystem

*(এটি রিয়েল ক্যামেরা থেকে গাছের ছবি নিয়ে ALIVE/DEAD ও VARI সূচক নির্ণয়কারী AI সার্ভিস চালু করবে)*

```bash

cd ~/ogrodoot_meer
source install/setup.bash
ros2 launch rover_vision tree_vitality.launch.py use_sim_time:=false

```

> ✅ **টিপ:** টার্মিনালে `Tree Vitality Node ready` মেসেজ এলে পরের ধাপে যাবেন।

---

### 🟢 Terminal 3 (Pi via SSH): Autonomous Mission Start Signal

*(এই কমান্ড দেওয়া মাত্র রোভার বেস স্টেশন থেকে একা একা যাত্রা শুরু করবে)*

#### অপশন (ক): ডিফল্ট কোঅর্ডিনেট দিয়ে চালাতে

```bash

cd ~/ogrodoot_meer
source install/setup.bash
ros2 run meer_mission_control autonomous_mission_runner.py

```

#### অপশন (খ): বিচারকদের দেওয়া নতুন কোঅর্ডিনেট দিয়ে চালাতে

*(মনে করুন Point 1: $X = 18.5, Y = 3.2$ এবং Point 2: $X = 28.0, Y = -5.0$)*

```bash
cd ~/ogrodoot_meer
source install/setup.bash
ros2 run meer_mission_control autonomous_mission_runner.py --ros-args -p tree_x:=18.5 -p tree_y:=3.2 -p pano_x:=28.0 -p pano_y:=-5.0

```

---

### 🔵 Terminal 4 (Laptop Local — SSH ছাড়া): Live RViz2 Monitoring

*(Pi-এর ওপর চাপ না দিয়ে আপনার ল্যাপটপের স্ক্রিনে রোভারের লাইভ LiDAR স্ক্যান, ম্যাপ ও পাথ দেখতে)*

```bash
export ROS_DOMAIN_ID=42
rviz2

```

---

## 🧪 ৪. ল্যাবে প্রথমবার হার্ডওয়্যার চেক করার কুইক কমান্ড (Debugging)

মিশন চালানোর আগে সেন্সর ও মোটর ঠিকমতো কাজ করছে কি না তা এই কমান্ডগুলো দিয়ে চেক করবেন:

* **সব অ্যাক্টিভ টপিক দেখতে:**
```bash
ros2 topic list

```


* **LiDAR ডেটা আসছে কি না চেক করতে (`/scan`):**
```bash
ros2 topic hz /scan

```


* **এনকোডার ওডোমেট্রি চেক করতে (`/odom`):**
```bash
ros2 topic echo /odom --once

```


* **কীবোর্ড দিয়ে ম্যানুয়ালি চাকা টেস্ট করতে (চাকা মাটি থেকে উপরে তুলে):**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```



---

## 📂 ৫. মিশন শেষে সাবমিশন ফাইলগুলো ল্যাপটপে নিয়ে আসা

মিশন শেষ হলে সব লগ ও ছবি Raspberry Pi-এর `~/ogrodoot_meer/log/mission_submissions/` ফোল্ডারে জমা হবে।

* **Pi-এর টার্মিনালে বসে ফাইলগুলো দেখতে:**
```bash

ls -la ~/ogrodoot_meer/log/mission_submissions/
cat ~/ogrodoot_meer/log/mission_submissions/tree_verdict.log

```


* **সব লগ ও ছবি Pi থেকে আপনার ল্যাপটপের Desktop-এ কপি করে আনতে (ল্যাপটপের লোকাল টার্মিনালে চালাবেন):**
```bash
scp -r rover@10.138.212.18:~/ogrodoot_meer/log/mission_submissions ~/Desktop/

```



---

## 🛑 ৬. জরুরি স্টপ ও সেফ শাটডাউন

* **জরুরি মুহূর্তে রোভার থামাতে:** **Terminal 3**-এ `Ctrl + C` চাপুন (রোভার সাথে সাথে থেমে যাবে)।
* **কাজ শেষে Pi বন্ধ করতে (সরাসরি পাওয়ার খুলবেন না):**
```bash

sudo shutdown now

```



```

---


```