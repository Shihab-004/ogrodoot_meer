# 🚀 NAGC 2026 Autonomous Navigation Mission — Operator Manual & Run Guide

এই নির্দেশিকাটি এমনভাবে তৈরি করা হয়েছে যেন যেকোনো সদস্য (এমনকি নন-ডেভেলপারও) খুব সহজে কোনো ভুল ছাড়াই টার্মিনালে কমান্ড কপি-পেস্ট করে সম্পূর্ণ **১০০ মার্কসের Autonomous Navigation Mission** সফলভাবে পরিচালনা করতে পারেন।

---

## 🧹 ১. প্রি-মিশন ক্লিনআপ (সব পুরোনো প্রসেস এক ক্লিকে কিল করা)

মিশন শুরু করার আগে অথবা পূর্বের রান আটকে গেলে ব্যাকগ্রাউন্ডের সব প্রসেস পরিষ্কার করতে যেকোনো টার্মিনালে নিচের কমান্ডটি রান করুন:

```bash
pkill -9 -f "gz sim|rviz2|nav2|ros2|tree_vitality|rqt" || true
```
> 💡 **কখন এটি চালাবেন?**  
> * প্রতিদিন প্রথমবার সিমুলেশন রান করার পূর্বে।  
> * কোনো কারণে Gazebo বা RViz2 হ্যাং হয়ে গেলে বা মিশন পুনরায় প্রথম থেকে শুরু করতে চাইলে।

---

## ⚙️ ২. `colcon build` কখন লাগবে এবং কখন লাগবে না?

| ক্ষেত্র | `colcon build` লাগবে কি? | কী করতে হবে? |
| :--- | :---: | :--- |
| **সাধারণ মিশন রান করার সময়** | **না ❌** | সরাসরি টার্মিনালে কমান্ড কপি-পেস্ট করে রান করবেন। বিল্ড করার দরকার নেই। |
| **টার্মিনালে নতুন GPS পয়েন্ট দেওয়ার সময়** | **না ❌** | টার্মিনালেই আর্গুমেন্ট পাস করা যায়, বিল্ড করার কোনো প্রয়োজন নেই। |
| **কোড বা কনফিগ ফাইল পরিবর্তন করলে** | **হ্যাঁ ✅** | পাইথন কোড, SDF এরেনা বা লাউঞ্চ ফাইল পরিবর্তন করলে একবার বিল্ড দিতে হবে। |

🛠️ **কোড পরিবর্তন করলে বিল্ড দেওয়ার কমান্ড:**
```bash
cd ~/autonomous_meer
colcon build --symlink-install
source install/setup.bash
```

---

## 🖥️ ৩. ধাপে ধাপে মিশন পরিচালনা (৩টি টার্মিনাল গাইড)

৩টি আলাদা টার্মিনাল উইন্ডো ওপেন করুন এবং ক্রমানুসারে নিচের কমান্ডগুলো চালান:

### 🟢 Terminal 1: Simulation, Robot & Nav2 Engine
*(এটি Gazebo সিমুলেশন, এরেনা, রোভার ফিজিক্স, Nav2 অটোনোমাস নেভিগেশন স্ট্যাক এবং RViz2 চালু করবে)*

```bash
cd ~/autonomous_meer
source install/setup.bash
ros2 launch meer_bringup sim.launch.py rviz:=true
```
> ⏳ **গুরুত্বপূর্ণ নোট:** কমান্ড দেওয়ার পর Gazebo ও RViz2 ওপেন হবে এবং ব্যাকগ্রাউন্ডে Nav2 ইঞ্জিন প্রস্তুত হতে **৮ থেকে ১০ সেকেন্ড** সময় দিন।

---

### 🟢 Terminal 2: Tree Vitality AI Vision Subsystem
*(এটি গাছের জীবন্ত/মৃত অবস্থা, ক্যানোপি ফলিয়েজ ও VARI সূচক নির্ণয়কারী AI অনবোর্ড সার্ভিস চালু করবে)*

```bash
cd ~/autonomous_meer
source install/setup.bash
ros2 launch rover_vision tree_vitality.launch.py
```
> ✅ **টিপ:** টার্মিনালে `Tree Vitality Node ready` মেসেজ দেখতে পাবেন।

---

### 🟢 Terminal 3: Autonomous Mission Start Signal
*(এটি আপনার অটোনোমাস মিশনের মূল স্টার্ট বাটন। এই কমান্ড দেওয়া মাত্র রোভার বেস স্টেশন থেকে একা একা যাত্রা শুরু করবে)*

#### অপশন (ক): ডিফল্ট পয়েন্ট দিয়ে চালাতে
```bash
cd ~/autonomous_meer
source install/setup.bash
ros2 run meer_mission_control autonomous_mission_runner.py
```

---

## 🎯 ৪. বিচারকরা নতুন GPS কোঅর্ডিনেট দিলে কীভাবে সেট করবেন?

প্রতিযোগিতার মাঠে বিচারকরা যখন চিরকুটে বা বোর্ডে Point 1 এবং Point 2 এর স্থানাঙ্ক দেবেন, তখন কীভাবে রান করবেন:

### ⚡ সবচেয়ে সহজ ও নিরাপদ উপায় (কোড এডিট ছাড়া এক লাইনে):
মনে করুন বিচারক আপনাকে নতুন পয়েন্ট দিলেন:
* **Tree Location (Point 1):** $X = 18.5\text{ m}, Y = 3.2\text{ m}$
* **Panorama Station (Point 2):** $X = 28.0\text{ m}, Y = -5.0\text{ m}$

তাহলে সরাসরি **Terminal 3**-এ এভাবে রান করবেন:
```bash
cd ~/autonomous_meer
source install/setup.bash
ros2 run meer_mission_control autonomous_mission_runner.py --ros-args -p tree_x:=18.5 -p tree_y:=3.2 -p pano_x:=28.0 -p pano_y:=-5.0
```
*(এখানে বিচারকের দেওয়া $X$ এবং $Y$ মানগুলো বসিয়ে এন্টার চাপলেই রোভার নতুন লোকেশনে চলে যাবে।)*

---

### 📝 কোড ফাইলে স্থায়ীভাবে পরিবর্তন করতে চাইলে:
ফাইল পাথ: [`src/meer_mission_control/scripts/autonomous_mission_runner.py`](file:///home/shihab/autonomous_meer/src/meer_mission_control/scripts/autonomous_mission_runner.py)  
ফাইলের ৪২ থেকে ৪৫ নম্বর লাইনে ডিফল্ট মান পরিবর্তন করুন:
```python
self.declare_parameter('tree_x', 18.5)
self.declare_parameter('tree_y', 3.2)
self.declare_parameter('pano_x', 28.0)
self.declare_parameter('pano_y', -5.0)
```
*(কোড ফাইল এডিট করলে টার্মিনালে একবার `colcon build --symlink-install` দিয়ে নিতে হবে।)*

---

## 📂 ৫. মিশন শেষে সাবমিশন ফাইলগুলো কোথায় পাবেন?

মিশন শেষ হওয়ামাত্র বিচারকদের কাছে জমা দেওয়ার জন্য সব ফাইল স্বয়ংক্রিয়ভাবে এই ফোল্ডারে সেভ হয়ে যাবে:

📁 **ফোল্ডার ডিরেক্টরি:**  
[`/home/shihab/autonomous_meer/log/mission_submissions/`](file:///home/shihab/autonomous_meer/log/mission_submissions/)

### 📋 জমাদানের ৩টি মূল ফাইল (রুলবুক ক্লজ 8.f.vi):
| ফাইলের নাম | বিবরণ |
| :--- | :--- |
| 📄 [**`tree_verdict.log`**](file:///home/shihab/autonomous_meer/log/mission_submissions/tree_verdict.log) | গাছের ডায়াগনস্টিক রিপোর্ট (ALIVE/DEAD রায়, VARI সূচক, ক্যানোপি কভারেজ % ও কনফিডেন্স) |
| 📸 [**`panorama_360.jpg`**](file:///home/shihab/autonomous_meer/log/mission_submissions/panorama_360.jpg) | ফুল ৩৬০° প্যানোরামা ছবি (**3840×1080 px** — রুলবুকের 1920×1080 এর দ্বিগুণ, হরাইজন লেভেল ±০°, নো ব্লার) |
| 📜 [**`onboard_run.log`**](file:///home/shihab/autonomous_meer/log/mission_submissions/onboard_run.log) | সম্পূর্ণ ফ্লাইট রেকর্ডার লগ (টাইমস্ট্যাম্প, জিপিএস ট্র্যাজেক্টরি পাথ, অবস্ট্যাকল এভয়ডেন্স ইভেন্ট ও মোট সময়) |
| 📸 [**`tree_snapshot.jpg`**](file:///home/shihab/autonomous_meer/log/mission_submissions/tree_snapshot.jpg) | পয়েন্ট ১-এ গাছের ক্রিস্টাল-ক্লিয়ার অপটিক্যাল ছবি |

---

## 🔍 ৬. ফাইল ও ছবি দেখার শর্টকাট কমান্ড

মিশন শেষে ফাইলগুলো এক ক্লিকে দেখার জন্য নতুন একটি টার্মিনালে নিচের কমান্ডগুলো ব্যবহার করতে পারেন:

* **ফোল্ডারটি ফাইল ম্যানেজারে খুলতে:**
  ```bash
  xdg-open ~/autonomous_meer/log/mission_submissions/
  ```
* **সব ছবি একবারে ইমেজ ভিউয়ারে দেখতে:**
  ```bash
  eog ~/autonomous_meer/log/mission_submissions/*.jpg
  ```
* **টার্মিনালে গাছের AI রায় দেখতে:**
  ```bash
  cat ~/autonomous_meer/log/mission_submissions/tree_verdict.log
  ```
* **টার্মিনালে অনবোর্ড ফ্লাইট লগ দেখতে:**
  ```bash
  cat ~/autonomous_meer/log/mission_submissions/onboard_run.log
  ```

---

## 🛑 ৭. সেফটি ও জরুরি কিল-সুইচ (Emergency Kill-Switch)

* জরুরী প্রয়োজনে যেকোনো মুহূর্তে রোভারের মুভমেন্ট থামাতে **Terminal 3**-এ কীবোর্ডের `Ctrl + C` চাপুন।
* কোড স্বয়ংক্রিয়ভাবে জিরো ভেলোসিটি পাঠিয়ে মোটরে ব্রেক প্রয়োগ করবে এবং রোভারকে সম্পূর্ণ নিরাপদভাবে সাথে সাথে থামিয়ে দেবে।
* রোভারের সর্বোচ্চ গতি ০.৬৫ m/s এ সীমাবদ্ধ রাখা হয়েছে (রুলবুকের ১.৫ m/s সীমার মধ্যে সম্পূর্ণ নিরাপদ)।
`