# 安装依赖
需要安装modbus库，使用以下命令安装：
```bash
sudo apt-get install -y protobuf-c-compiler libprotobuf-c-dev
```
# 如何绑定串口硬件设备？
添加设备的udev规则和自动设置latency_timer

```
先确定两只手爪的串口号，如：“/dev/ttyUSB0”,然后执行generate_serial_rules.sh两次文件，将左手手爪的设备别名设置为"stark_serial_L",右手设置为"stark_serial_R"
然后执行以下命令：
sudo udevadm control --reload-rules
sudo udevadm trigger
之后重启设备

```
# 测试方式
- 在~/kuavo/build$ 目录下执行 sudo ./lib/hand_sdk/hand_test  
