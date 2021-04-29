# USB CAN分析仪驱动节点

## overview
    读取CAN分析仪数据并以ros消息的形式发布，[CAN分析仪购买地址](https://item.taobao.com/item.htm?spm=a1z2k.11010449.931864.62.275f509dPWvRhX&scm=1007.13982.82927.0&id=18286496283&last_time=1589127269)

## USB 权限设置，创建新的udev规则，99-usbcan.rules
    1. 创建 /etc/udev/rules.d/99-usbcan.rules ,创建文件时需要sudo权限
    2. 文件内容 ACTION=="add",SUBSYSTEMS=="usb", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="0053", GROUP="users", MODE="0777"
    3. 重新插拔设备

## Nodes

### Node usbcan_driver_node
    读取CAN分析仪数据并以ros消息的形式发布

#### Params
* **`filter_mode`** (vector<int>(2))
    vector类型参数，分别配置两个CAN通道
    滤波模式: 0/1 接收所有类型帧  2，只接收标准帧  3，只接收拓展帧

* **`mAccCode`** (vector<int>(2))
    vector类型参数，分别配置两个CAN通道
    滤波器屏蔽位

* **`mAccCode`** (vector<int>(2))
    vector类型参数，分别配置两个CAN通道
    滤波器掩码位

* **`baudrate`** (vector<int>(2))
    vector类型参数，分别配置两个CAN通道
    波特率，目前支持 125/250/500/1000kbps，可补充

* **`frame_id`** (vector<string>(2))
    vector类型参数，分别配置两个CAN通道
    ROS消息frame_id ,不同的frame_id,用于区分不同通道

#### Published Topics

* **`/from_usbcan`** ([can_msgs/FrameArray])
    发布从CAN分析仪读取的消息帧，不同的frame_id用于区分不同通道

#### Subscribed Topics

* **`/to_usbcan`** ([can_msgs/FrameArray])
    订阅CAN消息并通过CAN分析仪发布到CAN总线，不同的frame_id用于区分不同通道