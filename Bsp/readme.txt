vcu-v1
2023.8.9
1.创建u6-vcu项目，git版本号vcu-v1。
2.创建can二值信号量，和整体事件1。
3.创建can线程，九轴线程。

问题：在串口重定向中，打算使用dma发送，但是无法实现串口打印。
vcu-v2
2023.8.11
1.编写4G-DTU设备驱动
2.创建onenet接受云端

2023.8.17
在调试九轴模时，通过CAN中断释放信号量同步下部处理数据，然后通过事件打印数据
再次过程中发生了数据丢失，并且稳定丢失中间两项数据，只打印acc与mag。
怀疑是CAN中断的频率远大于系统切换的频率，出现稳态数据丢失现象。
固将此部分数据放在中断上不处理。


vcu-v3
2023.8.17
取消onenet发送线程，将发送封装成一个接口，提供给其他线程进行4g通讯。
vcu-v3.1
2023.8.18
增加获取4g信号和时间功能。
vcu-v3.1.2
2023.8.19
通过标志置位实现dtu模块的配置和发送模式的相互随机抢占，导致模式进入失败问题，使其能够正常切换。
vcu-v3.2
2023.8.21
使用消息队列完成dtu的数据接受功能。
使用事件来互斥dtu进入配置模式和透传模式。
编写AT+CLK和AT+CSQ的数据机械代码。
调试了整体程序能够稳定运行。
具备功能：1.九轴数据CAN通讯采集，串口打印+4g传输至onenet云端
		  2.油门行程adc采集，串口打印+4g传输至onenet云端
		  3.4g-dtu，获取北京时间，和4g信号强度
