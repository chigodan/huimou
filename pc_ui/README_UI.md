# 慧眸电脑控制台

Windows串口界面，用于选择视频模式、设置图像参数和读取目标坐标。视频由FPGA经HDMI输出。

## 编译和启动

需要Windows与.NET Framework 4.x。在仓库根目录的PowerShell中运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File pc_ui/tools/Build.ps1
```

编译生成`pc_ui/bin/FpgaControl.exe`。随后双击`Launch.cmd`启动，或双击`Demo.cmd`进入离线演示。演示标有模拟数据，不打开COM口。

## 操作

1. 选择实际COM口，串口参数为115200、8N1、无流控。
2. 打开串口，选择视频模式，等待FPGA确认。
3. 勾选AWB、AE并点击“应用图像设置”。界面在FPGA确认后更新实际状态。
4. 显示亮度范围为-128～+127，默认0；“亮度归零”恢复0。该设置只影响显示，不进入测光和检测。
5. 状态区显示测光值、曝光码、增益档、红蓝增益和目标坐标，日志记录设置及状态变化。

AWB、AE上电默认开启。关闭AWB使用单位增益，空间颜色校正保留；关闭AE保持最近曝光与增益。曝光码不等于毫秒，增益档不等于百分比，红蓝增益显示为Q16换算倍率。

观察自动调节时先将显示亮度归零。固定照明并关闭AE，再切换AWB比较颜色；改变照明后开启AE，观察测光、曝光、增益和状态变化。

## 离线检查

在仓库根目录运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File pc_ui/tools/Test-Ui.ps1
```

脚本编译应用及测试程序，检查协议和界面交互，在`verification`目录中生成普通及紧凑窗口预览。此过程不连接实板串口，也不运行TD。生成目录不纳入Git。

## 源码与协议

| 文件 | 作用 |
| --- | --- |
| src/MainForm.cs | 界面与交互 |
| src/SerialController.cs | 串口后台与状态模型 |
| src/Protocol.cs | 协议编解码 |
| src/Program.cs | 程序入口 |
| src/Tests.cs | 离线回归与预览 |

协议采用AA55帧头、版本1、小端数据及CRC；STATUS为28字节。亮度使用0x11/0x12，图像开关与查询使用0x13/0x14。界面按FPGA能力位启用功能，图像设置不接管算法控制权。

完整参数见[图像调节与串口协议](../fpga/doc/IMAGE_AUTO_UART.md)。
