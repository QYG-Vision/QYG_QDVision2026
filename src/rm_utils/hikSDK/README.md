# 海康 SDK 架构说明

`lib/amd64` 配合 `include` 用于 x86-64；`lib/arm64` 配合 `include/arm64`
用于 Jetson / AArch64。两个架构的头文件与库分别配套，不混用版本。

本次 ARM64 文件取自本机已有的 `open-cv_-calibration/hikSDK`；该项目中库文件的
最近提交为 `440a35d`。包含相机控制、USB3 传输、格式转换、视频处理和渲染库，
替换了原 `arm64` 目录内误放的 x86-64 文件。厂商 SDK 文件保留原始内容。

构建时会检查相机控制库和 USB3 传输库的 ELF 架构。安装时会复制完整的选定架构
库目录，保证 USB 传输等动态加载依赖可用；ARM64 USB 库依赖 `libusb-1.0-0`。

使用单独安装的厂商 SDK 时，两个包均可传入同一根目录：

```bash
colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DHIK_SDK_ROOT=/opt/MVS -DBUILD_TESTING=OFF
```

支持 `include/` + `lib/64/` 的官方单架构安装布局，以及仓库当前的分架构布局。
