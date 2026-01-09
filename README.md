# NDT Robot Workspace

这是一个 STM32 嵌入式开发工作区，包含主要的机器人控制代码和相关的测试/示例代码。

## 目录结构

- **CODE/**
  - **NDT_Robot_Main/**: 机器人主控程序 (基于 STM32 HAL 库)
  - **STM32F103VCT6/**: STM32F103VCT6 开发板相关的示例代码 (基于标准外设库)
    - **5_Official_Demo_Code/**: 官方演示代码
    - **6_LED_Test_Code/**: LED 测试代码

## 开发环境配置

本工作区已配置 VS Code 环境，支持 C/C++ 代码浏览和 IntelliSense。

### 依赖项

- **VS Code 插件**:
  - C/C++ (ms-vscode.cpptools)
  - (可选) Cortex-Debug (用于调试)
  - (可选) Embedded Tools (用于寄存器查看等)

- **工具链**:
  - GNU Arm Embedded Toolchain (推荐)
  - 或者 Keil MDK / IAR (如果是使用外部 IDE 进行编译)

### 编译说明

- 如果项目使用 Keil MDK (`.uvprojx`)，请直接用 Keil 打开对应的工程文件。
- 如果使用 GCC + Make/CMake，请确保已安装工具链并在终端运行构建命令。

## 备注

`.vscode` 文件夹中包含了 `c_cpp_properties.json`，配置了递归包含路径，以便 IntelliSense 能找到所有头文件。
