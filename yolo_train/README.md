# YOLO 训练文档说明

本目录包含 YOLO 目标检测训练、推理及环境测试的相关文件。

### 文件说明

| 文件名 | 说明 |
| :--- | :--- |
| train.py | 模型训练文件。读取 marker.yaml 中的配置启动训练。 |
| test_env.py | 环境检测文件。用于验证 GPU (CUDA) 环境及 PyTorch 是否安装正确。若仅使用 CPU 训练可忽略。 |
| test_video.py | 视频推理文件。输入待检测视频，输出包含检测框和标签的视频结果。 |
| marker.yaml | 数据集配置文件。定义了训练集/验证集的路径、检测类别数及类别名称。 |

### 训练流程

#### 1. 数据采集
采集图像数据，并存入 dataset/images 目录下（目录结构见下文）。

#### 2. 数据标注
使用 LabelImg 对图像进行标注。

**注意**：
* 标签格式需选择 YOLO 格式
* 源数据目录：dataset/images
* 标签输出目录：dataset/labels

#### 3. 数据集划分与整理
标注完成后，按照 8:2 的比例将数据划分为训练集和验证集。最终目录结构应如下所示：

```text
dataset/
├── images/
│   ├── train/    # 存放训练集图片
│   └── val/      # 存放验证集图片
└── labels/
    ├── train/    # 存放训练集标签
    └── val/      # 存放验证集标签
```

#### 4. 启动训练
1.  修改 marker.yaml 文件中的路径配置。
2.  运行 train.py 开始训练。
3.  训练结果将保存在 runs/detect 目录下。

### 环境配置
运行以下命令安装 YOLO 依赖：
```bash
pip install ultralytics
```
默认的命令仅安装 CPU 版本的 torch ，若需使用 GPU 进行训练，则需卸载当前的 torch、torchvision 和 torchaudio ，并在官网上安装 GPU 版本