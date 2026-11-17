#!/usr/bin/env python3
"""在目标 Jetson 上将四角点装甲板 ONNX 转为 TensorRT 10 engine。"""

import argparse
from pathlib import Path


def main():
    """验证输入输出约定后构建模型，失败时不写入 engine 文件。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('onnx', type=Path, help='0526 对应的原始四角点装甲板 ONNX')
    parser.add_argument('--output', type=Path,
                        default=Path('src/rm_auto_aim/armor_detector/model/0526.engine'))
    parser.add_argument('--workspace-mib', type=int, default=1024)
    parser.add_argument('--fp16', action='store_true', help='允许内部 FP16 计算，I/O 仍为 FP32')
    args = parser.parse_args()
    if not args.onnx.is_file():
        parser.error(f'ONNX 不存在：{args.onnx}；不能直接使用 OpenVINO XML/BIN 或数字分类模型')
    if args.workspace_mib <= 0:
        parser.error('--workspace-mib 必须大于 0')
    try:
        import tensorrt as trt
    except ImportError:
        parser.error('缺少 TensorRT Python 模块；请在目标 Jetson 的开发环境中运行')
    if trt.__version__.split('.')[0] != '10':
        parser.error(f'需要 TensorRT 10.x，当前版本为 {trt.__version__}')

    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(0)
    onnx_parser = trt.OnnxParser(network, logger)
    if not onnx_parser.parse(args.onnx.read_bytes()):
        errors = '\n'.join(str(onnx_parser.get_error(i)) for i in range(onnx_parser.num_errors))
        parser.error(f'ONNX 解析失败：\n{errors}')
    if network.num_inputs != 1 or network.num_outputs != 1:
        parser.error('模型必须恰有一个输入和一个输出，不支持附带 NMS 的普通目标框模型')
    input_tensor = network.get_input(0)
    expected_shape = (1, 3, 640, 640)
    if len(input_tensor.shape) != 4 or any(
        actual not in (-1, expected)
        for actual, expected in zip(input_tensor.shape, expected_shape)
    ):
        parser.error(f'需要 NCHW [1,3,640,640] 输入，实际为 {input_tensor.shape}')
    output_tensor = network.get_output(0)
    if len(output_tensor.shape) != 3 or output_tensor.shape[0] not in (-1, 1) \
            or output_tensor.shape[2] != 22:
        parser.error(f'需要四角点输出 [1,N,22]，实际为 {output_tensor.shape}')
    if input_tensor.dtype != trt.float32 or output_tensor.dtype != trt.float32:
        parser.error('模型输入输出须为 FP32；预处理和角点后处理由检测器执行')
    input_tensor.shape = expected_shape
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, args.workspace_mib << 20)
    if args.fp16:
        config.set_flag(trt.BuilderFlag.FP16)
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        parser.error('engine 构建失败，请检查 CUDA、TensorRT 日志及可用内存')

    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(serialized)
    if engine is None:
        parser.error('生成的 engine 无法反序列化')
    for i in range(engine.num_io_tensors):
        name = engine.get_tensor_name(i)
        shape = tuple(engine.get_tensor_shape(name))
        if engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
            valid_shape = shape == expected_shape
        else:
            valid_shape = len(shape) == 3 and shape[0] == 1 and 0 < shape[1] <= 1000000 \
                and shape[2] == 22
        if (not valid_shape or engine.get_tensor_dtype(name) != trt.float32
                or engine.get_tensor_format(name) != trt.TensorFormat.LINEAR
                or engine.get_tensor_location(name) != trt.TensorLocation.DEVICE):
            parser.error(f'不兼容的 engine 张量：{name} {shape}；要求 FP32 LINEAR，输出 [1,N,22]')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(bytes(serialized))
    print(f'已生成 {args.output.resolve()}，TensorRT {trt.__version__}；请仅用于当前设备环境。')


if __name__ == '__main__':
    main()
