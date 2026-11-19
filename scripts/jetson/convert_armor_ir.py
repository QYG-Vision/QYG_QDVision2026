#!/usr/bin/env python3
"""转换现有装甲板 IR，并使用参考图像和视频验证转换后的有效预测。"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", type=Path,
                        default=root / "src/rm_auto_aim/armor_detector/model/IR/0526.xml")
    parser.add_argument("--output", type=Path, default=root / "record-test/models/0526.onnx")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--video", type=Path, default=root / "test.mp4")
    args = parser.parse_args()

    import cv2
    import numpy as np
    import onnx
    import openvino as ov

    if not args.xml.is_file() or not args.xml.with_suffix(".bin").is_file():
        parser.error("必须同时提供现有 IR 的 XML 和 BIN")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not args.validate_only:
        subprocess.run([sys.executable, "-m", "openvino2onnx", str(args.xml),
                        str(args.output), "-v", "17", "-s"], check=True)

    model = onnx.load(str(args.output))
    # 转换器保留了未使用的池化索引；TensorRT 只支持 MaxPool 的数值输出。
    used = {name for node in model.graph.node for name in node.input}
    used.update(tensor.name for tensor in model.graph.output)
    for node in model.graph.node:
        if node.op_type == "MaxPool" and len(node.output) == 2:
            if node.output[1] in used:
                parser.error("池化索引参与后续计算，不能安全移除")
            del node.output[1:]
    onnx.save(model, str(args.output))
    onnx.checker.check_model(model)
    for tensor, expected in ((model.graph.input[0], [1, 3, 640, 640]),
                             (model.graph.output[0], [1, 25200, 22])):
        shape = [dim.dim_value for dim in tensor.type.tensor_type.shape.dim]
        if tensor.type.tensor_type.elem_type != onnx.TensorProto.FLOAT or shape != expected:
            parser.error(f"转换结果不符合 FP32 模型约定：{tensor.name} {shape}")

    core = ov.Core()
    settings = {"INFERENCE_NUM_THREADS": 2, "INFERENCE_PRECISION_HINT": "f32"}
    original = core.compile_model(core.read_model(str(args.xml)), "CPU", settings)
    converted = core.compile_model(core.read_model(str(args.output)), "CPU", settings)
    images = []
    reference = cv2.imread(str(root / "src/rm_auto_aim/armor_detector/docs/test.png"))
    if reference is not None:
        images.append(("reference", reference))
    cap = cv2.VideoCapture(str(args.video))
    if cap.isOpened():
        frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        for fraction in (0.0, 0.2, 0.4, 0.6, 0.8):
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(max(0, frames - 1) * fraction))
            success, image = cap.read()
            if success:
                images.append((f"video-{fraction:.1f}", image))
    cap.release()
    if not images:
        parser.error("没有可用参考图片或视频，不能验收转换结果")
    images.append(("black", np.zeros((480, 640, 3), dtype=np.uint8)))

    results = []
    accepted = 0
    for name, bgr in images:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        height, width = rgb.shape[:2]
        scale = min(640 / width, 640 / height)
        canvas = np.zeros((640, 640, 3), dtype=np.uint8)
        resized = cv2.resize(rgb, (int(width * scale), int(height * scale)))
        canvas[:resized.shape[0], :resized.shape[1]] = resized
        data = np.ascontiguousarray(canvas.transpose(2, 0, 1)[None], dtype=np.float32) / 255
        source_type = original.input().get_element_type().to_dtype()
        expected = original([data.astype(source_type)])[original.output()][0]
        actual = converted([data])[converted.output()][0]
        if not np.isfinite(expected).all() or not np.isfinite(actual).all():
            raise RuntimeError(f"{name}: 模型输出含 NaN/Inf")
        confidence_a = 1 / (1 + np.exp(-np.clip(expected[:, 8], -80, 80)))
        confidence_b = 1 / (1 + np.exp(-np.clip(actual[:, 8], -80, 80)))
        mask = (confidence_a >= 0.65) | (confidence_b >= 0.65)
        accepted += int(mask.sum())
        corner_error = float(np.max(np.abs(expected[mask, :8] - actual[mask, :8]))) if mask.any() else 0
        score_error = float(np.max(np.abs(confidence_a[mask] - confidence_b[mask]))) if mask.any() else 0
        classes_equal = bool(np.array_equal(expected[mask, 13:22].argmax(1),
                                            actual[mask, 13:22].argmax(1)))
        colors_equal = bool(np.array_equal(expected[mask, 9:13].argmax(1),
                                           actual[mask, 9:13].argmax(1)))
        threshold_equal = bool(np.array_equal(confidence_a >= 0.65, confidence_b >= 0.65))
        passed = corner_error <= 2.0 and score_error <= 0.02 and classes_equal and colors_equal and threshold_equal
        results.append({"sample": name, "candidates": int(mask.sum()), "passed": passed,
                        "max_corner_error_model_pixels": corner_error,
                        "max_confidence_error": score_error,
                        "classes_equal": classes_equal, "colors_equal": colors_equal,
                        "confidence_threshold_equal": threshold_equal})
        print(json.dumps(results[-1], ensure_ascii=False), flush=True)

    passed = accepted > 0 and all(result["passed"] for result in results)
    report = {"passed": passed, "method": "openvino2onnx-1.0.0-opset17",
              "samples": results, "sha256": {
                  str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                  for path in (args.xml, args.xml.with_suffix(".bin"), args.output)},
              "scope": "转换前后 OpenVINO CPU 数值检查；不代替 TensorRT 检测结果回归或实机精度验收"}
    report_path = args.output.with_suffix(".conversion.json")
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"转换检查{'通过' if passed else '未通过'}；报告：{report_path}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
