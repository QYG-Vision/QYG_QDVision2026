#!/usr/bin/env bash
set -euo pipefail

# 在 Jetson GPU 容器中生成可追溯模型，先通过转换检查再构建 engine。
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_dir"
set +u
load_openvino() { source /opt/openvino/setupvars.sh; }
load_openvino
set -u

model_dir="$repo_dir/record-test/models"
tools_dir="$repo_dir/record-test/model-tools"
mkdir -p "$model_dir"
python3 -m venv --system-site-packages "$tools_dir"
# pip 对同版本仍可能优先访问远端，显式复用已下载的大型 ONNX wheel。
for wheel in openvino2onnx-1.0.0-py3-none-any.whl \
    onnx-1.17.0-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl; do
    if [[ -f "$repo_dir/record-test/tool-wheels/$wheel" ]]; then
        "$tools_dir/bin/python" -m pip install --no-deps "$repo_dir/record-test/tool-wheels/$wheel"
    fi
done
"$tools_dir/bin/python" -m pip install -r scripts/jetson/requirements-model-tools.txt
"$tools_dir/bin/python" scripts/jetson/convert_armor_ir.py --output "$model_dir/0526.onnx"
python3 scripts/jetson/build_armor_engine.py "$model_dir/0526.onnx" \
    --output "$model_dir/0526-fp32.engine"
python3 scripts/jetson/build_armor_engine.py "$model_dir/0526.onnx" \
    --output "$model_dir/0526-fp16.engine" --fp16

echo "模型已生成；请继续运行 CPU/GPU 检测结果回归后再验收视频链路。"
