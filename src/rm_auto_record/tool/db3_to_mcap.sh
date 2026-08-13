#!/bin/bash
# 将指定目录下的所有 .db3 文件转换为 .mcap 文件
# 需要确保 mcap 工具在脚本所在目录下
INPUT_DIR="record_2025_9_20_10_33_51"

for db3_file in "$INPUT_DIR"/*.db3; do
    base_name=$(basename "$db3_file" .db3)
    mcap_file="$INPUT_DIR/${base_name}.mcap"

    echo "转换: $db3_file -> $mcap_file"
    ./mcap convert "$db3_file" "$mcap_file"
done
