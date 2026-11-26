#!/usr/bin/env bash
# 在小电脑宿主机注册 Docker 自瞄服务，可从任意目录调用。
set -euo pipefail

print_only=false
if [[ "${1:-}" == --print-unit && $# == 1 ]]; then
    print_only=true
elif [[ $# != 0 ]]; then
    echo "用法：sudo bash $0 | bash $0 --print-unit" >&2
    exit 1
fi

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
service_user="${SUDO_USER:-$(id -un)}"
docker_bin="$(command -v docker)" || { echo "请先安装 Docker。" >&2; exit 1; }
service_path=/etc/systemd/system/rm.service

# ExecStart/ExecStop 使用双引号包裹路径；拒绝需要额外展开的特殊字符。
for value in "$repo_dir" "$service_user" "$docker_bin"; do
    if [[ "$value" == *[\$\%\"\\]* || "$value" == *$'\n'* || "$value" == *$'\r'* ]]; then
        echo '仓库路径、用户名或 Docker 路径含不支持的特殊字符。' >&2
        exit 1
    fi
done

compose=("$docker_bin" compose -f "$repo_dir/docker-compose.yaml"
    -f "$repo_dir/docker-compose.autostart.yaml")

unit_content="$(cat <<EOF
[Unit]
Description=QD Vision auto aim (Docker)
Requires=docker.service
Wants=network-online.target
After=docker.service network-online.target
StartLimitIntervalSec=0

[Service]
Type=simple
User=$service_user
WorkingDirectory=$repo_dir
ExecStart="$docker_bin" compose -f "$repo_dir/docker-compose.yaml" -f "$repo_dir/docker-compose.autostart.yaml" up --no-build --pull never --abort-on-container-exit --exit-code-from rv_devel rv_devel
ExecStop="$docker_bin" compose -f "$repo_dir/docker-compose.yaml" -f "$repo_dir/docker-compose.autostart.yaml" stop --timeout 30 rv_devel
Restart=always
RestartSec=10s
KillSignal=SIGINT
TimeoutStartSec=0
TimeoutStopSec=60s

[Install]
WantedBy=multi-user.target
EOF
)"

if "$print_only"; then
    printf '%s\n' "$unit_content"
    exit 0
fi

if [[ "$EUID" != 0 ]]; then
    echo "请在小电脑宿主机运行：sudo bash $repo_dir/src/rm_upstart/register_service.sh" >&2
    exit 1
fi
if [[ -f /.dockerenv || ! -d /run/systemd/system ]]; then
    echo "请在运行 systemd 的小电脑宿主机注册服务，不要在容器中运行。" >&2
    exit 1
fi
if [[ ! -r "$repo_dir/install/setup.bash" ]]; then
    echo "工作区尚未编译：请先在容器 /ros_ws 下完成 colcon build。" >&2
    exit 1
fi

# 以实际服务用户验证配置、Docker 权限和本地镜像，注册时不构建或下载镜像。
runuser -u "$service_user" -- "${compose[@]}" config --quiet
runuser -u "$service_user" -- "$docker_bin" info >/dev/null
vision_image="$(runuser -u "$service_user" -- "${compose[@]}" config --images)"
runuser -u "$service_user" -- "$docker_bin" image inspect "$vision_image" >/dev/null

# 防止复用同名但挂载其他仓库的开发容器。
if "$docker_bin" container inspect rv_devel_ >/dev/null 2>&1; then
    mounted_workspace="$("$docker_bin" inspect rv_devel_ --format \
        '{{range .Mounts}}{{if eq .Destination "/ros_ws"}}{{.Source}}{{end}}{{end}}')"
    if [[ "$mounted_workspace" != "$repo_dir" ]]; then
        echo "rv_devel_ 挂载的不是当前仓库：$mounted_workspace。请先确认容器归属。" >&2
        exit 1
    fi
fi

# 更新已有服务前先停止它，使旧 ExecStop 有机会完成清理。
if systemctl is-active --quiet rm.service; then
    systemctl stop rm.service
fi
printf '%s\n' "$unit_content" > "$service_path"
systemctl daemon-reload
systemctl enable docker.service
systemctl enable rm.service
systemctl restart rm.service
echo "已注册并启动 rm.service。查看运行日志：journalctl -u rm.service -f"
