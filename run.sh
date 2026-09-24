#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

PYTHON_BIN="python3"
if [[ -x .venv/bin/python ]]; then
	PYTHON_BIN=".venv/bin/python"
fi

XVFB_PID=""
APP_PID=""

display_available() {
	[[ -n "${DISPLAY:-}" ]] || return 1
	command -v xdpyinfo >/dev/null 2>&1 || return 1
	xdpyinfo -display "$DISPLAY" >/dev/null 2>&1
}

find_free_display() {
	local display_num
	for display_num in {99..109}; do
		if [[ ! -e "/tmp/.X${display_num}-lock" ]]; then
			printf ':%s\n' "$display_num"
			return 0
		fi
	done
	return 1
}

start_virtual_display() {
	command -v Xvfb >/dev/null 2>&1 || {
		echo "错误: 未找到 Xvfb。请先运行 ./setup_env.sh 安装无显示器录制依赖。" >&2
		exit 1
	}

	local display_name
	display_name="${DARTSYSTEM_VIRTUAL_DISPLAY:-}"
	if [[ -z "$display_name" ]]; then
		display_name="$(find_free_display)"
	fi

	local screen_size="${DARTSYSTEM_VIRTUAL_DISPLAY_SIZE:-1920x1080x24}"
	Xvfb "$display_name" -screen 0 "$screen_size" -nolisten tcp -ac >/tmp/dartsystem-xvfb.log 2>&1 &
	XVFB_PID=$!
	export DISPLAY="$display_name"
	export QT_QPA_PLATFORM=xcb

	for _ in {1..30}; do
		if display_available; then
			echo "已启动虚拟显示器 DISPLAY=$DISPLAY screen=$screen_size"
			return 0
		fi
		sleep 0.1
	done

	echo "错误: Xvfb 启动失败，请查看 /tmp/dartsystem-xvfb.log" >&2
	exit 1
}

cleanup() {
	if [[ -n "$XVFB_PID" ]]; then
		kill "$XVFB_PID" >/dev/null 2>&1 || true
	fi
}

terminate() {
	if [[ -n "$APP_PID" ]] && kill -0 "$APP_PID" >/dev/null 2>&1; then
		kill -TERM "$APP_PID" >/dev/null 2>&1 || true
		wait "$APP_PID" >/dev/null 2>&1 || true
	fi
	cleanup
	exit 0
}

trap terminate INT TERM
trap cleanup EXIT

if [[ "${DARTSYSTEM_FORCE_XVFB:-0}" == "1" ]]; then
	start_virtual_display
elif [[ "${DARTSYSTEM_AUTO_XVFB:-1}" == "1" ]] && ! display_available; then
	start_virtual_display
fi

"$PYTHON_BIN" main.py &
APP_PID=$!
wait "$APP_PID"
exit $?
