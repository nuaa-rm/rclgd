#!/bin/sh
echo -ne '\033c\033]0;rclgd\a'
base_path="$(dirname "$(realpath "$0")")"
"$base_path/rclgd.x86_64" "$@"
