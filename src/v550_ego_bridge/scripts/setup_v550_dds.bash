#!/usr/bin/env bash

# 本文件必须用 source 执行，才能把变量写入当前终端，而不是只写入子进程。
_v550_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# file:// 后必须是绝对路径；CycloneDDS 启动每个 ROS 进程时都会读取此 XML。
export CYCLONEDDS_URI="file://${_v550_script_dir}/../config/cyclonedds_local.xml"

# 临时变量只用于计算配置路径，完成后删除，避免污染用户终端环境。
unset _v550_script_dir

echo "V550 DDS: DOMAIN=${ROS_DOMAIN_ID}, LOCALHOST=${ROS_LOCALHOST_ONLY}, RMW=${RMW_IMPLEMENTATION}"
echo "V550 DDS: CYCLONEDDS_URI=${CYCLONEDDS_URI}"
