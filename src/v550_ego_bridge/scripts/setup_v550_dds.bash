#!/usr/bin/env bash

# 本文件必须用 source 执行，才能把变量写入当前终端，而不是只写入子进程。
if [ -n "${ZSH_VERSION:-}" ]; then # source不使用shebang选解释器，zsh需要自己的脚本路径语法。
    _v550_script_file="${(%):-%x}" # 获取被source文件，不能用当前工作目录代替。
else # Bash仍使用原生BASH_SOURCE，保持已有启动方式兼容。
    _v550_script_file="${BASH_SOURCE[0]}" # 获取实际脚本路径，允许从任意目录source。
fi # 两种shell统一由脚本位置定位配置。
_v550_script_dir="$(cd -- "$(dirname -- "$_v550_script_file")" && pwd)" # 解析成绝对目录。
if [ ! -r "${_v550_script_dir}/../config/cyclonedds_local.xml" ]; then # 配置不存在时立即报告，而非留到DDS初始化失败。
    echo "V550 DDS: 找不到 ${_v550_script_dir}/../config/cyclonedds_local.xml" >&2 # 显示实际查找路径便于定位。
    unset _v550_script_file _v550_script_dir # 清理source写入调用shell的临时变量。
    return 1 # 失败时不覆盖用户已有DDS环境。
fi # 配置有效才设置环境变量。

export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# file:// 后必须是绝对路径；CycloneDDS 启动每个 ROS 进程时都会读取此 XML。
export CYCLONEDDS_URI="file://${_v550_script_dir}/../config/cyclonedds_local.xml"

# 临时变量只用于计算配置路径，完成后删除，避免污染用户终端环境。
unset _v550_script_file _v550_script_dir

echo "V550 DDS: DOMAIN=${ROS_DOMAIN_ID}, LOCALHOST=${ROS_LOCALHOST_ONLY}, RMW=${RMW_IMPLEMENTATION}"
echo "V550 DDS: CYCLONEDDS_URI=${CYCLONEDDS_URI}"
