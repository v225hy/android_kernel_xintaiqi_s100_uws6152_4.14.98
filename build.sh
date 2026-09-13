#!/bin/bash
###############################################################################
# build.sh — w527/ums6152_1h10 (SPRD SharkL5) 4.14.98 kernel build script
#
# 目标: 在已有 .config 基础上增量重编出 boot 用 Image/Image.gz。
#       兼容 CI (gcc4build.yml) 的 gcc 4.9 工具链与预处理补丁, 但默认复用
#       当前 .config, 不重复跑会改配置的 sed/scripts/config 步骤。
#
# 用法:
#   ./build.sh              增量编译 (默认, 复用现 .config)
#   ./build.sh --fresh      defconfig + CI 预处理补丁 + 编译 (从零)
#   ./build.sh menu         先 make menuconfig 再编译
#   ./build.sh --clean      先 make clean 再增量编译 (保守回滚点)
#
# 产物:
#   arch/arm64/boot/Image      未压缩内核 (boot 用 raw Image)
#   arch/arm64/boot/Image.gz   gzip 内核 (同棵树 dtc 修复后由 make 生成)
#   同目录下另留 *.img.bak_<ts> 时间戳备份, 供一键回滚。
###############################################################################

set -u
# 不要 set -e: 我们想让错误日志落盘后仍能给出可读诊断。

# ----------------------------- 路径常量 ------------------------------------
# 脚本允许从任意 cwd 调用, 自动切到内核源码根。
KROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$KROOT"

# gcc 4.9 交叉工具链 (和 CI 一致, 纯 gcc 不用 clang)
TC="/home/ubuntu/tc/aarch64-linux-android-4.9/bin"
[ -x "$TC/aarch64-linux-android-gcc" ] || TC="/home/ubuntu/lineage-toolchain/bin"
if [ ! -x "$TC/aarch64-linux-android-gcc" ]; then
    echo "!! ERROR: 找不到 aarch64-linux-android-gcc 工具链"
    echo "  查找过: $TC"
    exit 2
fi

export ARCH=arm64
export CROSS_COMPILE="$TC/aarch64-linux-android-"
# gcc 4.9 对 4.14 新头文件的告警, 与 CI 相同, 抑制错误上升
export KCFLAGS="-Wno-error=sizeof-pointer-memaccess -Wno-error=array-bounds -Wno-error=missing-braces -Wno-error=format"

DEFCONFIG="sprd_sharkl5_s100_defconfig"
JOBS="$(nproc)"
LOG="build.log"

# ----------------------------- 预处理补丁 (仅 --fresh) --------------------
# 与 .github/workflows/gcc4build.yml 的 gcc 4.9 适配步骤保持一致。
# 幂等: 用脚本判断, 避免重复 sed 插入重复行。
apply_prep_patches() {
    echo "== [--fresh] 应用 gcc4build.yml 预处理补丁 =="

    scripts/config --disable CONFIG_APPENDED_DTB
    scripts/config --disable CONFIG_EFI
    scripts/config --disable CONFIG_EFI_STUB

    # dtc yaml 修复 (避免 yamltree 编译失败) — 幂等
    if grep -q 'yamltree\.o' scripts/dtc/Makefile; then
        sed -i '/yamltree\.o/d' scripts/dtc/Makefile
    fi
    if ! grep -q '// dt_to_yaml(' scripts/dtc/dtc.c; then
        sed -i 's/dt_to_yaml(/\/\/ dt_to_yaml(/g' scripts/dtc/dtc.c
    fi

    # stack protector 修复 — 幂等
    sed -i '/ifdef CONFIG_CC_STACKPROTECTOR_STRONG/{n;s/stackp-flag :=.*/stackp-flag := -fstack-protector-strong/}' Makefile
    sed -i '/ifdef stackp-name/,/endif/{s/exit 1/true/}' Makefile
    sed -i '/ifdef stackp-check/,/endif/{s/exit 1/true/}' Makefile
}

# ----------------------------- 参数解析 ------------------------------------
FRESH=0; MENU=0; CLEAN=0
for a in "$@"; do
    case "$a" in
        --fresh) FRESH=1 ;;
        menu|--menu) MENU=1 ;;
        --clean) CLEAN=1 ;;
        *) echo "!! 未知参数: $a"; echo "用法见脚本头部注释"; exit 1 ;;
    esac
done

# ----------------------------- 主流程 --------------------------------------
echo "=============================================================="
echo " KERNEL BUILD — $DEFCONFIG"
echo " 根目录 : $KROOT"
echo " 工具链 : $CROSS_COMPILE"
echo " jobs   : $JOBS   模式: fresh=$FRESH menu=$MENU clean=$CLEAN"
echo "=============================================================="

# 1. 备份旧 Image, 保证可回滚
TS="$(date +%Y%m%d_%H%M%S)"
for img in arch/arm64/boot/Image arch/arm64/boot/Image.gz; do
    if [ -f "$img" ]; then
        cp -a "$img" "${img}.bak_${TS}"
        echo "[backup] $img -> ${img}.bak_${TS}"
    fi
done

# 2. 配置准备
if [ "$FRESH" -eq 1 ]; then
    echo "== 生成 defconfig: $DEFCONFIG =="
    make "$DEFCONFIG" || { echo "!! defconfig 失败"; exit 1; }
    apply_prep_patches
    make olddefconfig || { echo "!! olddefconfig 失败"; exit 1; }
elif [ "$MENU" -eq 1 ]; then
    if [ ! -f .config ]; then
        echo "!! 无 .config, 先用 defconfig 生成"
        make "$DEFCONFIG" || exit 1
        apply_prep_patches
        make olddefconfig || exit 1
    fi
    echo "== 进入 menuconfig (保存退出后继续编译) =="
    make menuconfig || exit 1
fi

if [ ! -f .config ]; then
    echo "!! 没有 .config 且未指定 --fresh/menu"
    echo "   将用 $DEFCONFIG 生成 (如需保留手工配置请先 make menuconfig 导出)"
    make "$DEFCONFIG" || exit 1
    apply_prep_patches
    make olddefconfig || exit 1
fi

# 3. clean (可选)
if [ "$CLEAN" -eq 1 ]; then
    echo "== make clean =="
    make clean || exit 1
fi

# 4. 编译内核本体 (Image / Image.gz)
echo "== 编译中 ... (日志: $LOG) =="
if make -j"$JOBS" Image Image.gz 2>&1 | tee "$LOG"; then
    # 管道退出码取 make
    MSTAT="${PIPESTATUS[0]}"
else
    MSTAT="${PIPESTATUS[0]}"
fi

if [ "${MSTAT:-0}" -ne 0 ]; then
    echo "!! 编译失败 (make exit=$MSTAT), 完整日志见 $LOG"
    echo "   回滚: 旧镜像仍在 *.bak_$TS"
    exit "$MSTAT"
fi

# 5. 产物汇总
echo "=============================================================="
echo " BUILD OK"
REL="$(make -s kernelrelease 2>/dev/null | grep -v '^#' | tail -1)"
for img in arch/arm64/boot/Image arch/arm64/boot/Image.gz; do
    if [ -f "$img" ]; then
        echo "  $img  $(du -h "$img" | cut -f1)  md5=$(md5sum "$img" | cut -d' ' -f1)"
    fi
done
echo " kernelrelease : $REL"
echo " vmlinux       : $(ls -lh vmlinux 2>/dev/null | awk '{print $5}')"
echo "=============================================================="
