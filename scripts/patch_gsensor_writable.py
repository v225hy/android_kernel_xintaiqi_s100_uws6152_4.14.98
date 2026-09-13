#!/usr/bin/env python3
"""
patch_gsensor_writable.py - 让 /sys/class/xr-gsensor/device/gsensor 节点可写

背景:
- sensors.ums312.so HAL 的 AccSensor::setEnable() 调用
    write_sys_attribute("/sys/class/xr-gsensor/device/", "gsensor", "0"/"1"/"2")
- 内核 da217.c 把这个 DEVICE_ATTR(gsensor, ...) 定义为 0444 (只读)
- 写只读 sysfs 文件 -> 返回 -EIO (-5) -> setEnable 返回 -5
- SensorService 报 "Error activating sensor 0 (I/O error)"
- 所有 user app 通过 SensorManager 拿不到加速度 xyz

修复:
- 将 DEVICE_ATTR(gsensor, 0444, gsensor_show, NULL) 改为
  DEVICE_ATTR(gsensor, 0644, gsensor_show, gsensor_enable_store)
  其中 gsensor_enable_store 复用原 enable 节点的 store 函数(写 1/0 控制 poll)
- "2" 是 HAL 在 enable=其它时写的字面量,kstrtoul("2")=2 进入 if(val) 分支,
  等同 1,自动兼容.
"""
import re
from pathlib import Path

CFILE = Path("/home/ubuntu/src/android_uws6152_4.14.98/drivers/iio/accel/da217.c")

assert CFILE.exists()
src = CFILE.read_text()

old = 'static DEVICE_ATTR(gsensor,    0444, gsensor_show,          NULL);'
new = ('static DEVICE_ATTR(gsensor,    0644, gsensor_show,          gsensor_enable_store);'
       '   /* Bug3(#13): HAL AccSensor::setEnable writes "0"/"1" to gsensor node;'
       ' was 0444 -> write returns -EIO(-5), activate fails, no xyz for user apps.'
       ' Reuse enable store: any non-zero starts polling, 0 stops it. */')

assert old in src, "old DEVICE_ATTR(gsensor, 0444 ...) not found"
assert src.count(old) == 1, f"expected exactly 1 match, got {src.count(old)}"

src2 = src.replace(old, new)
assert src2 != src

CFILE.write_text(src2)
print("[ok] patched", CFILE)
print("     DEVICE_ATTR(gsensor)  0444 -> 0644 + gsensor_enable_store")
