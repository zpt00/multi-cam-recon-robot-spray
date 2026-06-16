# list_realsense_devices.py
import pyrealsense2 as rs

ctx = rs.context()
devices = ctx.query_devices()

if len(devices) == 0:
    print("没有检测到 RealSense 设备")
else:
    print(f"检测到 {len(devices)} 台设备:\n")
    for i, dev in enumerate(devices):
        name = dev.get_info(rs.camera_info.name) if dev.supports(rs.camera_info.name) else "Unknown"
        serial = dev.get_info(rs.camera_info.serial_number) if dev.supports(rs.camera_info.serial_number) else "Unknown"
        firmware = dev.get_info(rs.camera_info.firmware_version) if dev.supports(rs.camera_info.firmware_version) else "Unknown"
        usb_type = dev.get_info(rs.camera_info.usb_type_descriptor) if dev.supports(rs.camera_info.usb_type_descriptor) else "Unknown"

        print(f"设备 {i}:")
        print(f"  Name     : {name}")
        print(f"  Serial   : {serial}")
        print(f"  Firmware : {firmware}")
        print(f"  USB Type : {usb_type}")
        print()
