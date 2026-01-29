########################################################################
# ZED headless point cloud capture (no visualization)
########################################################################

import argparse
import pyzed.sl as sl
import sys

def parse_args(init, opt):
    if opt.input_svo_file and opt.input_svo_file.endswith((".svo", ".svo2")):
        init.set_from_svo_file(opt.input_svo_file)
        print("[Sample] Using SVO file:", opt.input_svo_file)

    elif opt.ip_address:
        if ":" in opt.ip_address:
            ip, port = opt.ip_address.split(":")
            init.set_from_stream(ip, int(port))
        else:
            init.set_from_stream(opt.ip_address)
        print("[Sample] Using stream:", opt.ip_address)

    res_map = {
        "HD2K": sl.RESOLUTION.HD2K,
        "HD1200": sl.RESOLUTION.HD1200,
        "HD1080": sl.RESOLUTION.HD1080,
        "HD720": sl.RESOLUTION.HD720,
        "SVGA": sl.RESOLUTION.SVGA,
        "VGA": sl.RESOLUTION.VGA,
    }
    if opt.resolution in res_map:
        init.camera_resolution = res_map[opt.resolution]

def main(opt):
    print("Running headless ZED point cloud capture")

    init = sl.InitParameters(
        depth_mode=sl.DEPTH_MODE.ULTRA,
        coordinate_units=sl.UNIT.METER,
        coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP,
        depth_maximum_distance=40.0,
    )

    parse_args(init, opt)

    zed = sl.Camera()
    if zed.open(init) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera")
        sys.exit(1)

    point_cloud = sl.Mat()

    print("Press Ctrl+C to save and exit")

    try:
        while True:
            if zed.grab() == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU)

    except KeyboardInterrupt:
        print("\nSaving point cloud to Pointcloud.ply")
        err = point_cloud.write("Pointcloud.ply")
        if err == sl.ERROR_CODE.SUCCESS:
            print("Saved Pointcloud.ply successfully")
        else:
            print("Failed to save point cloud:", err)

    zed.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_svo_file', type=str, default='')
    parser.add_argument('--ip_address', type=str, default='')
    parser.add_argument('--resolution', type=str, default='')
    opt = parser.parse_args()

    if opt.input_svo_file and opt.ip_address:
        print("Specify only one input source")
        sys.exit(1)

    main(opt)
