import numpy as np
import cv2
from open_karto import (Mapper, Dataset, Name, LaserRangeFinder,
                        LocalizedRangeScan, process_localized_range_scan,
                        Pose2, add_range_finder_to_dataset,
                        add_localized_range_scan_to_dataset,
                        LaserRangeFinderType, create_occupancy_grid,
                        GridStates, MapperWrapper)


def main():
    mw = MapperWrapper('laser0', 0.5, -1.0, 1.0)

    # mapper = mw.mapper
    # for param in dir(mapper):
    #     if param[0] == '_':
    #         continue
    #     print(param, eval('mapper.' + param))

    mw2 = MapperWrapper('laser3', 0.5, -1.0, 1.0)
    mw.reset()
    mw.range_finder.set_offset_pose(Pose2(1.0, 0.0, 0.0))
    mw.range_finder.set_angular_resolution(np.deg2rad(0.5))
    mw.range_finder.set_range_threshold(12.0)

    mw.process_scan([3.0] * 230, 0, 0, 0)
    mw.process_scan([3.0] * 230, 1.0, 0, 1.57)

    grid = mw.create_occupancy_grid(0.1)
    print(grid.width, grid.height, grid.offset)

    # TODO this should really happen on the C++ side
    map_img = np.zeros((grid.height, grid.width), np.uint8)
    for y in range(grid.height):
        for x in range(grid.width):
            val = grid.get_value(x, y)
            if val == GridStates.Unknown:
                map_img[y, x] = 128
            if val == GridStates.Free:
                map_img[y, x] = 255
            if val == GridStates.Occupied:
                map_img[y, x] = 0

    cv2.imwrite('/tmp/map.png', map_img)

    scan = mw.get_processed_scans()[-1]

    return scan


if __name__ == '__main__':
    s = main()
    print("out")
    print(s.get_corrected_pose())
