#!/usr/bin/env python3

import argparse
import logging
from dataclasses import dataclass
from os import getenv
from typing import Tuple

import numpy as np
from detection import Point, dist, draw_map, path_to_destination, setup_graph
from management import execute_bash_command

logger = logging.getLogger(__file__)


def graph_loop(mapfile="map.txt", enlarging_safety=2):
    rects, vertices, g, m = setup_graph(mapfile, enlarging_safety)

    def get_next(init: Point, end: Point, dryrun=False):
        path = path_to_destination(init, end, rects=rects, vertices=vertices, graph=g)
        if dryrun:
            draw_map(m, rects, lines=list(np.array(p) for p in path), point=end)

        return np.array(path[1])

    return get_next


SCALE_MAP_MM = 100  # map scale: 1 map unit = X mm
MIN_D_MM = 100  # 100 mm,


@dataclass
class Options:
    init: Tuple[int, int, int]
    end: Tuple[int, int]
    speed: int
    enlarge: int
    mapfile: str
    dryrun: bool = False

    def __post_init__(self):
        if len(self.init) != 3:
            raise TypeError("Init should be x, y, theta. Use as `--init X Y THETA`")
        if len(self.end) != 2:
            raise TypeError("end should be x, y. Use as `--end X Y`")
        if self.speed < 0 or self.speed > 100:
            raise TypeError("Speed should be between 10 100")
        self.init = tuple(self.init)
        self.end = tuple(self.end)

    @staticmethod
    def parse_args():
        parser = argparse.ArgumentParser(
            description="Program to manage the path of the main robot",
            epilog="""
        Environment variables for extra control:
        DEBUG (If value is 1, print extra information).
        ROBOT_USER (Username on machine of ROBOT_USER).
        ROBOT_IP (IP address to connect you).
        ROBOT_FOLDER (Folder of main robot executable on target machine).
        ROBOT_EXE (filename of robot executable on target machine).
        """,
        )
        parser.add_argument(
            "--init",
            nargs=3,
            type=int,
            default=[0, 0, 0],
            help="Initial position (x y theta).",
        )
        parser.add_argument(
            "--end",
            nargs=2,
            type=int,
            default=[700, 800],
            help="Destination (x y).",
        )
        parser.add_argument(
            "--speed",
            type=int,
            default=30,
            help="Expected speed of robot on a scale from 10 to 100.",
        )

        parser.add_argument(
            "--enlarge",
            type=int,
            default=200,
            help="Safety enlargement of obstacles in mm, it will get rounded to the map scale.",
        )

        parser.add_argument(
            "--dry-run",
            dest="dryrun",
            type=bool,
            default=False,
            action=argparse.BooleanOptionalAction,
            help="When this is true, do not actually execute movement, just draw a path.",
        )
        parser.add_argument(
            "--map-file",
            dest="mapfile",
            default="map.txt",
            type=str,
            help="File location of the map.",
        )
        args = parser.parse_args()
        return Options(
            init=args.init,
            end=args.end,
            speed=args.speed,
            dryrun=args.dryrun,
            mapfile=args.mapfile,
            enlarge=args.enlarge,
        )


def main():
    level = logging.INFO
    if getenv("DEBUG"):
        level = logging.DEBUG

    logging.basicConfig(level=level, format="%(message)s")
    o = Options.parse_args()
    logging.info("Options %s", o)

    init = o.init
    end = o.end

    get_next = graph_loop(
        mapfile=o.mapfile, enlarging_safety=int(o.enlarge / SCALE_MAP_MM)
    )

    x_mm, y_mm, theta = init
    x_mm_end, y_mm_end = end
    p_init = np.array([x_mm / SCALE_MAP_MM, y_mm / SCALE_MAP_MM])
    p_end = np.array([x_mm_end / SCALE_MAP_MM, y_mm_end / SCALE_MAP_MM])
    while (d := dist(p_end, p_init)) > MIN_D_MM / SCALE_MAP_MM:
        logger.info("Distance is %s", d)
        map_x_target, map_y_target = get_next(p_init, p_end, dryrun=o.dryrun)
        if o.dryrun:
            p_init = np.array([map_x_target, map_y_target])
            continue

        x_mm, y_mm, theta = execute_bash_command(
            x_mm,
            y_mm,
            theta,
            SCALE_MAP_MM * map_x_target,
            SCALE_MAP_MM * map_y_target,
            speed=o.speed,
        )
        p_init = np.array([x_mm / SCALE_MAP_MM, y_mm / SCALE_MAP_MM])


if __name__ == "__main__":
    main()
