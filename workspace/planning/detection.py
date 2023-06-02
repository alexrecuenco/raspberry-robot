# https://www.jeffreythompson.org/collision-detection/line-line.php
# https://pypi.org/project/Dijkstar/
# pollygon collision detection https://www.jeffreythompson.org/collision-detection/poly-point.php

import dataclasses
import logging
from contextlib import suppress
from enum import Enum, auto
from functools import cached_property
from itertools import chain, repeat, tee
from typing import Callable, Iterable, List, Optional, Tuple, Union

import matplotlib.pyplot as plt
import numpy as np
from dijkstar import Graph, find_path
from numpy.typing import NDArray

logger = logging.getLogger(__file__)


class Corner(Enum):
    TL = auto()
    TR = auto()
    BL = auto()
    BR = auto()


class Direction(Enum):
    UP = auto()
    DOWN = auto()
    LEFT = auto()
    RIGHT = auto()


UP = np.array([0, 1])
DOWN = np.array([0, -1])
LEFT = np.array([-1, 0])
RIGHT = np.array([1, 0])


Point = NDArray


@dataclasses.dataclass
class Rect:
    """Bottom left corner"""

    _p: Point
    h: float = 1
    w: float = 1
    index: int = 2

    def segments(self):
        yield self.bl, self.br
        yield self.br, self.tr
        yield self.tr, self.tl
        yield self.tl, self.bl

    def points(self):
        yield self.bl
        yield self.br
        yield self.tr
        yield self.tl

    def has_inside(self, p: Point, *, threshold: float = 0.0) -> bool:
        """
        Returns True if the point p is inside the rectangle (with an optional threshold),
        False otherwise.

        Positive threshold means more leanient, we allow if it is just on the edges
        Negative threshold means that if you are in the edge or near it you are also considered inside

        """
        x, y = p

        if (
            self.bl[0] + threshold <= x <= self.tr[0] - threshold
            and self.bl[1] + threshold <= y <= self.tr[1] - threshold
        ):
            return True
        else:
            return False

    def p(self, corner: Corner) -> Point:
        if corner == Corner.BL:
            return self._p.copy()
        if corner == Corner.BR:
            return self._p + self.w * RIGHT
        if corner == Corner.TL:
            return self._p + self.h * UP
        if corner == Corner.TR:
            return self._p + self.h * UP + self.w * RIGHT

        raise ValueError("Incorrect corner")

    @cached_property
    def bl(self):
        return self.p(Corner.BL)

    @cached_property
    def br(self):
        return self.p(Corner.BR)

    @cached_property
    def tl(self):
        return self.p(Corner.TL)

    @cached_property
    def tr(self):
        return self.p(Corner.TR)


def v(p1: Point, p2: Point) -> Point:
    return p2 - p1


def dot(p1: Point, p2: Point) -> Point:
    return np.vdot(p1, p2)


def cross(p1: Point, p2: Point) -> float:
    return np.cross(p1, p2)


def dist(p1: Point, p2: Point):
    return np.linalg.norm(p2 - p1)


#   float uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));
#   float uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));


def draw_map(m: NDArray, rects: Iterable[Rect], lines: Iterable[Point], point: Point):
    # Create a figure and axis object using matplotlib
    fig, ax = plt.subplots()

    # Display the data as an image
    # Remember that array indexing is (row, column) -> (y, x) but by math conventions / plotting we specify points (x, y).
    # our m, we have decided, comes with the convention m[x, y]
    im = ax.imshow(m.T, origin="lower")

    # Display the collision rectangles
    for rect in rects:
        for (x1, y1), (x2, y2) in rect.segments():
            ax.plot([x1, x2], [y1, y2], "g-")

    linetype = "r-"

    for p1, p2 in pairwise(lines):
        ix, iy = p1
        x, y = p2
        ax.plot([ix, x], [iy, y], linetype)
        linetype = "y-"

    # Add a marker at the end point
    ex, ey = point
    ax.plot(ex, ey, "bo")

    # Add a colorbar for reference
    plt.colorbar(im)

    # Show the plot
    plt.show()


def line_detector(
    a1: Point, a2: Point, b1: Point, b2: Point, *, epsilon=0.1
) -> Optional[Point]:
    # epsilon is giving a small margin of error
    # https://stackoverflow.com/a/565282
    # segment point a1, a2 = a1+v1, b1, b2=b1+v3

    # p + r s = q + t r, r,t in [0,1]
    # s x p = s x q + t s x r
    # t = s x (p - q) / s x r
    #
    # a1 + r va = b1 + t vb, r,t in [0,1]
    # va x a1 = va x b1 + t va x vb
    # t = - va x (b1 - a1) / va x vb
    # r = vb x (b1 - a1) / vb x va
    # r in (0,1) is good

    v_a = v(a1, a2)
    v_b = v(b1, b2)
    v_ab = v(a1, b1)
    va_cross_vb = cross(v_a, v_b)
    if va_cross_vb == 0:
        return None

    va_cross_ab = cross(v_a, v_ab)

    t = -va_cross_ab / va_cross_vb
    if t < -epsilon or t > 1 + epsilon:
        return None

    vb_cross_ab = cross(v_b, v_ab)
    r = -vb_cross_ab / va_cross_vb

    if r < -epsilon or r > 1 + epsilon:
        return None

    return b1 + t * v_b

    """
    we could also do (which works on more than 2D, with some extra help)
    Using https://web.archive.org/web/20111108065352/https://www.cs.mun.ca/~rod/2500/notes/numpy-arrays/numpy-arrays.html

    def seg_intersect(a1,a2, b1,b2) :
    va = a2-a1
    vb = b2-b1
    vab = a1-b1
    vap = perp(va)
    denom = dot( vap, vb)
    num = dot( vap, vab )
    return (num / denom)*vb + b1

    explanation:

    a1 + x va = b1 + y vb
    vap = orthogonal to va

    vap(a1 + x va) = vap (b1 + y vb)
    vap * a1 = vap * b1 + y vap* vb
    y  = vap * (a1-b1) / (vap * vb)
    p = b1 + y vb = b1 + vb \cdot vap * (a1-b1) / (vap * vb)

    You need to do this for both
    """


def rect_detector(a1: Point, a2: Point, rect: Rect) -> Iterable[Point]:
    for b1, b2 in rect.segments():
        p = line_detector(a1, a2, b1, b2)
        if p is not None:
            yield p


def get_rect(x: float, y: float) -> Rect:
    return Rect(np.array([x, y]))


def find_vertex(x, y, dir: Direction, *, map: NDArray, edge_val) -> Optional[Point]:
    xsize, ysize = map.shape
    itx: Iterable[int] = repeat(x)
    ity: Iterable[int] = repeat(y)

    if dir == Direction.RIGHT:
        itx = range(x + 1, xsize)
    if dir == Direction.LEFT:
        itx = range(x - 1, -1, -1)
    if dir == Direction.DOWN:
        ity = range(y - 1, -1, -1)
    if dir == Direction.UP:
        ity = range(y + 1, ysize)

    for x, y in zip(itx, ity):
        v = map[x, y]
        if v == edge_val:
            return np.array((x, y))
        if v == 0:
            break

    return None


def iterate_corners(x, y, f: Callable[[int, int, NDArray], None], *, map: NDArray):
    xsize, ysize = map.shape
    if x > 0:
        f(x - 1, y, map)
    if x + 1 < xsize:
        f(x + 1, y, map)
    if y > 0:
        f(x, y - 1, map)
    if y + 1 < ysize:
        f(x, y + 1, map)


def count_around(x, y, *, map: NDArray):
    count = 0

    def count_up(x, y, map):
        nonlocal count
        if map[x, y]:
            count += 1

    iterate_corners(x, y, count_up, map=map)
    return count


def get_collision_rects(map: NDArray) -> Iterable[Rect]:
    iter_map = np.nditer(map, flags=["multi_index"])
    # First index will be 2
    rect_idx = 2
    for map_value in iter_map:
        value = float(map_value)
        if value > 1:
            x, y = iter_map.multi_index
            if count_around(x, y, map=map) != 2:
                raise ValueError(
                    "Rectangles are touching, we don't know how to do this"
                )
            up = find_vertex(x, y, Direction.UP, map=map, edge_val=value)
            right = find_vertex(x, y, Direction.RIGHT, map=map, edge_val=value)
            if up is not None and right is not None:
                rect_idx += 1
                _, upy = up
                rightx, _ = right
                rect = Rect(np.array((x, y)), h=upy - y, w=rightx - x, index=rect_idx)
                logging.debug("Adding rect %s", rect)
                yield rect


def get_vertices(rects: List[Rect], d: int):
    t = np.array([0, d])
    r = np.array([d, 0])
    b = np.array([0, -d])
    l = np.array([-d, 0])
    for rect in rects:
        yield rect.bl + b + l
        yield rect.br + b + r
        yield rect.tl + t + l
        yield rect.tr + t + r


def cp(graph: Graph) -> Graph:
    return graph.subgraph(list(graph.get_data().keys()))


def pairwise(iterable, connect_back=False):
    # modified https://docs.python.org/3.8/library/itertools.html#itertools-recipes
    """s -> (s0, s1), (s1, s2), (s2, s3), ...

    If connect_back is True, the last step is (sn, s0)
    """
    a, b = tee(iterable)
    last = next(b, None)
    if connect_back and last is not None:
        b = chain(b, [last])
    return zip(a, b)


def connect_rects(
    collision_rects: Iterable[Rect],
    *,
    graph: Graph,
    cost=dist,
):
    for rect in collision_rects:
        for p1, p2 in rect.segments():
            if collides_with_rectangles(
                (p1, p2), list(nrect for nrect in collision_rects if nrect is not rect)
            ):
                continue

            c = cost(p1, p2)
            idx_1 = tuple(p1)
            idx_2 = tuple(p2)
            graph.add_edge(idx_1, idx_2, c)
            graph.add_edge(idx_2, idx_1, c)

    return graph


def collides_with_rectangles(
    line: Tuple[Point, Point], collision_rects: Iterable[Rect], *, threshold=0.01
):
    v1, v2 = line
    for rect in collision_rects:
        if rect.has_inside(v1, threshold=threshold) or rect.has_inside(
            v2, threshold=threshold
        ):
            return True
        if rect.has_inside((v1 + v2) / 2, threshold=threshold):
            return True
        for p in rect_detector(v1, v2, rect):
            # Allow the corners to be added
            if dist(p, v1) > threshold and dist(p, v2) > threshold:
                return True
    return False


def connect_points(
    points: List[Point],
    collision_rects: Iterable[Rect],
    *,
    graph: Graph,
    cost=dist,
):
    n = len(points)
    for i, vertex_1 in enumerate(points):
        for j in range(i + 1, n):
            vertex_2 = points[j]
            if collides_with_rectangles(
                line=(vertex_1, vertex_2), collision_rects=collision_rects
            ):
                continue
            c = cost(vertex_1, vertex_2)
            idx_1 = tuple(vertex_1)
            idx_2 = tuple(vertex_2)
            graph.add_edge(idx_1, idx_2, c)
            graph.add_edge(idx_2, idx_1, c)

    return graph


def connect_point(
    point: Point,
    *,
    graph: Graph,
    vertices: Iterable[Point],
    rects: List[Rect],
    cost=dist,
    to_point=False,
    from_point=False,
):
    # objects = [get_rect(collision) for collision in get_collision_points(map)]
    point_id = tuple(point)
    for vertex in vertices:
        if collides_with_rectangles(line=(vertex, point), collision_rects=rects):
            continue
        c = cost(point, vertex)
        idx = tuple(vertex)
        if to_point:
            graph.add_edge(idx, point_id, c)
        if from_point:
            graph.add_edge(point_id, idx, c)

    return point_id


def dijkstar_path(
    init: int,
    end: int,
    *,
    graph: Graph,
):
    path = find_path(graph, init, end)
    return path


def read_file_to_array(file_path) -> NDArray:
    array = np.loadtxt(file_path)
    return array


def safety_enlarging(rects: Iterable[Rect], d: int):
    assert d >= 0
    left_corner_displacement = np.array([d, d])
    for rect in rects:
        w = rect.w + 2 * d
        h = rect.h + 2 * d
        new_bl = rect.bl - left_corner_displacement
        new_rect = Rect(new_bl, h=h, w=w, index=rect.index)
        yield new_rect


def main():
    logging.basicConfig(level=logging.DEBUG)
    mapfile = "map.txt"
    enlarging_safety = 2
    rects, vertices, g, _ = setup_graph(mapfile, enlarging_safety)
    init = np.array([1.3, 1.2])
    end = np.array([30, 30])
    path_to_destination(init, end, rects=rects, vertices=vertices, graph=g)


def read_map(mapfile: str) -> NDArray:
    """Read map

    Args:
        mapfile (str): Convention

        If map is:

        3 4 5
        0 1 2

        m[0,0]= 0
        m[1,0]= 1
        m[2,0]= 2
        m[0,1]= 3
        m[1,1]= 4
        m[2,1]= 5

    Returns:
        NDArray: m, map with coordinate convention given above
    """
    # We obtain mapfile from top left to bottom right with
    # x = read_file_to_array(mapfile);
    # x(row, column) ; x[y,x] in math notation
    # # x[0,0] top left
    # x[-1, 0] last row, so on left bottom
    # x[0, -1] last column, so top right
    # x[-1, -1] is bottom right
    # but we want m(x,y) with x being column, y being row, and
    # # x going from 0,0 (bottom left) to [-1,0]  bottom right
    # # y going from 0,0 (bottom left) to [0,-1]  top left

    # so first we transpose it
    x = read_file_to_array(mapfile)
    xT = x.T
    # then we flip it so the y coordinate is changed
    m = np.flip(xT, 1)
    return m


def setup_graph(mapfile, enlarging_safety):
    m = read_map(mapfile)

    rects = list(safety_enlarging(get_collision_rects(m), enlarging_safety))
    vertices = list(get_vertices(rects, d=1))
    g = Graph()
    g = connect_points(vertices, rects, graph=g)
    # g = connect_rects(rects, graph=g)
    return rects, vertices, g, m


def path_to_destination(
    init: Point, end: Point, *, rects: List[Rect], vertices: List[Point], graph: Graph
) -> List[Tuple[Union[float, int], Union[float, int]]]:
    init_idx = connect_point(
        point=init,
        graph=graph,
        vertices=[*vertices, end],
        rects=rects,
        from_point=True,
    )
    end_idx = connect_point(
        point=end,
        graph=graph,
        vertices=vertices,
        rects=rects,
        to_point=True,
    )
    logger.debug("graph %s", graph)
    logger.debug("init %s; end %s", init_idx, end_idx)

    path = find_path(graph, init_idx, end_idx)
    logger.info("Recommended path %s\n\t go to %s", path, path.nodes[1])
    with suppress(KeyError):
        graph.remove_node(end_idx)
    with suppress(KeyError):
        graph.remove_node(init_idx)
    return path.nodes


if __name__ == "__main__":
    main()
