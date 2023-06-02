from detection import (
    List,
    NDArray,
    Rect,
    collides_with_rectangles,
    draw_map,
    get_collision_rects,
    line_detector,
    np,
    read_file_to_array,
)


def collides_tests(m: NDArray, rects: List[Rect]):
    init = np.array([0, 0])
    end = np.array([13, 9])
    line = (init, end)
    print("collisions", collides_with_rectangles(line, rects))
    draw_map(m, rects, lines=(init, end), point=end)


def assert_valid_rects(m: NDArray, rects: List[Rect]):
    for rect in rects:
        assert rect.w != 0
        assert rect.h != 0
        assert m[tuple(rect.br)] == 2.0
        assert m[tuple(rect.bl)] == 2.0
        assert m[tuple(rect.tr)] == 2.0
        assert m[tuple(rect.tl)] == 2.0


def test_rects():
    m = read_file_to_array("map.txt")
    rects = list(get_collision_rects(m))
    assert_valid_rects(m, rects)
    collides_tests(m, rects)


def test_collide():
    points = ([0, 0], [1, 1], [0, 1], [1, 0])
    np_points = list(np.array(p) for p in points)
    assert line_detector(*np_points) is not None, "should collide"
    points = ([0, 0], [1, 1], [0, -1], [-1, 0])
    np_points = list(np.array(p) for p in points)
    assert line_detector(*np_points) is None, "should not collide"


if __name__ == "__main__":
    test_collide()
    test_rects()
