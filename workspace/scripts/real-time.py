"""Read realtime data
Requirements:
pip install matplotlib iterators

Testing:

# First test, test data at large speed
python real-time.py  --test --nargs=10
# Second test, real stdin
python -c 'import random; [print(random.random(), ", ", random.random()+1) for _ in range(500)]' | python real-time.py

Usage:

<your command with output> | python real-time.py

For example

ssh -t pi@192.168.1.1 "DEBUG_SENSORS=1 sudo -E /home/pi/alex/Notes_Master_UCM_Tech_Fotonica/Robots/programming/pi/workspace" | python real-time.py

"""

import argparse
import random
import sys
from collections import namedtuple
from dataclasses import dataclass, field
from itertools import islice
from time import time
from typing import Iterable, List, Tuple

import matplotlib.pyplot as plt
from iterators import TimeoutIterator
from matplotlib.animation import FuncAnimation

DataType = List[float]
LineData = namedtuple("LineData", ["x", "y"])


@dataclass
class Options:
    test: bool
    nargs: int
    maxlen: int
    framerate: int
    batching: int
    interval: int = field(init=False)
    data_iterator: Iterable[DataType] = field(init=False)
    min: int
    max: int

    def __post_init__(self):
        if self.test:
            input_iter = Options.fake_input(self.nargs)
        else:
            input_iter = sys.stdin
        self.data_iterator = Options.extract_data(input_iter)
        self.interval = int(1 / self.framerate * 1000)

    @staticmethod
    def fake_input(nargs):
        while True:
            yield ", ".join(
                str(random.random() * (1 + i / nargs) + i * 4 / nargs)
                for i in range(nargs)
            )

    @staticmethod
    def parse_args():
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--test",
            type=bool,
            default=False,
            action=argparse.BooleanOptionalAction,
            help="Whether to use test data",
        )
        parser.add_argument(
            "--nargs",
            type=int,
            default=4,
            help="Number of arguments passed (Only used for test data)",
        )
        parser.add_argument(
            "--maxlen",
            type=int,
            default=500,
            help="Max number of datapoints to represet",
        )
        parser.add_argument(
            "--framerate",
            type=int,
            default=50,
            help="Framerate of animation in Hz",
        )
        parser.add_argument(
            "--batching",
            type=int,
            default=20,
            help="How many points to collect for each frame",
        )
        parser.add_argument(
            "--min",
            type=int,
            default=0,
            help="Minimum input value",
        )
        parser.add_argument(
            "--max",
            type=int,
            default=1023,
            help="Maximum input value",
        )
        args = parser.parse_args()
        return Options(
            test=args.test,
            nargs=args.nargs,
            maxlen=args.maxlen,
            min=args.min,
            max=args.max,
            framerate=args.framerate,
            batching=args.batching,
        )

    @staticmethod
    def extract_data(iter: Iterable[str]):
        for line in iter:
            try:
                values = line.split(",")
                values = [float(value.strip()) for value in values]
                yield values
            except ValueError:
                pass


def batched(iterable, n: int, timeout: float = 0.01):
    """Batch data into tuples of length n. With timeout

    Recipe from https://docs.python.org/3/library/itertools.html#itertools-recipes
    # batched('ABCDEFG', 3) --> ABC DEF G
    """
    if n < 1:
        raise ValueError("n must be at least one")
    it = iter(iterable)
    sentinel = object()
    # The total timeout needs to be at most timeout
    it = TimeoutIterator(it, timeout=timeout / (2 * n), sentinel=sentinel)
    count_empty = 0

    while True:
        tin = time()
        batch = tuple(islice(it, n))
        batch = [x for x in batch if x != sentinel]
        if batch:
            count_empty = 0
        else:
            count_empty += 1
            if count_empty > 15:
                # Some break condition?
                break
        tout = time()
        if (tin - tout) > timeout:
            print(tin - tout, "WARNING")
        yield batch


def plot_lines(
    data: Iterable[List[float]],
    *,
    maxlen: int,
    interval: int,
    batching: int,
    y_min: int = 0,
    y_max: int = 1023,
):
    # Set up the figure and axis
    fig, ax = plt.subplots()
    ax.set_xlim(0, maxlen)
    ax.set_ylim(y_min, y_max)  # Change the limits as per your needs

    def empty_Line2D(data, i=0):
        (line,) = ax.plot(data.x, data.y, lw=1, label=f"{i}")
        return line

    # Initialize the line object that will be plotted
    lines_data = [LineData(x=[0], y=[v]) for v in next(data)]
    lines = [empty_Line2D(line_data, i=idx) for idx, line_data in enumerate(lines_data)]
    plt.legend()

    def update_one(frame: Tuple[int, DataType]):
        # Get the values from the input stream (replace this with your own code to get the values)
        idx, values = frame

        for line, line_data, value in zip(lines, lines_data, values):
            line_data.y.append(value)
            if len(line_data.y) < maxlen:
                line_data.x.append(idx)
            else:
                line_data.y.pop(0)
            line.set_data(line_data.x, line_data.y)

        return lines

    def update(frames: Iterable[Tuple[int, DataType]], timeout=0.001):
        for frame in frames:
            update_one(frame)
        return lines

    print("animation")

    # Create the animation object and start it
    animation = FuncAnimation(
        fig,
        update,
        # Total timeout needs to be shorter than the framerate spec to not get screen freezes
        frames=batched(enumerate(data), batching, timeout=interval / 10),
        blit=True,
        interval=interval,
        cache_frame_data=False,
        save_count=0,
        repeat=False,
    )  # Change the interval as per your needs
    plt.show()


def runtime(o: Options):
    plot_lines(
        data=o.data_iterator,
        maxlen=o.maxlen,
        interval=o.interval,
        batching=o.batching,
        y_min=o.min,
        y_max=o.max,
    )


def main():
    options = Options.parse_args()
    runtime(options)


if __name__ == "__main__":
    main()
