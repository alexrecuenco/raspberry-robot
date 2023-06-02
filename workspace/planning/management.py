import logging
import re
import subprocess
from os import getenv
from typing import Optional, Tuple

logger = logging.getLogger(__file__)

USER = getenv("ROBOT_USER", "pi")
IP = getenv("ROBOT_IP", "192.168.1.38")
PWD = getenv("ROBOT_FOLDER", "/home/pi/")
CMD = getenv("ROBOT_EXE", "main-robot")

DEBUG = True


def debug_print(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)


def execute_bash_command(
    x: int, y: int, theta: int, x2: int, y2: int, *, speed=30
) -> Tuple[float, float, float]:
    command = f"""ssh -tt {USER}@{IP} "SPEED={int(speed)} X_INIT={int(x)} Y_INIT={int(y)} THETA_INIT={int(theta)} sudo -E {PWD}/{CMD} movereckless {int(x2)} {int(y2)}" """
    logger.info(command)
    process = subprocess.Popen(
        command,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    stdout, stderr = process.communicate(timeout=5 * 60)

    logger.debug("stdout:%s", stdout)
    logger.debug("stderr:%s", stderr)

    if process.returncode != 0:
        logger.warn(
            "Error executing command:\nstdout:\n%s\nstderr:\n%s", stdout, stderr
        )

    pattern = (
        r".*DEBUG.*p_out: \(x, y, theta\) (-?\d+\.\d+), (-?\d+\.\d+), (-?\d+\.\d+)"
    )
    lines_out = [line for line in stdout.split("\n") if "p_out" in line]
    lines_target = [line for line in stdout.split("\n") if "p_target" in line]
    logger.info(lines_out)
    logger.info(lines_target)

    if not lines_out:
        raise TypeError("Could not find the line with the p_out value.")
    p_out_line = lines_out[-1]
    match = re.findall(pattern, p_out_line)

    if not match:
        raise TypeError("Format of p_out value has changed, not found x,y,theta")

    # match = re.search(pattern, stderr)
    # if not match: raise... whatever
    # new_x = float(match.group(1))
    # new_y = float(match.group(2))
    # new_theta = float(match.group(3))
    # return new_x, new_y, new_theta

    x, y, theta = match[-1]
    return float(x), float(y), float(theta)


def main():
    logging.basicConfig(level=logging.DEBUG)

    logger.debug("TEST ONLY")
    (x, y, theta) = execute_bash_command(0, 50, 90, 100, 100)
    logger.debug("REACHED")
    logger.info(x, y, theta)


if __name__ == "__main__":
    main()
