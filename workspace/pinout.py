import re

import yaml

with open("pinout.yaml") as f:
    data = yaml.safe_load(f)


def hasscheme(pin: dict):
    return bool(pin.get("scheme"))


def getscheme(pin):
    return pin["scheme"]["bcm"], pin["scheme"]["wiringpi"]


def varname(pin):
    ty: str = pin["type"]
    return re.sub("\W+", "_", ty)


pinmap = [pin for pin in data["pins"].values() if hasscheme(pin)]
items = [
    (k := getscheme(pin), f"#define {varname(pin)}_{k[0]} {k[1]}") for pin in pinmap
]
items.sort()
for key, value in items:
    print(value)
