"""
Snapshot / restore of the kinematic state of a vehicle (all of its parts).

Used by the "Undo last run" feature. It deliberately does not depend on the
internals of vehicle.py: every simple attribute (numbers, bools, strings) and
every point-like attribute (an object with numeric .x and .y) of each vehicle
part is saved and later put back. No QGIS imports, so it can be tested anywhere.
"""


def _is_simple(value) -> bool:
    return value is None or isinstance(value, (bool, int, float, str))


def _is_point(value) -> bool:
    if _is_simple(value):
        return False
    try:
        x, y = value.x, value.y
    except AttributeError:
        return False
    num = (int, float)
    return isinstance(x, num) and isinstance(y, num) \
        and not isinstance(x, bool) and not isinstance(y, bool)


def _is_simple_seq(value) -> bool:
    return isinstance(value, (list, tuple)) and all(_is_simple(i) for i in value)


def snapshot_vehicle(vehicle):
    """
    Return an opaque state object, or None if nothing could be captured.
    Works for single vehicles and for articulated ones (vehicle.vehicle_parts).
    """
    parts = getattr(vehicle, "vehicle_parts", None) or [vehicle]
    state = []
    captured = 0

    for part in parts:
        attrs = getattr(part, "__dict__", None)
        if not attrs:
            continue

        saved = {}
        for name, value in list(attrs.items()):
            if _is_simple(value):
                saved[name] = ("v", value)
            elif _is_point(value):
                saved[name] = ("p", value, value.x, value.y)
            elif _is_simple_seq(value):
                saved[name] = ("s", type(value), list(value))

        state.append((part, saved))
        captured += len(saved)

    return state if captured else None


def restore_vehicle(state):
    """
    Put a snapshot back. Returns (restored, failed) attribute counts.
    Point objects are reinstated (original object, original coordinates), so it
    works whether the vehicle mutates its points or replaces them on each step.
    """
    restored = 0
    failed = 0

    for part, saved in state or []:
        for name, item in saved.items():
            try:
                kind = item[0]
                if kind == "v":
                    setattr(part, name, item[1])
                elif kind == "p":
                    _, obj, x, y = item
                    obj.x = x
                    obj.y = y
                    setattr(part, name, obj)
                elif kind == "s":
                    _, seq_type, items = item
                    try:
                        setattr(part, name, seq_type(items))
                    except TypeError:
                        setattr(part, name, list(items))
                restored += 1
            except (AttributeError, TypeError):
                failed += 1

    return restored, failed
