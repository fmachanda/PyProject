import math


def quaternion_to_euler(q: list[float]) -> tuple[float]:
    """Convert a quaternion angle to Euler. 

    Copied on 10/2/2023 from automaticaddison.com
    
    Parameters
    ----------
    q : tuple[float, float, float, float]
        The quaternion values x, y, z, w
    
    Returns
    -------
    tuple[float, float, float]
        A tuple of roll, pitch, yaw as radians in Euler format
    
    Raises
    ------
    ValueError
        If the input is not of length 4
    """

    if len(q)!=4:
        raise ValueError("Quaternion input must be tuple with length 4")

    x, y, z, w = q

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw # in radians


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> list[float, float, float, float]:
    """Convert an Euler angle to a quaternion.

    Copied on 10/2/2023 from automaticaddison.com

    Parameters
    ----------
        roll : float
            The roll (rotation around x-axis) angle in radians.
        pitch : float
            The pitch (rotation around y-axis) angle in radians.
        yaw : float
            The yaw (rotation around z-axis) angle in radians.

    Returns
    list[float, float, float, float]
        The orientation in quaternion [w,x,y,z] format
    """

    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return [qw, qx, qy, qz]

def _clip(value: float):
    """Clips value to valid domain for inverse trig functions."""
    return max(min(value, 1), -1)

def py_to_rp(Pc: float, Yc: float) -> tuple[float, float]:
    """Convert angles from pitch/yaw frame to roll/pitch frame."""
    assert -90<=Pc<=90, "Input pitch angle must be between -90 degrees and 90 degrees."
    # print("------------------")
    # print("Input: ", Pc, Yc)
    Pc = math.radians(Pc)
    Yc = math.radians(Yc)

    Pd = math.acos(math.cos(Pc) * math.cos(Yc))
    # print("Pd: ", math.degrees(Pd))

    try:
        # print("asin: ", _clip(math.cos(Pc) * math.sin(Yc) / math.sin(Pd)))
        Rd = math.asin(_clip(math.cos(Pc) * math.sin(Yc) / math.sin(Pd)))
        # print("Rd v1: ", math.degrees(Rd))
        Rd = -(Rd + math.pi) if Pc<0 else Rd
        # print("Rd v2: ", math.degrees(Rd))
    except ZeroDivisionError:
        Rd = 0.0
        # print("Rd err: ", math.degrees(Rd))

    Pd = round(math.degrees(Pd), 5)
    Rd = round(math.degrees(Rd), 5)

    # assert abs(math.sin(Pd)*math.cos(Rd) - math.sin(Pc)) < 1e-6
    return Rd, Pd

def rp_to_py(Rd: float, Pd: float) -> tuple[float, float]:
    """Convert angles from roll/pitch frame to pitch/yaw frame."""
    assert 0<=Pd<=180, "Input pitch angle must be between 0 degrees and 180 degrees."
    print("------------------")
    print("Input: ", Rd, Pd)
    Rd = math.radians(Rd)
    Pd = math.radians(Pd)

    Pc = math.asin(math.sin(Pd) * math.cos(Rd))
    print("Pc: ", math.degrees(Pc))

    try:
        print("acos: ", _clip(math.cos(Pd) / math.cos(Pc)))
        Yc = math.acos(_clip(math.cos(Pd) / math.cos(Pc)))
        print("Yc v1: ", math.degrees(Yc))
        Yc *= -1 if (-math.pi<Rd<0 or math.pi<Rd<2*math.pi) else 1
        print("Yc v2: ", math.degrees(Yc))
    except ZeroDivisionError:
        Yc = 0.0
        print("Yc err: ", math.degrees(Yc))

    Pc = round(math.degrees(Pc), 5)
    Yc = round(math.degrees(Yc), 5)

    # assert abs(math.cos(Pc)*math.sin(Yc) - math.sin(Pd)*math.sin(Rd)) < 1e-6
    return Pc, Yc

def _tests() -> bool:
    assert py_to_rp(0, 0)==(0, 0)

    assert py_to_rp(12, 0)==(0, 12)
    assert py_to_rp(-16, 0)==(180, 16) or py_to_rp(-16, 0)==(-180, 16)
    assert py_to_rp(0, 16)==(90, 16)
    assert py_to_rp(0, -78)==(-90, 78)

    assert py_to_rp(0, 90)==(90, 90)
    assert py_to_rp(12, 90)==(78, 90)
    assert py_to_rp(-16, 90)==(106, 90) or py_to_rp(-16, 90)==(106-360, 90)
    assert py_to_rp(90, 0)==(0, 90)
    assert py_to_rp(90, 16)==(0, 90)
    assert py_to_rp(90, -78)==(0, 90)

    assert py_to_rp(0, -90)==(-90, 90)
    assert py_to_rp(12, -90)==(-78, 90)
    assert py_to_rp(-16, -90)==(-106, 90)
    assert py_to_rp(-90, 0)==(180, 90) or py_to_rp(-90, 0)==(-180, 90)
    assert py_to_rp(-90, 16)==(180, 90) or py_to_rp(-90, 16)==(-180, 90)
    assert py_to_rp(-90, -78)==(180, 90) or py_to_rp(-90, -78)==(-180, 90)

    assert py_to_rp(90, 90)==(0, 90)
    assert py_to_rp(-90, 90)==(-180, 90)

    assert py_to_rp(30, 45)==(50.76848, 52.23876)
    assert py_to_rp(30, -45)==(-50.76848, 52.23876)
    assert py_to_rp(-30, 45)==(129.23152, 52.23876) or py_to_rp(-30, 45)==(129.23152-360, 52.23876)
    assert py_to_rp(-30, -45)==(-129.23152, 52.23876) or py_to_rp(-30, -45)==(-129.23152+360, 52.23876)

    for point in [(0, 0), (12, 0), (-16, 0), (0, 16), (0, -78), (-16, 90), (-30, 45)]:
        # TODO: incomplete test set
        assert rp_to_py(*py_to_rp(*point)) == point

    print("Tests passed!")

if __name__=='__main__':
    _tests()