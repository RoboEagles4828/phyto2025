import functools
import math
import operator
import typing
from wpimath.geometry import Pose3d, Rotation2d, Rotation3d, Translation2d, Pose2d

number = typing.Union[float, int]


def clamp(inputValue: float, minimum: float, maximum: float) -> float:
    return max(min(inputValue, maximum), minimum)


def normalizeRotation(inputRotation: Rotation2d) -> Rotation2d:
    """
    Normalize the given rotation to the range [-pi, pi)
    """
    inputAngle = inputRotation.radians()
    return Rotation2d(
        inputAngle - 2 * math.pi * math.floor((inputAngle + math.pi) / (2 * math.pi))
    )


def translationFromDistanceAndRotation(
    distance: float, rotation: Rotation2d
) -> Translation2d:
    return Translation2d(distance * rotation.cos(), distance * rotation.sin())


def rotationFromTranslation(translation: Translation2d) -> Rotation2d:
    return Rotation2d(math.atan2(translation.Y(), translation.X()))


def rotateAroundPoint(
    pose: Pose2d, position: Translation2d, rotation: Rotation2d
) -> Pose2d:
    deltaTranslation = pose.translation() - position
    newRotation = rotation + pose.rotation()

    rotatedTranslation = translationFromDistanceAndRotation(
        deltaTranslation.distance(Translation2d()),
        rotationFromTranslation(deltaTranslation) + rotation,
    )

    return Pose2d(rotatedTranslation + position, newRotation)


def map_range(
    value: number,
    inputMin: number,
    inputMax: number,
    outputMin: number,
    outputMax: number,
):
    return (value - inputMin) * (outputMax - outputMin) / (
        inputMax - inputMin
    ) + outputMin


def average(averageable_list: typing.List[typing.Any], initial: typing.Any):
    length = len(averageable_list)
    return functools.reduce(operator.add, averageable_list, initial) / length


def addPose2d(a: Pose2d, b: Pose2d):
    return Pose2d(
        a.X() + b.X(),
        a.Y() + b.Y(),
        Rotation2d(a.rotation().radians() + b.rotation().radians()),
    )


def pose3dFrom2d(pose: Pose2d) -> Pose3d:
    return Pose3d(pose.X(), pose.Y(), 0, Rotation3d(0, 0, pose.rotation().radians()))


def translationDotProduct(a: Translation2d, b: Translation2d) -> float:
    return a.x * b.x + a.y * b.y


def pointInRectangle(
    rect: typing.Tuple[Translation2d, Translation2d, Translation2d, Translation2d],
    point: Translation2d,
) -> bool:
    # https://stackoverflow.com/a/37865332 cuz aint no way I can magic my way this one
    a, b, c, _ = rect
    ab = a - b
    am = a - point
    bc = b - c
    bm = b - point
    dot = translationDotProduct

    dotABAM = dot(ab, am)
    dotABAB = dot(ab, ab)
    dotBCBM = dot(bc, bm)
    dotBCBC = dot(bc, bc)
    return 0 <= dotABAM <= dotABAB and 0 <= dotBCBM <= dotBCBC


def pointInCircle(p1: Translation2d, c: Translation2d, r: float) -> bool:
    return (p1 - c).norm() <= r


def inputModulus(input: number, min_value: number, max_value: number) -> number:
    return (input - min_value) % (max_value - min_value) + min_value