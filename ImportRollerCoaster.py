import adsk.core
import adsk.fusion
import traceback
import os
import math
import datetime
import xml.etree.ElementTree as ET

LAST_SWEEP_ERROR = ''

# ============================================================
# SETTINGS
# ============================================================

RAIL_NAMES = ['left', 'center', 'right', 'bottom']

# Preferred 0-based crosstie points.
# The first and last points are reserved for the 51% rectangular spine lofts.
# Short final track pieces use the preferred indices that fit their point count.
CROSSTIE_INDICES = [1, 3, 5, 7, 9, 11]

# Fusion API internal units are cm.
DEFAULT_GAUGE_CM = 2.6  # 26 mm
TRACK_PATH_SCALE_CM_PER_UNIT = DEFAULT_GAUGE_CM / 100.0
SCALE_RAIL_OFFSETS_FROM_CENTER = True

CREATE_CROSSTIES = True
CREATE_TRACK_SPINE_SWEEPS = True
CREATE_SUPPORTS_FROM_XML = True
CREATE_RAIL_SPLINE_SKETCHES = False
CREATE_SPINE_CENTER_SKETCH = False
HIDE_HELPER_SKETCHES = True
HIDE_CONSTRUCTION_GEOMETRY = True

# Set this to True whenever you want the track-folder and support-XML dialogs back.
PROMPT_FOR_INPUT_PATHS = False
DEFAULT_TRACK_BASE_FOLDER = r'C:\Users\rpowe\Documents\SimpleCorkscrew'
DEFAULT_SUPPORT_XML_PATH = r'C:\Users\rpowe\Documents\SimpleCorkscrew\corkscrew_supports.xml'

DEBUG_SWEEP_LOGGING = False
DEBUG_LOG_FILENAME = 'ImportAllTrack_sweep_debug.log'
DEBUG_POINT_SAMPLE_COUNT = 3
SUPPORT_DEBUG_LOGGING = True
SUPPORT_DEBUG_LOG_FILENAME = 'ImportAllTrack_support_debug.log'

# NL2 support XML positions are meters. The corkscrew CSV path coordinates
# line up with 100 CSV path units per NL2 meter, then use the same path scale.
SUPPORT_XML_METERS_TO_CSV_UNITS = 100.0
SUPPORT_DEFAULT_BEAM_SIZE_M = 0.6096
CREATE_SUPPORT_RASC_CONNECTORS = True
CREATE_SUPPORT_FOOTER_PADS = True
CREATE_SUPPORT_TRACK_INTERFACES = True
CLIP_SUPPORT_BEAMS_TO_FOOTER_EXIT = True
SUPPORT_FOOTER_PAD_DIAMETER_M = 1.0
SUPPORT_FOOTER_PAD_HEIGHT_M = 0.16
SUPPORT_TRACK_INTERFACE_DIAMETER_M = 0.70
SUPPORT_TRACK_INTERFACE_STANDOFF_CM = 0.35
SUPPORT_BEAM_DIAMETER_SCALE = 0.75
SUPPORT_INTERFACE_START_MAX_SIZE_CM = 0.90
SUPPORT_INTERFACE_TRACK_MAX_SIZE_CM = 0.55
SUPPORT_CROSSTIE_CLEARANCE_CM = 0.1636
SUPPORT_STRAIGHT_CLEARANCE_DROP_MAX_CM = 1.50
SUPPORT_TRACK_PAD_STANDOFF_CM = 1.635 / 10.0
SUPPORT_CROSSTIE_MARGIN_CM = 0.3 / 10.0
SUPPORT_MAX_SNAP_CM = 1.0
SUPPORT_TRACK_COLLAR_LENGTH_CM = 0.12
SUPPORT_MAX_BEAM_EXTENSION_CM = 0.5
SUPPORT_MIN_CONNECTOR_DIAMETER_CM = 0.05
SUPPORT_TRACK_PAD_SQUARE_SCALE = 2.0
SUPPORT_TRACK_PAD_CIRCLE_SCALE = 2.0
SUPPORT_TRACK_COLLAR_DIAMETER_SCALE = 2.0
SUPPORT_MIN_ANGLED_CONNECTOR_LENGTH_CM = 0.20
SUPPORT_CONNECTOR_OVERLAP_CM = 0.02
SUPPORT_FOOTER_BEAM_EMBED_CM = 0.04
SUPPORT_MAX_CROSSTIE_BAY_CM = 4.0
SUPPORT_SINGLE_DOUBLE_TRACK_SQUARE_SIZE_MM = 11.285
SUPPORT_SINGLE_DOUBLE_PAD_STANDOFF_CM = 1.7 / 10.0
SUPPORT_SINGLE_DOUBLE_COLLAR_LENGTH_CM = 5.1 / 10.0
SUPPORT_OVERHANG_MIN_HEAD_LENGTH_CM = 1.0 / 10.0
SUPPORT_REGULAR_MAX_ROLL_DEG = 75.0
SUPPORT_90_MAX_ROLL_DEG = 145.0

# Crosstie extrusion depth along track direction.
CROSSTIE_DEPTH_MM = 3.0

# Crosstie rail reference/boss circles.
RAIL_TUBE_OUTER_DIAMETER_MM = 3.0
RAIL_CROSSTIE_BOSS_DIAMETER_MM = 3.0

# Center/spine loft rectangle. Each end loft covers this path fraction.
END_BOX_SWEEP_FRACTION = 0.51

NOMINAL_GAUGE_MM = 26.0
NOMINAL_BOTTOM_DROP_MM = 13.0

# Crosstie profile dimensions from the reference sketch.
# X = across track, center between rails is X=0.
# Y = vertical profile. Rail centers are Y=0.
# Negative Y points downward toward the bottom/spine point.
CROSSTIE_RIGHT_RAIL_ORIGIN_POINTS_MM = {
    'right_rail_center': (0.0, 0.0),
    'left_rail_center': (-26.0, 0.0),
    'bottom_point': (-13.0, -13.0),
}

CROSSTIE_OUTLINE_MM = [
    {'type': 'line', 'to': (-5.887, -4.612)},
    {'type': 'line', 'to': (-5.681, -3.840)},
    {'type': 'line', 'to': (-1.243, -0.840)},
    {'type': 'arc', 'center': (0.0, 0.0), 'to': (-1.136, 0.980)},
    {'type': 'line', 'to': (-2.244, 0.980)},
    {'type': 'line', 'to': (-8.803, -0.778)},
    {'type': 'line', 'to': (-17.197, -0.778)},
    {'type': 'line', 'to': (-23.757, 0.980)},
    {'type': 'line', 'to': (-24.864, 0.980)},
    {'type': 'arc', 'center': (-26.0, 0.0), 'to': (-24.757, -0.840)},
    {'type': 'line', 'to': (-20.319, -3.840)},
    {'type': 'line', 'to': (-20.113, -4.612)},
    {'type': 'line', 'to': (-20.113, -11.512)},
    {'type': 'line', 'to': (-5.887, -11.512)},
]
CROSSTIE_OUTLINE_START_MM = (-5.887, -11.512)

SPINE_RECT_POINTS_MM = [
    (-18.646, -9.850),
    (-7.354, -9.850),
    (-7.354, -3.153),
    (-18.646, -3.153),
]
SPINE_RECT_WIDTH_MM = max(p[0] for p in SPINE_RECT_POINTS_MM) - min(p[0] for p in SPINE_RECT_POINTS_MM)
SPINE_RECT_HEIGHT_MM = max(p[1] for p in SPINE_RECT_POINTS_MM) - min(p[1] for p in SPINE_RECT_POINTS_MM)
SPINE_CENTER_RIGHT_ORIGIN_MM = (
    (SPINE_RECT_POINTS_MM[0][0] + SPINE_RECT_POINTS_MM[2][0]) * 0.5,
    (SPINE_RECT_POINTS_MM[0][1] + SPINE_RECT_POINTS_MM[2][1]) * 0.5,
)


# ============================================================
# BASIC HELPERS
# ============================================================

def mm_to_cm(mm):
    return mm / 10.0


def debugLogPath():
    try:
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), DEBUG_LOG_FILENAME)
    except:
        return os.path.join(os.getcwd(), DEBUG_LOG_FILENAME)


def resetDebugLog():
    if not DEBUG_SWEEP_LOGGING:
        return

    try:
        with open(debugLogPath(), 'w', encoding='utf-8') as logFile:
            logFile.write('ImportAllTrack sweep debug log\n')
            logFile.write(f'Started: {datetime.datetime.now().isoformat(timespec="seconds")}\n')
            logFile.write('=' * 72 + '\n')
    except:
        pass


def debugLog(message):
    if not DEBUG_SWEEP_LOGGING:
        return

    try:
        with open(debugLogPath(), 'a', encoding='utf-8') as logFile:
            timestamp = datetime.datetime.now().isoformat(timespec='seconds')
            logFile.write(f'[{timestamp}] {message}\n')
    except:
        pass


def supportDebugLogPath():
    try:
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), SUPPORT_DEBUG_LOG_FILENAME)
    except:
        return os.path.join(os.getcwd(), SUPPORT_DEBUG_LOG_FILENAME)


def resetSupportDebugLog():
    if not SUPPORT_DEBUG_LOGGING:
        return

    try:
        with open(supportDebugLogPath(), 'w', encoding='utf-8') as logFile:
            logFile.write('ImportAllTrack support debug log\n')
            logFile.write(f'Started: {datetime.datetime.now().isoformat(timespec="seconds")}\n')
            logFile.write('=' * 72 + '\n')
    except:
        pass


def supportDebugLog(message):
    if not SUPPORT_DEBUG_LOGGING:
        return

    try:
        with open(supportDebugLogPath(), 'a', encoding='utf-8') as logFile:
            timestamp = datetime.datetime.now().isoformat(timespec='seconds')
            logFile.write(f'[{timestamp}] {message}\n')
    except:
        pass


def formatPoint(point):
    if not point:
        return 'None'
    return f'({point.x:.6f}, {point.y:.6f}, {point.z:.6f})'


def formatVector(vector):
    if not vector:
        return 'None'
    return f'({vector.x:.6f}, {vector.y:.6f}, {vector.z:.6f}) len={vector.length:.6f}'


def formatPointTuple(pointTuple):
    if not pointTuple or len(pointTuple) < 3:
        return 'None'
    return f'({pointTuple[0]:.6f}, {pointTuple[1]:.6f}, {pointTuple[2]:.6f})'


def pathPointStats(points):
    if not points:
        return 'count=0'

    totalLength = 0.0
    minSegment = None
    maxSegment = 0.0
    zeroSegments = 0
    segmentLengths = []

    for i in range(len(points) - 1):
        p1 = toPoint3D(points[i])
        p2 = toPoint3D(points[i + 1])
        length = p1.distanceTo(p2)
        segmentLengths.append(length)
        totalLength += length
        if length < 0.000001:
            zeroSegments += 1
        if minSegment is None or length < minSegment:
            minSegment = length
        if length > maxSegment:
            maxSegment = length

    sampleCount = min(DEBUG_POINT_SAMPLE_COUNT, len(points))
    firstPoints = ', '.join(formatPointTuple(points[i]) for i in range(sampleCount))
    lastStart = max(sampleCount, len(points) - DEBUG_POINT_SAMPLE_COUNT)
    lastPoints = ', '.join(formatPointTuple(points[i]) for i in range(lastStart, len(points)))
    segmentSample = ', '.join(f'{length:.6f}' for length in segmentLengths[:DEBUG_POINT_SAMPLE_COUNT])

    return (
        f'count={len(points)}, totalLen={totalLength:.6f}cm, '
        f'minSeg={(minSegment if minSegment is not None else 0.0):.6f}cm, '
        f'maxSeg={maxSegment:.6f}cm, zeroSegs={zeroSegments}, '
        f'first=[{firstPoints}], last=[{lastPoints}], '
        f'segSample=[{segmentSample}]'
    )


def describeProfile(profile):
    if not profile:
        return 'None'

    pieces = []
    try:
        pieces.append(f'type={profile.objectType}')
    except:
        pass

    try:
        sketch = profile.parentSketch
        pieces.append(f'sketch={sketch.name}')
        pieces.append(f'profileCount={sketch.profiles.count}')
    except:
        pass

    try:
        props = profile.areaProperties(adsk.fusion.CalculationAccuracy.LowCalculationAccuracy)
        pieces.append(f'area={props.area:.6f}cm2')
        pieces.append(f'centroid={formatPoint(props.centroid)}')
    except Exception as err:
        pieces.append(f'areaError={err}')

    return ', '.join(pieces) if pieces else 'profile'


def promptForBaseFolder(ui):
    folderDialog = ui.createFolderDialog()
    folderDialog.title = "Select Base Folder Containing Track Folders"
    if folderDialog.showDialog() == adsk.core.DialogResults.DialogOK:
        return True, folderDialog.folder
    return False, None


def promptForSupportXmlFile(ui):
    try:
        fileDialog = ui.createFileDialog()
        fileDialog.title = "Select NL2 Supports XML (Cancel to skip)"
        fileDialog.filter = "XML files (*.xml)"
        fileDialog.isMultiSelectEnabled = False

        if fileDialog.showOpen() == adsk.core.DialogResults.DialogOK:
            return True, fileDialog.filename

        return False, None
    except Exception as err:
        debugLog(f'promptForSupportXmlFile: file dialog failed, falling back to text input: {err}')
        xmlPathInput, userCancelled = ui.inputBox(
            'Optional: enter NL2 supports XML path, or leave blank to skip:',
            'Supports XML',
            ''
        )

        if userCancelled:
            return False, None

        xmlPath = xmlPathInput.strip().strip('"')
        if not xmlPath:
            return False, None

        return True, xmlPath


def readPointsFromCSV(csvFilePath):
    pointList = []
    with open(csvFilePath, 'r', encoding='utf-8-sig') as csvFile:
        for rawLine in csvFile:
            cleanedLine = rawLine.strip()
            if not cleanedLine:
                continue

            csvValues = [v.strip() for v in cleanedLine.split(',') if v.strip()]
            if len(csvValues) < 3:
                continue

            try:
                pointList.append((
                    float(csvValues[0]),
                    float(csvValues[1]),
                    float(csvValues[2])
                ))
            except ValueError:
                continue

    return pointList


def trackFolderSortKey(name):
    prefix = 'track'
    suffix = name[len(prefix):] if name.lower().startswith(prefix) else name
    try:
        return int(suffix)
    except:
        return suffix


def scalePoints(pointList, scaleFactor):
    return [(x * scaleFactor, y * scaleFactor, z * scaleFactor) for x, y, z in pointList]


def scalePointFromCenter(rawPoint, rawCenter, pathScaleFactor, offsetScaleFactor):
    return (
        rawCenter[0] * pathScaleFactor + (rawPoint[0] - rawCenter[0]) * offsetScaleFactor,
        rawCenter[1] * pathScaleFactor + (rawPoint[1] - rawCenter[1]) * offsetScaleFactor,
        rawCenter[2] * pathScaleFactor + (rawPoint[2] - rawCenter[2]) * offsetScaleFactor
    )


def scaleTrackRailData(rawRailData, pathScaleFactor, offsetScaleFactor):
    scaledRailData = {}
    rawCenter = rawRailData.get('center', [])

    for railName in RAIL_NAMES:
        rawPoints = rawRailData.get(railName)
        if not rawPoints:
            continue

        if (
            SCALE_RAIL_OFFSETS_FROM_CENTER
            and rawCenter
            and railName != 'center'
        ):
            scaledPoints = []
            for i, rawPoint in enumerate(rawPoints):
                if i < len(rawCenter):
                    scaledPoints.append(
                        scalePointFromCenter(
                            rawPoint,
                            rawCenter[i],
                            pathScaleFactor,
                            offsetScaleFactor
                        )
                    )
                else:
                    scaledPoints.append((
                        rawPoint[0] * pathScaleFactor,
                        rawPoint[1] * pathScaleFactor,
                        rawPoint[2] * pathScaleFactor
                    ))
        else:
            scaledPoints = scalePoints(rawPoints, pathScaleFactor)

        scaledRailData[railName] = scaledPoints

    return scaledRailData


def toPoint3D(pointTuple):
    return adsk.core.Point3D.create(pointTuple[0], pointTuple[1], pointTuple[2])


def toObjectCollection(pointList):
    collection = adsk.core.ObjectCollection.create()
    for x, y, z in pointList:
        collection.add(adsk.core.Point3D.create(x, y, z))
    return collection


def vectorBetween(a, b):
    return adsk.core.Vector3D.create(
        b.x - a.x,
        b.y - a.y,
        b.z - a.z
    )


def midpoint(a, b):
    return adsk.core.Point3D.create(
        (a.x + b.x) * 0.5,
        (a.y + b.y) * 0.5,
        (a.z + b.z) * 0.5
    )


def addVec(point, vec, scale=1.0):
    return adsk.core.Point3D.create(
        point.x + vec.x * scale,
        point.y + vec.y * scale,
        point.z + vec.z * scale
    )


def safeNormalize(vec):
    if vec.length < 0.000001:
        return False
    vec.normalize()
    return True


def copyVector(vec):
    return adsk.core.Vector3D.create(vec.x, vec.y, vec.z)


def scaledVector(vec, scale):
    result = copyVector(vec)
    result.scaleBy(scale)
    return result


def dotPointVector(a, b, axis):
    return (
        (b.x - a.x) * axis.x
        + (b.y - a.y) * axis.y
        + (b.z - a.z) * axis.z
    )


def perpendicularUnitVector(normal):
    test = adsk.core.Vector3D.create(0, 0, 1)
    normalCopy = adsk.core.Vector3D.create(normal.x, normal.y, normal.z)
    normalCopy.normalize()

    if abs(normalCopy.dotProduct(test)) > 0.95:
        test = adsk.core.Vector3D.create(0, 1, 0)

    perp = normalCopy.crossProduct(test)
    if not safeNormalize(perp):
        return None

    return perp


def makeLocalPoint(origin, xAxis, yAxis, xCm, yCm):
    p = addVec(origin, xAxis, xCm)
    p = addVec(p, yAxis, yCm)
    return p


def toSketchPoint(sketch, modelPoint):
    try:
        return sketch.modelToSketchSpace(modelPoint)
    except:
        return modelPoint


def hideEntity(entity):
    try:
        entity.isLightBulbOn = False
    except:
        pass


def createConstructionPoint(component, point):
    cpInput = component.constructionPoints.createInput()
    cpInput.setByPoint(point)
    constructionPoint = component.constructionPoints.add(cpInput)
    if HIDE_CONSTRUCTION_GEOMETRY:
        hideEntity(constructionPoint)
    return constructionPoint


def createPlaneByThreePoints(component, p1, p2, p3, name):
    cp1 = createConstructionPoint(component, p1)
    cp2 = createConstructionPoint(component, p2)
    cp3 = createConstructionPoint(component, p3)

    planeInput = component.constructionPlanes.createInput()
    planeInput.setByThreePoints(cp1, cp2, cp3)
    plane = component.constructionPlanes.add(planeInput)
    plane.name = name
    if HIDE_CONSTRUCTION_GEOMETRY:
        hideEntity(plane)
    return plane


def createPlaneNormalToVector(component, origin, normal, name):
    xAxis = perpendicularUnitVector(normal)
    if not xAxis:
        return None

    yAxis = normal.crossProduct(xAxis)
    if not safeNormalize(yAxis):
        return None

    p2 = addVec(origin, xAxis, 1.0)
    p3 = addVec(origin, yAxis, 1.0)
    return createPlaneByThreePoints(component, origin, p2, p3, name)


def localProfileScale(leftPoint, rightPoint, bottomPoint, origin, yAxis):
    gaugeMm = leftPoint.distanceTo(rightPoint) * 10.0
    bottomDropCm = -dotPointVector(origin, bottomPoint, yAxis)
    bottomDropMm = bottomDropCm * 10.0

    xScale = gaugeMm / NOMINAL_GAUGE_MM if NOMINAL_GAUGE_MM > 0 else 1.0
    yScale = bottomDropMm / NOMINAL_BOTTOM_DROP_MM if NOMINAL_BOTTOM_DROP_MM > 0 else 1.0

    if xScale <= 0:
        xScale = 1.0
    if yScale <= 0:
        yScale = xScale

    return xScale, yScale


def rightOriginToNominalMm(pointMm):
    return pointMm[0] + (NOMINAL_GAUGE_MM * 0.5), pointMm[1]


def shortestArcMidpointMm(centerMm, startMm, endMm):
    startAngle = math.atan2(startMm[1] - centerMm[1], startMm[0] - centerMm[0])
    endAngle = math.atan2(endMm[1] - centerMm[1], endMm[0] - centerMm[0])
    delta = endAngle - startAngle

    while delta > math.pi:
        delta -= math.pi * 2.0
    while delta < -math.pi:
        delta += math.pi * 2.0

    midAngle = startAngle + delta * 0.5
    radius = math.sqrt(
        (startMm[0] - centerMm[0]) ** 2
        + (startMm[1] - centerMm[1]) ** 2
    )

    return (
        centerMm[0] + math.cos(midAngle) * radius,
        centerMm[1] + math.sin(midAngle) * radius
    )


class TrackSectionFrame:
    def __init__(self, leftPoint, rightPoint, bottomPoint):
        self.leftPoint = leftPoint
        self.rightPoint = rightPoint
        self.bottomPoint = bottomPoint
        self.origin = midpoint(leftPoint, rightPoint)

        self.xPerMm = vectorBetween(leftPoint, rightPoint)
        self.xPerMm.scaleBy(1.0 / NOMINAL_GAUGE_MM)

        self.downPerMm = vectorBetween(self.origin, bottomPoint)
        self.downPerMm.scaleBy(1.0 / NOMINAL_BOTTOM_DROP_MM)

        self.xAxis = adsk.core.Vector3D.create(self.xPerMm.x, self.xPerMm.y, self.xPerMm.z)
        self.yAxis = adsk.core.Vector3D.create(-self.downPerMm.x, -self.downPerMm.y, -self.downPerMm.z)

        if not safeNormalize(self.xAxis):
            raise ValueError('bad rail gauge')
        if not safeNormalize(self.yAxis):
            raise ValueError('bad bottom point')

        self.zAxis = self.xAxis.crossProduct(self.yAxis)
        if not safeNormalize(self.zAxis):
            raise ValueError('bad section plane')

        self.yAxis = self.zAxis.crossProduct(self.xAxis)
        if not safeNormalize(self.yAxis):
            raise ValueError('bad section up axis')

        rawDown = vectorBetween(self.origin, bottomPoint)
        if safeNormalize(rawDown) and self.yAxis.dotProduct(rawDown) > 0:
            self.yAxis.scaleBy(-1.0)

    def pointFromNominalMm(self, xMm, yMm):
        p = addVec(self.origin, self.xPerMm, xMm)
        p = addVec(p, self.downPerMm, -yMm)
        return p

    def pointFromRightOriginMm(self, xMm, yMm):
        nominalX, nominalY = rightOriginToNominalMm((xMm, yMm))
        return self.pointFromNominalMm(nominalX, nominalY)

    def rectangleCornersFromNominalMm(self, centerXmm, centerYmm, widthMm, heightMm):
        halfW = widthMm * 0.5
        halfH = heightMm * 0.5
        return [
            self.pointFromNominalMm(centerXmm - halfW, centerYmm - halfH),
            self.pointFromNominalMm(centerXmm + halfW, centerYmm - halfH),
            self.pointFromNominalMm(centerXmm + halfW, centerYmm + halfH),
            self.pointFromNominalMm(centerXmm - halfW, centerYmm + halfH),
        ]


# ============================================================
# SPLINE CREATION
# ============================================================

def createSplineSketch(component, scaledPoints, railName):
    sketch = component.sketches.add(component.xYConstructionPlane)
    sketch.name = f'{railName}_spline'
    try:
        sketch.is3D = True
    except:
        pass
    sketch.isComputeDeferred = True

    spline = sketch.sketchCurves.sketchFittedSplines.add(
        toObjectCollection(scaledPoints)
    )

    sketch.isComputeDeferred = False
    if HIDE_HELPER_SKETCHES:
        hideEntity(sketch)
    return sketch, spline


def createVisibleSplineSketch(component, points, sketchName):
    debugLog(f'createVisibleSplineSketch {sketchName}: {pathPointStats(points)}')

    if len(points) < 2:
        debugLog(f'createVisibleSplineSketch {sketchName}: not enough points')
        return None, None

    sketch = component.sketches.add(component.xYConstructionPlane)
    sketch.name = sketchName
    try:
        sketch.is3D = True
    except:
        pass
    sketch.isComputeDeferred = True

    spline = None
    try:
        spline = sketch.sketchCurves.sketchFittedSplines.add(
            toObjectCollection(points)
        )
    except Exception as err:
        debugLog(f'createVisibleSplineSketch {sketchName}: fitted spline add failed: {err}')

    sketch.isComputeDeferred = False
    if HIDE_HELPER_SKETCHES:
        hideEntity(sketch)
    return sketch, spline


# ============================================================
# PROFILE / FEATURE HELPERS
# ============================================================

def createCircleProfileOnPlane(component, plane, centerPoint, diameterCm, sketchName):
    sketch = component.sketches.add(plane)
    sketch.name = sketchName
    sketch.isComputeDeferred = True

    sketch.sketchCurves.sketchCircles.addByCenterRadius(
        toSketchPoint(sketch, centerPoint),
        diameterCm * 0.5
    )

    sketch.isComputeDeferred = False
    if HIDE_HELPER_SKETCHES:
        hideEntity(sketch)

    profile = largestProfile(sketch)
    if not profile:
        return None, None

    return sketch, profile


def largestProfile(sketch):
    if not sketch or sketch.profiles.count < 1:
        return None

    bestProfile = sketch.profiles.item(0)
    bestArea = -1.0

    for i in range(sketch.profiles.count):
        profile = sketch.profiles.item(i)
        area = 0.0

        try:
            props = profile.areaProperties(
                adsk.fusion.CalculationAccuracy.LowCalculationAccuracy
            )
            area = props.area
        except:
            try:
                props = profile.areaProperties()
                area = props.area
            except:
                area = float(i)

        if area > bestArea:
            bestArea = area
            bestProfile = profile

    return bestProfile


def createCenteredRectangleProfileOnPlane(component, plane, center, xAxis, yAxis, widthCm, heightCm, sketchName):
    sketch = component.sketches.add(plane)
    sketch.name = sketchName
    sketch.isComputeDeferred = True

    halfW = widthCm * 0.5
    halfH = heightCm * 0.5

    p1 = makeLocalPoint(center, xAxis, yAxis, -halfW, -halfH)
    p2 = makeLocalPoint(center, xAxis, yAxis, halfW, -halfH)
    p3 = makeLocalPoint(center, xAxis, yAxis, halfW, halfH)
    p4 = makeLocalPoint(center, xAxis, yAxis, -halfW, halfH)

    lines = sketch.sketchCurves.sketchLines
    lines.addByTwoPoints(toSketchPoint(sketch, p1), toSketchPoint(sketch, p2))
    lines.addByTwoPoints(toSketchPoint(sketch, p2), toSketchPoint(sketch, p3))
    lines.addByTwoPoints(toSketchPoint(sketch, p3), toSketchPoint(sketch, p4))
    lines.addByTwoPoints(toSketchPoint(sketch, p4), toSketchPoint(sketch, p1))

    sketch.isComputeDeferred = False
    if HIDE_HELPER_SKETCHES:
        hideEntity(sketch)

    profile = largestProfile(sketch)
    if not profile:
        return None, None

    return sketch, profile


# ============================================================
# CROSSTIE CREATION
# ============================================================

def extrudeProfile(component, profile, depthCm, operation, name):
    extrudes = component.features.extrudeFeatures
    extInput = extrudes.createInput(profile, operation)

    distance = adsk.core.ValueInput.createByReal(depthCm)

    try:
        extInput.setSymmetricExtent(distance, True)
    except:
        extInput.setDistanceExtent(False, distance)

    feature = extrudes.add(extInput)
    feature.name = name
    return feature


def createCrosstie(component, leftPoint, rightPoint, bottomPoint, sequenceNumber, pointIndex):
    try:
        frame = TrackSectionFrame(leftPoint, rightPoint, bottomPoint)
    except:
        return False

    planeName = f'crosstie_{sequenceNumber:02d}_plane'

    plane = createPlaneByThreePoints(
        component,
        leftPoint,
        rightPoint,
        bottomPoint,
        planeName
    )

    # Main crosstie profile.
    sketch = component.sketches.add(plane)
    sketch.name = f'crosstie_{sequenceNumber:02d}_profile'
    sketch.isComputeDeferred = True

    lines = sketch.sketchCurves.sketchLines
    arcs = sketch.sketchCurves.sketchArcs

    currentMm = CROSSTIE_OUTLINE_START_MM
    for entity in CROSSTIE_OUTLINE_MM:
        nextMm = entity['to']
        startPoint = frame.pointFromRightOriginMm(currentMm[0], currentMm[1])
        endPoint = frame.pointFromRightOriginMm(nextMm[0], nextMm[1])

        if entity['type'] == 'arc':
            centerMm = entity['center']
            midMm = shortestArcMidpointMm(centerMm, currentMm, nextMm)
            midPoint = frame.pointFromRightOriginMm(midMm[0], midMm[1])
            arcs.addByThreePoints(
                toSketchPoint(sketch, startPoint),
                toSketchPoint(sketch, midPoint),
                toSketchPoint(sketch, endPoint)
            )
        else:
            lines.addByTwoPoints(
                toSketchPoint(sketch, startPoint),
                toSketchPoint(sketch, endPoint)
            )

        currentMm = nextMm

    # Ø4 mm outer tubing construction circles.
    spineCorners = [
        frame.pointFromRightOriginMm(xMm, yMm)
        for xMm, yMm in SPINE_RECT_POINTS_MM
    ]

    for i in range(len(spineCorners)):
        rectLine = lines.addByTwoPoints(
            toSketchPoint(sketch, spineCorners[i]),
            toSketchPoint(sketch, spineCorners[(i + 1) % len(spineCorners)])
        )
        rectLine.isConstruction = True

    tubeRadiusCm = mm_to_cm(RAIL_TUBE_OUTER_DIAMETER_MM) * 0.5

    leftTube = sketch.sketchCurves.sketchCircles.addByCenterRadius(
        toSketchPoint(sketch, leftPoint),
        tubeRadiusCm
    )
    rightTube = sketch.sketchCurves.sketchCircles.addByCenterRadius(
        toSketchPoint(sketch, rightPoint),
        tubeRadiusCm
    )

    leftTube.isConstruction = True
    rightTube.isConstruction = True

    sketch.isComputeDeferred = False
    if HIDE_HELPER_SKETCHES:
        hideEntity(sketch)

    mainProfile = largestProfile(sketch)
    if not mainProfile:
        return False

    depthCm = mm_to_cm(CROSSTIE_DEPTH_MM)

    # Extrude the main web.
    extrudeProfile(
        component,
        mainProfile,
        depthCm,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        f'crosstie_{sequenceNumber:02d}_web'
    )

    # Make Ø3 mm circular printed bosses as separate solids.
    # They are intentionally separate because Fusion profile-region selection
    # can get confused when circles overlap the main polygon.
    bossSketch = component.sketches.add(plane)
    bossSketch.name = f'crosstie_{sequenceNumber:02d}_rail_bosses'
    bossSketch.isComputeDeferred = True

    bossRadiusCm = mm_to_cm(RAIL_CROSSTIE_BOSS_DIAMETER_MM) * 0.5

    bossSketch.sketchCurves.sketchCircles.addByCenterRadius(
        toSketchPoint(bossSketch, leftPoint),
        bossRadiusCm
    )
    bossSketch.sketchCurves.sketchCircles.addByCenterRadius(
        toSketchPoint(bossSketch, rightPoint),
        bossRadiusCm
    )

    bossSketch.isComputeDeferred = False
    if HIDE_HELPER_SKETCHES:
        hideEntity(bossSketch)

    for i in range(bossSketch.profiles.count):
        try:
            extrudeProfile(
                component,
                bossSketch.profiles.item(i),
                depthCm,
                adsk.fusion.FeatureOperations.JoinFeatureOperation,
                f'crosstie_{sequenceNumber:02d}_boss_{i + 1}'
            )
        except:
            extrudeProfile(
                component,
                bossSketch.profiles.item(i),
                depthCm,
                adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
                f'crosstie_{sequenceNumber:02d}_boss_{i + 1}'
            )

    return True


def usableCrosstieIndices(pointCount):
    if pointCount < 3:
        return []

    lastInteriorIndex = pointCount - 2
    return [
        pointIndex
        for pointIndex in CROSSTIE_INDICES
        if 0 < pointIndex <= lastInteriorIndex
    ]


def createCrossties(component, scaledLeft, scaledRight, scaledBottom):
    usablePointCount = min(len(scaledLeft), len(scaledRight), len(scaledBottom))
    crosstieIndices = usableCrosstieIndices(usablePointCount)

    debugLog(
        'createCrossties: '
        f'leftCount={len(scaledLeft)}, rightCount={len(scaledRight)}, '
        f'bottomCount={len(scaledBottom)}, usablePointCount={usablePointCount}, '
        f'configuredIndices={CROSSTIE_INDICES}, usedIndices={crosstieIndices}'
    )

    if not crosstieIndices:
        return 0, f'not enough interior points; usable point count {usablePointCount}'

    count = 0

    for sequenceNumber, pointIndex in enumerate(crosstieIndices, start=1):
        leftPoint = toPoint3D(scaledLeft[pointIndex])
        rightPoint = toPoint3D(scaledRight[pointIndex])
        bottomPoint = toPoint3D(scaledBottom[pointIndex])

        ok = createCrosstie(
            component,
            leftPoint,
            rightPoint,
            bottomPoint,
            sequenceNumber,
            pointIndex
        )

        if ok:
            count += 1
        else:
            debugLog(
                f'createCrossties: failed sequence={sequenceNumber}, '
                f'pointIndex={pointIndex}, left={formatPoint(leftPoint)}, '
                f'right={formatPoint(rightPoint)}, bottom={formatPoint(bottomPoint)}'
            )

    return count, None


# ============================================================
# TRACK SPINE LOFTS
# ============================================================

def spineRecordPoints(records):
    return [record['point'] for record in records]


def computeSpineCenterRecords(scaledLeft, scaledRight, scaledBottom):
    maxCount = min(len(scaledLeft), len(scaledRight), len(scaledBottom))
    records = []
    skipped = 0

    for i in range(maxCount):
        leftPoint = toPoint3D(scaledLeft[i])
        rightPoint = toPoint3D(scaledRight[i])
        bottomPoint = toPoint3D(scaledBottom[i])

        try:
            frame = TrackSectionFrame(leftPoint, rightPoint, bottomPoint)
        except Exception as err:
            skipped += 1
            debugLog(
                f'computeSpineCenterRecords: skipped index={i}: {err}; '
                f'left={formatPoint(leftPoint)}, right={formatPoint(rightPoint)}, bottom={formatPoint(bottomPoint)}'
            )
            continue

        spineCenter = frame.pointFromRightOriginMm(
            SPINE_CENTER_RIGHT_ORIGIN_MM[0],
            SPINE_CENTER_RIGHT_ORIGIN_MM[1]
        )
        records.append({
            'index': i,
            'point': (spineCenter.x, spineCenter.y, spineCenter.z)
        })

    if DEBUG_SWEEP_LOGGING:
        debugLog(
            'computeSpineCenterRecords: '
            f'maxCount={maxCount}, valid={len(records)}, skipped={skipped}, '
            f'{pathPointStats(spineRecordPoints(records))}'
        )

    return records


def createTrackSpineSweep(component, scaledLeft, scaledRight, scaledBottom):
    debugLog(
        'createTrackSpineSweep: '
        f'leftCount={len(scaledLeft)}, rightCount={len(scaledRight)}, bottomCount={len(scaledBottom)}'
    )

    if len(scaledLeft) < 2 or len(scaledRight) < 2 or len(scaledBottom) < 2:
        debugLog('createTrackSpineSweep: not enough rail points')
        return False

    spineCenterRecords = computeSpineCenterRecords(scaledLeft, scaledRight, scaledBottom)
    if len(spineCenterRecords) < 2:
        debugLog('createTrackSpineSweep: not enough valid spine center points')
        return False

    if CREATE_SPINE_CENTER_SKETCH:
        createVisibleSplineSketch(
            component,
            spineRecordPoints(spineCenterRecords),
            'spine_center_spline'
        )

    startOk = createPartialCenterSpineSweep(
        component,
        scaledLeft,
        scaledRight,
        spineCenterRecords,
        scaledBottom,
        False,
        'start_track_spine'
    )

    endOk = createPartialCenterSpineSweep(
        component,
        scaledLeft,
        scaledRight,
        spineCenterRecords,
        scaledBottom,
        True,
        'end_track_spine'
    )

    debugLog(f'createTrackSpineSweep: startOk={startOk}, endOk={endOk}')
    return startOk and endOk


def interpolateTuple(a, b, t):
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t
    )


def partialSpineStations(orderedRecords, fraction):
    if len(orderedRecords) < 2:
        return [
            {
                'point': orderedRecords[0]['point'],
                'fromIndex': orderedRecords[0]['index'],
                'toIndex': orderedRecords[0]['index'],
                't': 0.0,
                'source': 'record'
            }
        ] if orderedRecords else []

    if fraction <= 0:
        firstRecord = orderedRecords[0]
        return [{
            'point': firstRecord['point'],
            'fromIndex': firstRecord['index'],
            'toIndex': firstRecord['index'],
            't': 0.0,
            'source': 'record'
        }]

    if fraction >= 1:
        stations = []
        for record in orderedRecords:
            stations.append({
                'point': record['point'],
                'fromIndex': record['index'],
                'toIndex': record['index'],
                't': 0.0,
                'source': 'record'
            })
        return stations

    segmentLengths = []
    totalLength = 0.0

    for i in range(len(orderedRecords) - 1):
        a = toPoint3D(orderedRecords[i]['point'])
        b = toPoint3D(orderedRecords[i + 1]['point'])
        length = a.distanceTo(b)
        segmentLengths.append(length)
        totalLength += length

    firstRecord = orderedRecords[0]
    result = [{
        'point': firstRecord['point'],
        'fromIndex': firstRecord['index'],
        'toIndex': firstRecord['index'],
        't': 0.0,
        'source': 'record'
    }]

    if totalLength < 0.000001:
        return result

    targetLength = totalLength * fraction
    walkedLength = 0.0

    for i, segmentLength in enumerate(segmentLengths):
        if segmentLength < 0.000001:
            continue

        if walkedLength + segmentLength >= targetLength:
            localFraction = (targetLength - walkedLength) / segmentLength
            endPoint = interpolateTuple(
                orderedRecords[i]['point'],
                orderedRecords[i + 1]['point'],
                localFraction
            )

            if toPoint3D(result[-1]['point']).distanceTo(toPoint3D(endPoint)) >= 0.000001:
                result.append({
                    'point': endPoint,
                    'fromIndex': orderedRecords[i]['index'],
                    'toIndex': orderedRecords[i + 1]['index'],
                    't': localFraction,
                    'source': 'interpolated'
                })

            if DEBUG_SWEEP_LOGGING:
                debugLog(
                    'partialSpineStations: '
                    f'fraction={fraction:.6f}, totalLength={totalLength:.6f}cm, '
                    f'targetLength={targetLength:.6f}cm, endSegment={i}, '
                    f'localFraction={localFraction:.6f}, stations={len(result)}, '
                    f'path={pathPointStats([station["point"] for station in result])}'
                )
            return result

        walkedLength += segmentLength
        nextRecord = orderedRecords[i + 1]
        nextPoint = nextRecord['point']
        if toPoint3D(result[-1]['point']).distanceTo(toPoint3D(nextPoint)) >= 0.000001:
            result.append({
                'point': nextPoint,
                'fromIndex': nextRecord['index'],
                'toIndex': nextRecord['index'],
                't': 0.0,
                'source': 'record'
            })

    return result


def describeStations(stations):
    if not stations:
        return 'count=0'

    sampleCount = min(DEBUG_POINT_SAMPLE_COUNT, len(stations))
    pieces = []

    for i in range(sampleCount):
        station = stations[i]
        pieces.append(
            f'{i}:idx{station["fromIndex"]}->{station["toIndex"]}'
            f'@{station["t"]:.4f}:{station["source"]}:'
            f'{formatPointTuple(station["point"])}'
        )

    return f'count={len(stations)}, sample=[{"; ".join(pieces)}]'


def railPointAtStation(railPoints, station):
    fromIndex = station['fromIndex']
    toIndex = station['toIndex']

    if fromIndex < 0 or fromIndex >= len(railPoints):
        raise IndexError(f'fromIndex {fromIndex} outside rail point count {len(railPoints)}')
    if toIndex < 0 or toIndex >= len(railPoints):
        raise IndexError(f'toIndex {toIndex} outside rail point count {len(railPoints)}')

    startPoint = toPoint3D(railPoints[fromIndex])
    if fromIndex == toIndex:
        return startPoint

    endPoint = toPoint3D(railPoints[toIndex])
    return addVec(startPoint, vectorBetween(startPoint, endPoint), station['t'])


def createSpineLoftProfile(component, scaledLeft, scaledRight, scaledBottom, station, sequenceNumber, namePrefix):
    leftPoint = railPointAtStation(scaledLeft, station)
    rightPoint = railPointAtStation(scaledRight, station)
    bottomPoint = railPointAtStation(scaledBottom, station)
    centerPoint = toPoint3D(station['point'])

    frame = TrackSectionFrame(leftPoint, rightPoint, bottomPoint)
    xScale, yScale = localProfileScale(
        leftPoint,
        rightPoint,
        bottomPoint,
        frame.origin,
        frame.yAxis
    )

    plane = createPlaneByThreePoints(
        component,
        centerPoint,
        addVec(centerPoint, frame.xAxis, 1.0),
        addVec(centerPoint, frame.yAxis, 1.0),
        f'{namePrefix}_loft_profile_plane_{sequenceNumber:02d}'
    )

    sketch, profile = createCenteredRectangleProfileOnPlane(
        component,
        plane,
        centerPoint,
        frame.xAxis,
        frame.yAxis,
        mm_to_cm(SPINE_RECT_WIDTH_MM * xScale),
        mm_to_cm(SPINE_RECT_HEIGHT_MM * yScale),
        f'{namePrefix}_loft_profile_{sequenceNumber:02d}'
    )

    if DEBUG_SWEEP_LOGGING:
        debugLog(
            f'createSpineLoftProfile {namePrefix}[{sequenceNumber}]: '
            f'station=idx{station["fromIndex"]}->{station["toIndex"]}@{station["t"]:.6f}, '
            f'center={formatPoint(centerPoint)}, profile={describeProfile(profile)}'
        )

    return profile


def createLoftFromProfiles(component, profiles, name):
    global LAST_SWEEP_ERROR

    if len(profiles) < 2:
        LAST_SWEEP_ERROR = f'{name}: not enough loft profiles ({len(profiles)})'
        debugLog(f'createLoftFromProfiles {name}: {LAST_SWEEP_ERROR}')
        return None

    try:
        debugLog(f'createLoftFromProfiles {name}: begin profileCount={len(profiles)}')
        lofts = component.features.loftFeatures
        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

        try:
            loftInput.isSolid = True
        except Exception as err:
            debugLog(f'createLoftFromProfiles {name}: could not set isSolid=True: {err}')

        try:
            loftInput.isClosed = False
        except Exception as err:
            debugLog(f'createLoftFromProfiles {name}: could not set isClosed=False: {err}')

        try:
            loftInput.isTangentEdgesMerged = False
        except Exception as err:
            debugLog(f'createLoftFromProfiles {name}: could not set isTangentEdgesMerged=False: {err}')

        sections = loftInput.loftSections
        for i, profile in enumerate(profiles):
            if DEBUG_SWEEP_LOGGING:
                debugLog(f'createLoftFromProfiles {name}: add section {i}: {describeProfile(profile)}')
            sections.add(profile)

        feature = lofts.add(loftInput)
        feature.name = name
        debugLog(f'createLoftFromProfiles {name}: success feature={feature.name}')
        return feature
    except Exception as err:
        LAST_SWEEP_ERROR = f'{name}: loft failed: {err}'
        debugLog(f'createLoftFromProfiles {name}: failed: {err}\n{traceback.format_exc()}')
        return None


def createPartialCenterSpineSweep(component, scaledLeft, scaledRight, spineCenterRecords, scaledBottom, reversePath, namePrefix):
    global LAST_SWEEP_ERROR

    if len(spineCenterRecords) < 2:
        LAST_SWEEP_ERROR = f'{namePrefix}_sweep_51_percent: not enough points'
        debugLog(
            f'createPartialCenterSpineSweep {namePrefix}: '
            f'validSpineRecords={len(spineCenterRecords)}, not enough points'
        )
        return False

    if reversePath:
        orderedRecords = list(reversed(spineCenterRecords))
    else:
        orderedRecords = spineCenterRecords[:]

    startRecord = orderedRecords[0]
    nextRecord = orderedRecords[1]
    idx = startRecord['index']
    nextIdx = nextRecord['index']
    stations = partialSpineStations(orderedRecords, END_BOX_SWEEP_FRACTION)

    if len(stations) < 2:
        LAST_SWEEP_ERROR = f'{namePrefix}_loft_51_percent: not enough loft stations'
        debugLog(
            f'createPartialCenterSpineSweep {namePrefix}: '
            f'not enough loft stations; {describeStations(stations)}'
        )
        return False

    if DEBUG_SWEEP_LOGGING:
        orderedPathPoints = spineRecordPoints(orderedRecords)
        pathPoints = [station['point'] for station in stations]
        debugLog(
            f'createPartialCenterSpineSweep {namePrefix}: '
            f'reversePath={reversePath}, validSpineRecords={len(spineCenterRecords)}, '
            f'idx={idx}, nextIdx={nextIdx}, fraction={END_BOX_SWEEP_FRACTION}, '
            f'orderedPath={pathPointStats(orderedPathPoints)}, '
            f'partialStations={pathPointStats(pathPoints)}, stations={describeStations(stations)}'
        )

    profiles = []
    for sequenceNumber, station in enumerate(stations):
        try:
            profile = createSpineLoftProfile(
                component,
                scaledLeft,
                scaledRight,
                scaledBottom,
                station,
                sequenceNumber,
                namePrefix
            )
        except Exception as err:
            LAST_SWEEP_ERROR = f'{namePrefix}_loft_51_percent: profile {sequenceNumber} failed: {err}'
            debugLog(
                f'createPartialCenterSpineSweep {namePrefix}: '
                f'profile {sequenceNumber} failed: {err}\n{traceback.format_exc()}'
            )
            return False

        if not profile:
            LAST_SWEEP_ERROR = f'{namePrefix}_loft_51_percent: profile {sequenceNumber} was empty'
            debugLog(
                f'createPartialCenterSpineSweep {namePrefix}: '
                f'profile {sequenceNumber} was empty; station={station}'
            )
            return False

        profiles.append(profile)

    loft = createLoftFromProfiles(
        component,
        profiles,
        f'{namePrefix}_loft_51_percent'
    )

    if loft:
        LAST_SWEEP_ERROR = ''
        debugLog(f'createPartialCenterSpineSweep {namePrefix}: loft succeeded')
        return True

    debugLog(
        f'createPartialCenterSpineSweep {namePrefix}: loft failed; '
        f'LAST_SWEEP_ERROR={LAST_SWEEP_ERROR}'
    )
    return False


def interpolatePoint(a, b, t):
    return adsk.core.Point3D.create(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    )


def interpolateTrackSample(startSample, endSample, targetCm, t, safeAnchor=False):
    return {
        'targetCm': targetCm,
        'distanceCm': targetCm,
        'folderName': startSample['folderName'],
        'index': startSample['index'],
        't': t,
        'safeAnchor': safeAnchor,
        'center': interpolatePoint(startSample['center'], endSample['center'], t),
        'spine': interpolatePoint(startSample['spine'], endSample['spine'], t),
        'left': interpolatePoint(startSample['left'], endSample['left'], t),
        'right': interpolatePoint(startSample['right'], endSample['right'], t),
        'bottom': interpolatePoint(startSample['bottom'], endSample['bottom'], t)
    }


def buildSupportTrackSamples(scaledTrackDataList):
    samples = []
    safeAnchors = []
    crosstieStations = []
    walkedCm = 0.0
    previousCenter = None

    for trackData in scaledTrackDataList:
        folderName = trackData['folderName']
        scaledRailData = trackData['scaledRailData']
        scaledLeft = scaledRailData.get('left', [])
        scaledCenter = scaledRailData.get('center', [])
        scaledRight = scaledRailData.get('right', [])
        scaledBottom = scaledRailData.get('bottom', [])
        maxCount = min(len(scaledLeft), len(scaledCenter), len(scaledRight), len(scaledBottom))
        trackSamples = []

        for i in range(maxCount):
            leftPoint = toPoint3D(scaledLeft[i])
            centerPoint = toPoint3D(scaledCenter[i])
            rightPoint = toPoint3D(scaledRight[i])
            bottomPoint = toPoint3D(scaledBottom[i])

            try:
                frame = TrackSectionFrame(leftPoint, rightPoint, bottomPoint)
                spinePoint = frame.pointFromRightOriginMm(
                    SPINE_CENTER_RIGHT_ORIGIN_MM[0],
                    SPINE_CENTER_RIGHT_ORIGIN_MM[1]
                )
            except:
                spinePoint = centerPoint

            if previousCenter:
                walkedCm += previousCenter.distanceTo(centerPoint)

            sample = {
                'distanceCm': walkedCm,
                'folderName': folderName,
                'index': i,
                'center': centerPoint,
                'spine': spinePoint,
                'left': leftPoint,
                'right': rightPoint,
                'bottom': bottomPoint
            }
            samples.append(sample)
            trackSamples.append(sample)
            previousCenter = centerPoint

        crosstieIndices = usableCrosstieIndices(maxCount)
        for crosstieIndex in crosstieIndices:
            if crosstieIndex < len(trackSamples):
                crosstieStations.append(trackSamples[crosstieIndex])

        for i in range(len(crosstieIndices) - 1):
            startIndex = crosstieIndices[i]
            endIndex = crosstieIndices[i + 1]
            if startIndex >= len(trackSamples) or endIndex >= len(trackSamples):
                continue

            startSample = trackSamples[startIndex]
            endSample = trackSamples[endIndex]
            targetCm = (startSample['distanceCm'] + endSample['distanceCm']) * 0.5
            safeAnchor = interpolateTrackSample(startSample, endSample, targetCm, 0.5, True)
            safeAnchor['betweenCrossties'] = (startIndex, endIndex)
            safeAnchor['nearestCrosstieDistanceCm'] = min(
                abs(targetCm - startSample['distanceCm']),
                abs(endSample['distanceCm'] - targetCm)
            )
            safeAnchors.append(safeAnchor)

    supportDebugLog(
        f'buildSupportTrackSamples: samples={len(samples)}, '
        f'safeAnchors={len(safeAnchors)}, '
        f'crosstieStations={len(crosstieStations)}, '
        f'totalLengthCm={(samples[-1]["distanceCm"] if samples else 0.0):.6f}'
    )
    return {
        'samples': samples,
        'safeAnchors': safeAnchors,
        'crosstieStations': crosstieStations
    }


def nearestCrosstieStationDistanceCm(supportTrackData, distanceCm):
    if not isinstance(supportTrackData, dict):
        return None

    crosstieStations = supportTrackData.get('crosstieStations', [])
    if not crosstieStations:
        return None

    return min(abs(station['distanceCm'] - distanceCm) for station in crosstieStations)


def addNearestCrosstieDistance(sample, supportTrackData):
    nearestCm = nearestCrosstieStationDistanceCm(supportTrackData, sample['distanceCm'])
    if nearestCm is not None:
        sample['nearestCrosstieDistanceCm'] = nearestCm
    return sample


def mergeIntervals(intervals):
    cleaned = [
        (min(start, end), max(start, end))
        for start, end in intervals
        if start is not None and end is not None
    ]
    if not cleaned:
        return []

    cleaned.sort(key=lambda item: item[0])
    merged = [cleaned[0]]

    for start, end in cleaned[1:]:
        lastStart, lastEnd = merged[-1]
        if start <= lastEnd + 0.000001:
            merged[-1] = (lastStart, max(lastEnd, end))
        else:
            merged.append((start, end))

    return merged


def complementIntervals(forbidden, startCm, endCm):
    if endCm < startCm:
        return []

    allowed = []
    cursor = startCm

    for start, end in forbidden:
        if end < startCm or start > endCm:
            continue

        start = max(start, startCm)
        end = min(end, endCm)

        if start > cursor:
            allowed.append((cursor, start))

        cursor = max(cursor, end)

    if cursor < endCm:
        allowed.append((cursor, endCm))

    return allowed


def supportTrackPadSquareSizeCm(connectorDiameterCm):
    return max(connectorDiameterCm * SUPPORT_TRACK_PAD_SQUARE_SCALE, SUPPORT_MIN_CONNECTOR_DIAMETER_CM)


def supportTrackPadCircleDiameterCm(beamDiameterCm):
    return max(beamDiameterCm * SUPPORT_TRACK_PAD_CIRCLE_SCALE, SUPPORT_MIN_CONNECTOR_DIAMETER_CM)


def supportTrackCollarDiameterCm(beamDiameterCm):
    return max(beamDiameterCm * SUPPORT_TRACK_COLLAR_DIAMETER_SCALE, SUPPORT_MIN_CONNECTOR_DIAMETER_CM)


def supportPadStandoffCm(supportKind):
    if supportKind in ('single', 'double', '90', '180'):
        return SUPPORT_SINGLE_DOUBLE_PAD_STANDOFF_CM

    return SUPPORT_TRACK_PAD_STANDOFF_CM


def supportCollarLengthCm(supportKind):
    if supportKind in ('single', 'double', '90', '180'):
        return SUPPORT_SINGLE_DOUBLE_COLLAR_LENGTH_CM

    return SUPPORT_TRACK_COLLAR_LENGTH_CM


def supportMinHeadLengthCm(supportKind):
    if supportKind in ('90', '180'):
        return SUPPORT_OVERHANG_MIN_HEAD_LENGTH_CM

    return SUPPORT_MIN_ANGLED_CONNECTOR_LENGTH_CM


def supportRequiredInterfaceStandoffCm(supportKind):
    return max(
        SUPPORT_TRACK_INTERFACE_STANDOFF_CM,
        supportPadStandoffCm(supportKind)
        + supportCollarLengthCm(supportKind)
        + supportMinHeadLengthCm(supportKind)
    )


def supportConnectorClearanceCm(connectorDiameterCm, squareSizeCm):
    crosstieHalfDepthCm = mm_to_cm(CROSSTIE_DEPTH_MM) * 0.5
    footprintHalfCm = max(connectorDiameterCm, squareSizeCm) * 0.5
    return crosstieHalfDepthCm + footprintHalfCm + SUPPORT_CROSSTIE_MARGIN_CM


def clampDistanceCm(value, startCm, endCm):
    return min(max(value, startCm), endCm)


def findCrosstieBayForDistance(supportTrackData, targetCm):
    if not isinstance(supportTrackData, dict):
        return None

    stations = sorted(
        [
            station for station in supportTrackData.get('crosstieStations', [])
            if station.get('distanceCm') is not None
        ],
        key=lambda station: station['distanceCm']
    )
    if len(stations) < 2:
        return None

    previousStation = None
    nextStation = None

    for station in stations:
        if station['distanceCm'] <= targetCm:
            previousStation = station
            continue

        nextStation = station
        break

    if not previousStation or not nextStation:
        return None

    previousCm = previousStation['distanceCm']
    nextCm = nextStation['distanceCm']
    bayGapCm = nextCm - previousCm
    if bayGapCm <= 0.0 or bayGapCm > SUPPORT_MAX_CROSSTIE_BAY_CM:
        return None

    bayCenterCm = (previousCm + nextCm) * 0.5
    return {
        'previousStation': previousStation,
        'nextStation': nextStation,
        'prevCrosstieCm': previousCm,
        'nextCrosstieCm': nextCm,
        'bayGapCm': bayGapCm,
        'bayCenterCm': bayCenterCm,
        'centerSnapDeltaCm': bayCenterCm - targetCm
    }


def availableSupportFootprintCm(bayInfo):
    if not bayInfo:
        return None

    return (
        bayInfo['bayGapCm']
        - mm_to_cm(CROSSTIE_DEPTH_MM)
        - SUPPORT_CROSSTIE_MARGIN_CM * 2.0
    )


def fittedConnectorSizesCm(beamDiameterCm, bayInfo, supportKind):
    if supportKind in ('single', 'double', '90', '180'):
        desiredSquareCm = mm_to_cm(SUPPORT_SINGLE_DOUBLE_TRACK_SQUARE_SIZE_MM)
        desiredCircleCm = beamDiameterCm
        desiredCollarCm = beamDiameterCm
    else:
        desiredSquareCm = supportTrackPadSquareSizeCm(beamDiameterCm)
        desiredCircleCm = supportTrackPadCircleDiameterCm(beamDiameterCm)
        desiredCollarCm = supportTrackCollarDiameterCm(beamDiameterCm)

    finalSquareCm = desiredSquareCm
    finalCircleCm = desiredCircleCm
    finalCollarCm = desiredCollarCm
    availableCm = availableSupportFootprintCm(bayInfo)
    capScale = 1.0
    fitStatus = 'uncapped'

    if availableCm is not None:
        largestCm = max(desiredSquareCm, desiredCircleCm, desiredCollarCm)
        if availableCm < SUPPORT_MIN_CONNECTOR_DIAMETER_CM:
            finalSquareCm = SUPPORT_MIN_CONNECTOR_DIAMETER_CM
            finalCircleCm = SUPPORT_MIN_CONNECTOR_DIAMETER_CM
            finalCollarCm = SUPPORT_MIN_CONNECTOR_DIAMETER_CM
            capScale = 0.0
            fitStatus = 'bay_too_tight_minimum'
        elif largestCm > availableCm:
            capScale = availableCm / largestCm
            finalSquareCm = max(desiredSquareCm * capScale, SUPPORT_MIN_CONNECTOR_DIAMETER_CM)
            finalCircleCm = max(desiredCircleCm * capScale, SUPPORT_MIN_CONNECTOR_DIAMETER_CM)
            finalCollarCm = max(desiredCollarCm * capScale, SUPPORT_MIN_CONNECTOR_DIAMETER_CM)
            fitStatus = 'capped_to_bay'

    return {
        'beamDiameterCm': beamDiameterCm,
        'desiredSquareSizeCm': desiredSquareCm,
        'squareSizeCm': finalSquareCm,
        'desiredCircleDiameterCm': desiredCircleCm,
        'circleDiameterCm': finalCircleCm,
        'desiredCollarDiameterCm': desiredCollarCm,
        'collarDiameterCm': finalCollarCm,
        'availableFootprintCm': availableCm,
        'capScale': capScale,
        'fitStatus': fitStatus,
        'nearTrackFootprintCm': max(finalSquareCm, finalCircleCm, finalCollarCm)
    }


def sampleSupportTrackInBay(supportTrackData, targetCm, bayInfo, requiredClearanceCm):
    if not bayInfo:
        return None

    sample = sampleSupportTrackAtDistance(supportTrackData, bayInfo['bayCenterCm'])
    if not sample:
        return None

    nearestCm = nearestCrosstieStationDistanceCm(supportTrackData, bayInfo['bayCenterCm'])
    sample.update({
        'snapStatus': 'bay_center',
        'originalTargetCm': targetCm,
        'snappedTargetCm': bayInfo['bayCenterCm'],
        'snapDeltaCm': bayInfo['bayCenterCm'] - targetCm,
        'requiredClearanceCm': requiredClearanceCm,
        'nearestCrosstieCm': nearestCm,
        'prevCrosstieCm': bayInfo['prevCrosstieCm'],
        'nextCrosstieCm': bayInfo['nextCrosstieCm'],
        'bayGapCm': bayInfo['bayGapCm'],
        'bayCenterCm': bayInfo['bayCenterCm'],
        'centerSnapDeltaCm': bayInfo['centerSnapDeltaCm']
    })
    sample['targetCm'] = bayInfo['bayCenterCm']
    sample['distanceCm'] = bayInfo['bayCenterCm']
    return sample


def safeSupportDistanceCm(supportTrackData, targetCm, requiredClearanceCm):
    if isinstance(supportTrackData, dict):
        samples = supportTrackData.get('samples', [])
    else:
        samples = supportTrackData or []

    if not samples:
        return targetCm, {
            'snapStatus': 'no_track_samples',
            'originalTargetCm': targetCm,
            'snappedTargetCm': targetCm,
            'snapDeltaCm': 0.0,
            'requiredClearanceCm': requiredClearanceCm,
            'nearestCrosstieCm': None
        }

    startCm = samples[0]['distanceCm']
    endCm = samples[-1]['distanceCm']
    clampedTargetCm = clampDistanceCm(targetCm, startCm, endCm)

    if requiredClearanceCm <= 0.0:
        nearestCm = nearestCrosstieStationDistanceCm(supportTrackData, clampedTargetCm)
        return clampedTargetCm, {
            'snapStatus': 'not_requested',
            'originalTargetCm': targetCm,
            'snappedTargetCm': clampedTargetCm,
            'snapDeltaCm': clampedTargetCm - targetCm,
            'requiredClearanceCm': requiredClearanceCm,
            'nearestCrosstieCm': nearestCm
        }

    crosstieStations = []
    if isinstance(supportTrackData, dict):
        crosstieStations = supportTrackData.get('crosstieStations', [])

    forbidden = []
    for station in crosstieStations:
        stationCm = station.get('distanceCm')
        if stationCm is None:
            continue
        forbidden.append((
            max(startCm, stationCm - requiredClearanceCm),
            min(endCm, stationCm + requiredClearanceCm)
        ))

    forbidden = mergeIntervals(forbidden)
    allowed = complementIntervals(forbidden, startCm, endCm)
    nearestOriginalCm = nearestCrosstieStationDistanceCm(supportTrackData, clampedTargetCm)

    if not allowed:
        return clampedTargetCm, {
            'snapStatus': 'no_safe_interval',
            'originalTargetCm': targetCm,
            'snappedTargetCm': clampedTargetCm,
            'snapDeltaCm': clampedTargetCm - targetCm,
            'requiredClearanceCm': requiredClearanceCm,
            'nearestCrosstieCm': nearestOriginalCm
        }

    for start, end in allowed:
        if start - 0.000001 <= clampedTargetCm <= end + 0.000001:
            nearestCm = nearestCrosstieStationDistanceCm(supportTrackData, clampedTargetCm)
            return clampedTargetCm, {
                'snapStatus': 'already_safe',
                'originalTargetCm': targetCm,
                'snappedTargetCm': clampedTargetCm,
                'snapDeltaCm': clampedTargetCm - targetCm,
                'requiredClearanceCm': requiredClearanceCm,
                'nearestCrosstieCm': nearestCm
            }

    candidates = []
    for start, end in allowed:
        candidates.append(start)
        candidates.append(end)

    snappedCm = min(candidates, key=lambda item: abs(item - clampedTargetCm))
    movementCm = abs(snappedCm - clampedTargetCm)
    if movementCm > SUPPORT_MAX_SNAP_CM:
        snappedCm = clampedTargetCm
        snapStatus = 'snap_exceeds_max'
    else:
        snapStatus = 'snapped'

    nearestCm = nearestCrosstieStationDistanceCm(supportTrackData, snappedCm)
    return snappedCm, {
        'snapStatus': snapStatus,
        'originalTargetCm': targetCm,
        'snappedTargetCm': snappedCm,
        'snapDeltaCm': snappedCm - targetCm,
        'requiredClearanceCm': requiredClearanceCm,
        'nearestCrosstieCm': nearestCm,
        'nearestOriginalCrosstieCm': nearestOriginalCm,
        'snapMovementCm': movementCm
    }


def sampleSupportTrackAtDistance(supportTrackData, distanceCm):
    if isinstance(supportTrackData, dict):
        samples = supportTrackData.get('samples', [])
    else:
        samples = supportTrackData or []

    if not samples:
        return None

    startCm = samples[0]['distanceCm']
    endCm = samples[-1]['distanceCm']
    distanceCm = clampDistanceCm(distanceCm, startCm, endCm)

    if distanceCm <= startCm:
        result = dict(samples[0])
        result['targetCm'] = startCm
        result['distanceCm'] = startCm
        result['t'] = 0.0
        return addNearestCrosstieDistance(result, supportTrackData)

    if distanceCm >= endCm:
        result = dict(samples[-1])
        result['targetCm'] = endCm
        result['distanceCm'] = endCm
        result['t'] = 1.0
        return addNearestCrosstieDistance(result, supportTrackData)

    for i in range(len(samples) - 1):
        startSample = samples[i]
        endSample = samples[i + 1]
        if endSample['distanceCm'] < distanceCm:
            continue

        segmentLength = endSample['distanceCm'] - startSample['distanceCm']
        t = 0.0 if segmentLength < 0.000001 else (distanceCm - startSample['distanceCm']) / segmentLength
        result = interpolateTrackSample(startSample, endSample, distanceCm, t)
        return addNearestCrosstieDistance(result, supportTrackData)

    result = dict(samples[-1])
    result['targetCm'] = endCm
    result['distanceCm'] = endCm
    result['t'] = 1.0
    return addNearestCrosstieDistance(result, supportTrackData)


def sampleSupportTrack(
    supportTrackData,
    centerRailsCoordMeters,
    supportScaleFactor,
    requiredClearanceCm=0.0,
    snapToSafe=True
):
    if isinstance(supportTrackData, dict):
        samples = supportTrackData.get('samples', [])
    else:
        samples = supportTrackData or []

    if not samples:
        return None

    originalTargetCm = centerRailsCoordMeters * supportScaleFactor
    if snapToSafe and requiredClearanceCm > 0.0:
        sampledDistanceCm, snapInfo = safeSupportDistanceCm(
            supportTrackData,
            originalTargetCm,
            requiredClearanceCm
        )
    else:
        startCm = samples[0]['distanceCm']
        endCm = samples[-1]['distanceCm']
        sampledDistanceCm = clampDistanceCm(originalTargetCm, startCm, endCm)
        nearestCm = nearestCrosstieStationDistanceCm(supportTrackData, sampledDistanceCm)
        snapInfo = {
            'snapStatus': 'not_requested',
            'originalTargetCm': originalTargetCm,
            'snappedTargetCm': sampledDistanceCm,
            'snapDeltaCm': sampledDistanceCm - originalTargetCm,
            'requiredClearanceCm': requiredClearanceCm,
            'nearestCrosstieCm': nearestCm
        }

    sample = sampleSupportTrackAtDistance(supportTrackData, sampledDistanceCm)
    if not sample:
        return None

    sample.update(snapInfo)
    sample['targetCm'] = sampledDistanceCm
    sample['distanceCm'] = sampledDistanceCm
    return sample


def supportTrackFrame(trackSample):
    try:
        return TrackSectionFrame(trackSample['left'], trackSample['right'], trackSample['bottom'])
    except:
        return None


def supportTrackInterfaceAnchor(trackSample, frame):
    if not frame:
        return trackSample['spine']

    halfSpineHeightCm = mm_to_cm(SPINE_RECT_HEIGHT_MM) * 0.5
    return addVec(trackSample['spine'], frame.yAxis, -halfSpineHeightCm)


def supportInterfaceKind(interface):
    source = (interface.get('sourcePath', '') + ' ' + str(interface.get('rascType', ''))).lower()

    if '90deg' in source or '90 deg' in source:
        return '90'
    if '180' in source:
        return '180'
    if 'double' in source:
        return 'double'
    if 'straight' in source:
        return 'straight'

    return 'single'


def supportTrackRollDegrees(frame):
    if not frame:
        return None

    tangent = copyVector(frame.zAxis)
    trackUp = copyVector(frame.yAxis)
    if not safeNormalize(tangent) or not safeNormalize(trackUp):
        return None

    referenceUp = adsk.core.Vector3D.create(0, 0, 1)
    axial = referenceUp.dotProduct(tangent)
    referenceUp = adsk.core.Vector3D.create(
        referenceUp.x - tangent.x * axial,
        referenceUp.y - tangent.y * axial,
        referenceUp.z - tangent.z * axial
    )

    if not safeNormalize(referenceUp):
        for fallback in (
            adsk.core.Vector3D.create(0, 1, 0),
            adsk.core.Vector3D.create(1, 0, 0)
        ):
            axial = fallback.dotProduct(tangent)
            referenceUp = adsk.core.Vector3D.create(
                fallback.x - tangent.x * axial,
                fallback.y - tangent.y * axial,
                fallback.z - tangent.z * axial
            )
            if safeNormalize(referenceUp):
                break
        else:
            return None

    cosine = max(-1.0, min(1.0, referenceUp.dotProduct(trackUp)))
    cross = referenceUp.crossProduct(trackUp)
    sine = tangent.dotProduct(cross)
    angleDegrees = math.degrees(math.atan2(sine, cosine))
    if angleDegrees < 0.0:
        angleDegrees += 360.0

    return angleDegrees


def supportKindForTrackRoll(xmlSupportKind, rollDegrees):
    if rollDegrees is None:
        return xmlSupportKind

    normalizedRoll = rollDegrees % 360.0
    mirroredRoll = min(normalizedRoll, 360.0 - normalizedRoll)

    if mirroredRoll >= SUPPORT_90_MAX_ROLL_DEG:
        return '180'
    if mirroredRoll >= SUPPORT_REGULAR_MAX_ROLL_DEG:
        return '90'

    return xmlSupportKind


def straightSupportClearanceDropCm(trackSample):
    clearanceCm = trackSample.get('nearestCrosstieDistanceCm', None)
    if clearanceCm is None or clearanceCm >= SUPPORT_CROSSTIE_CLEARANCE_CM:
        return 0.0

    neededDrop = math.sqrt(max(0.0, SUPPORT_CROSSTIE_CLEARANCE_CM ** 2 - clearanceCm ** 2))
    return min(SUPPORT_STRAIGHT_CLEARANCE_DROP_MAX_CM, neededDrop + 0.02)


def supportOffsetForPrefab(prefabOffsets, prefabIndex):
    return prefabOffsets.get(prefabIndex, zeroVector())


def supportBeamLabel(beam):
    if not beam:
        return 'None'
    return f'{beam.get("startId")}->{beam.get("endId")}:{beam.get("kind")}'


def supportTupleDistance(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2
        + (a[1] - b[1]) ** 2
        + (a[2] - b[2]) ** 2
    )


def buildSupportBeamAdjacency(supportData):
    beamsByNode = {}
    for beam in supportData.get('beams', []):
        beamsByNode.setdefault((beam['prefabIndex'], beam['startId']), []).append(beam)
        beamsByNode.setdefault((beam['prefabIndex'], beam['endId']), []).append(beam)
    return beamsByNode


def supportBeamOtherMeters(beam, nodeId):
    if beam.get('startId') == nodeId:
        return beam.get('endMeters')
    if beam.get('endId') == nodeId:
        return beam.get('startMeters')
    return None


def chooseConnectedSupportBeam(interface, beamsByNode):
    nodeKey = (interface['prefabIndex'], interface['nodeId'])
    connected = beamsByNode.get(nodeKey, [])
    if not connected:
        supportDebugLog(
            f'interface plan warning: prefab={interface["prefabIndex"]}, '
            f'node={interface["nodeId"]} has no connected beam'
        )
        return None

    interfaceMeters = interface['pointMeters']

    def beamPriority(beam):
        otherMeters = supportBeamOtherMeters(beam, interface['nodeId'])
        if not otherMeters:
            drop = 0.0
            length = 0.0
        else:
            drop = interfaceMeters[1] - otherMeters[1]
            length = supportTupleDistance(interfaceMeters, otherMeters)
        rascScore = 1 if beam.get('kind') == 'rasc_connector' else 0
        return rascScore, max(drop, 0.0), length

    selected = max(connected, key=beamPriority)
    if len(connected) > 1:
        supportDebugLog(
            f'interface plan ambiguity: prefab={interface["prefabIndex"]}, '
            f'node={interface["nodeId"]}, selected={supportBeamLabel(selected)}, '
            f'choices={[supportBeamLabel(beam) for beam in connected]}'
        )

    return selected


def generatedSupportBeamDiameterCm(beam, interface, supportScaleFactor):
    diameterMeters = beam['diameterMeters'] if beam else interface['diameterMeters']
    return max(diameterMeters * supportScaleFactor * SUPPORT_BEAM_DIAMETER_SCALE, 0.05)


def planSupportInterfaces(supportData, supportTrackSamples, supportScaleFactor, beamsByNode):
    plans = []

    if not supportTrackSamples:
        return plans

    for sequenceNumber, interface in enumerate(supportData.get('interfaces', []), start=1):
        xmlSupportKind = supportInterfaceKind(interface)
        supportKind = xmlSupportKind
        selectedBeam = chooseConnectedSupportBeam(interface, beamsByNode)
        targetCm = interface['centerRailsCoord'] * supportScaleFactor
        beamDiameterCm = generatedSupportBeamDiameterCm(
            selectedBeam,
            interface,
            supportScaleFactor
        )
        bayInfo = findCrosstieBayForDistance(supportTrackSamples, targetCm)
        connectorSizes = fittedConnectorSizesCm(beamDiameterCm, bayInfo, supportKind)
        requiredClearanceCm = supportConnectorClearanceCm(
            connectorSizes['nearTrackFootprintCm'],
            connectorSizes['squareSizeCm']
        )

        trackSample = sampleSupportTrackInBay(
            supportTrackSamples,
            targetCm,
            bayInfo,
            requiredClearanceCm
        )
        if not trackSample:
            trackSample = sampleSupportTrack(
                supportTrackSamples,
                interface['centerRailsCoord'],
                supportScaleFactor,
                requiredClearanceCm=requiredClearanceCm,
                snapToSafe=True
            )

        if not trackSample:
            supportDebugLog(
                f'interface plan {sequenceNumber}: skipped; no track sample, '
                f'prefab={interface["prefabIndex"]}, node={interface["nodeId"]}'
            )
            continue

        frame = supportTrackFrame(trackSample)
        rollDegrees = supportTrackRollDegrees(frame)
        reclassifiedKind = supportKindForTrackRoll(xmlSupportKind, rollDegrees)

        if reclassifiedKind != supportKind:
            supportKind = reclassifiedKind
            connectorSizes = fittedConnectorSizesCm(beamDiameterCm, bayInfo, supportKind)
            requiredClearanceCm = supportConnectorClearanceCm(
                connectorSizes['nearTrackFootprintCm'],
                connectorSizes['squareSizeCm']
            )

            reclassifiedTrackSample = sampleSupportTrackInBay(
                supportTrackSamples,
                targetCm,
                bayInfo,
                requiredClearanceCm
            )
            if not reclassifiedTrackSample:
                reclassifiedTrackSample = sampleSupportTrack(
                    supportTrackSamples,
                    interface['centerRailsCoord'],
                    supportScaleFactor,
                    requiredClearanceCm=requiredClearanceCm,
                    snapToSafe=True
                )

            if reclassifiedTrackSample:
                trackSample = reclassifiedTrackSample
                frame = supportTrackFrame(trackSample)
                rollDegrees = supportTrackRollDegrees(frame)
                supportKind = supportKindForTrackRoll(xmlSupportKind, rollDegrees)
                connectorSizes = fittedConnectorSizesCm(beamDiameterCm, bayInfo, supportKind)
                requiredClearanceCm = supportConnectorClearanceCm(
                    connectorSizes['nearTrackFootprintCm'],
                    connectorSizes['squareSizeCm']
                )

        trackAnchor = supportTrackInterfaceAnchor(trackSample, frame)
        standoffCm = supportRequiredInterfaceStandoffCm(supportKind)
        if frame:
            desiredTop = addVec(trackAnchor, frame.yAxis, -standoffCm)
        else:
            desiredTop = adsk.core.Point3D.create(trackAnchor.x, trackAnchor.y, trackAnchor.z - standoffCm)

        plan = {
            'sequenceNumber': sequenceNumber,
            'interface': interface,
            'supportKind': supportKind,
            'xmlSupportKind': xmlSupportKind,
            'rollDegrees': rollDegrees,
            'selectedBeam': selectedBeam,
            'trackSample': trackSample,
            'frame': frame,
            'trackAnchor': trackAnchor,
            'desiredSupportTop': desiredTop,
            'beamDiameterCm': beamDiameterCm,
            'connectorDiameterCm': connectorSizes['circleDiameterCm'],
            'circleDiameterCm': connectorSizes['circleDiameterCm'],
            'collarDiameterCm': connectorSizes['collarDiameterCm'],
            'squareSizeCm': connectorSizes['squareSizeCm'],
            'desiredSquareSizeCm': connectorSizes['desiredSquareSizeCm'],
            'desiredCircleDiameterCm': connectorSizes['desiredCircleDiameterCm'],
            'desiredCollarDiameterCm': connectorSizes['desiredCollarDiameterCm'],
            'availableFootprintCm': connectorSizes['availableFootprintCm'],
            'nearTrackFootprintCm': connectorSizes['nearTrackFootprintCm'],
            'connectorFitStatus': connectorSizes['fitStatus'],
            'connectorCapScale': connectorSizes['capScale'],
            'requiredClearanceCm': requiredClearanceCm,
            'requiredInterfaceStandoffCm': standoffCm,
            'bayInfo': bayInfo
        }
        plans.append(plan)

        nearestLogCm = trackSample.get(
            'nearestCrosstieCm',
            trackSample.get('nearestCrosstieDistanceCm', -1.0)
        )
        if nearestLogCm is None:
            nearestLogCm = -1.0
        availableFootprintCm = connectorSizes['availableFootprintCm']
        if availableFootprintCm is None:
            availableFootprintCm = -1.0

        supportDebugLog(
            f'interface plan {sequenceNumber}: prefab={interface["prefabIndex"]}, '
            f'node={interface["nodeId"]}, xmlKind={xmlSupportKind}, '
            f'kind={supportKind}, rollDeg={(rollDegrees if rollDegrees is not None else -1.0):.6f}, '
            f'selectedBeam={supportBeamLabel(selectedBeam)}, '
            f'originalTargetCm={trackSample.get("originalTargetCm", trackSample["targetCm"]):.6f}, '
            f'prevCrosstieCm={trackSample.get("prevCrosstieCm", -1.0):.6f}, '
            f'nextCrosstieCm={trackSample.get("nextCrosstieCm", -1.0):.6f}, '
            f'bayGapCm={trackSample.get("bayGapCm", -1.0):.6f}, '
            f'bayCenterCm={trackSample.get("bayCenterCm", -1.0):.6f}, '
            f'snappedTargetCm={trackSample.get("snappedTargetCm", trackSample["targetCm"]):.6f}, '
            f'snapStatus={trackSample.get("snapStatus", "unknown")}, '
            f'snapDeltaCm={trackSample.get("snapDeltaCm", 0.0):.6f}, '
            f'requiredClearanceCm={requiredClearanceCm:.6f}, '
            f'availableFootprintCm={availableFootprintCm:.6f}, '
            f'nearestCrosstieCm={nearestLogCm:.6f}, '
            f'beamDiameterCm={beamDiameterCm:.6f}, '
            f'desiredSquareSizeCm={connectorSizes["desiredSquareSizeCm"]:.6f}, '
            f'finalSquareSizeCm={connectorSizes["squareSizeCm"]:.6f}, '
            f'desiredCircleDiameterCm={connectorSizes["desiredCircleDiameterCm"]:.6f}, '
            f'finalCircleDiameterCm={connectorSizes["circleDiameterCm"]:.6f}, '
            f'finalCollarDiameterCm={connectorSizes["collarDiameterCm"]:.6f}, '
            f'fitStatus={connectorSizes["fitStatus"]}, capScale={connectorSizes["capScale"]:.6f}, '
            f'requiredInterfaceStandoffCm={standoffCm:.6f}, '
            f'track={trackSample["folderName"]}[{trackSample["index"]}], t={trackSample["t"]:.6f}'
        )

    return plans


def controllingInterfacePlanForPrefab(plans):
    return max(
        plans,
        key=lambda plan: (
            plan['interface']['pointMeters'][1],
            -plan['sequenceNumber']
        )
    )


def computeSupportPrefabOffsetsFromPlans(supportData, interfacePlans, supportScaleFactor):
    offsets = {}
    plansByPrefab = {}

    for plan in interfacePlans:
        plansByPrefab.setdefault(plan['interface']['prefabIndex'], []).append(plan)

    for prefabIndex, plans in plansByPrefab.items():
        controllingPlan = controllingInterfacePlanForPrefab(plans)
        controllingInterface = controllingPlan['interface']

        if len(plans) > 1:
            otherNodes = [
                plan['interface']['nodeId']
                for plan in plans
                if plan is not controllingPlan
            ]
            supportDebugLog(
                f'prefab {prefabIndex} has {len(plans)} interfaces; '
                f'controlling node={controllingInterface["nodeId"]}; other nodes={otherNodes}'
            )

        nativeTop = supportPointMetersToFusion(
            controllingInterface['pointMeters'],
            supportScaleFactor
        )
        desiredTop = controllingPlan['desiredSupportTop']
        offset = vectorBetween(nativeTop, desiredTop)
        offsets[prefabIndex] = offset

        supportDebugLog(
            f'prefab offset {prefabIndex}: controllingNode={controllingInterface["nodeId"]}, '
            f'nativeTop={formatPoint(nativeTop)}, desiredSupportTop={formatPoint(desiredTop)}, '
            f'offset={formatVector(offset)}, '
            f'snapStatus={controllingPlan["trackSample"].get("snapStatus", "unknown")}'
        )

    return offsets


def supportBeamEndpointPoint(beam, endpointName, supportScaleFactor, prefabOffsets, floorZ):
    if endpointName == 'start':
        return supportPointForKind(
            beam['startMeters'],
            beam.get('startKind'),
            beam['prefabIndex'],
            supportScaleFactor,
            prefabOffsets,
            floorZ
        )

    return supportPointForKind(
        beam['endMeters'],
        beam.get('endKind'),
        beam['prefabIndex'],
        supportScaleFactor,
        prefabOffsets,
        floorZ
    )


def selectedBeamOtherPoint(beam, nodeId, supportScaleFactor, prefabOffsets, floorZ):
    if not beam:
        return None

    if beam.get('startId') == nodeId:
        return supportBeamEndpointPoint(beam, 'end', supportScaleFactor, prefabOffsets, floorZ)

    if beam.get('endId') == nodeId:
        return supportBeamEndpointPoint(beam, 'start', supportScaleFactor, prefabOffsets, floorZ)

    return None


def buildSupportInterfacePlacements(
    supportData,
    interfacePlans,
    prefabOffsets,
    supportScaleFactor,
    floorZ,
    beamsByNode
):
    placements = []
    endpointOverrides = {}

    for plan in interfacePlans:
        interface = plan['interface']
        frame = plan['frame']
        if not frame:
            supportDebugLog(
                f'placement {plan["sequenceNumber"]}: skipped; no track frame, '
                f'prefab={interface["prefabIndex"]}, node={interface["nodeId"]}'
            )
            continue

        offset = supportOffsetForPrefab(prefabOffsets, interface['prefabIndex'])
        supportTop = supportPointMetersToFusion(
            interface['pointMeters'],
            supportScaleFactor,
            offset
        )
        trackAnchor = plan['trackAnchor']
        padNormal = scaledVector(frame.yAxis, -1.0)
        padAcross = copyVector(frame.xAxis)
        padAlongTrack = copyVector(frame.zAxis)
        padStandoffCm = supportPadStandoffCm(plan['supportKind'])
        collarLengthCm = supportCollarLengthCm(plan['supportKind'])
        circleCenter = addVec(trackAnchor, padNormal, padStandoffCm)
        neckEnd = addVec(circleCenter, padNormal, collarLengthCm)
        beamEnd = supportTop

        selectedBeam = plan['selectedBeam']
        otherPoint = selectedBeamOtherPoint(
            selectedBeam,
            interface['nodeId'],
            supportScaleFactor,
            prefabOffsets,
            floorZ
        )

        if otherPoint:
            beamAxis = vectorBetween(beamEnd, otherPoint)
            beamAxisSource = 'selected_beam'
        else:
            beamAxis = vectorBetween(neckEnd, beamEnd)
            beamAxisSource = 'neck_to_beam_fallback'

        if not safeNormalize(beamAxis):
            beamAxis = copyVector(padNormal)
            beamAxisSource = 'pad_normal_fallback'

        collarStart = addVec(circleCenter, padNormal, -SUPPORT_CONNECTOR_OVERLAP_CM)
        collarEnd = addVec(neckEnd, padNormal, SUPPORT_CONNECTOR_OVERLAP_CM)
        angledStart = addVec(neckEnd, padNormal, -SUPPORT_CONNECTOR_OVERLAP_CM)
        angledEnd = addVec(beamEnd, beamAxis, SUPPORT_CONNECTOR_OVERLAP_CM)
        beamExtensionCm = 0.0
        endpointOverrides[(interface['prefabIndex'], interface['nodeId'])] = beamEnd

        placement = {
            'sequenceNumber': plan['sequenceNumber'],
            'interface': interface,
            'supportKind': plan['supportKind'],
            'xmlSupportKind': plan['xmlSupportKind'],
            'rollDegrees': plan['rollDegrees'],
            'trackSample': plan['trackSample'],
            'frame': frame,
            'trackAnchor': trackAnchor,
            'padNormal': padNormal,
            'padAcross': padAcross,
            'padAlongTrack': padAlongTrack,
            'beamDiameterCm': plan['beamDiameterCm'],
            'connectorDiameterCm': plan['connectorDiameterCm'],
            'circleDiameterCm': plan['circleDiameterCm'],
            'collarDiameterCm': plan['collarDiameterCm'],
            'squareSizeCm': plan['squareSizeCm'],
            'padStandoffCm': padStandoffCm,
            'collarLengthCm': collarLengthCm,
            'circleCenter': circleCenter,
            'collarStart': collarStart,
            'collarEnd': collarEnd,
            'neckEnd': neckEnd,
            'angledStart': angledStart,
            'angledEnd': angledEnd,
            'supportTop': supportTop,
            'beamEnd': beamEnd,
            'beamAxis': beamAxis,
            'beamAxisSource': beamAxisSource,
            'otherBeamEndpoint': otherPoint,
            'selectedBeam': selectedBeam,
            'beamExtensionCm': beamExtensionCm,
            'requiredInterfaceStandoffCm': plan['requiredInterfaceStandoffCm']
        }
        placements.append(placement)

        neckToBeamCm = neckEnd.distanceTo(beamEnd)
        supportDebugLog(
            f'placement {plan["sequenceNumber"]}: prefab={interface["prefabIndex"]}, '
            f'node={interface["nodeId"]}, selectedBeam={supportBeamLabel(selectedBeam)}, '
            f'beamAxisSource={beamAxisSource}, trackAnchor={formatPoint(trackAnchor)}, '
            f'circleCenter={formatPoint(circleCenter)}, neckEnd={formatPoint(neckEnd)}, '
            f'collarStart={formatPoint(collarStart)}, collarEnd={formatPoint(collarEnd)}, '
            f'angledStart={formatPoint(angledStart)}, angledEnd={formatPoint(angledEnd)}, '
            f'beamRawTop={formatPoint(supportTop)}, beamEnd={formatPoint(beamEnd)}, '
            f'otherBeamEndpoint={formatPoint(otherPoint)}, padNormal={formatVector(padNormal)}, '
            f'padAcross={formatVector(padAcross)}, padAlongTrack={formatVector(padAlongTrack)}, '
            f'beamAxis={formatVector(beamAxis)}, '
            f'trackAnchorToCircleCm={trackAnchor.distanceTo(circleCenter):.6f}, '
            f'circleToNeckCm={circleCenter.distanceTo(neckEnd):.6f}, '
            f'neckToBeamCm={neckToBeamCm:.6f}, '
            f'padStandoffCm={padStandoffCm:.6f}, collarLengthCm={collarLengthCm:.6f}, '
            f'requiredInterfaceStandoffCm={plan["requiredInterfaceStandoffCm"]:.6f}, '
            f'beamExtensionCm={beamExtensionCm:.6f}, '
            f'connectorOverlapCm={SUPPORT_CONNECTOR_OVERLAP_CM:.6f}'
        )

    return placements, endpointOverrides


# ============================================================
# NL2 SUPPORT XML CREATION
# ============================================================

def parseSupportFloat(value, default=None):
    try:
        return float(value)
    except:
        return default


def parseSupportPosition(text):
    if not text:
        return None

    parts = text.replace(',', ' ').split()
    if len(parts) < 3:
        return None

    try:
        return float(parts[0]), float(parts[1]), float(parts[2])
    except:
        return None


def supportElementPosition(element):
    position = parseSupportPosition(element.get('pos'))
    if position:
        return position

    posElement = element.find('pos')
    if posElement is None:
        return None

    return parseSupportPosition(posElement.text)


def supportBeamSizeMeters(element):
    sizes = []

    for attrName in ('size1', 'size2', 'size'):
        size = parseSupportFloat(element.get(attrName), None)
        if size and size > 0:
            sizes.append(size)

    if not sizes:
        return SUPPORT_DEFAULT_BEAM_SIZE_M

    return sum(sizes) / len(sizes)


def zeroVector():
    return adsk.core.Vector3D.create(0, 0, 0)


def supportPointMetersToFusion(pointMeters, supportScaleFactor, offset=None):
    # NL2: X lateral, Y vertical, Z forward/back. CSV/Fusion path: X, -Z, Y.
    point = adsk.core.Point3D.create(
        pointMeters[0] * supportScaleFactor,
        -pointMeters[2] * supportScaleFactor,
        pointMeters[1] * supportScaleFactor
    )

    if offset:
        point = addVec(point, offset, 1.0)

    return point


def computeSupportFloorZ(supportData, supportScaleFactor, prefabOffsets):
    if not supportData.get('footers'):
        return None

    heightCm = SUPPORT_FOOTER_PAD_HEIGHT_M * supportScaleFactor
    floorZ = None

    for footer in supportData['footers']:
        offset = supportOffsetForPrefab(prefabOffsets, footer['prefabIndex'])
        centerPoint = supportPointMetersToFusion(footer['pointMeters'], supportScaleFactor, offset)
        bottomZ = centerPoint.z - heightCm * 0.5
        floorZ = bottomZ if floorZ is None else min(floorZ, bottomZ)

    supportDebugLog(f'common support floorZ={floorZ:.6f}cm, footerHeightCm={heightCm:.6f}')
    return floorZ


def supportPointForKind(pointMeters, kind, prefabIndex, supportScaleFactor, prefabOffsets, floorZ):
    offset = supportOffsetForPrefab(prefabOffsets, prefabIndex)
    point = supportPointMetersToFusion(pointMeters, supportScaleFactor, offset)

    if kind == 'footernode' and floorZ is not None:
        heightCm = SUPPORT_FOOTER_PAD_HEIGHT_M * supportScaleFactor
        point = adsk.core.Point3D.create(point.x, point.y, floorZ + heightCm * 0.5)

    return point


def horizontalDistanceTo(point, centerPoint):
    return math.sqrt((point.x - centerPoint.x) ** 2 + (point.y - centerPoint.y) ** 2)


def addSupportNode(nodes, nodeId, pointMeters, kind, prefabIndex, sourcePath):
    if nodeId is None or pointMeters is None:
        return False

    nodes[str(nodeId)] = {
        'pointMeters': pointMeters,
        'kind': kind,
        'prefabIndex': prefabIndex,
        'sourcePath': sourcePath
    }
    return True


def readSupportXmlData(xmlPath):
    tree = ET.parse(xmlPath)
    root = tree.getroot()

    prefabs = root.findall('./supports/prefab')
    if not prefabs:
        prefabs = root.findall('.//prefab')

    nodes = {}
    footers = []
    beams = []
    interfaces = []
    skipped = []

    for prefabIndex, prefab in enumerate(prefabs):
        sourcePath = prefab.get('path', '')
        centerRailsCoord = parseSupportFloat(prefab.get('center_rails_coord'), 0.0)
        customTrackIndex = prefab.get('custom_track_index', '')
        supportContainer = prefab.find('./atomization/supports')

        if supportContainer is None:
            skipped.append(f'prefab {prefabIndex}: no atomization/supports')
            continue

        for footer in supportContainer.findall('footernode'):
            nodeId = footer.get('id')
            pointMeters = supportElementPosition(footer)
            if addSupportNode(nodes, nodeId, pointMeters, 'footernode', prefabIndex, sourcePath):
                footers.append({
                    'id': str(nodeId),
                    'pointMeters': pointMeters,
                    'prefabIndex': prefabIndex,
                    'sourcePath': sourcePath
                })
            else:
                skipped.append(f'prefab {prefabIndex}: bad footernode id={nodeId}')

        for rasc in supportContainer.findall('rasc'):
            rascNodes = []
            rascSize = supportBeamSizeMeters(rasc)
            rascType = rasc.get('type', '')
            for subnode in rasc.findall('subnode'):
                nodeId = subnode.get('id')
                pointMeters = supportElementPosition(subnode)
                if addSupportNode(nodes, nodeId, pointMeters, 'subnode', prefabIndex, sourcePath):
                    rascNodes.append(str(nodeId))
                else:
                    skipped.append(f'prefab {prefabIndex}: bad subnode id={nodeId}')

            if CREATE_SUPPORT_RASC_CONNECTORS and len(rascNodes) >= 2:
                for i in range(len(rascNodes) - 1):
                    startId = rascNodes[i]
                    endId = rascNodes[i + 1]
                    beams.append({
                        'startId': startId,
                        'endId': endId,
                        'startMeters': nodes[startId]['pointMeters'],
                        'endMeters': nodes[endId]['pointMeters'],
                        'diameterMeters': rascSize,
                        'kind': 'rasc_connector',
                        'prefabIndex': prefabIndex,
                        'sourcePath': sourcePath,
                        'centerRailsCoord': centerRailsCoord,
                        'customTrackIndex': customTrackIndex,
                        'startKind': nodes[startId]['kind'],
                        'endKind': nodes[endId]['kind']
                    })

            if CREATE_SUPPORT_TRACK_INTERFACES and rascNodes:
                topNodeId = max(
                    rascNodes,
                    key=lambda item: nodes[item]['pointMeters'][1]
                )
                interfaces.append({
                    'nodeId': topNodeId,
                    'pointMeters': nodes[topNodeId]['pointMeters'],
                    'diameterMeters': rascSize,
                    'kind': 'track_interface',
                    'prefabIndex': prefabIndex,
                    'sourcePath': sourcePath,
                    'centerRailsCoord': centerRailsCoord,
                    'customTrackIndex': customTrackIndex,
                    'rascType': rascType
                })

        for beam in supportContainer.findall('beam'):
            startId = str(beam.get('start'))
            endId = str(beam.get('end'))

            if startId not in nodes or endId not in nodes:
                skipped.append(
                    f'prefab {prefabIndex}: beam skipped; missing node '
                    f'start={startId} end={endId}'
                )
                continue

            startMeters = nodes[startId]['pointMeters']
            endMeters = nodes[endId]['pointMeters']
            diameterMeters = supportBeamSizeMeters(beam)

            for beamNode in beam.findall('beamnode'):
                beamNodeId = beamNode.get('id')
                posFraction = parseSupportFloat(beamNode.get('pos'), None)
                if posFraction is None:
                    skipped.append(f'prefab {prefabIndex}: bad beamnode pos id={beamNodeId}')
                    continue

                beamNodePoint = interpolateTuple(startMeters, endMeters, posFraction)
                if not addSupportNode(nodes, beamNodeId, beamNodePoint, 'beamnode', prefabIndex, sourcePath):
                    skipped.append(f'prefab {prefabIndex}: bad beamnode id={beamNodeId}')

            beams.append({
                'startId': startId,
                'endId': endId,
                'startMeters': startMeters,
                'endMeters': endMeters,
                'diameterMeters': diameterMeters,
                'kind': 'beam',
                'prefabIndex': prefabIndex,
                'sourcePath': sourcePath,
                'centerRailsCoord': centerRailsCoord,
                'customTrackIndex': customTrackIndex,
                'startKind': nodes[startId]['kind'],
                'endKind': nodes[endId]['kind']
            })

    return {
        'prefabCount': len(prefabs),
        'nodes': nodes,
        'footers': footers,
        'beams': beams,
        'interfaces': interfaces,
        'skipped': skipped
    }


def footerPadExitPoint(outerPoint, footerCenter, radiusCm, heightCm):
    eps = 0.000001
    topZ = footerCenter.z + heightCm * 0.5
    bottomZ = footerCenter.z - heightCm * 0.5
    direction = vectorBetween(outerPoint, footerCenter)
    candidates = []

    if outerPoint.z > topZ + eps and direction.z < -eps:
        topT = (topZ - outerPoint.z) / direction.z
        if 0.0 <= topT <= 1.0:
            topPoint = addVec(outerPoint, direction, topT)
            if horizontalDistanceTo(topPoint, footerCenter) <= radiusCm + eps:
                candidates.append((topT, topPoint, 'top'))

    if outerPoint.z < bottomZ - eps and direction.z > eps:
        bottomT = (bottomZ - outerPoint.z) / direction.z
        if 0.0 <= bottomT <= 1.0:
            bottomPoint = addVec(outerPoint, direction, bottomT)
            if horizontalDistanceTo(bottomPoint, footerCenter) <= radiusCm + eps:
                candidates.append((bottomT, bottomPoint, 'bottom'))

    dx = outerPoint.x - footerCenter.x
    dy = outerPoint.y - footerCenter.y
    vx = direction.x
    vy = direction.y
    a = vx * vx + vy * vy
    b = 2.0 * (dx * vx + dy * vy)
    c = dx * dx + dy * dy - radiusCm * radiusCm

    if a > 0.000001:
        discriminant = b * b - 4.0 * a * c
        if discriminant >= 0.0:
            root = math.sqrt(discriminant)
            for t in ((-b - root) / (2.0 * a), (-b + root) / (2.0 * a)):
                if 0.0 <= t <= 1.0:
                    point = addVec(outerPoint, direction, t)
                    if bottomZ - eps <= point.z <= topZ + eps:
                        candidates.append((t, point, 'side'))

    if candidates:
        candidates.sort(key=lambda item: item[0])
        return candidates[0][1], candidates[0][2], candidates

    def clampedCapPoint(zValue):
        dx = outerPoint.x - footerCenter.x
        dy = outerPoint.y - footerCenter.y
        distance = math.sqrt(dx * dx + dy * dy)
        if distance > radiusCm and distance > eps:
            scale = radiusCm / distance
            return adsk.core.Point3D.create(
                footerCenter.x + dx * scale,
                footerCenter.y + dy * scale,
                zValue
            )
        return adsk.core.Point3D.create(outerPoint.x, outerPoint.y, zValue)

    if outerPoint.z >= topZ:
        return clampedCapPoint(topZ), 'fallback_top', candidates

    if outerPoint.z <= bottomZ:
        return clampedCapPoint(bottomZ), 'fallback_bottom', candidates

    distance = math.sqrt(dx * dx + dy * dy)
    if distance > eps:
        scale = radiusCm / distance
        sideZ = clampDistanceCm(outerPoint.z, bottomZ, topZ)
        return adsk.core.Point3D.create(
            footerCenter.x + dx * scale,
            footerCenter.y + dy * scale,
            sideZ
        ), 'fallback_side', candidates

    return adsk.core.Point3D.create(footerCenter.x + radiusCm, footerCenter.y, outerPoint.z), 'fallback_side', candidates


def clampPointInsideFooterCylinder(point, footerCenter, radiusCm, heightCm):
    eps = 0.000001
    topZ = footerCenter.z + heightCm * 0.5
    bottomZ = footerCenter.z - heightCm * 0.5
    zValue = clampDistanceCm(point.z, bottomZ, topZ)
    dx = point.x - footerCenter.x
    dy = point.y - footerCenter.y
    distance = math.sqrt(dx * dx + dy * dy)

    if distance > radiusCm and distance > eps:
        scale = radiusCm / distance
        return adsk.core.Point3D.create(
            footerCenter.x + dx * scale,
            footerCenter.y + dy * scale,
            zValue
        )

    return adsk.core.Point3D.create(point.x, point.y, zValue)


def embeddedFooterExitPoint(surfacePoint, outerPoint, footerCenter, radiusCm, heightCm):
    ray = vectorBetween(outerPoint, footerCenter)
    if not safeNormalize(ray):
        return surfacePoint

    embeddedPoint = addVec(surfacePoint, ray, SUPPORT_FOOTER_BEAM_EMBED_CM)
    return clampPointInsideFooterCylinder(embeddedPoint, footerCenter, radiusCm, heightCm)


def formatFooterCandidates(candidates):
    pieces = []
    for tValue, point, surface in candidates[:6]:
        pieces.append(f'{surface}@{tValue:.6f}:{formatPoint(point)}')
    return '[' + '; '.join(pieces) + ']'


def clampPointToFloor(point, floorZ):
    if floorZ is None or point.z >= floorZ:
        return point

    return adsk.core.Point3D.create(point.x, point.y, floorZ)


def clipSupportBeamToFooterExit(beam, startPoint, endPoint, supportScaleFactor, floorZ=None):
    if not CLIP_SUPPORT_BEAMS_TO_FOOTER_EXIT:
        return startPoint, endPoint, []

    radiusCm = SUPPORT_FOOTER_PAD_DIAMETER_M * supportScaleFactor * 0.5
    heightCm = SUPPORT_FOOTER_PAD_HEIGHT_M * supportScaleFactor

    if beam.get('startKind') == 'footernode':
        footerCenter = startPoint
        rawStart, surface, candidates = footerPadExitPoint(endPoint, footerCenter, radiusCm, heightCm)
        clippedStart = embeddedFooterExitPoint(rawStart, endPoint, footerCenter, radiusCm, heightCm)
        clippedStart = clampPointToFloor(clippedStart, floorZ)
        topZ = footerCenter.z + heightCm * 0.5
        bottomZ = footerCenter.z - heightCm * 0.5
        supportDebugLog(
            f'footer exit start beam={beam["startId"]}->{beam["endId"]}: '
            f'surface={surface}, center={formatPoint(footerCenter)}, outer={formatPoint(endPoint)}, '
            f'candidates={formatFooterCandidates(candidates)}, rawExit={formatPoint(rawStart)}, '
            f'embeddedExit={formatPoint(clippedStart)}, floorZ={floorZ}, '
            f'topZ={topZ:.6f}, bottomZ={bottomZ:.6f}, radius={radiusCm:.6f}'
        )
        startPoint = clippedStart

    if beam.get('endKind') == 'footernode':
        footerCenter = endPoint
        rawEnd, surface, candidates = footerPadExitPoint(startPoint, footerCenter, radiusCm, heightCm)
        clippedEnd = embeddedFooterExitPoint(rawEnd, startPoint, footerCenter, radiusCm, heightCm)
        clippedEnd = clampPointToFloor(clippedEnd, floorZ)
        topZ = footerCenter.z + heightCm * 0.5
        bottomZ = footerCenter.z - heightCm * 0.5
        supportDebugLog(
            f'footer exit end beam={beam["startId"]}->{beam["endId"]}: '
            f'surface={surface}, center={formatPoint(footerCenter)}, outer={formatPoint(startPoint)}, '
            f'candidates={formatFooterCandidates(candidates)}, rawExit={formatPoint(rawEnd)}, '
            f'embeddedExit={formatPoint(clippedEnd)}, floorZ={floorZ}, '
            f'topZ={topZ:.6f}, bottomZ={bottomZ:.6f}, radius={radiusCm:.6f}'
        )
        endPoint = clippedEnd

    return startPoint, endPoint, []


def createSupportBeamBody(component, startPoint, endPoint, diameterCm, name):
    distance = startPoint.distanceTo(endPoint)
    if distance < 0.000001 or diameterCm <= 0:
        debugLog(f'createSupportBeamBody {name}: skipped')
        return False

    tangent = vectorBetween(startPoint, endPoint)
    if not safeNormalize(tangent):
        debugLog(f'createSupportBeamBody {name}: bad tangent')
        return False

    try:
        centerPoint = midpoint(startPoint, endPoint)
        plane = createPlaneNormalToVector(
            component,
            centerPoint,
            tangent,
            f'{name}_plane'
        )

        if not plane:
            debugLog(f'createSupportBeamBody {name}: could not create plane')
            return False

        sketch, profile = createCircleProfileOnPlane(
            component,
            plane,
            centerPoint,
            diameterCm,
            f'{name}_profile'
        )

        if not profile:
            debugLog(f'createSupportBeamBody {name}: profile failed')
            return False

        extrude = extrudeProfile(
            component,
            profile,
            distance,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
            name
        )
        return extrude is not None
    except Exception as err:
        debugLog(f'createSupportBeamBody {name}: failed: {err}\n{traceback.format_exc()}')
        return False


def closestPointTo(target, points):
    bestPoint = points[0]
    bestDistance = target.distanceTo(bestPoint)

    for point in points[1:]:
        distance = target.distanceTo(point)
        if distance < bestDistance:
            bestDistance = distance
            bestPoint = point

    return bestPoint


def supportInterfaceStartPoint(interface, supportTop, trackAnchor, frame, sizeCm):
    supportKind = supportInterfaceKind(interface)

    if frame and supportKind == '90':
        sideOffset = sizeCm * 0.5
        return closestPointTo(trackAnchor, [
            addVec(supportTop, frame.xAxis, sideOffset),
            addVec(supportTop, frame.xAxis, -sideOffset)
        ])

    if frame and supportKind == '180':
        return addVec(supportTop, frame.yAxis, -sizeCm * 0.5)

    return supportTop


def createSupportInterfaceLoft(component, startPoint, endPoint, frame, startSizeCm, endSizeCm, name):
    if startPoint.distanceTo(endPoint) < 0.000001:
        return False

    if frame:
        xAxis = frame.xAxis
        yAxis = frame.yAxis
    else:
        tangent = vectorBetween(startPoint, endPoint)
        if not safeNormalize(tangent):
            return False
        xAxis = perpendicularUnitVector(tangent)
        if not xAxis:
            return False
        yAxis = tangent.crossProduct(xAxis)
        if not safeNormalize(yAxis):
            return False

    try:
        startPlane = createPlaneByThreePoints(
            component,
            startPoint,
            addVec(startPoint, xAxis, 1.0),
            addVec(startPoint, yAxis, 1.0),
            f'{name}_start_plane'
        )
        endPlane = createPlaneByThreePoints(
            component,
            endPoint,
            addVec(endPoint, xAxis, 1.0),
            addVec(endPoint, yAxis, 1.0),
            f'{name}_end_plane'
        )
        if not startPlane or not endPlane:
            return False

        startSketch, startProfile = createCenteredRectangleProfileOnPlane(
            component,
            startPlane,
            startPoint,
            xAxis,
            yAxis,
            startSizeCm,
            startSizeCm,
            f'{name}_start_profile'
        )
        endSketch, endProfile = createCenteredRectangleProfileOnPlane(
            component,
            endPlane,
            endPoint,
            xAxis,
            yAxis,
            endSizeCm,
            endSizeCm,
            f'{name}_end_profile'
        )

        if not startProfile or not endProfile:
            return False

        return createLoftFromProfiles(component, [startProfile, endProfile], name) is not None
    except Exception as err:
        supportDebugLog(f'createSupportInterfaceLoft {name}: failed: {err}')
        return False


def createSupportTrackConnector(component, placement, name):
    trackAnchor = placement['trackAnchor']
    circleCenter = placement['circleCenter']
    neckEnd = placement['neckEnd']
    beamEnd = placement['beamEnd']
    padAcross = placement['padAcross']
    padAlongTrack = placement['padAlongTrack']
    padNormal = placement['padNormal']
    beamAxis = placement['beamAxis']
    beamDiameterCm = placement['beamDiameterCm']
    circleDiameterCm = placement['circleDiameterCm']
    collarDiameterCm = placement['collarDiameterCm']
    squareSizeCm = placement['squareSizeCm']
    collarStart = placement.get('collarStart', addVec(circleCenter, padNormal, -SUPPORT_CONNECTOR_OVERLAP_CM))
    collarEnd = placement.get('collarEnd', addVec(neckEnd, padNormal, SUPPORT_CONNECTOR_OVERLAP_CM))
    angledStart = placement.get('angledStart', addVec(neckEnd, padNormal, -SUPPORT_CONNECTOR_OVERLAP_CM))
    angledEnd = placement.get('angledEnd', addVec(beamEnd, beamAxis, SUPPORT_CONNECTOR_OVERLAP_CM))

    try:
        squarePlane = createPlaneByThreePoints(
            component,
            trackAnchor,
            addVec(trackAnchor, padAcross, 1.0),
            addVec(trackAnchor, padAlongTrack, 1.0),
            f'{name}_square_plane'
        )
        circlePlane = createPlaneByThreePoints(
            component,
            circleCenter,
            addVec(circleCenter, padAcross, 1.0),
            addVec(circleCenter, padAlongTrack, 1.0),
            f'{name}_circle_plane'
        )
        if not squarePlane or not circlePlane:
            return False

        squareSketch, squareProfile = createCenteredRectangleProfileOnPlane(
            component,
            squarePlane,
            trackAnchor,
            padAcross,
            padAlongTrack,
            squareSizeCm,
            squareSizeCm,
            f'{name}_square_profile'
        )
        circleSketch, circleProfile = createCircleProfileOnPlane(
            component,
            circlePlane,
            circleCenter,
            circleDiameterCm,
            f'{name}_circle_profile'
        )
        if not squareProfile or not circleProfile:
            return False

        padLoft = createLoftFromProfiles(
            component,
            [squareProfile, circleProfile],
            f'{name}_pad_loft'
        )
        if not padLoft:
            return False

        collarOk = createSupportBeamBody(
            component,
            collarStart,
            collarEnd,
            collarDiameterCm,
            f'{name}_collar'
        )
        if not collarOk:
            return False

        if placement.get('supportKind') in ('90', '180'):
            supportDebugLog(
                f'{name}: preview connector created without final beam loft; '
                f'kind={placement.get("supportKind")}'
            )
            return True

        if angledStart.distanceTo(angledEnd) < 0.000001:
            return True

        angledStartPlane = createPlaneByThreePoints(
            component,
            angledStart,
            addVec(angledStart, padAcross, 1.0),
            addVec(angledStart, padAlongTrack, 1.0),
            f'{name}_angled_start_plane'
        )
        angledEndPlane = createPlaneNormalToVector(
            component,
            angledEnd,
            beamAxis,
            f'{name}_angled_end_plane'
        )
        if not angledStartPlane or not angledEndPlane:
            supportDebugLog(f'{name}: angled planes failed; using circular beam fallback')
            return createSupportBeamBody(
                component,
                angledStart,
                angledEnd,
                max(collarDiameterCm, beamDiameterCm),
                f'{name}_angled_fallback'
            )

        startSketch, startProfile = createCircleProfileOnPlane(
            component,
            angledStartPlane,
            angledStart,
            collarDiameterCm,
            f'{name}_angled_start_profile'
        )
        endSketch, endProfile = createCircleProfileOnPlane(
            component,
            angledEndPlane,
            angledEnd,
            beamDiameterCm,
            f'{name}_angled_end_profile'
        )
        if not startProfile or not endProfile:
            supportDebugLog(f'{name}: angled profiles failed; using circular beam fallback')
            return createSupportBeamBody(
                component,
                angledStart,
                angledEnd,
                max(collarDiameterCm, beamDiameterCm),
                f'{name}_angled_fallback'
            )

        angledLoft = createLoftFromProfiles(
            component,
            [startProfile, endProfile],
            f'{name}_angled_loft'
        )
        if angledLoft:
            return True

        supportDebugLog(f'{name}: angled loft failed; using circular beam fallback')
        fallbackOk = createSupportBeamBody(
            component,
            angledStart,
            angledEnd,
            max(collarDiameterCm, beamDiameterCm),
            f'{name}_angled_fallback'
        )
        if fallbackOk:
            supportDebugLog(
                f'{name}: fallback cylinder created, start={formatPoint(angledStart)}, '
                f'end={formatPoint(angledEnd)}, diameter={max(collarDiameterCm, beamDiameterCm):.6f}'
            )
        return fallbackOk
    except Exception as err:
        supportDebugLog(f'createSupportTrackConnector {name}: failed: {err}\n{traceback.format_exc()}')
        return False


def createSupportFooterPad(component, footer, supportScaleFactor, prefabOffsets, floorZ, sequenceNumber):
    centerPoint = supportPointForKind(
        footer['pointMeters'],
        'footernode',
        footer['prefabIndex'],
        supportScaleFactor,
        prefabOffsets,
        floorZ
    )
    heightCm = SUPPORT_FOOTER_PAD_HEIGHT_M * supportScaleFactor
    diameterCm = SUPPORT_FOOTER_PAD_DIAMETER_M * supportScaleFactor
    startPoint = adsk.core.Point3D.create(centerPoint.x, centerPoint.y, centerPoint.z - heightCm * 0.5)
    endPoint = adsk.core.Point3D.create(centerPoint.x, centerPoint.y, centerPoint.z + heightCm * 0.5)

    return createSupportBeamBody(
        component,
        startPoint,
        endPoint,
        diameterCm,
        f'support_footer_{sequenceNumber:03d}'
    )


def createSupportsFromXml(component, xmlPath, pathScaleFactor, supportTrackSamples=None):
    if not os.path.exists(xmlPath):
        return {
            'ok': False,
            'error': f'XML file does not exist: {xmlPath}',
            'beamCount': 0,
            'beamTotal': 0,
            'footerCount': 0,
            'footerTotal': 0,
            'prefabCount': 0
        }

    supportScaleFactor = pathScaleFactor * SUPPORT_XML_METERS_TO_CSV_UNITS
    debugLog(
        f'createSupportsFromXml: xmlPath={xmlPath}, '
        f'supportScaleFactor={supportScaleFactor:.9f}cm_per_meter'
    )

    try:
        supportData = readSupportXmlData(xmlPath)
    except Exception as err:
        debugLog(f'createSupportsFromXml: XML parse failed: {err}\n{traceback.format_exc()}')
        return {
            'ok': False,
            'error': f'XML parse failed: {err}',
            'beamCount': 0,
            'beamTotal': 0,
            'footerCount': 0,
            'footerTotal': 0,
            'prefabCount': 0
        }

    for skippedMessage in supportData['skipped'][:20]:
        debugLog(f'createSupportsFromXml skipped: {skippedMessage}')

    debugLog(
        'createSupportsFromXml parsed: '
        f'prefabs={supportData["prefabCount"]}, nodes={len(supportData["nodes"])}, '
        f'beams={len(supportData["beams"])}, interfaces={len(supportData["interfaces"])}, '
        f'footers={len(supportData["footers"])}, '
        f'skipped={len(supportData["skipped"])}'
    )
    supportDebugLog(
        'parsed XML: '
        f'prefabs={supportData["prefabCount"]}, nodes={len(supportData["nodes"])}, '
        f'beams={len(supportData["beams"])}, interfaces={len(supportData["interfaces"])}, '
        f'footers={len(supportData["footers"])}, skipped={len(supportData["skipped"])}'
    )
    beamsByNode = buildSupportBeamAdjacency(supportData)
    interfacePlans = []
    if CREATE_SUPPORT_TRACK_INTERFACES and supportTrackSamples:
        interfacePlans = planSupportInterfaces(
            supportData,
            supportTrackSamples,
            supportScaleFactor,
            beamsByNode
        )

    prefabOffsets = computeSupportPrefabOffsetsFromPlans(
        supportData,
        interfacePlans,
        supportScaleFactor
    )
    floorZ = computeSupportFloorZ(supportData, supportScaleFactor, prefabOffsets)
    interfacePlacements, endpointOverrides = buildSupportInterfacePlacements(
        supportData,
        interfacePlans,
        prefabOffsets,
        supportScaleFactor,
        floorZ,
        beamsByNode
    )

    supportKindsByPrefab = {
        plan['interface']['prefabIndex']: plan['supportKind']
        for plan in interfacePlans
    }
    prefabComponents = {}
    for prefabIndex in range(supportData['prefabCount']):
        supportKind = supportKindsByPrefab.get(prefabIndex, 'support')
        componentName = f'support_piece_{prefabIndex + 1:03d}_{supportKind}'
        prefabOccurrence = component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        prefabComponent = adsk.fusion.Component.cast(prefabOccurrence.component)
        prefabComponent.name = componentName
        prefabComponents[prefabIndex] = prefabComponent

    beamCount = 0
    for sequenceNumber, beam in enumerate(supportData['beams'], start=1):
        startPoint = supportPointForKind(
            beam['startMeters'],
            beam.get('startKind'),
            beam['prefabIndex'],
            supportScaleFactor,
            prefabOffsets,
            floorZ
        )
        endPoint = supportPointForKind(
            beam['endMeters'],
            beam.get('endKind'),
            beam['prefabIndex'],
            supportScaleFactor,
            prefabOffsets,
            floorZ
        )
        diameterCm = beam['diameterMeters'] * supportScaleFactor * SUPPORT_BEAM_DIAMETER_SCALE
        name = f'support_{sequenceNumber:03d}_{beam["kind"]}'

        startOverride = endpointOverrides.get((beam['prefabIndex'], beam['startId']))
        if startOverride:
            startPoint = startOverride

        endOverride = endpointOverrides.get((beam['prefabIndex'], beam['endId']))
        if endOverride:
            endPoint = endOverride

        if DEBUG_SWEEP_LOGGING:
            debugLog(
                f'createSupportsFromXml beam {sequenceNumber}: '
                f'kind={beam["kind"]}, startId={beam["startId"]}, '
                f'endId={beam["endId"]}, diameterCm={diameterCm:.6f}'
            )

        startPoint, endPoint, stubs = clipSupportBeamToFooterExit(
            beam,
            startPoint,
            endPoint,
            supportScaleFactor,
            floorZ
        )

        supportPieceComponent = prefabComponents.get(beam['prefabIndex'], component)

        if createSupportBeamBody(supportPieceComponent, startPoint, endPoint, diameterCm, name):
            beamCount += 1

    interfaceCount = 0
    if CREATE_SUPPORT_TRACK_INTERFACES and supportTrackSamples:
        for placement in interfacePlacements:
            sequenceNumber = placement['sequenceNumber']
            interface = placement['interface']
            trackSample = placement['trackSample']
            name = f'support_interface_{sequenceNumber:03d}'

            nearestLogCm = trackSample.get(
                'nearestCrosstieCm',
                trackSample.get('nearestCrosstieDistanceCm', -1.0)
            )
            if nearestLogCm is None:
                nearestLogCm = -1.0

            supportDebugLog(
                f'interface {sequenceNumber}: prefab={interface["prefabIndex"]}, '
                f'node={interface["nodeId"]}, centerRails={interface["centerRailsCoord"]:.6f}m, '
                f'xmlKind={placement["xmlSupportKind"]}, kind={placement["supportKind"]}, '
                f'rollDeg={(placement["rollDegrees"] if placement["rollDegrees"] is not None else -1.0):.6f}, '
                f'targetCm={trackSample["targetCm"]:.6f}, '
                f'originalTargetCm={trackSample.get("originalTargetCm", trackSample["targetCm"]):.6f}, '
                f'snapStatus={trackSample.get("snapStatus", "unknown")}, '
                f'snapDeltaCm={trackSample.get("snapDeltaCm", 0.0):.6f}, '
                f'nearestCrosstieCm={nearestLogCm:.6f}, '
                f'track={trackSample["folderName"]}[{trackSample["index"]}] '
                f't={trackSample["t"]:.6f}, trackAnchor={formatPoint(placement["trackAnchor"])}, '
                f'circleCenter={formatPoint(placement["circleCenter"])}, '
                f'collarStart={formatPoint(placement["collarStart"])}, '
                f'neckEnd={formatPoint(placement["neckEnd"])}, '
                f'angledStart={formatPoint(placement["angledStart"])}, '
                f'beamEnd={formatPoint(placement["beamEnd"])}, '
                f'angledEnd={formatPoint(placement["angledEnd"])}, '
                f'beamAxisSource={placement["beamAxisSource"]}, '
                f'beamAxis={formatVector(placement["beamAxis"])}, '
                f'beamDiameter={placement["beamDiameterCm"]:.6f}cm, '
                f'circleDiameter={placement["circleDiameterCm"]:.6f}cm, '
                f'collarDiameter={placement["collarDiameterCm"]:.6f}cm, '
                f'square={placement["squareSizeCm"]:.6f}cm, source={interface["sourcePath"]}'
            )

            if createSupportTrackConnector(
                prefabComponents.get(interface['prefabIndex'], component),
                placement,
                name
            ):
                interfaceCount += 1

    footerCount = 0
    if CREATE_SUPPORT_FOOTER_PADS:
        for sequenceNumber, footer in enumerate(supportData['footers'], start=1):
            if createSupportFooterPad(
                prefabComponents.get(footer['prefabIndex'], component),
                footer,
                supportScaleFactor,
                prefabOffsets,
                floorZ,
                sequenceNumber
            ):
                footerCount += 1

    expectedInterfaceCount = len(interfacePlacements) if (CREATE_SUPPORT_TRACK_INTERFACES and supportTrackSamples) else 0
    ok = beamCount == len(supportData['beams']) and interfaceCount == expectedInterfaceCount
    error = None if ok else (
        f'created {beamCount} of {len(supportData["beams"])} support beams, '
        f'{interfaceCount} of {expectedInterfaceCount} interfaces'
    )

    return {
        'ok': ok,
        'error': error,
        'beamCount': beamCount,
        'beamTotal': len(supportData['beams']),
        'interfaceCount': interfaceCount,
        'interfaceTotal': expectedInterfaceCount,
        'footerCount': footerCount,
        'footerTotal': len(supportData['footers']) if CREATE_SUPPORT_FOOTER_PADS else 0,
        'prefabCount': supportData['prefabCount'],
        'skippedCount': len(supportData['skipped'])
    }


# ============================================================
# MAIN
# ============================================================

def run(context):
    ui = None

    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        resetDebugLog()
        resetSupportDebugLog()
        debugLog('run: started')
        debugLog(
            'settings: '
            f'CREATE_RAIL_SPLINE_SKETCHES={CREATE_RAIL_SPLINE_SKETCHES}, '
            f'CREATE_CROSSTIES={CREATE_CROSSTIES}, '
            f'CREATE_TRACK_SPINE_SWEEPS={CREATE_TRACK_SPINE_SWEEPS}, '
            f'CREATE_SUPPORTS_FROM_XML={CREATE_SUPPORTS_FROM_XML}, '
            f'END_BOX_SWEEP_FRACTION={END_BOX_SWEEP_FRACTION}, '
            f'SPINE_RECT_WIDTH_MM={SPINE_RECT_WIDTH_MM:.6f}, '
            f'SPINE_RECT_HEIGHT_MM={SPINE_RECT_HEIGHT_MM:.6f}'
        )
        design = adsk.fusion.Design.cast(app.activeProduct)

        if not design:
            debugLog('run: no active Fusion design')
            ui.messageBox('Please open a Fusion design first.')
            return

        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        rootComponent = design.rootComponent

        if PROMPT_FOR_INPUT_PATHS:
            userSelectedFolder, baseFolderPath = promptForBaseFolder(ui)
            if not userSelectedFolder:
                debugLog('run: user cancelled folder selection')
                ui.messageBox('Cancelled.')
                return
        else:
            baseFolderPath = DEFAULT_TRACK_BASE_FOLDER
            userSelectedFolder = os.path.isdir(baseFolderPath)
            debugLog(f'run: using default base folder={baseFolderPath}')

        if not userSelectedFolder:
            debugLog(f'run: base folder unavailable: {baseFolderPath}')
            ui.messageBox(
                f'Track folder not found:\n{baseFolderPath}\n\n'
                'Update DEFAULT_TRACK_BASE_FOLDER or set PROMPT_FOR_INPUT_PATHS = True.'
            )
            return

        debugLog(f'run: selected base folder={baseFolderPath}')

        trackFolders = sorted([
            name for name in os.listdir(baseFolderPath)
            if os.path.isdir(os.path.join(baseFolderPath, name))
            and name.lower().startswith('track')
        ], key=trackFolderSortKey)

        debugLog(f'run: trackFolders={trackFolders}')

        if not trackFolders:
            debugLog('run: no track folders found')
            ui.messageBox('No track folders found.')
            return

        allTrackData = {}

        for folderName in trackFolders:
            folderPath = os.path.join(baseFolderPath, folderName)
            allTrackData[folderName] = {}

            for railName in RAIL_NAMES:
                csvPath = os.path.join(folderPath, f'{railName}.csv')
                if os.path.exists(csvPath):
                    allTrackData[folderName][railName] = readPointsFromCSV(csvPath)
                    debugLog(
                        f'run: loaded {folderName}/{railName}.csv '
                        f'points={len(allTrackData[folderName][railName])}, path={csvPath}'
                    )
                else:
                    debugLog(f'run: missing {folderName}/{railName}.csv path={csvPath}')

        try:
            firstTrack = allTrackData[trackFolders[0]]
            leftFirst = adsk.core.Point3D.create(*firstTrack['left'][0])
            rightFirst = adsk.core.Point3D.create(*firstTrack['right'][0])
            currentGauge = leftFirst.distanceTo(rightFirst)
            debugLog(
                f'run: rawLeftRightDistance={currentGauge:.6f}, '
                f'leftFirst={formatPoint(leftFirst)}, rightFirst={formatPoint(rightFirst)}'
            )
        except Exception as err:
            debugLog(f'run: could not measure gauge: {err}\n{traceback.format_exc()}')
            ui.messageBox(
                'Could not measure gauge.\n'
                'Check that left.csv and right.csv exist in the first track folder.'
            )
            return

        gaugeInputValue, userCancelled = ui.inputBox(
            f'Raw left/right distance: {currentGauge:.4f}\n'
            f'Default target gauge: {DEFAULT_GAUGE_CM:.4f} cm / 26 mm\n\n'
            f'Enter desired gauge in cm:',
            'Track Gauge Scaler',
            f'{DEFAULT_GAUGE_CM:.4f}'
        )

        if userCancelled:
            debugLog('run: user cancelled gauge input')
            ui.messageBox('Cancelled.')
            return

        try:
            desiredGauge = float(gaugeInputValue)
        except ValueError:
            debugLog(f'run: invalid gauge input={gaugeInputValue}')
            ui.messageBox('Invalid input — enter a number.')
            return

        if currentGauge <= 0:
            debugLog(f'run: current gauge <= 0: {currentGauge}')
            ui.messageBox('Current gauge measured as zero. Check CSV points.')
            return

        gaugeScaleFactor = desiredGauge / currentGauge
        pathScaleFactor = TRACK_PATH_SCALE_CM_PER_UNIT
        debugLog(
            f'run: desiredGauge={desiredGauge:.6f}cm, '
            f'gaugeScaleFactor={gaugeScaleFactor:.9f}, '
            f'pathScaleFactor={pathScaleFactor:.9f}, '
            f'SCALE_RAIL_OFFSETS_FROM_CENTER={SCALE_RAIL_OFFSETS_FROM_CENTER}'
        )

        supportXmlSelected = False
        supportXmlPath = None
        if CREATE_SUPPORTS_FROM_XML:
            if PROMPT_FOR_INPUT_PATHS:
                supportXmlSelected, supportXmlPath = promptForSupportXmlFile(ui)
            else:
                supportXmlPath = DEFAULT_SUPPORT_XML_PATH
                supportXmlSelected = os.path.isfile(supportXmlPath)
                debugLog(f'run: using default supports XML={supportXmlPath}')

            if supportXmlSelected and supportXmlPath:
                debugLog(f'run: selected supports XML={supportXmlPath}')
            else:
                debugLog(f'run: supports XML unavailable: {supportXmlPath}')
                if not PROMPT_FOR_INPUT_PATHS:
                    ui.messageBox(
                        f'Supports XML not found:\n{supportXmlPath}\n\n'
                        'Update DEFAULT_SUPPORT_XML_PATH or set PROMPT_FOR_INPUT_PATHS = True.'
                    )
                    return

        identity = adsk.core.Matrix3D.create()
        topOcc = rootComponent.occurrences.addNewComponent(identity)
        topComp = adsk.fusion.Component.cast(topOcc.component)
        topComp.name = 'RollerCoaster_Tracks'

        summaryLines = []
        scaledTrackDataList = []

        for folderName in trackFolders:
            debugLog('=' * 72)
            debugLog(f'run: processing track folder={folderName}')
            trackOcc = topComp.occurrences.addNewComponent(identity)
            trackComp = adsk.fusion.Component.cast(trackOcc.component)
            trackComp.name = folderName

            imported = []
            missing = []

            scaledRailData = scaleTrackRailData(
                allTrackData[folderName],
                pathScaleFactor,
                gaugeScaleFactor
            )
            scaledTrackDataList.append({
                'folderName': folderName,
                'scaledRailData': scaledRailData
            })

            for railName in RAIL_NAMES:
                rawPoints = allTrackData[folderName].get(railName)

                if not rawPoints or len(rawPoints) < 2:
                    debugLog(
                        f'run: {folderName}/{railName}: missing or not enough points; '
                        f'rawCount={len(rawPoints) if rawPoints else 0}'
                    )
                    missing.append(railName)
                    continue

                scaledPoints = scaledRailData.get(railName, [])
                if DEBUG_SWEEP_LOGGING:
                    debugLog(
                        f'run: {folderName}/{railName}: rawCount={len(rawPoints)}, '
                        f'scaledStats={pathPointStats(scaledPoints)}'
                    )

                if CREATE_RAIL_SPLINE_SKETCHES:
                    createSplineSketch(trackComp, scaledPoints, railName)
                imported.append(railName)

            spineSweepText = ''
            if CREATE_TRACK_SPINE_SWEEPS:
                scaledLeft = scaledRailData.get('left', [])
                scaledRight = scaledRailData.get('right', [])
                scaledBottom = scaledRailData.get('bottom', [])

                if scaledLeft and scaledRight and scaledBottom:
                    spineOk = createTrackSpineSweep(
                        trackComp,
                        scaledLeft,
                        scaledRight,
                        scaledBottom
                    )
                    debugLog(
                        f'run: {folderName}: createTrackSpineSweep result={spineOk}, '
                        f'LAST_SWEEP_ERROR={LAST_SWEEP_ERROR}'
                    )
                    if spineOk:
                        spineSweepText = ' | track spine loft: ok'
                    elif LAST_SWEEP_ERROR:
                        spineSweepText = f' | track spine loft failed: {LAST_SWEEP_ERROR}'
                    else:
                        spineSweepText = ' | track spine loft: failed'
                else:
                    spineSweepText = ' | track spine loft skipped'

            crosstieText = ''
            if CREATE_CROSSTIES:
                scaledLeft = scaledRailData.get('left', [])
                scaledRight = scaledRailData.get('right', [])
                scaledBottom = scaledRailData.get('bottom', [])

                if scaledLeft and scaledRight and scaledBottom:
                    count, err = createCrossties(
                        trackComp,
                        scaledLeft,
                        scaledRight,
                        scaledBottom
                    )

                    if err:
                        crosstieText = f' | crossties skipped: {err}'
                    else:
                        crosstieText = f' | crossties: {count}'
                else:
                    crosstieText = ' | crossties skipped: missing left/right/bottom'

            line = f'{folderName}: imported {", ".join(imported)}'

            if missing:
                line += f' | missing: {", ".join(missing)}'

            line += spineSweepText
            line += crosstieText

            summaryLines.append(line)
            debugLog(f'run: summary line: {line}')

        if CREATE_SUPPORTS_FROM_XML:
            if supportXmlSelected and supportXmlPath:
                supportOcc = topComp.occurrences.addNewComponent(identity)
                supportComp = adsk.fusion.Component.cast(supportOcc.component)
                supportComp.name = 'NL2_Supports'
                supportTrackSamples = buildSupportTrackSamples(scaledTrackDataList)

                supportResult = createSupportsFromXml(
                    supportComp,
                    supportXmlPath,
                    pathScaleFactor,
                    supportTrackSamples
                )

                if supportResult['ok']:
                    supportLine = (
                        f'Supports: {supportResult["beamCount"]}/{supportResult["beamTotal"]} beams, '
                        f'{supportResult["interfaceCount"]}/{supportResult["interfaceTotal"]} interfaces, '
                        f'{supportResult["footerCount"]}/{supportResult["footerTotal"]} footers, '
                        f'{supportResult["prefabCount"]} prefabs'
                    )
                else:
                    supportLine = (
                        f'Supports failed: {supportResult["error"]} | '
                        f'{supportResult["beamCount"]}/{supportResult["beamTotal"]} beams, '
                        f'{supportResult["interfaceCount"]}/{supportResult["interfaceTotal"]} interfaces, '
                        f'{supportResult["footerCount"]}/{supportResult["footerTotal"]} footers'
                    )

                if supportResult.get('skippedCount', 0):
                    supportLine += f' | skipped XML items: {supportResult["skippedCount"]}'

                summaryLines.append(supportLine)
                debugLog(f'run: support summary line: {supportLine}')
            else:
                supportLine = 'Supports: skipped'
                summaryLines.append(supportLine)
                debugLog(f'run: support summary line: {supportLine}')

        debugParts = []
        if DEBUG_SWEEP_LOGGING:
            debugParts.append(f'Sweep log:\n{debugLogPath()}')
        if SUPPORT_DEBUG_LOGGING:
            debugParts.append(f'Support log:\n{supportDebugLogPath()}')
        debugText = '\n\n' + '\n\n'.join(debugParts) if debugParts else ''

        ui.messageBox(
            f'Done!\n\n'
            f'Target gauge: {desiredGauge:.4f} cm / {desiredGauge * 10.0:.2f} mm\n'
            f'Path scale factor: {pathScaleFactor:.6f}\n'
            f'Gauge offset scale factor: {gaugeScaleFactor:.6f}\n\n'
            + '\n'.join(summaryLines)
            + debugText
        )
        debugLog('run: completed successfully')

    except:
        debugLog(f'run: fatal error\n{traceback.format_exc()}')
        if ui:
            ui.messageBox('Fatal error:\n' + traceback.format_exc())
