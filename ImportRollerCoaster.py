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

PERFORMANCE_BUFFER_LOGS = False
PERFORMANCE_LOG_FLUSH_LINE_COUNT = 100
PERFORMANCE_SUPPRESS_VIEWPORT_REFRESH = True

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
SUPPORT_TRACK_MIN_CLEARANCE_CM = 0.20
SUPPORT_INTERFACE_TRANSITION_STANDOFF_CM = 0.30
SUPPORT_BEAM_DIAMETER_SCALE = 0.75
SUPPORT_INTERFACE_START_MAX_SIZE_CM = 0.90
SUPPORT_INTERFACE_TRACK_MAX_SIZE_CM = 0.55
SUPPORT_CROSSTIE_CLEARANCE_CM = 0.1636
SUPPORT_STRAIGHT_CLEARANCE_DROP_MAX_CM = 1.50
SUPPORT_TRACK_INTERSECTION_CLEARANCE_CM = 1.00
SUPPORT_TRACK_INTERSECTION_STEP_CM = 0.10
SUPPORT_TRACK_INTERSECTION_MAX_STEPS = 20
SUPPORT_TRACK_INTERSECTION_EPSILON_CM3 = 0.000001
SUPPORT_CONNECTOR_DIAMETER_SCALE = 0.62
SUPPORT_CONNECTOR_MIN_DIAMETER_CM = 0.35
SUPPORT_CONNECTOR_OVERLAP_CM = 0.12
SUPPORT_CONNECTOR_CREATE_COLLAR = False
SUPPORT_CONNECTOR_COLLAR_SCALE = 1.35
SUPPORT_CONNECTOR_COLLAR_THICKNESS_SCALE = 0.35

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
    global DEBUG_LOG_BUFFER

    DEBUG_LOG_BUFFER = []
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
    global DEBUG_LOG_BUFFER

    if not DEBUG_SWEEP_LOGGING:
        return

    timestamp = datetime.datetime.now().isoformat(timespec='seconds')
    line = f'[{timestamp}] {message}\n'

    if PERFORMANCE_BUFFER_LOGS:
        DEBUG_LOG_BUFFER.append(line)
        if len(DEBUG_LOG_BUFFER) >= PERFORMANCE_LOG_FLUSH_LINE_COUNT:
            flushDebugLogBuffer()
        return

    try:
        with open(debugLogPath(), 'a', encoding='utf-8') as logFile:
            logFile.write(line)
    except:
        pass


def flushDebugLogBuffer():
    global DEBUG_LOG_BUFFER

    if not DEBUG_SWEEP_LOGGING or not DEBUG_LOG_BUFFER:
        return

    try:
        with open(debugLogPath(), 'a', encoding='utf-8') as logFile:
            logFile.writelines(DEBUG_LOG_BUFFER)
        DEBUG_LOG_BUFFER = []
    except:
        pass


def supportDebugLogPath():
    try:
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), SUPPORT_DEBUG_LOG_FILENAME)
    except:
        return os.path.join(os.getcwd(), SUPPORT_DEBUG_LOG_FILENAME)


def resetSupportDebugLog():
    global SUPPORT_DEBUG_LOG_BUFFER

    SUPPORT_DEBUG_LOG_BUFFER = []
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
    global SUPPORT_DEBUG_LOG_BUFFER

    if not SUPPORT_DEBUG_LOGGING:
        return

    timestamp = datetime.datetime.now().isoformat(timespec='seconds')
    line = f'[{timestamp}] {message}\n'

    if PERFORMANCE_BUFFER_LOGS:
        SUPPORT_DEBUG_LOG_BUFFER.append(line)
        if len(SUPPORT_DEBUG_LOG_BUFFER) >= PERFORMANCE_LOG_FLUSH_LINE_COUNT:
            flushSupportDebugLogBuffer()
        return

    try:
        with open(supportDebugLogPath(), 'a', encoding='utf-8') as logFile:
            logFile.write(line)
    except:
        pass


def flushSupportDebugLogBuffer():
    global SUPPORT_DEBUG_LOG_BUFFER

    if not SUPPORT_DEBUG_LOGGING or not SUPPORT_DEBUG_LOG_BUFFER:
        return

    try:
        with open(supportDebugLogPath(), 'a', encoding='utf-8') as logFile:
            logFile.writelines(SUPPORT_DEBUG_LOG_BUFFER)
        SUPPORT_DEBUG_LOG_BUFFER = []
    except:
        pass


def flushDebugLogs():
    flushDebugLogBuffer()
    flushSupportDebugLogBuffer()


def suppressViewportRefresh(app):
    global VIEWPORT_REFRESH_WAS_SUPPRESSED

    if not PERFORMANCE_SUPPRESS_VIEWPORT_REFRESH or not app:
        return

    for methodName in ('disableViewportRefresh', 'disableViewportUpdates'):
        method = getattr(app, methodName, None)
        if not callable(method):
            continue
        try:
            method()
            VIEWPORT_REFRESH_WAS_SUPPRESSED = True
            debugLog(f'performance: called app.{methodName}()')
            return
        except Exception as err:
            debugLog(f'performance: app.{methodName}() failed: {err}')


def restoreViewportRefresh(app):
    global VIEWPORT_REFRESH_WAS_SUPPRESSED

    if not VIEWPORT_REFRESH_WAS_SUPPRESSED or not app:
        return

    for methodName in ('enableViewportRefresh', 'enableViewportUpdates'):
        method = getattr(app, methodName, None)
        if not callable(method):
            continue
        try:
            method()
            VIEWPORT_REFRESH_WAS_SUPPRESSED = False
            debugLog(f'performance: called app.{methodName}()')
            return
        except Exception as err:
            debugLog(f'performance: app.{methodName}() failed: {err}')

    VIEWPORT_REFRESH_WAS_SUPPRESSED = False


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


def describeLoftSection(section):
    if not section:
        return 'None'

    pieces = []
    try:
        pieces.append(f'type={section.objectType}')
    except:
        pass

    try:
        props = section.areaProperties(adsk.fusion.CalculationAccuracy.LowCalculationAccuracy)
        pieces.append(f'area={props.area:.6f}cm2')
        pieces.append(f'centroid={formatPoint(props.centroid)}')
    except Exception as err:
        pieces.append(f'areaError={err}')

    return ', '.join(pieces) if pieces else 'section'


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


def extrudeCutProfile(component, profile, depthCm, name, participantBody=None):
    if depthCm <= 0:
        return None

    extrudes = component.features.extrudeFeatures
    extInput = extrudes.createInput(
        profile,
        adsk.fusion.FeatureOperations.CutFeatureOperation
    )

    distance = adsk.core.ValueInput.createByReal(-depthCm)
    try:
        extInput.setDistanceExtent(False, distance)
    except:
        return None

    if participantBody:
        try:
            bodies = adsk.core.ObjectCollection.create()
            bodies.add(participantBody)
            extInput.participantBodies = bodies
        except:
            pass

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


def createLoftFromProfiles(
    component,
    profiles,
    name,
    operation=adsk.fusion.FeatureOperations.NewBodyFeatureOperation
):
    global LAST_SWEEP_ERROR

    if len(profiles) < 2:
        LAST_SWEEP_ERROR = f'{name}: not enough loft profiles ({len(profiles)})'
        debugLog(f'createLoftFromProfiles {name}: {LAST_SWEEP_ERROR}')
        return None

    try:
        debugLog(f'createLoftFromProfiles {name}: begin profileCount={len(profiles)}')
        lofts = component.features.loftFeatures
        loftInput = lofts.createInput(operation)

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
                debugLog(f'createLoftFromProfiles {name}: add section {i}: {describeLoftSection(profile)}')
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


def sampleSupportTrack(supportTrackData, centerRailsCoordMeters, supportScaleFactor, snapToSafe=True, forceSafe=False):
    if isinstance(supportTrackData, dict):
        samples = supportTrackData.get('samples', [])
        safeAnchors = supportTrackData.get('safeAnchors', [])
    else:
        samples = supportTrackData or []
        safeAnchors = []

    if not samples:
        return None

    targetCm = centerRailsCoordMeters * supportScaleFactor
    targetNearestCrosstieCm = nearestCrosstieStationDistanceCm(supportTrackData, targetCm)
    if forceSafe and safeAnchors:
        validAnchors = [
            item for item in safeAnchors
            if item.get('nearestCrosstieDistanceCm', 999999.0) >= SUPPORT_CROSSTIE_CLEARANCE_CM
        ]
        anchorChoices = validAnchors if validAnchors else safeAnchors
        anchor = min(anchorChoices, key=lambda item: abs(item['distanceCm'] - targetCm))
        result = dict(anchor)
        result['originalTargetCm'] = targetCm
        result['snapDeltaCm'] = result['distanceCm'] - targetCm
        result['requestedNearestCrosstieCm'] = targetNearestCrosstieCm
        result['forcedSafeAnchor'] = True
        return result

    shouldSnap = (
        snapToSafe
        and safeAnchors
        and targetNearestCrosstieCm is not None
        and targetNearestCrosstieCm < SUPPORT_CROSSTIE_CLEARANCE_CM
    )

    if shouldSnap:
        validAnchors = [
            item for item in safeAnchors
            if item.get('nearestCrosstieDistanceCm', 999999.0) >= SUPPORT_CROSSTIE_CLEARANCE_CM
        ]
        anchorChoices = validAnchors if validAnchors else safeAnchors
        anchor = min(anchorChoices, key=lambda item: abs(item['distanceCm'] - targetCm))
        result = dict(anchor)
        result['originalTargetCm'] = targetCm
        result['snapDeltaCm'] = result['distanceCm'] - targetCm
        result['requestedNearestCrosstieCm'] = targetNearestCrosstieCm
        return result

    if targetCm <= samples[0]['distanceCm']:
        result = dict(samples[0], targetCm=targetCm, t=0.0)
        return addNearestCrosstieDistance(result, supportTrackData)
    if targetCm >= samples[-1]['distanceCm']:
        result = dict(samples[-1], targetCm=targetCm, t=1.0)
        return addNearestCrosstieDistance(result, supportTrackData)

    for i in range(len(samples) - 1):
        startSample = samples[i]
        endSample = samples[i + 1]
        if endSample['distanceCm'] < targetCm:
            continue

        segmentLength = endSample['distanceCm'] - startSample['distanceCm']
        t = 0.0 if segmentLength < 0.000001 else (targetCm - startSample['distanceCm']) / segmentLength
        result = interpolateTrackSample(startSample, endSample, targetCm, t)
        return addNearestCrosstieDistance(result, supportTrackData)

    result = dict(samples[-1], targetCm=targetCm, t=1.0)
    return addNearestCrosstieDistance(result, supportTrackData)


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


def supportTrackInterfaceAnchorForKind(trackSample, frame, supportKind, referencePoint=None):
    if not frame:
        return supportTrackInterfaceAnchor(trackSample, frame)

    halfSpineHeightCm = mm_to_cm(SPINE_RECT_HEIGHT_MM) * 0.5

    if supportKind == '180':
        return addVec(trackSample['spine'], frame.yAxis, halfSpineHeightCm)

    return addVec(trackSample['spine'], frame.yAxis, -halfSpineHeightCm)


def supportInterfaceCenterClearanceCm(diameterMeters, supportScaleFactor):
    beamDiameterCm = diameterMeters * supportScaleFactor * SUPPORT_BEAM_DIAMETER_SCALE
    return max(
        SUPPORT_TRACK_INTERFACE_STANDOFF_CM,
        beamDiameterCm * 0.5 + SUPPORT_TRACK_MIN_CLEARANCE_CM,
        SUPPORT_TRACK_INTERSECTION_CLEARANCE_CM
    )


def supportInterfaceSideSign(nativeTop, trackAnchor, frame):
    if not frame:
        return 1.0

    sideScore = vectorBetween(trackAnchor, nativeTop).dotProduct(frame.xAxis)
    return 1.0 if sideScore >= 0.0 else -1.0


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


def straightSupportClearanceDropCm(trackSample):
    clearanceCm = trackSample.get('nearestCrosstieDistanceCm', None)
    if clearanceCm is None or clearanceCm >= SUPPORT_CROSSTIE_CLEARANCE_CM:
        return 0.0

    neededDrop = math.sqrt(max(0.0, SUPPORT_CROSSTIE_CLEARANCE_CM ** 2 - clearanceCm ** 2))
    return min(SUPPORT_STRAIGHT_CLEARANCE_DROP_MAX_CM, neededDrop + 0.02)


def supportOffsetForPrefab(prefabOffsets, prefabIndex):
    return prefabOffsets.get(prefabIndex, zeroVector())


def computeSupportPrefabOffsets(supportData, supportTrackSamples, supportScaleFactor):
    offsets = {}

    if not supportTrackSamples:
        return offsets

    for interface in supportData['interfaces']:
        prefabIndex = interface['prefabIndex']
        if prefabIndex in offsets:
            continue

        supportKind = supportInterfaceKind(interface)
        trackSample = sampleSupportTrack(
            supportTrackSamples,
            interface['centerRailsCoord'],
            supportScaleFactor,
            snapToSafe=True,
            forceSafe=True
        )
        if not trackSample:
            continue

        frame = supportTrackFrame(trackSample)
        nativeTop = supportPointMetersToFusion(interface['pointMeters'], supportScaleFactor)
        trackAnchor = supportTrackInterfaceAnchorForKind(trackSample, frame, supportKind, nativeTop)
        clearanceCm = supportInterfaceCenterClearanceCm(interface['diameterMeters'], supportScaleFactor)
        clearanceDropCm = 0.0
        if frame:
            desiredTop = addVec(trackAnchor, frame.yAxis, -clearanceCm)
        else:
            desiredTop = adsk.core.Point3D.create(trackAnchor.x, trackAnchor.y, trackAnchor.z - clearanceCm)

        if supportKind in ('single', 'double', 'straight'):
            clearanceDropCm = straightSupportClearanceDropCm(trackSample)
            if clearanceDropCm > 0.0:
                if frame:
                    desiredTop = addVec(desiredTop, frame.yAxis, -clearanceDropCm)
                else:
                    desiredTop = adsk.core.Point3D.create(
                        desiredTop.x,
                        desiredTop.y,
                        desiredTop.z - clearanceDropCm
                    )

        offset = vectorBetween(nativeTop, desiredTop)
        offsets[prefabIndex] = offset

        supportDebugLog(
            f'prefab offset {prefabIndex}: centerRails={interface["centerRailsCoord"]:.6f}m, '
            f'kind={supportKind}, '
            f'nativeTop={formatPoint(nativeTop)}, desiredTop={formatPoint(desiredTop)}, '
            f'offset={formatVector(offset)}, track={trackSample["folderName"]}[{trackSample["index"]}] '
            f't={trackSample["t"]:.6f}, safeAnchor={trackSample.get("safeAnchor", False)}, '
            f'between={trackSample.get("betweenCrossties", None)}, '
            f'snapDeltaCm={trackSample.get("snapDeltaCm", 0.0):.6f}, '
            f'nearestCrosstieCm={trackSample.get("nearestCrosstieDistanceCm", -1.0):.6f}, '
            f'clearanceCm={clearanceCm:.6f}, clearanceDropCm={clearanceDropCm:.6f}'
        )

    return offsets


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
    topZ = footerCenter.z + heightCm * 0.5
    bottomZ = footerCenter.z - heightCm * 0.5
    direction = vectorBetween(outerPoint, footerCenter)
    candidates = []

    if abs(direction.z) > 0.000001:
        topT = (topZ - outerPoint.z) / direction.z
        if 0.0 <= topT <= 1.0:
            topPoint = addVec(outerPoint, direction, topT)
            if horizontalDistanceTo(topPoint, footerCenter) <= radiusCm + 0.000001:
                candidates.append((topT, topPoint))

        bottomT = (bottomZ - outerPoint.z) / direction.z
        if bottomT >= 0.0:
            bottomPoint = addVec(outerPoint, direction, bottomT)
            if horizontalDistanceTo(bottomPoint, footerCenter) <= radiusCm + 0.000001:
                candidates.append((bottomT, bottomPoint))

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
                if t >= 0.0:
                    point = addVec(outerPoint, direction, t)
                    if bottomZ - 0.000001 <= point.z <= topZ + 0.000001:
                        candidates.append((t, point))

    if candidates:
        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[0][1]

    return adsk.core.Point3D.create(footerCenter.x, footerCenter.y, bottomZ)


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
        clippedStart = clampPointToFloor(
            footerPadExitPoint(endPoint, footerCenter, radiusCm, heightCm),
            floorZ
        )
        supportDebugLog(
            f'footer exit start beam={beam["startId"]}->{beam["endId"]}: '
            f'center={formatPoint(footerCenter)}, exit={formatPoint(clippedStart)}, floorZ={floorZ}'
        )
        startPoint = clippedStart

    if beam.get('endKind') == 'footernode':
        footerCenter = endPoint
        clippedEnd = clampPointToFloor(
            footerPadExitPoint(startPoint, footerCenter, radiusCm, heightCm),
            floorZ
        )
        supportDebugLog(
            f'footer exit end beam={beam["startId"]}->{beam["endId"]}: '
            f'center={formatPoint(footerCenter)}, exit={formatPoint(clippedEnd)}, floorZ={floorZ}'
        )
        endPoint = clippedEnd

    return startPoint, endPoint, []


def createSupportBeamBody(component, startPoint, endPoint, diameterCm, name):
    distance = startPoint.distanceTo(endPoint)
    if distance < 0.000001 or diameterCm <= 0:
        debugLog(f'createSupportBeamBody {name}: skipped')
        return None

    tangent = vectorBetween(startPoint, endPoint)
    if not safeNormalize(tangent):
        debugLog(f'createSupportBeamBody {name}: bad tangent')
        return None

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
            return None

        sketch, profile = createCircleProfileOnPlane(
            component,
            plane,
            centerPoint,
            diameterCm,
            f'{name}_profile'
        )

        if not profile:
            debugLog(f'createSupportBeamBody {name}: profile failed')
            return None

        return extrudeProfile(
            component,
            profile,
            distance,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
            name
        )
    except Exception as err:
        debugLog(f'createSupportBeamBody {name}: failed: {err}\n{traceback.format_exc()}')
        return None


def bodyFromFeature(feature):
    try:
        if feature and feature.bodies and feature.bodies.count > 0:
            return feature.bodies.item(0)
    except:
        pass
    return None


def createChildComponent(parentComponent, name):
    identity = adsk.core.Matrix3D.create()
    occurrence = parentComponent.occurrences.addNewComponent(identity)
    childComponent = adsk.fusion.Component.cast(occurrence.component)
    childComponent.name = name
    try:
        occurrence.name = name
    except:
        pass
    return childComponent


def collectComponentBodies(component, bodies=None):
    if bodies is None:
        bodies = []

    if not component:
        return bodies

    try:
        for i in range(component.bRepBodies.count):
            body = component.bRepBodies.item(i)
            if body:
                bodies.append(body)
    except:
        pass

    try:
        for i in range(component.occurrences.count):
            occurrence = component.occurrences.item(i)
            collectComponentBodies(occurrence.component, bodies)
    except:
        pass

    return bodies


def bodyBoundingBox(body):
    try:
        return body.boundingBox
    except:
        return None


def bodyCenterPoint(body):
    bbox = bodyBoundingBox(body)
    if not bbox:
        return None

    return midpoint(bbox.minPoint, bbox.maxPoint)


def boundingBoxesOverlap(a, b, paddingCm=0.0):
    if not a or not b:
        return False

    return not (
        a.maxPoint.x + paddingCm < b.minPoint.x or a.minPoint.x - paddingCm > b.maxPoint.x
        or a.maxPoint.y + paddingCm < b.minPoint.y or a.minPoint.y - paddingCm > b.maxPoint.y
        or a.maxPoint.z + paddingCm < b.minPoint.z or a.minPoint.z - paddingCm > b.maxPoint.z
    )


def bodyPhysicalVolume(body):
    try:
        props = body.physicalProperties
        return props.volume
    except:
        try:
            props = body.getPhysicalProperties(
                adsk.fusion.CalculationAccuracy.LowCalculationAccuracy
            )
            return props.volume
        except:
            return None


def fusionBodiesIntersect(bodyA, bodyB):
    boxA = bodyBoundingBox(bodyA)
    boxB = bodyBoundingBox(bodyB)
    if not boundingBoxesOverlap(boxA, boxB):
        return False

    try:
        tempMgr = adsk.fusion.TemporaryBRepManager.get()
        tempA = tempMgr.copy(bodyA)
        tempB = tempMgr.copy(bodyB)
        tempMgr.booleanOperation(
            tempA,
            tempB,
            adsk.fusion.BooleanTypes.IntersectionBooleanType
        )
        volume = bodyPhysicalVolume(tempA)
        if volume is not None:
            return volume > SUPPORT_TRACK_INTERSECTION_EPSILON_CM3
    except Exception as err:
        supportDebugLog(f'fusionBodiesIntersect: temporary BRep intersect failed: {err}')

    return True


def bodyIntersectsAnyTrackBody(supportBody, trackBodies):
    supportBox = bodyBoundingBox(supportBody)
    if not supportBox:
        return False, None

    for trackBody in trackBodies or []:
        trackBox = bodyBoundingBox(trackBody)
        if not boundingBoxesOverlap(supportBox, trackBox):
            continue

        if fusionBodiesIntersect(supportBody, trackBody):
            return True, trackBody

    return False, None


def bodyTooCloseToAnyTrackBody(supportBody, trackBodies, clearanceCm):
    supportBox = bodyBoundingBox(supportBody)
    if not supportBox:
        return False, None

    for trackBody in trackBodies or []:
        trackBox = bodyBoundingBox(trackBody)
        if boundingBoxesOverlap(supportBox, trackBox, clearanceCm):
            return True, trackBody

    return False, None


def bodyViolatesTrackClearance(supportBody, trackBodies):
    intersects, trackBody = bodyIntersectsAnyTrackBody(supportBody, trackBodies)
    if intersects:
        return True, True, trackBody

    tooClose, trackBody = bodyTooCloseToAnyTrackBody(
        supportBody,
        trackBodies,
        SUPPORT_TRACK_INTERSECTION_CLEARANCE_CM
    )
    return tooClose, False, trackBody


def moveBodyByVector(component, body, direction, distanceCm, name):
    try:
        owningComponent = component
        try:
            if body.parentComponent:
                owningComponent = body.parentComponent
        except:
            pass

        moveVector = adsk.core.Vector3D.create(
            direction.x * distanceCm,
            direction.y * distanceCm,
            direction.z * distanceCm
        )
        transform = adsk.core.Matrix3D.create()
        transform.translation = moveVector

        bodies = adsk.core.ObjectCollection.create()
        bodies.add(body)

        moveFeatures = owningComponent.features.moveFeatures
        try:
            moveInput = moveFeatures.createInput2(bodies, transform)
        except:
            moveInput = moveFeatures.createInput(bodies, transform)

        feature = moveFeatures.add(moveInput)
        if feature:
            feature.name = name
        else:
            supportDebugLog(f'moveBodyByVector {name}: moveFeatures.add returned None; rechecking body position')
        return True
    except Exception as err:
        supportDebugLog(f'moveBodyByVector {name}: failed: {err}')
        return False


def nearestTrackSampleForPoint(supportTrackSamples, point):
    if not point or not supportTrackSamples:
        return None

    samples = supportTrackSamples.get('samples', []) if isinstance(supportTrackSamples, dict) else supportTrackSamples
    if not samples:
        return None

    bestSample = None
    bestDistance = None
    for sample in samples:
        samplePoint = sample.get('spine') or sample.get('center') or sample.get('left')
        if not samplePoint:
            continue
        distance = point.distanceTo(samplePoint)
        if bestDistance is None or distance < bestDistance:
            bestDistance = distance
            bestSample = sample

    return bestSample


def supportClearanceMoveDirection(body, supportTrackSamples):
    center = bodyCenterPoint(body)
    sample = nearestTrackSampleForPoint(supportTrackSamples, center)
    frame = supportTrackFrame(sample) if sample else None
    if not center or not sample or not frame:
        return adsk.core.Vector3D.create(1.0, 0.0, 0.0)

    trackPoint = sample.get('spine') or frame.origin
    away = vectorBetween(trackPoint, center)
    tangentComponent = away.dotProduct(frame.zAxis)
    direction = adsk.core.Vector3D.create(
        away.x - frame.zAxis.x * tangentComponent,
        away.y - frame.zAxis.y * tangentComponent,
        away.z - frame.zAxis.z * tangentComponent
    )

    if not safeNormalize(direction):
        direction = adsk.core.Vector3D.create(frame.yAxis.x, frame.yAxis.y, frame.yAxis.z)
        if not safeNormalize(direction):
            direction = adsk.core.Vector3D.create(frame.xAxis.x, frame.xAxis.y, frame.xAxis.z)
            safeNormalize(direction)

    return direction


def resolveSupportTrackIntersections(component, supportBodies, trackBodies, supportTrackSamples):
    supportDebugLog(
        f'resolveSupportTrackIntersections begin: '
        f'supportBodies={len(supportBodies) if supportBodies else 0}, '
        f'trackBodies={len(trackBodies) if trackBodies else 0}, '
        f'clearance={SUPPORT_TRACK_INTERSECTION_CLEARANCE_CM:.6f}, '
        f'step={SUPPORT_TRACK_INTERSECTION_STEP_CM:.6f}, '
        f'maxSteps={SUPPORT_TRACK_INTERSECTION_MAX_STEPS}'
    )

    if not supportBodies or not trackBodies:
        supportDebugLog(
            f'resolveSupportTrackIntersections skipped: '
            f'supportBodies={len(supportBodies) if supportBodies else 0}, '
            f'trackBodies={len(trackBodies) if trackBodies else 0}'
        )
        return 0

    movedCount = 0
    for index, body in enumerate(supportBodies, start=1):
        try:
            bodyName = body.name
        except:
            bodyName = f'support_body_{index:03d}'

        if index == 1 or index % 10 == 0:
            supportDebugLog(
                f'resolveSupportTrackIntersections progress: body={index}/{len(supportBodies)}, '
                f'name={bodyName}'
            )

        violatesClearance, intersects, trackBody = bodyViolatesTrackClearance(body, trackBodies)
        if not violatesClearance:
            continue

        direction = supportClearanceMoveDirection(body, supportTrackSamples)
        supportDebugLog(
            f'resolveSupportTrackIntersections {bodyName}: initial violation=True, '
            f'intersects={intersects}, clearance={SUPPORT_TRACK_INTERSECTION_CLEARANCE_CM:.6f}, '
            f'direction={formatVector(direction)}, step={SUPPORT_TRACK_INTERSECTION_STEP_CM:.6f}, '
            f'maxSteps={SUPPORT_TRACK_INTERSECTION_MAX_STEPS}'
        )

        movedDistance = 0.0
        cleared = False
        for stepIndex in range(1, SUPPORT_TRACK_INTERSECTION_MAX_STEPS + 1):
            moveSucceeded = moveBodyByVector(
                component,
                body,
                direction,
                SUPPORT_TRACK_INTERSECTION_STEP_CM,
                f'{bodyName}_track_clearance_move_{stepIndex:02d}'
            )
            if not moveSucceeded:
                supportDebugLog(
                    f'resolveSupportTrackIntersections {bodyName}: '
                    f'step={stepIndex}, move failed; stopping this body'
                )
                break
            movedDistance += SUPPORT_TRACK_INTERSECTION_STEP_CM

            violatesClearance, intersects, trackBody = bodyViolatesTrackClearance(body, trackBodies)
            supportDebugLog(
                f'resolveSupportTrackIntersections {bodyName}: '
                f'step={stepIndex}, moved={movedDistance:.6f}, '
                f'intersects={intersects}, violatesClearance={violatesClearance}'
            )
            if not violatesClearance:
                cleared = True
                break

        if movedDistance > 0.0:
            movedCount += 1
        if not cleared:
            supportDebugLog(
                f'resolveSupportTrackIntersections {bodyName}: still violates clearance after '
                f'{movedDistance:.6f}cm; leaving best moved position'
            )

    supportDebugLog(f'resolveSupportTrackIntersections complete: movedBodies={movedCount}')
    flushSupportDebugLogBuffer()
    return movedCount


def registerSupportBeamDebugRecord(recordsByNodeId, nodeId, endpoint, otherEndpoint, diameterCm, name):
    if not nodeId or not endpoint or not otherEndpoint:
        return

    key = str(nodeId)
    if key not in recordsByNodeId:
        recordsByNodeId[key] = []

    recordsByNodeId[key].append({
        'endpoint': endpoint,
        'otherEndpoint': otherEndpoint,
        'diameterCm': diameterCm,
        'name': name
    })


def nearestSupportBeamDebugRecord(recordsByNodeId, nodeId, targetPoint):
    records = recordsByNodeId.get(str(nodeId), []) if recordsByNodeId else []
    bestRecord = None
    bestDistance = None

    for record in records:
        endpoint = record.get('endpoint')
        if not endpoint:
            continue

        distance = endpoint.distanceTo(targetPoint)
        if bestDistance is None or distance < bestDistance:
            bestDistance = distance
            bestRecord = record

    return bestRecord, bestDistance


def trimSupportBeamBelowFloor(component, beamFeature, startPoint, endPoint, diameterCm, floorZ, name):
    if floorZ is None or not beamFeature:
        return None

    body = bodyFromFeature(beamFeature)
    if not body:
        supportDebugLog(f'trimSupportBeamBelowFloor {name}: skipped; no body')
        return None

    lowCenterZ = min(startPoint.z, endPoint.z)
    cutDepth = max(diameterCm * 3.0, floorZ - lowCenterZ + diameterCm * 3.0, 0.5)
    center = midpoint(startPoint, endPoint)
    planeCenter = adsk.core.Point3D.create(center.x, center.y, floorZ)
    xAxis = adsk.core.Vector3D.create(1, 0, 0)
    yAxis = adsk.core.Vector3D.create(0, 1, 0)
    span = max(
        abs(startPoint.x - endPoint.x),
        abs(startPoint.y - endPoint.y),
        diameterCm,
        1.0
    ) + diameterCm * 6.0 + 2.0

    try:
        plane = createPlaneByThreePoints(
            component,
            planeCenter,
            addVec(planeCenter, xAxis, 1.0),
            addVec(planeCenter, yAxis, 1.0),
            f'{name}_floor_cut_plane'
        )
        if not plane:
            supportDebugLog(f'trimSupportBeamBelowFloor {name}: skipped; cut plane failed')
            return None

        sketch, profile = createCenteredRectangleProfileOnPlane(
            component,
            plane,
            planeCenter,
            xAxis,
            yAxis,
            span,
            span,
            f'{name}_floor_cut_profile'
        )
        if not profile:
            supportDebugLog(f'trimSupportBeamBelowFloor {name}: skipped; cut profile failed')
            return None

        cutFeature = extrudeCutProfile(
            component,
            profile,
            cutDepth,
            f'{name}_floor_trim',
            body
        )
        supportDebugLog(
            f'trimSupportBeamBelowFloor {name}: '
            f'floorZ={floorZ:.6f}, cutDepth={cutDepth:.6f}, span={span:.6f}, '
            f'success={cutFeature is not None}'
        )
        return cutFeature
    except Exception as err:
        supportDebugLog(f'trimSupportBeamBelowFloor {name}: failed: {err}\n{traceback.format_exc()}')
        return None


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

    if frame and supportKind == '180':
        return addVec(supportTop, frame.yAxis, -sizeCm * 0.5)

    return supportTop


def supportInterfaceProfileAxes(startPoint, endPoint, frame, supportKind):
    if frame:
        return frame.xAxis, frame.zAxis

    tangent = vectorBetween(startPoint, endPoint)
    if not safeNormalize(tangent):
        return None, None

    xAxis = perpendicularUnitVector(tangent)
    if not xAxis:
        return None, None

    yAxis = tangent.crossProduct(xAxis)
    if not safeNormalize(yAxis):
        return None, None

    return xAxis, yAxis


def supportInterfaceTrackSizeCm(supportKind):
    return mm_to_cm(SPINE_RECT_WIDTH_MM)


def supportInterfaceStartProfile(
    component,
    plane,
    startPoint,
    xAxis,
    yAxis,
    startSizeCm,
    supportKind,
    name
):
    if supportKind in ('single', 'double', 'straight'):
        return createCircleProfileOnPlane(
            component,
            plane,
            startPoint,
            startSizeCm,
            f'{name}_start_tube_profile'
        )

    return createCenteredRectangleProfileOnPlane(
        component,
        plane,
        startPoint,
        xAxis,
        yAxis,
        startSizeCm,
        startSizeCm,
        f'{name}_start_profile'
    )


def supportInterfaceTransitionPoint(startPoint, endPoint, startSizeCm):
    direction = vectorBetween(startPoint, endPoint)
    distance = direction.length
    if distance < 0.000001:
        return None

    direction.normalize()
    transitionDistance = max(
        SUPPORT_INTERFACE_TRANSITION_STANDOFF_CM,
        startSizeCm * 0.75
    )
    transitionDistance = min(transitionDistance, distance * 0.75)
    if transitionDistance < 0.000001:
        return None

    return addVec(startPoint, direction, transitionDistance)


def supportConnectorEmbedPoint(startPoint, transitionPoint, diameterCm):
    direction = vectorBetween(transitionPoint, startPoint)
    if not safeNormalize(direction):
        return startPoint

    return addVec(startPoint, direction, max(diameterCm * 0.5, SUPPORT_TRACK_MIN_CLEARANCE_CM))


def supportInterfaceFinalOverlapPoint(extensionPoint, beamPoint, diameterCm):
    direction = vectorBetween(extensionPoint, beamPoint)
    if not safeNormalize(direction):
        return beamPoint

    overlapCm = max(diameterCm * 0.55, SUPPORT_TRACK_MIN_CLEARANCE_CM)
    return addVec(beamPoint, direction, overlapCm)


def supportConnectorDiameterCm(startSizeCm, beamDiameterCm=None):
    candidates = [
        startSizeCm * SUPPORT_CONNECTOR_DIAMETER_SCALE,
        startSizeCm
    ]
    if beamDiameterCm and beamDiameterCm > 0:
        candidates.append(beamDiameterCm * SUPPORT_CONNECTOR_DIAMETER_SCALE)

    diameter = min(candidates)
    return max(SUPPORT_CONNECTOR_MIN_DIAMETER_CM, diameter)


def vectorComponentPerpendicularToAxis(fromPoint, toPoint, axis):
    vec = vectorBetween(fromPoint, toPoint)
    axisCopy = adsk.core.Vector3D.create(axis.x, axis.y, axis.z)
    if not safeNormalize(axisCopy):
        return None

    axial = vec.dotProduct(axisCopy)
    return adsk.core.Vector3D.create(
        vec.x - axisCopy.x * axial,
        vec.y - axisCopy.y * axial,
        vec.z - axisCopy.z * axial
    )


def supportBeamContactPoint(connectorStart, beamEndpoint, beamOtherEndpoint, beamDiameterCm):
    if not connectorStart or not beamEndpoint or not beamOtherEndpoint:
        return beamEndpoint

    beamAxis = vectorBetween(beamOtherEndpoint, beamEndpoint)
    if not safeNormalize(beamAxis):
        return beamEndpoint

    fallbackMode = 'perpendicular'
    radial = vectorComponentPerpendicularToAxis(beamEndpoint, connectorStart, beamAxis)
    if not radial or not safeNormalize(radial):
        fallbackMode = 'connector_projection'
        connectorDirection = vectorBetween(connectorStart, beamEndpoint)
        radial = adsk.core.Vector3D.create(
            connectorDirection.x,
            connectorDirection.y,
            connectorDirection.z
        )
        axial = radial.dotProduct(beamAxis)
        radial = adsk.core.Vector3D.create(
            radial.x - beamAxis.x * axial,
            radial.y - beamAxis.y * axial,
            radial.z - beamAxis.z * axial
        )
        if not safeNormalize(radial):
            fallbackMode = 'axis_perpendicular'
            radial = perpendicularUnitVector(beamAxis)
            if not radial:
                return beamEndpoint

    radiusCm = max(beamDiameterCm * 0.5, SUPPORT_TRACK_MIN_CLEARANCE_CM)
    contactPoint = addVec(beamEndpoint, radial, radiusCm * 0.85)
    supportDebugLog(
        f'supportBeamContactPoint: start={formatPoint(connectorStart)}, '
        f'beamEndpoint={formatPoint(beamEndpoint)}, beamOther={formatPoint(beamOtherEndpoint)}, '
        f'axis={formatVector(beamAxis)}, radial={formatVector(radial)}, '
        f'beamDiameter={beamDiameterCm:.6f}, radius={radiusCm:.6f}, '
        f'contact={formatPoint(contactPoint)}, mode={fallbackMode}'
    )
    return contactPoint


def supportConnectorEmbeddedEnd(contactPoint, beamEndpoint):
    direction = vectorBetween(contactPoint, beamEndpoint)
    if not safeNormalize(direction):
        return contactPoint

    return addVec(contactPoint, direction, SUPPORT_CONNECTOR_OVERLAP_CM)


def distancePointToLine(point, linePoint, lineOtherPoint):
    if not point or not linePoint or not lineOtherPoint:
        return None

    axis = vectorBetween(linePoint, lineOtherPoint)
    if not safeNormalize(axis):
        return None

    offset = vectorBetween(linePoint, point)
    axial = offset.dotProduct(axis)
    perpendicular = adsk.core.Vector3D.create(
        offset.x - axis.x * axial,
        offset.y - axis.y * axial,
        offset.z - axis.z * axial
    )
    return perpendicular.length


def supportConnectorDebugLog(
    name,
    transitionPoint,
    contactPoint,
    embeddedEndPoint,
    beamEndpoint,
    beamOtherEndpoint,
    connectorDiameterCm,
    beamDiameterCm,
    connectorFeature,
    collarFeature
):
    beamRadius = beamDiameterCm * 0.5 if beamDiameterCm else None
    contactRadius = distancePointToLine(contactPoint, beamEndpoint, beamOtherEndpoint)
    embeddedRadius = distancePointToLine(embeddedEndPoint, beamEndpoint, beamOtherEndpoint)
    transitionToContact = transitionPoint.distanceTo(contactPoint) if transitionPoint and contactPoint else -1.0
    contactToEmbedded = contactPoint.distanceTo(embeddedEndPoint) if contactPoint and embeddedEndPoint else -1.0
    transitionToEmbedded = transitionPoint.distanceTo(embeddedEndPoint) if transitionPoint and embeddedEndPoint else -1.0

    supportDebugLog(
        f'createSupportInterfaceLoft {name}: planB connector detail '
        f'transition={formatPoint(transitionPoint)}, contact={formatPoint(contactPoint)}, '
        f'embeddedEnd={formatPoint(embeddedEndPoint)}, beamEndpoint={formatPoint(beamEndpoint)}, '
        f'beamOther={formatPoint(beamOtherEndpoint)}, '
        f'transitionToContact={transitionToContact:.6f}, '
        f'contactToEmbedded={contactToEmbedded:.6f}, transitionToEmbedded={transitionToEmbedded:.6f}, '
        f'connectorDiameter={connectorDiameterCm:.6f}, '
        f'beamDiameter={(beamDiameterCm if beamDiameterCm else -1.0):.6f}, '
        f'beamRadius={(beamRadius if beamRadius is not None else -1.0):.6f}, '
        f'contactRadius={(contactRadius if contactRadius is not None else -1.0):.6f}, '
        f'embeddedRadius={(embeddedRadius if embeddedRadius is not None else -1.0):.6f}, '
        f'connector={connectorFeature is not None}, collar={collarFeature is not None}, '
        f'collarEnabled={SUPPORT_CONNECTOR_CREATE_COLLAR}'
    )


def createSupportJointCollar(
    component,
    beamEndpoint,
    beamOtherEndpoint,
    beamDiameterCm,
    connectorDiameterCm,
    name
):
    if not beamEndpoint or not beamOtherEndpoint:
        return None

    beamAxis = vectorBetween(beamOtherEndpoint, beamEndpoint)
    if not safeNormalize(beamAxis):
        return None

    thickness = max(
        connectorDiameterCm * SUPPORT_CONNECTOR_COLLAR_THICKNESS_SCALE,
        SUPPORT_CONNECTOR_OVERLAP_CM
    )
    startPoint = addVec(beamEndpoint, beamAxis, -thickness * 0.35)
    endPoint = addVec(beamEndpoint, beamAxis, thickness * 0.65)
    collarDiameter = max(
        beamDiameterCm if beamDiameterCm else 0.0,
        connectorDiameterCm * SUPPORT_CONNECTOR_COLLAR_SCALE
    )
    return createSupportBeamBody(
        component,
        startPoint,
        endPoint,
        collarDiameter,
        name
    )


def supportInterfaceCircleExtensionEndPoint(circlePoint, beamPoint):
    direction = vectorBetween(circlePoint, beamPoint)
    distance = direction.length
    if distance < 0.000001:
        return None

    direction.normalize()
    extensionDistance = min(distance * 0.65, max(SUPPORT_INTERFACE_TRANSITION_STANDOFF_CM, distance * 0.5))
    if extensionDistance <= 0.000001:
        return None

    return addVec(circlePoint, direction, extensionDistance)


def createSupportTrackSquareToCircleLoft(
    component,
    circlePoint,
    trackPoint,
    xAxis,
    yAxis,
    circleSizeCm,
    squareSizeCm,
    name
):
    circlePlane = createPlaneByThreePoints(
        component,
        circlePoint,
        addVec(circlePoint, xAxis, 1.0),
        addVec(circlePoint, yAxis, 1.0),
        f'{name}_p1_circle_plane'
    )
    squarePlane = createPlaneByThreePoints(
        component,
        trackPoint,
        addVec(trackPoint, xAxis, 1.0),
        addVec(trackPoint, yAxis, 1.0),
        f'{name}_track_square_plane'
    )
    if not circlePlane or not squarePlane:
        supportDebugLog(f'createSupportInterfaceLoft {name}: square-to-circle plane creation failed')
        return None, None

    circleSketch, circleProfile = createCircleProfileOnPlane(
        component,
        circlePlane,
        circlePoint,
        circleSizeCm,
        f'{name}_p1_circle_profile'
    )
    squareSketch, squareProfile = createCenteredRectangleProfileOnPlane(
        component,
        squarePlane,
        trackPoint,
        xAxis,
        yAxis,
        squareSizeCm,
        squareSizeCm,
        f'{name}_track_square_profile'
    )

    if not circleProfile or not squareProfile:
        supportDebugLog(
            f'createSupportInterfaceLoft {name}: square-to-circle profile failed; '
            f'circle={describeProfile(circleProfile)}, square={describeProfile(squareProfile)}'
        )
        return None, None

    trackLoft = createLoftFromProfiles(
        component,
        [squareProfile, circleProfile],
        f'{name}_track_square_to_p1_circle'
    )
    supportDebugLog(
        f'createSupportInterfaceLoft {name}: square-to-circle loft={trackLoft is not None}, '
        f'circle={formatPoint(circlePoint)}, track={formatPoint(trackPoint)}, '
        f'distance={circlePoint.distanceTo(trackPoint):.6f}, '
        f'circleProfile={describeProfile(circleProfile)}, squareProfile={describeProfile(squareProfile)}'
    )
    return trackLoft, circleProfile


def createSupportCircleToCircleLoft(
    component,
    startPoint,
    endPoint,
    xAxis,
    yAxis,
    diameterCm,
    name,
    featureSuffix='p1_circle_to_beam_top',
    throughPoints=None,
    operation=adsk.fusion.FeatureOperations.NewBodyFeatureOperation
):
    profilePoints = [startPoint]
    if throughPoints:
        profilePoints.extend(point for point in throughPoints if point)
    profilePoints.append(endPoint)

    gapDirection = vectorBetween(startPoint, endPoint)
    gapDistance = startPoint.distanceTo(endPoint)
    if not safeNormalize(gapDirection):
        supportDebugLog(f'createSupportInterfaceLoft {name}: gap direction failed')
        return None

    supportDebugLog(
        f'createSupportInterfaceLoft {name}: {featureSuffix} input '
        f'start={formatPoint(startPoint)}, end={formatPoint(endPoint)}, '
        f'distance={gapDistance:.6f}, diameter={diameterCm:.6f}, '
        f'direction={formatVector(gapDirection)}, sections={len(profilePoints)}'
    )

    profiles = []
    for pointIndex, profilePoint in enumerate(profilePoints):
        plane = createPlaneNormalToVector(
            component,
            profilePoint,
            gapDirection,
            f'{name}_{featureSuffix}_{pointIndex:02d}_plane'
        )
        if not plane:
            supportDebugLog(
                f'createSupportInterfaceLoft {name}: gap circle plane creation failed '
                f'at section {pointIndex}, point={formatPoint(profilePoint)}'
            )
            return None

        sketch, profile = createCircleProfileOnPlane(
            component,
            plane,
            profilePoint,
            diameterCm,
            f'{name}_{featureSuffix}_{pointIndex:02d}_profile'
        )
        if not profile:
            supportDebugLog(
                f'createSupportInterfaceLoft {name}: circle-to-circle profile failed '
                f'at section {pointIndex}, point={formatPoint(profilePoint)}, '
                f'profile={describeProfile(profile)}'
            )
            return None
        profiles.append(profile)

    loft = createLoftFromProfiles(
        component,
        profiles,
        f'{name}_{featureSuffix}',
        operation
    )
    supportDebugLog(
        f'createSupportInterfaceLoft {name}: {featureSuffix} loft={loft is not None}, '
        f'sections={len(profiles)}, LAST_SWEEP_ERROR={LAST_SWEEP_ERROR}'
    )
    return loft


def createSupportInterfaceLoft(
    component,
    startPoint,
    endPoint,
    frame,
    startSizeCm,
    endSizeCm,
    name,
    supportKind='single',
    beamEndpoint=None,
    beamOtherEndpoint=None,
    beamDiameterCm=None
):
    if startPoint.distanceTo(endPoint) < 0.000001:
        supportDebugLog(f'createSupportInterfaceLoft {name}: skipped; zero distance')
        return False

    xAxis, yAxis = supportInterfaceProfileAxes(startPoint, endPoint, frame, supportKind)
    if not xAxis or not yAxis:
        supportDebugLog(f'createSupportInterfaceLoft {name}: skipped; bad profile axes')
        return False

    try:
        supportDebugLog(
            f'createSupportInterfaceLoft {name}: begin kind={supportKind}, '
            f'start={formatPoint(startPoint)}, end={formatPoint(endPoint)}, '
            f'xAxis={formatVector(xAxis)}, yAxis={formatVector(yAxis)}, '
            f'sizes={startSizeCm:.6f}->{endSizeCm:.6f}'
        )
        if supportKind in ('single', 'double', 'straight', '90'):
            transitionPoint = supportInterfaceTransitionPoint(startPoint, endPoint, startSizeCm)
            if not transitionPoint:
                supportDebugLog(f'createSupportInterfaceLoft {name}: transition point failed')
                return False
            supportDebugLog(
                f'createSupportInterfaceLoft {name}: transition '
                f'p1={formatPoint(transitionPoint)}, '
                f'p1ToStart={transitionPoint.distanceTo(startPoint):.6f}, '
                f'p1ToTrack={transitionPoint.distanceTo(endPoint):.6f}'
            )

            trackLoft, transitionProfile = createSupportTrackSquareToCircleLoft(
                component,
                transitionPoint,
                endPoint,
                xAxis,
                yAxis,
                startSizeCm,
                endSizeCm,
                name
            )
            if not trackLoft or not transitionProfile:
                return False

            if supportKind == '90':
                embeddedStartPoint = supportConnectorEmbedPoint(startPoint, transitionPoint, startSizeCm)
                connectorFeature = createSupportBeamBody(
                    component,
                    embeddedStartPoint,
                    transitionPoint,
                    startSizeCm,
                    f'{name}_p1_circle_to_support_beam'
                )
                supportDebugLog(
                    f'createSupportInterfaceLoft {name}: 90 connector beam={connectorFeature is not None}, '
                    f'from={formatPoint(embeddedStartPoint)}, supportTop={formatPoint(startPoint)}, '
                    f'to={formatPoint(transitionPoint)}'
                )
                return connectorFeature is not None

            extensionEndPoint = supportInterfaceCircleExtensionEndPoint(transitionPoint, startPoint)
            if not extensionEndPoint:
                supportDebugLog(f'createSupportInterfaceLoft {name}: extension end point failed')
                return False

            finalBeamTarget = supportInterfaceFinalOverlapPoint(
                extensionEndPoint,
                startPoint,
                startSizeCm
            )
            extensionLoft = createSupportCircleToCircleLoft(
                component,
                transitionPoint,
                extensionEndPoint,
                xAxis,
                yAxis,
                startSizeCm,
                name,
                'p1_circle_extension'
            )
            finalLoft = createSupportCircleToCircleLoft(
                component,
                extensionEndPoint,
                finalBeamTarget,
                xAxis,
                yAxis,
                startSizeCm,
                name,
                'extension_to_beam_top',
                throughPoints=[startPoint]
            )
            supportDebugLog(
                f'createSupportInterfaceLoft {name}: staged non-90 '
                f'extensionEnd={formatPoint(extensionEndPoint)}, '
                f'beamTop={formatPoint(startPoint)}, finalBeamTarget={formatPoint(finalBeamTarget)}, '
                f'overlap={startPoint.distanceTo(finalBeamTarget):.6f}, '
                f'extensionLoft={extensionLoft is not None}, finalLoft={finalLoft is not None}'
            )
            flushSupportDebugLogBuffer()
            return extensionLoft is not None and finalLoft is not None

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
            supportDebugLog(f'createSupportInterfaceLoft {name}: plane creation failed')
            return False

        startSketch, startProfile = supportInterfaceStartProfile(
            component,
            startPlane,
            startPoint,
            xAxis,
            yAxis,
            startSizeCm,
            supportKind,
            name
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
            supportDebugLog(
                f'createSupportInterfaceLoft {name}: profile failed; '
                f'start={describeProfile(startProfile)}, end={describeProfile(endProfile)}'
            )
            return False

        loft = createLoftFromProfiles(component, [startProfile, endProfile], name)
        if loft:
            supportDebugLog(f'createSupportInterfaceLoft {name}: loft succeeded')
            return True

        supportDebugLog(f'createSupportInterfaceLoft {name}: loft failed; LAST_SWEEP_ERROR={LAST_SWEEP_ERROR}')
        return False
    except Exception as err:
        supportDebugLog(f'createSupportInterfaceLoft {name}: failed: {err}\n{traceback.format_exc()}')
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


def createSupportsFromXml(component, xmlPath, pathScaleFactor, supportTrackSamples=None, trackBodies=None):
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
    prefabOffsets = computeSupportPrefabOffsets(
        supportData,
        supportTrackSamples,
        supportScaleFactor
    )
    floorZ = computeSupportFloorZ(supportData, supportScaleFactor, prefabOffsets)

    beamCount = 0
    beamDebugRecordsByNodeId = {}
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

        beamComponent = createChildComponent(component, name)
        beamFeature = createSupportBeamBody(beamComponent, startPoint, endPoint, diameterCm, name)
        if beamFeature:
            registerSupportBeamDebugRecord(
                beamDebugRecordsByNodeId,
                beam.get('startId'),
                startPoint,
                endPoint,
                diameterCm,
                name
            )
            registerSupportBeamDebugRecord(
                beamDebugRecordsByNodeId,
                beam.get('endId'),
                endPoint,
                startPoint,
                diameterCm,
                name
            )
            trimSupportBeamBelowFloor(
                beamComponent,
                beamFeature,
                startPoint,
                endPoint,
                diameterCm,
                floorZ,
                name
            )
            beamCount += 1

    interfaceCount = 0
    if CREATE_SUPPORT_TRACK_INTERFACES and supportTrackSamples:
        for sequenceNumber, interface in enumerate(supportData['interfaces'], start=1):
            trackSample = sampleSupportTrack(
                supportTrackSamples,
                interface['centerRailsCoord'],
                supportScaleFactor,
                snapToSafe=True,
                forceSafe=True
            )
            if not trackSample:
                continue

            offset = supportOffsetForPrefab(prefabOffsets, interface['prefabIndex'])
            supportTop = supportPointMetersToFusion(interface['pointMeters'], supportScaleFactor, offset)
            frame = supportTrackFrame(trackSample)
            supportKind = supportInterfaceKind(interface)
            trackAnchor = supportTrackInterfaceAnchorForKind(trackSample, frame, supportKind, supportTop)
            baseSizeCm = interface['diameterMeters'] * supportScaleFactor * SUPPORT_BEAM_DIAMETER_SCALE
            startSizeCm = max(baseSizeCm, 0.05)
            endSizeCm = max(supportInterfaceTrackSizeCm(supportKind), 0.05)
            interfaceStart = supportInterfaceStartPoint(
                interface,
                supportTop,
                trackAnchor,
                frame,
                startSizeCm
            )
            name = f'support_interface_{sequenceNumber:03d}'
            nearestBeamRecord, nearestBeamDistance = nearestSupportBeamDebugRecord(
                beamDebugRecordsByNodeId,
                interface.get('nodeId'),
                interfaceStart
            )
            nearestBeamEndpoint = nearestBeamRecord.get('endpoint') if nearestBeamRecord else None
            nearestBeamOther = nearestBeamRecord.get('otherEndpoint') if nearestBeamRecord else None
            nearestBeamName = nearestBeamRecord.get('name') if nearestBeamRecord else None
            nearestBeamDiameter = nearestBeamRecord.get('diameterCm') if nearestBeamRecord else None

            supportDebugLog(
                f'interface {sequenceNumber}: centerRails={interface["centerRailsCoord"]:.6f}m, '
                f'kind={supportKind}, targetCm={trackSample["targetCm"]:.6f}, '
                f'originalTargetCm={trackSample.get("originalTargetCm", trackSample["targetCm"]):.6f}, '
                f'snapDeltaCm={trackSample.get("snapDeltaCm", 0.0):.6f}, '
                f'nearestCrosstieCm={trackSample.get("nearestCrosstieDistanceCm", -1.0):.6f}, '
                f'between={trackSample.get("betweenCrossties", None)}, '
                f'track={trackSample["folderName"]}[{trackSample["index"]}] '
                f't={trackSample["t"]:.6f}, supportTop={formatPoint(supportTop)}, '
                f'interfaceStart={formatPoint(interfaceStart)}, trackAnchor={formatPoint(trackAnchor)}, '
                f'gap={interfaceStart.distanceTo(trackAnchor):.6f}cm, '
                f'nearestBeam={nearestBeamName}, '
                f'nearestBeamEndpoint={formatPoint(nearestBeamEndpoint)}, '
                f'nearestBeamOther={formatPoint(nearestBeamOther)}, '
                f'nearestBeamDiameter={(nearestBeamDiameter if nearestBeamDiameter else -1.0):.6f}, '
                f'nearestBeamDistance={(nearestBeamDistance if nearestBeamDistance is not None else -1.0):.6f}, '
                f'sizes={startSizeCm:.6f}->{endSizeCm:.6f}cm, '
                f'source={interface["sourcePath"]}'
            )

            if createSupportInterfaceLoft(
                component,
                interfaceStart,
                trackAnchor,
                frame,
                startSizeCm,
                endSizeCm,
                name,
                supportKind,
                nearestBeamEndpoint,
                nearestBeamOther,
                nearestBeamDiameter
            ):
                interfaceCount += 1

    footerCount = 0
    if CREATE_SUPPORT_FOOTER_PADS:
        for sequenceNumber, footer in enumerate(supportData['footers'], start=1):
            if createSupportFooterPad(component, footer, supportScaleFactor, prefabOffsets, floorZ, sequenceNumber):
                footerCount += 1

    supportBodies = collectComponentBodies(component)
    movedBodyCount = resolveSupportTrackIntersections(
        component,
        supportBodies,
        trackBodies,
        supportTrackSamples
    )

    expectedInterfaceCount = len(supportData['interfaces']) if (CREATE_SUPPORT_TRACK_INTERFACES and supportTrackSamples) else 0
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
        'skippedCount': len(supportData['skipped']),
        'movedBodyCount': movedBodyCount
    }


# ============================================================
# MAIN
# ============================================================

def run(context):
    app = None
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

        userSelectedFolder, baseFolderPath = promptForBaseFolder(ui)
        if not userSelectedFolder:
            debugLog('run: user cancelled folder selection')
            ui.messageBox('Cancelled.')
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
            supportXmlSelected, supportXmlPath = promptForSupportXmlFile(ui)
            if supportXmlSelected and supportXmlPath:
                debugLog(f'run: selected supports XML={supportXmlPath}')
            else:
                debugLog('run: supports XML selection skipped')

        suppressViewportRefresh(app)

        identity = adsk.core.Matrix3D.create()
        topOcc = rootComponent.occurrences.addNewComponent(identity)
        topComp = adsk.fusion.Component.cast(topOcc.component)
        topComp.name = 'RollerCoaster_Tracks'

        summaryLines = []
        scaledTrackDataList = []
        allTrackBodies = []

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
            trackBodies = collectComponentBodies(trackComp)
            allTrackBodies.extend(trackBodies)
            debugLog(f'run: {folderName}: collected track bodies={len(trackBodies)}')

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
                    supportTrackSamples,
                    allTrackBodies
                )

                if supportResult['ok']:
                    supportLine = (
                        f'Supports: {supportResult["beamCount"]}/{supportResult["beamTotal"]} beams, '
                        f'{supportResult["interfaceCount"]}/{supportResult["interfaceTotal"]} interfaces, '
                        f'{supportResult["footerCount"]}/{supportResult["footerTotal"]} footers, '
                        f'{supportResult["prefabCount"]} prefabs, '
                        f'{supportResult.get("movedBodyCount", 0)} moved supports'
                    )
                else:
                    supportLine = (
                        f'Supports failed: {supportResult["error"]} | '
                        f'{supportResult["beamCount"]}/{supportResult["beamTotal"]} beams, '
                        f'{supportResult["interfaceCount"]}/{supportResult["interfaceTotal"]} interfaces, '
                        f'{supportResult["footerCount"]}/{supportResult["footerTotal"]} footers, '
                        f'{supportResult.get("movedBodyCount", 0)} moved supports'
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

        restoreViewportRefresh(app)
        flushDebugLogs()

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
        restoreViewportRefresh(app)
        flushDebugLogs()
        if ui:
            ui.messageBox('Fatal error:\n' + traceback.format_exc())
    finally:
        restoreViewportRefresh(app)
        flushDebugLogs()
