import math
import cadquery as cq
from datetime import datetime

WIDTH = 140
HEIGHT = 75
LEDS = 65
WALL_TH = 3
STAND_W = 7
WITH_LABELS = True
TOL = 0.5


def gen_slot_data(w, h, nh, a=None):
    assert(nh > 1)
    assert((nh % 2) == 0)

    a_rad = math.atan(h / w)
    if a:
        ac = math.tan(math.radians(a - 90))
    else:
        ac = - w / h
    dy = h / nh
    dx = dy / math.tan(a_rad)

    p = (dx / 2, dy / 2)
    pts = []

    while True:
        if (p[0] <= w / 2) or (p[1] <= h / 2):
            pts.append(p)
            pts.append((-p[0], -p[1]))
            p = (p[0] + dx, p[1] + dy)
        else:
            break

    assert(len(pts) == nh)

    pts.sort()

    def find_on_x(pt, x):
        return (x, ac * (x - pt[0]) + pt[1])

    def find_on_y(pt, y):
        return (pt[0] + ((y - pt[1]) / ac), y)

    def width(pt):
        ps = [find_on_x(pt, w / 2), find_on_x(pt, -w / 2),
              find_on_y(pt, h / 2), find_on_y(pt, -h / 2)]

        ps = list(filter(lambda p: (p[0] <= w / 2) and (p[1] <= h / 2)
                         and (p[0] >= -w / 2) and (p[1] >= -h / 2), ps))

        d = math.dist(ps[0], ps[1])
        return (((ps[0][0] + ps[1][0]) / 2), ((ps[0][1] + ps[1][1]) / 2)), d

    wd = (math.hypot(dx, dy) / 2)
    if a:
        wd *= math.cos(math.radians(a) - a_rad)

    return a if a else math.degrees(a_rad), wd, map(width, pts)


def slots(w, h, n, a=None):
    sk = cq.Sketch()
    a, sw, data = gen_slot_data(w, h, n, a)

    for c, w in data:
        sk = sk.push([c]).slot(w, sw, a+90)

    return sk


def screw_shape(l):
    assert(l >= 4)
    a = cq.Workplane("XY")\
        .circle(1.5).extrude(l - 3)\
        .faces(">Z")\
        .circle(1.5 + (TOL / 2)).extrude(3)
    return a


def standoffs():
    pcb_w = 70
    pcb_h = 50
    h = 6

    s = screw_shape(h - 2).translate((0, 0, 2))
    a = cq.Workplane("XY")\
        .circle(6 / 2).extrude(h).faces("<Z")\
        .cut(s).translate((pcb_w / 2 - 2, pcb_h / 2 - 2))\
        .mirror("XZ", union=True)\
        .mirror("YZ", union=True)
    return a


def grating(w, h, num, th=WALL_TH):
    outline = (
        cq.Sketch()
        .trapezoid(w, h, 90)
        .vertices()
        .circle(w / 10, mode='s')
        .reset()
        .vertices()
        .fillet(1)
    )

    dw = (w - (w / 3)) / (num - 1)
    dh = h - (1.5 * w / 5)
    cut = (
        cq.Sketch()
        .rarray(dw, 1, num, 1)
        .slot(dh, dw / 2, mode='a', angle=90)
    )
    return cq.Workplane().workplane(offset=-th / 4).box(w, h, th / 2)\
        .faces('>Z').workplane()\
        .placeSketch(outline).extrude(th / 2)\
        .faces('>Z').edges().fillet(0.3)\
        .faces('>Z').workplane()\
        .placeSketch(cut).cutThruAll()


def base_shape(h):
    return cq.Workplane("XY").box(WIDTH, HEIGHT, h)\
        .edges("|Z").fillet(2)


def section_1(h):
    return base_shape(h).cut(cq.Workplane("XY")
                             .box(2 * LEDS + 2 * TOL,
                                  LEDS + 2 * TOL, h))


def section_2(h):
    wx = 5
    wy = 9
    dx = LEDS - 8
    dy = LEDS / 2 - (wy / 2)

    notch = cq.Workplane("XY").box(5, 9, h)
    a = notch.translate((dx, - dy, 0))
    b = notch.translate((LEDS - dx, dy, 0))
    a = a.union(b)
    a = a.union(a.translate((-LEDS, 0, 0)))
    a = base_shape(h).cut(a)
    a = a.union(standoffs().translate((0, 0, h / 2)))
    return a


def mid_section(th, w=WIDTH, h=HEIGHT):
    base = base_shape(th)
    base = base.cut(cq.Workplane("XY")
                    .box(w - 2 * WALL_TH, h - 2 * WALL_TH, th))
    dx = (w - 2 * WALL_TH - STAND_W) / 2
    dy = (h - 2 * WALL_TH - STAND_W) / 2
    a = cq.Workplane("XY").box(STAND_W, STAND_W, th)\
        .edges("|Z and <X and <Y").fillet(1)
    a = a.translate((dx, dy))
    a = a.mirror("XZ", union=True)
    a = a.mirror("YZ", union=True)
    return base.union(a).edges("|Z").fillet(0.5)


def section_3(h):
    dx = (WIDTH - WALL_TH - STAND_W) / 2
    dy = (HEIGHT - WALL_TH - STAND_W) / 2
    sl = 12
    a = screw_shape(sl).translate((dx, dy, h / 2 - sl))
    a = a.union(a.mirror("XZ"))
    a = a.union(a.mirror("YZ"))
    a = mid_section(h).faces(">Z").cut(a)
    a = a.faces(">X").workplane().move(-HEIGHT / 4, 0)\
        .hole(11 + 2 * TOL, WALL_TH)
    if WITH_LABELS:
        a = a.transformed(offset=cq.Vector(-HEIGHT / 4 - 11, 0, 0),
                          rotate=cq.Vector(0, 0, -90))\
            .text("5V/~3-4A", 4, -1, font="Ubuntu Mono",
                  kind="bold")
    a = a.faces(">Y").workplane(centerOption="CenterOfMass")\
        .move(-WIDTH / 3, 0)\
        .hole(9.5 + 2 * TOL, WALL_TH)
    (disp_w, disp_h) = (38 + 2 * TOL, 12 + 2 * TOL)
    disp = cq.Workplane("XZ").box(disp_w, disp_h, 3)\
        .faces(">Y").workplane(centerOption="CenterOfMass")\
        .move(-(6-5), 0).rect(26, 12)\
        .extrude(1).faces(">Y").rect(26, 12)\
        .workplane(centerOption="CenterOfMass", offset=10)\
        .rect(26 * 4.5, 12 * 1.8).loft(combine=True)\
        .translate((0, 1.5, 0))

    box = cq.Workplane("XZ").box(disp_w + 3, disp_h + 3, 5)\
        .translate((0, 2.5)).edges("|Z or |X").fillet(0.5)
    a = a.cut(box.translate((0, HEIGHT / 2 - WALL_TH)))
    window = box.cut(disp).translate((0, HEIGHT / 2 - WALL_TH))
    gv = (-WIDTH / 4 - 8, HEIGHT / 2 - WALL_TH / 2, 0)
    g = grating(22 + TOL, 22 + TOL, 8, WALL_TH)\
        .rotateAboutCenter((1, 0, 0), -90)\
        .translate(gv).translate((0, WALL_TH / 2, 0))
    a = a.cut(cq.Workplane('XZ').box(22 + TOL, 22 + TOL, WALL_TH)
              .translate(gv))
    return a.union(window).union(g)


def section_4(h):
    a = base_shape(h).cut(
        mid_section(h, WIDTH - 2 * TOL, HEIGHT - 2 * TOL))
    return a


def section_5(h):
    assert(h >= 3)
    dx = WIDTH - WALL_TH - STAND_W
    dy = HEIGHT - WALL_TH - STAND_W
    a = base_shape(h)
    return a.faces(">Z").workplane() \
        .rect(dx, dy, forConstruction=True)\
        .vertices()\
        .cboreHole(3, 5, 2)


def process(parts):
    offset = parts[0][0]
    p1 = parts[0][1](parts[0][0])\
        .translate((0, 0, parts[0][0] / 2))

    for h, p in parts[1:]:
        a = p(h).translate((0, 0, offset + h / 2))
        p1 = p1.union(a)
        offset += h

    return p1


box = [(7, section_1), (3, section_2), (40, section_3)]
cover = [(1, section_4), (3, section_5)]

box = process(box).edges(">X and <Z").fillet(0.5)
cover = process(cover).edges(">X and >Z").fillet(0.5)
box = box.faces(">Z[2]").edges("(>X or >Y or <X or <Y)").fillet(0.4)

sl = slots((WIDTH / 2) - 20, HEIGHT - 25, 14, 45)
cover = cover.faces(">Z").workplane().pushPoints(
    [(-WIDTH / 4 + 2, 0), (WIDTH / 4 - 2, 0)])\
    .placeSketch(sl).cutThruAll()

cover = cover.rotateAboutCenter((1, 0, 0), 180)\
    .translate((0, 100))

cq.exporters.export(box.union(cover), 'enclosure.svg')
cq.exporters.export(box.union(cover), 'enclosure.stl')
cq.exporters.export(cover.section(height=-2.5), 'cover_section.dxf')
cq.exporters.export(base_shape(5).section(), 'front_section.dxf')
