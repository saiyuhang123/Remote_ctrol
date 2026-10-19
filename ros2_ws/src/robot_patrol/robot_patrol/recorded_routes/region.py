"""Shared polygon predicate; boundary counts as inside."""
def inside_polygon(x, y, polygon):
    inside = False
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        ax, ay = a
        bx, by = b
        cross = (x-ax)*(by-ay)-(y-ay)*(bx-ax)
        if abs(cross) < 1e-9 and min(ax,bx)-1e-9 <= x <= max(ax,bx)+1e-9 and min(ay,by)-1e-9 <= y <= max(ay,by)+1e-9:
            return True
        if (ay > y) != (by > y) and x < (bx-ax)*(y-ay)/(by-ay)+ax:
            inside = not inside
    return inside
