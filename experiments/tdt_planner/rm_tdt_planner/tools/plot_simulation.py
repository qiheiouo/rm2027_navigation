#!/usr/bin/env python3
"""Export a static SVG for manual review; no ROS or plotting package required."""
import argparse
from html import escape
import json
import math
from pathlib import Path
from simulation_geometry import OBSTACLES


def rows(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('observation_directory', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    trajectory = rows(args.observation_directory/'trajectory.jsonl')
    plans = rows(args.observation_directory/'plans.jsonl')
    if not trajectory:
        parser.error('no trajectory samples; cannot plot a navigation result')
    points = [(s['x'], s['y']) for s in trajectory] + [(4.3,0.0)]
    for x0, y0, x1, y1 in OBSTACLES:
        points.extend(((x0,y0),(x1,y1)))
    for plan in plans:
        points.extend(plan['xy'])
    if not all(math.isfinite(v) for point in points for v in point):
        parser.error('non-finite plot input')
    xmin, xmax = min(p[0] for p in points)-.6, max(p[0] for p in points)+.6
    ymin, ymax = min(p[1] for p in points)-.6, max(p[1] for p in points)+.6
    scale = min(880/(xmax-xmin), 580/(ymax-ymin))
    def point(x, y):
        return f'{60+(x-xmin)*scale:.3f},{100+(ymax-y)*scale:.3f}'
    def polyline(path, color, width, opacity=1):
        return f'<polyline points="{" ".join(point(*p) for p in path)}" fill="none" stroke="{color}" stroke-width="{width}" opacity="{opacity}"/>'
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="750" viewBox="0 0 1000 750">',
           '<rect width="1000" height="750" fill="white"/>',
           '<text x="40" y="28" font-size="20">P2B static simulation: recorded plans and poses</text>',
           f'<text x="40" y="50" font-size="12">{escape(str(args.observation_directory))}</text>',
           '<text x="40" y="72" font-size="13">Gray: all plans; blue: poses; green: sampled padded footprint; black: fixture obstacles</text>']
    for x0,y0,x1,y1 in OBSTACLES:
        svg.append(f'<polygon points="{point(x0,y0)} {point(x1,y0)} {point(x1,y1)} {point(x0,y1)}" fill="#444"/>')
    for plan in plans:
        svg.append(polyline(plan['xy'], '#999', 1, .45))
    svg.append(polyline([(s['x'],s['y']) for s in trajectory], '#1765c1', 2.5))
    for row in trajectory[::max(1,len(trajectory)//20)]:
        c,s = math.cos(row['yaw']),math.sin(row['yaw'])
        outline = [(row['x']+c*x-s*y,row['y']+s*x+c*y) for x,y in
                   ((-.33,-.28),(.33,-.28),(.33,.28),(-.33,.28),(-.33,-.28))]
        svg.append(polyline(outline, '#168055', 1, .6))
    for label,x,y in [('start',trajectory[0]['x'],trajectory[0]['y']),('goal',4.3,0.0)]:
        px,py=point(x,y).split(',')
        svg.append(f'<circle cx="{px}" cy="{py}" r="4" fill="#bb2233"/>')
        svg.append(f'<text x="{float(px)+8}" y="{py}" font-size="12">{label}</text>')
    svg.append('<text x="40" y="725" font-size="12">Visualization of recorded samples; inspect JSON evidence and timing gaps before acceptance.</text></svg>')
    with args.output.open('x') as stream:
        stream.write('\n'.join(svg)+'\n')
    print(args.output)


if __name__ == '__main__':
    main()
