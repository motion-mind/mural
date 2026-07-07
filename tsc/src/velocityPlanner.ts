import { Command, CoordinateCommand } from './types';

// Computes a per-point target pen speed (mm/s) across the WHOLE job, treating every
// coordinate command (pen up travel moves included, not just pen-down strokes) as one
// continuous polyline. This never reorders, adds, or removes points, and never changes
// any point's (x, y) - it only annotates a `speed` value onto each CoordinateCommand.
//
// Units are physical (mm/s, mm/s^2) rather than steps/sec, matching how positions are
// already sent in mm - the firmware's step/mm calibration constants stay entirely
// firmware-side, exactly as they do for position today.
//
// IMPORTANT, and easy to get wrong (I got it wrong on the first pass of this): this does
// NOT need to force speed to 0 at every pen up/down transition. Tracing AccelStepper's
// actual runSpeed()/setSpeed() implementation shows there is no persisted momentum
// between separate move commands - every discrete move already ends in a genuine full
// stop before the next one begins, regardless of what speed either one used, because
// there's a real tick where distanceToGo() hits 0 and no further steps are issued until
// the next moveTo()/setSpeed() call. So pen transitions are already "at rest" today, for
// free, by construction - what this planner actually changes is the SPEED (and therefore
// the mechanical shock) at which those unavoidable stops happen: low near sharp corners
// and near the true start/end of the whole job, higher through long straight runs -
// rather than every single move using the same fixed speed regardless of what's around
// it. Only the very first and very last point of the entire job need to be pinned to a
// true standing start/stop; treating every other point (including ones adjacent to a
// pen up/down marker) as just another point on the polyline is both simpler and correct.

// Starting points, not verified values - see MURAL_FORK_README.md. MAX_SPEED_MM_PER_SEC
// is set close to the ~12.5mm/s the firmware's old hardcoded printSpeedSteps constant
// already implied (500 steps/sec at the existing pulley circumference/microstepping),
// so cruise speed on straight runs stays roughly comparable to current behavior.
// MAX_ACCEL_MM_PER_SEC2 has no existing equivalent to anchor to - today's firmware
// effectively uses a new, independent constant-speed command for every ~1mm segment
// with no ramp at all - so this is a genuine guess pending real-hardware tuning.
export const MAX_SPEED_MM_PER_SEC = 15;
export const MAX_ACCEL_MM_PER_SEC2 = 100;

// Cornering speed limit: at each interior point of the polyline, the allowed speed is
// scaled by a factor in [0, 1] based on the angle between the incoming and outgoing
// direction - 1.0 for continuing straight, 0.0 for a full reversal. CORNER_FACTOR_POWER
// controls how sharply that factor falls off approaching a tight corner (higher = more
// tolerant of gentle bends, more aggressive slowdown for sharp ones).
const CORNER_FACTOR_POWER = 2;
const MIN_SEGMENT_DISTANCE = 1e-6; // guards against divide-by-zero on duplicate points

interface Point {
    x: number;
    y: number;
}

function distance(a: Point, b: Point): number {
    return Math.hypot(b.x - a.x, b.y - a.y);
}

function isCoordinateCommand(cmd: Command): cmd is CoordinateCommand {
    return typeof cmd !== 'string';
}

// Speed achievable after accelerating from `fromSpeed` over `dist` at MAX_ACCEL_MM_PER_SEC2.
function speedAfterAccelerating(fromSpeed: number, dist: number): number {
    return Math.sqrt(fromSpeed * fromSpeed + 2 * MAX_ACCEL_MM_PER_SEC2 * dist);
}

function cornerSpeedFactor(prev: Point, current: Point, next: Point): number {
    const d1x = current.x - prev.x, d1y = current.y - prev.y;
    const d2x = next.x - current.x, d2y = next.y - current.y;
    const len1 = Math.hypot(d1x, d1y), len2 = Math.hypot(d2x, d2y);
    if (len1 < MIN_SEGMENT_DISTANCE || len2 < MIN_SEGMENT_DISTANCE) {
        return 1; // degenerate segment - don't let it force an artificial slowdown
    }
    const cosTheta = (d1x * d2x + d1y * d2y) / (len1 * len2); // -1 (reversal) .. 1 (straight)
    const factor = Math.max(0, Math.min(1, (cosTheta + 1) / 2));
    return Math.pow(factor, CORNER_FACTOR_POWER);
}

// speeds[i] is the target constant speed for the move that ARRIVES at points[i]; speeds[0]
// is a placeholder (nothing moves "to" the very first point of the job - it's wherever the
// bot already is when planning starts).
function planSpeedsForPolyline(points: Point[]): number[] {
    const n = points.length;
    if (n === 0) {
        return [];
    }
    if (n === 1) {
        return [0];
    }

    const segmentDistances: number[] = [];
    for (let i = 0; i < n - 1; i++) {
        segmentDistances.push(Math.max(distance(points[i], points[i + 1]), MIN_SEGMENT_DISTANCE));
    }

    const cornerLimits: number[] = new Array(n).fill(MAX_SPEED_MM_PER_SEC);
    for (let i = 1; i < n - 1; i++) {
        cornerLimits[i] = MAX_SPEED_MM_PER_SEC * cornerSpeedFactor(points[i - 1], points[i], points[i + 1]);
    }

    // Forward pass: accelerate as fast as possible from a standing start at the very
    // first point, capped by cruise speed and each point's cornering limit.
    const forward: number[] = new Array(n);
    forward[0] = 0;
    for (let i = 1; i < n; i++) {
        const reachable = speedAfterAccelerating(forward[i - 1], segmentDistances[i - 1]);
        forward[i] = Math.min(reachable, MAX_SPEED_MM_PER_SEC, cornerLimits[i]);
    }

    // Backward pass: cap every point so decelerating to a standing stop at the very last
    // point is physically achievable, without exceeding what the forward pass allows.
    const speeds: number[] = new Array(n);
    speeds[n - 1] = 0;
    for (let i = n - 2; i >= 0; i--) {
        const reachable = speedAfterAccelerating(speeds[i + 1], segmentDistances[i]);
        speeds[i] = Math.min(forward[i], reachable);
    }

    return speeds;
}

export function planVelocities(commands: Command[]): Command[] {
    const coordIndices: number[] = [];
    for (let i = 0; i < commands.length; i++) {
        if (isCoordinateCommand(commands[i])) {
            coordIndices.push(i);
        }
    }

    if (coordIndices.length === 0) {
        return commands;
    }

    const points = coordIndices.map(i => commands[i] as CoordinateCommand);
    const speeds = planSpeedsForPolyline(points);

    const result = [...commands];
    for (let k = 0; k < coordIndices.length; k++) {
        const original = points[k];
        result[coordIndices[k]] = { x: original.x, y: original.y, speed: speeds[k] };
    }
    return result;
}

