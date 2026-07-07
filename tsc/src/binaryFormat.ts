import { Command } from './types';
import { MAX_SPEED_MM_PER_SEC } from './velocityPlanner';

// Binary job format (little-endian throughout; matches ESP32 native byte order,
// so the firmware can read fields directly with no byte-swapping):
//
// Header (9 bytes):
//   offset 0: uint8   formatVersion   (currently 3)
//   offset 1: float32 totalDistance   (mm)
//   offset 5: float32 height          (mm, drawing height, informational only)
//
// Records (repeated until EOF):
//   opcode 0x00: pen up                                          -> 1 byte total
//   opcode 0x01: pen down                                         -> 1 byte total
//   opcode 0x02: move, followed by int32 x, int32 y, uint16 speed -> 11 bytes total
//              (x, y fixed-point per COORDINATE_SCALE; speed fixed-point per SPEED_SCALE)
//
// Coordinates are fixed-point rather than float32: exact decimal values with no
// IEEE-754 rounding, at the cost of a fixed maximum resolution (see COORDINATE_SCALE).
//
// Format version bumped 2 -> 3 to add the per-move speed field (velocityPlanner.ts) -
// old firmware will reject files in this format (and vice versa) via the version byte
// check in initTaskProvider().

export const FORMAT_VERSION = 3;

export const OPCODE_PEN_UP = 0x00;
export const OPCODE_PEN_DOWN = 0x01;
export const OPCODE_MOVE = 0x02;

// Fixed-point scale for move coordinates: 1 unit = 1/COORDINATE_SCALE mm.
// At 100, that's 0.01mm resolution with a range of ±21,474,836mm (int32) -
// vastly more than a wall plotter needs in either precision or range, and it
// still leaves headroom versus float32's ~0.0003mm precision at these magnitudes.
export const COORDINATE_SCALE = 100;

// Fixed-point scale for the per-move speed field: 1 unit = 1/SPEED_SCALE mm/s.
// At 10, that's 0.1mm/s resolution with a uint16 range of 0-6553.5mm/s - far beyond
// MAX_SPEED_MM_PER_SEC, so there's no risk of overflow from any value the planner
// can actually produce.
export const SPEED_SCALE = 10;

const HEADER_SIZE = 9;
const PEN_RECORD_SIZE = 1;
const MOVE_RECORD_SIZE = 11;

function toFixedPoint(mm: number): number {
    return Math.round(mm * COORDINATE_SCALE);
}

function speedToFixedPoint(speedMmPerSec: number): number {
    const clamped = Math.max(0, Math.min(speedMmPerSec, MAX_SPEED_MM_PER_SEC));
    return Math.round(clamped * SPEED_SCALE);
}

// `commands` is expected to be the output of velocityPlanner.ts's planVelocities(),
// applied to the deduped command list produced in toCommands.ts. Any 'd...'/'h...'
// marker strings in it are ignored here since totalDistance and height are passed in
// explicitly and written into the binary header instead.
export function encodeCommandsBinary(commands: Command[], totalDistance: number, height: number): ArrayBuffer {
    let size = HEADER_SIZE;
    for (const cmd of commands) {
        if (typeof cmd === 'string') {
            if (cmd === 'p0' || cmd === 'p1') {
                size += PEN_RECORD_SIZE;
            }
            // any other string command (d.../h...) is a header marker, not a record - skip
        } else {
            size += MOVE_RECORD_SIZE;
        }
    }

    const buffer = new ArrayBuffer(size);
    const view = new DataView(buffer);
    let offset = 0;

    view.setUint8(offset, FORMAT_VERSION);
    offset += 1;

    view.setFloat32(offset, totalDistance, true);
    offset += 4;

    view.setFloat32(offset, height, true);
    offset += 4;

    for (const cmd of commands) {
        if (typeof cmd === 'string') {
            if (cmd === 'p0') {
                view.setUint8(offset, OPCODE_PEN_UP);
                offset += 1;
            } else if (cmd === 'p1') {
                view.setUint8(offset, OPCODE_PEN_DOWN);
                offset += 1;
            }
        } else {
            view.setUint8(offset, OPCODE_MOVE);
            offset += 1;
            view.setInt32(offset, toFixedPoint(cmd.x), true);
            offset += 4;
            view.setInt32(offset, toFixedPoint(cmd.y), true);
            offset += 4;
            // cmd.speed is optional on the type (see types.ts) for callers that never run
            // it through the planner; fall back to max speed rather than silently emitting 0
            // and stalling the job.
            view.setUint16(offset, speedToFixedPoint(cmd.speed ?? MAX_SPEED_MM_PER_SEC), true);
            offset += 2;
        }
    }

    return buffer;
}

