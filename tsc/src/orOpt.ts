import { loadPaper } from './paperLoader';
import { updateStatusFn } from './types';

const paper = loadPaper();

// Or-opt: a local-search pass that runs after optimizer.ts's greedy nearest-neighbor
// ordering. It repeatedly tries relocating short chains of 1-3 consecutive paths to a
// different position in the draw order, keeping the move only if it shortens total
// pen-up travel distance. Unlike 2-opt, it never reverses a path's own drawn direction -
// only the *order* paths are visited in - so it can't change what ends up on the wall,
// only how long it takes to get there.
//
// Greedy nearest-neighbor construction is typically within ~25% of optimal travel
// distance; Or-opt on top of it typically closes a good chunk of that gap for
// comparatively little compute, since each candidate move is O(1) to evaluate (only the
// edges touching the moved chain change) rather than the O(chain length) cost a
// direction-preserving block-reversal (2-opt) move would need here.

const MAX_CHAIN_LENGTH = 3;
const MAX_PASSES = 15;
const MAX_MILLISECONDS = 4000; // safety budget - bail out with whatever improvement was found so far
const EPSILON = 1e-6;

function pathStart(p: paper.Path): paper.Point {
    return p.firstSegment.point;
}

function pathEnd(p: paper.Path): paper.Point {
    return p.lastSegment.point;
}

export function orOptimizePaths(paths: paper.Path[], startX: number, startY: number, updateStatusFn: updateStatusFn): paper.Path[] {
    if (paths.length < 4) {
        return paths; // not enough paths for a relocation to ever help
    }

    updateStatusFn("Reducing travel distance");

    let tour = [...paths];
    const startPoint = new paper.Point(startX, startY);
    const deadline = Date.now() + MAX_MILLISECONDS;

    function pointBefore(t: paper.Path[], index: number): paper.Point {
        return index === 0 ? startPoint : pathEnd(t[index - 1]);
    }

    let improved = true;
    let pass = 0;

    while (improved && pass < MAX_PASSES && Date.now() < deadline) {
        improved = false;
        pass++;

        for (let chainLength = 1; chainLength <= MAX_CHAIN_LENGTH; chainLength++) {
            for (let i = 0; i + chainLength <= tour.length; i++) {
                if (Date.now() >= deadline) {
                    break;
                }

                const chain = tour.slice(i, i + chainLength);
                const chainStart = pathStart(chain[0]);
                const chainEnd = pathEnd(chain[chainLength - 1]);

                const beforeChain = pointBefore(tour, i);
                const afterIndex = i + chainLength;
                const afterChain = afterIndex < tour.length ? pathStart(tour[afterIndex]) : null;

                // How much total distance shrinks by lifting this chain out and closing the gap.
                const removedCost = beforeChain.getDistance(chainStart) + (afterChain ? chainEnd.getDistance(afterChain) : 0);
                const bridgeCost = afterChain ? beforeChain.getDistance(afterChain) : 0;
                const removalGain = removedCost - bridgeCost;

                if (removalGain <= EPSILON) {
                    continue; // even a free re-insertion wouldn't beat leaving it in place
                }

                const remainingTour = tour.slice(0, i).concat(tour.slice(i + chainLength));

                let bestInsertionIndex = -1;
                let bestDelta = -EPSILON; // only accept strict improvements

                for (let k = 0; k <= remainingTour.length; k++) {
                    const before = pointBefore(remainingTour, k);
                    const after = k < remainingTour.length ? pathStart(remainingTour[k]) : null;

                    const oldBridge = after ? before.getDistance(after) : 0;
                    const insertionCost = before.getDistance(chainStart) + (after ? chainEnd.getDistance(after) : 0) - oldBridge;

                    const delta = insertionCost - removalGain;
                    if (delta < bestDelta) {
                        bestDelta = delta;
                        bestInsertionIndex = k;
                    }
                }

                if (bestInsertionIndex >= 0) {
                    tour = remainingTour.slice(0, bestInsertionIndex)
                        .concat(chain)
                        .concat(remainingTour.slice(bestInsertionIndex));
                    improved = true;
                }
            }
        }
    }

    return tour;
}
