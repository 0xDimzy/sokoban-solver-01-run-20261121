// Push-optimal Sokoban solver for hallway-and-rooms levels.
//
// Strategy: the level decomposes into independent rooms off a one-wide hallway
// (a box pushed through a door into the hallway can never come back, so every
// box stays in its room and the level optimum is the sum of the rooms'
// optima). Each room is solved push-optimally with A* over pushes:
//   - state = sorted box squares + the player's reachable region (named by
//     its lowest square), so walking between pushes is free in the search;
//   - heuristic = minimum-cost matching of boxes to goals over precomputed
//     single-box push distances (admissible: ignores the other boxes);
//   - pruning = dead squares (no lone box can be pushed from there to any
//     goal) and frozen 2x2 blocks of walls/boxes with a box off its goal.
// If the room decomposition does not apply, the same A* runs on the whole
// level. Work is budgeted by expansions, never by the clock, and every tie
// breaks by square order, so two runs write identical bytes.

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Expansion budgets (work, not time). Rooms in this generator are small
// (<= 4 boxes in a 6x6 slot); A* with the matching heuristic needs far fewer
// expansions than these caps on solvable rooms.
const ROOM_EXPANSIONS = 1_500_000;
const LEVEL_EXPANSIONS = 1_500_000;
const TOTAL_EXPANSIONS = 6_000_000;
let totalUsed = 0;

const DIRS: [number, number, string, string][] = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

type Level = {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
};

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x]!;
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++) if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

/** Squares from which a lone box can be pushed onto some goal (pulled back from the goals). */
function liveSquares(w: number, wall: Uint8Array, goals: number[]): Uint8Array {
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi]!;
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

/**
 * Single-box push distances: dist[gi * size + s] = fewest pushes to bring a
 * lone box from square s onto goal gi (other boxes ignored; the square the
 * player must stand on only has to be non-wall). -1 when impossible.
 */
function pushDistances(w: number, wall: Uint8Array, goals: number[]): Int32Array {
  const size = wall.length;
  const dist = new Int32Array(goals.length * size).fill(-1);
  goals.forEach((g, gi) => {
    const base = gi * size;
    const queue = [g];
    dist[base + g] = 0;
    for (let qi = 0; qi < queue.length; qi++) {
      const c = queue[qi]!;
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const p = c - d;
        if (!wall[p] && !wall[p - d] && dist[base + p] < 0) {
          dist[base + p] = dist[base + c]! + 1;
          queue.push(p);
        }
      }
    }
  });
  return dist;
}

/** Minimum-cost perfect matching (Hungarian, O(n^3)) on a square cost matrix; Infinity if impossible. */
function matchingCost(cost: number[][]): number {
  const n = cost.length;
  if (n === 0) return 0;
  const u = new Array<number>(n + 1).fill(0);
  const v = new Array<number>(n + 1).fill(0);
  const p = new Array<number>(n + 1).fill(0); // p[j] = row assigned to column j (1-based)
  const way = new Array<number>(n + 1).fill(0);
  for (let i = 1; i <= n; i++) {
    p[0] = i;
    let j0 = 0;
    const minv = new Array<number>(n + 1).fill(Infinity);
    const used = new Array<boolean>(n + 1).fill(false);
    do {
      used[j0] = true;
      const i0 = p[j0]!;
      let delta = Infinity;
      let j1 = 0;
      for (let j = 1; j <= n; j++) {
        if (used[j]) continue;
        const cur = cost[i0 - 1]![j - 1]! - u[i0]! - v[j]!;
        if (cur < minv[j]!) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j]! < delta) {
          delta = minv[j]!;
          j1 = j;
        }
      }
      for (let j = 0; j <= n; j++) {
        if (used[j]) {
          u[p[j]!] = u[p[j]!]! + delta;
          v[j] = v[j]! - delta;
        } else {
          minv[j] = minv[j]! - delta;
        }
      }
      j0 = j1;
    } while (p[j0] !== 0);
    do {
      const j1 = way[j0]!;
      p[j0] = p[j1]!;
      j0 = j1;
    } while (j0 !== 0);
  }
  let total = 0;
  for (let j = 1; j <= n; j++) {
    if (p[j] === 0) return Infinity;
    const c = cost[p[j]! - 1]![j - 1]!;
    if (!Number.isFinite(c)) return Infinity;
    total += c;
  }
  return total;
}

/** Flood fill the player's reachable region; marks `seen` with `stamp`, returns the lowest square. */
function reach(
  w: number,
  wall: Uint8Array,
  occupied: Uint8Array,
  from: number,
  seen: Int32Array,
  stamp: number,
  stack: Int32Array,
): number {
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n]!;
    if (p < low) low = p;
    for (let k = 0; k < 4; k++) {
      const q = k === 0 ? p - w : k === 1 ? p + w : k === 2 ? p - 1 : p + 1;
      if (wall[q] === 0 && occupied[q] === 0 && seen[q] !== stamp) {
        seen[q] = stamp;
        stack[n++] = q;
      }
    }
  }
  return low;
}

/** Whether the box just pushed to `t` sits in a frozen 2x2 block of walls/boxes with a box off its goal. */
function frozenBlock(w: number, wall: Uint8Array, occupied: Uint8Array, goal: Uint8Array, t: number): boolean {
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) || !(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) || (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

/** Shortest walk from `from` to `to` around walls and boxes, in lower LURD letters. */
function walkTo(w: number, wall: Uint8Array, occupied: Uint8Array, from: number, to: number): string {
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array<string>(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi]!;
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  let s = '';
  for (let p = to; p !== from; p = prev[p]!) s = how[p]! + s;
  return s;
}

type Solved = { pushes: number; moves: string };

/**
 * Push-optimal A* over pushes for one (sub-)level. Returns null when the
 * expansion budget runs out or the level is unsolvable.
 */
function solvePushOptimal(level: Level, maxExpansions: number): Solved | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(w, wall, goals);
  const dist = pushDistances(w, wall, goals);
  const nGoals = goals.length;

  const BIG = 1_000_000;
  const heuristic = (boxes: number[]): number => {
    const cost: number[][] = boxes.map((b) => {
      const row: number[] = [];
      for (let gi = 0; gi < nGoals; gi++) {
        const d = dist[gi * size + b]!;
        row.push(d < 0 ? BIG : d);
      }
      return row;
    });
    const total = matchingCost(cost);
    return total >= BIG ? Infinity : total;
  };

  if (level.boxes.some((b) => !live[b])) return null;
  const h0 = heuristic(level.boxes);
  if (!Number.isFinite(h0)) return null;

  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b]);

  // State storage.
  const stBoxes: number[][] = [level.boxes];
  const stPlayer: number[] = [level.player];
  const stG: number[] = [0];
  const stParent: number[] = [-1];
  const stPushFrom: number[] = [-1]; // box square before the push that made this state
  const stPushDir: number[] = [-1]; // direction index of that push

  const keyOf = (boxes: number[], low: number): string => {
    let k = String(low);
    for (const b of boxes) k += ',' + b;
    return k;
  };

  const best = new Map<string, number>();
  for (const b of level.boxes) occupied[b] = 1;
  best.set(keyOf(level.boxes, reach(w, wall, occupied, level.player, seen, ++stamp, stack)), 0);
  for (const b of level.boxes) occupied[b] = 0;

  if (solved(level.boxes)) return { pushes: 0, moves: '' };

  // Heap of [f, -g, seq, stateIndex]; ties break toward deeper states, then insertion order.
  type Entry = [number, number, number, number];
  const heap: Entry[] = [[h0, 0, 0, 0]];
  let seq = 1;
  const less = (a: Entry, b: Entry): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const hpush = (e: Entry): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const hpop = (): Entry => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  let found = -1;
  let expansions = 0;
  while (heap.length > 0 && found < 0) {
    const [, , , s] = hpop();
    const boxes = stBoxes[s]!;
    const g = stG[s]!;
    // Skip stale heap entries superseded by a cheaper path to the same state.
    for (const b of boxes) occupied[b] = 1;
    const lowHere = reach(w, wall, occupied, stPlayer[s]!, seen, ++stamp, stack);
    const here = stamp;
    if (best.get(keyOf(boxes, lowHere)) !== s) {
      for (const b of boxes) occupied[b] = 0;
      continue;
    }
    if (expansions >= maxExpansions || totalUsed >= TOTAL_EXPANSIONS) {
      for (const b of boxes) occupied[b] = 0;
      return null;
    }
    expansions++;
    totalUsed++;

    for (let bi = 0; bi < boxes.length && found < 0; bi++) {
      const b = boxes[bi]!;
      for (let di = 0; di < 4; di++) {
        const [dx, dy] = DIRS[di]!;
        const d = dy * w + dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(w, wall, occupied, goal, t);
        let next: number[] | null = null;
        let key = '';
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          key = keyOf(next, reach(w, wall, occupied, b, mark, ++stamp, stack));
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || next === null) continue;
        const prevIdx = best.get(key);
        if (prevIdx !== undefined && stG[prevIdx]! <= g + 1) continue;
        const h = heuristic(next);
        if (!Number.isFinite(h)) continue;
        const idx = stBoxes.length;
        stBoxes.push(next);
        stPlayer.push(b);
        stG.push(g + 1);
        stParent.push(s);
        stPushFrom.push(b);
        stPushDir.push(di);
        best.set(key, idx);
        if (solved(next)) {
          found = idx;
          break;
        }
        hpush([g + 1 + h, -(g + 1), seq++, idx]);
      }
    }
    for (const b of boxes) occupied[b] = 0;
  }
  if (found < 0) return null;

  // Reconstruct: walk to each push square, then push.
  const path: number[] = [];
  for (let s = found; s > 0; s = stParent[s]!) path.push(s);
  path.reverse();
  let moves = '';
  let player = level.player;
  let boxes = level.boxes;
  for (const s of path) {
    const from = stPushFrom[s]!;
    const di = stPushDir[s]!;
    const [dx, dy, , pushLetter] = DIRS[di]!;
    const d = dy * w + dx;
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(w, wall, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += pushLetter;
    player = from;
    boxes = stBoxes[s]!;
  }
  return { pushes: path.length, moves };
}

type Room = { boxes: number[]; goals: number[]; done?: boolean };

/**
 * The level's rooms: cut the dead squares of the player's row (the hallway)
 * out of the floor; each remaining component holding boxes is a room. Null
 * when the level is not a set of independent rooms.
 */
function roomsOf(level: Level): Room[] | null {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(w, wall, goals);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i]! + dy * w + dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ boxes: [], goals: [] });
  }
  for (const b of boxes) {
    if (part[b] < 0) return null;
    rooms[part[b]!]!.boxes.push(b);
  }
  for (const g of goals) {
    if (part[g] < 0) return null;
    rooms[part[g]!]!.goals.push(g);
  }
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

function endOf(w: number, from: number, moves: string): number {
  let p = from;
  for (const ch of moves.toLowerCase()) p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  return p;
}

/** Solve room by room (other rooms' boxes stand as walls), else the whole level at once. */
function solve(text: string): string {
  const level = parseLevel(text);
  const rooms = roomsOf(level);
  if (rooms !== null) {
    let player = level.player;
    let moves = '';
    let ok = true;
    for (const room of rooms) {
      const wall = level.wall.slice();
      for (const other of rooms) {
        if (other === room) continue;
        for (const b of other.done ? other.goals : other.boxes) wall[b] = 1;
      }
      const sub: Level = {
        w: level.w,
        h: level.h,
        wall,
        goals: room.goals.slice().sort((a, b) => a - b),
        boxes: room.boxes.slice().sort((a, b) => a - b),
        player,
      };
      const solved = solvePushOptimal(sub, ROOM_EXPANSIONS);
      if (solved === null) {
        ok = false;
        break;
      }
      moves += solved.moves;
      player = endOf(level.w, player, solved.moves);
      room.done = true;
    }
    if (ok) return moves;
  }
  const whole = solvePushOptimal(level, LEVEL_EXPANSIONS);
  return whole === null ? '' : whole.moves;
}

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
