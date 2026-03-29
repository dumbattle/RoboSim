# AI Benchmark - Detailed Results

## Overview

Each LLM was given the robot API documentation and asked to write an exploration algorithm in a single prompt. The raw solutions were then debugged (fixing compile errors and crashes) and optimized (reducing wasted sensor calls, fixing turn inefficiencies) before benchmarking. An instructor-written solution serves as the baseline.

The robot explores a 2D grid of randomly generated walls on a limited battery budget. It can only see one tile ahead and must use sensor queries, movement, and turning - all of which cost energy - to navigate. Score equals unique tiles visited before the battery runs out or the robot crashes into a wall.

---

## Algorithm Descriptions

**Instructor** - Flood-fills unexplored tiles into connected groups, BFS-paths to each, and picks the group with the best size-to-distance ratio. Recalculates every step. Greedy phase checks forward first, skips known tiles from memory.

**Claude** - Dijkstra pathfinding weighted by turn + move cost to find the cheapest frontier. Greedy forward-first phase with mid-path re-evaluation. Accurate pathing but the frontier ranking is cost-only, no cluster sizing.

**DeepSeek** - Scores frontier tiles by uncertainty-based info-gain divided by A* distance. In practice, the greedy forward step dominates - it just runs straight until blocked, then the pathfinder kicks in.

**Gemini** - Picks nearest frontier tile by Manhattan distance, BFS-paths to that tile. No explicit greedy phase - every single move involves a full grid scan and pathfind.

**Copilot** - Right-hand wall follow with BFS frontier pathfinding when stuck.

**Grok** - Stack-based DFS with right-hand wall-following priority (right > forward > left). Simple backtracking.

**ChatGPT** - Stack-based DFS with forward movement priority. Simple backtracking.

---

## LLM Output Correctness

All LLMs used the free versions on the web. Outputs that produced overly basic solutions were rerolled. Outputs that did not compile or had faulty logic were given feedback for correction. If unable to correct, the conversation was restarted until a working script was produced, or was close enough to manually fix.

**ChatGPT** - Compiled and ran correctly without any bugs. Only 1 attempt needed.

**Gemini** - Initial attempt produced a slightly weaker version of ChatGPT's version, so opted to reroll for variety. Several small bugs in new version, including a minor typo.

**DeepSeek** - Failed repeatedly to generate a working script. Many attempts, all attempting complex information-theoretic solutions. A working solution was eventually achieved by having the instructor fix various bugs with Claude's help.

**Grok** - Several attempts. Often resorts to wall hugging. Further prompts for more complex algorithms often produce nonfunctional scripts. Required manual fixing.

**Claude** - First attempt compiled, but had flawed robot logic. A single feedback loop was enough for Claude to fix and produce a working script.

**Copilot** - Often produced overly simple algorithms despite instructions and iteration. Instructor instead opted to provide a specific algorithm description. Copilot was able to produce a working implementation on the first attempt.

**Instructor** - Algorithm was provided to and produced by Claude. Initial script was debugged and optimized by the instructor.

---

## Global Optimizations

In order to fairly compare each algorithm, each of these optimizations were applied to every algorithm when applicable.

- **Forward Bias** - For any algorithm that had a greedy phase, neighbor ordering was modified to prioritize the forward direction to minimize turns. Does not apply to wall-hugging algorithms.
- **Scan Memory** - Each algorithm was given a memory if it didn't already have one. Redundant scans were replaced with memory lookups.
- **Short Turn** - Most algorithms had some version of `TurnToDirection(Direction d)`. These functions were optimized to calculate the optimal direction (left or right) to turn.

---

## Standard Trial Results

**Parameters:** 300×200 map · 200,000 battery · Move cost 5 · Turn cost 3 · Scan cost 1–3
Map characterized by variable-sized caverns with many thin corridors.

### Scores by Seed

| Solution | Seed 3 | Seed 31 | Seed 314 | Seed 3141 | Seed 31415 | Average | Wins |
|----------|--------|---------|----------|-----------|------------|---------|------|
| Instructor | 20,332 | 20,457 | 19,849 | 20,326 | 20,056 | **20,204** | 2/5 |
| Claude | 19,307 | **20,497** | **19,986** | 20,109 | 19,628 | **19,905** | 2/5 |
| DeepSeek | 19,839 | 20,296 | 18,354 | **20,651** | 18,358 | **19,500** | 1/5 |
| Gemini | 18,915 | 17,935 | 17,035 | 17,738 | 18,484 | **18,021** | 0/5 |
| Copilot | 17,154 | 17,900 | 17,126 | 17,434 | 17,235 | **17,370** | 0/5 |
| Grok | 14,754 | 16,991 | 14,770 | 15,788 | 14,458 | **15,352** | 0/5 |
| ChatGPT | 15,539 | 15,531 | 15,466 | 14,340 | 15,272 | **15,230** | 0/5 |

### Efficiency Ratios (Averaged)

| Solution | Score/Step | Steps/Score | Turns/Step | Scans/Score | Move % | Turn % | Scan % |
|----------|-----------|-------------|------------|-------------|--------|--------|--------|
| Instructor | 0.822 | 1.22 | 0.38 | 1.22 | 61% | 14% | 25% |
| Claude | 0.804 | 1.24 | 0.38 | 1.22 | 62% | 14% | 24% |
| DeepSeek | 0.794 | 1.26 | 0.40 | 1.22 | 62% | 15% | 23% |
| Gemini | 0.764 | 1.31 | 0.53 | 1.23 | 59% | 19% | 22% |
| Copilot | 0.880 | 1.14 | 0.83 | 1.49 | 49% | 25% | 26% |
| Grok | 0.711 | 1.41 | 0.77 | 1.36 | 54% | 25% | 21% |
| ChatGPT | 0.566 | 1.77 | 0.34 | 1.23 | 67% | 14% | 19% |

**Key metrics:**
- **Score/Step** - tiles gained per move. Higher = better greedy collection. Copilot leads (0.880).
- **Steps/Score** - moves per new tile (1.0 = no backtracking). Copilot leads (1.14).
- **Turns/Step** - turning overhead. Instructor and Claude lead (0.38).
- **Move %** - fraction of battery spent on actual movement. ChatGPT leads (67%) but wastes it on backtracking.

---

## Cost Scenario Experiments

All run on seed 31415 to isolate the effect of cost changes.
*Max reachable tiles: 32,444*

### Scores Under Different Cost Rules

| Solution | Standard | Turns Free | Scans Free | Both Free | Gain |
|----------|----------|------------|------------|-----------|------|
| Instructor | 20,056 | 23,091 | 25,138 | 29,600 | +9,544 |
| Claude | 19,628 | 21,683 | 24,693 | 28,701 | +9,073 |
| DeepSeek | 18,358 | 21,503 | 23,588 | 29,227 | +10,869 |
| Gemini | 18,484 | 22,480 | 23,049 | 28,416 | +9,932 |
| Copilot | 17,235 | 22,806 | 23,337 | **32,419** | **+15,184** |
| Grok | 14,458 | 19,964 | 18,444 | 25,424 | +10,966 |
| ChatGPT | 15,272 | 17,232 | 18,051 | 21,564 | +6,292 |

### Ranking Shifts

| Solution | Standard | Turns Free | Scans Free | Both Free |
|----------|----------|------------|------------|-----------|
| Instructor | 1st | 1st | 1st | 2nd |
| Claude | 2nd | 4th | 2nd | 4th |
| DeepSeek | 4th | 5th | 3rd | 3rd |
| Gemini | 3rd | 3rd | 5th | 5th |
| Copilot | 5th | 2nd | 4th | **1st** |
| Grok | 7th | 6th | 7th | 6th |
| ChatGPT | 6th | 7th | 6th | 7th |

### Notes

- **Wall-hug efficiency** - Wall hugging tends to have a higher score/step, which is offset by a higher turn count.
- **Free turns help wall-followers the most** - Copilot gains +5,571, Grok gains +5,506. The two highest turn-overhead algorithms benefit disproportionately.
- **Algorithmic complexity** - DeepSeek's information-based approach runs by far the slowest. Instructor's frontier grouping approach is also slow, but to a much lesser degree. Other algorithms run fine.

---

## Architectural Patterns

Every algorithm follows mostly the same two-phase loop:

1. **Greedy phase** - collect nearby points efficiently
2. **Frontier phase** - find new territory when local options dry up

The differences are in how each phase is implemented:

| Solution | Greedy Strategy | Frontier Strategy |
|----------|----------------|-------------------|
| Instructor | Forward > left > right > back | Flood-fill groups, heuristic size/distance scoring |
| Claude | Forward > left > right > back | Dijkstra with turn costs, cheapest-first |
| DeepSeek | Go straight if open, otherwise fall through | A* per-tile, info-gain / distance scoring |
| Gemini | None (pathfinds every move) | BFS to closest by Manhattan distance |
| Copilot | Right-hand wall follow | BFS to nearest unexplored region |
| ChatGPT | DFS push (try all neighbors) | Stack backtrack |
| Grok | DFS push with right-hand priority | Stack backtrack |

---

## Next Steps

### Algorithm

The instructor solution performs best in many scenarios, but has a major weakness: prioritizing larger regions often leaves scattered points, making cleanup during the later stages highly inefficient. Copilot and Grok's wall following cleans up local regions very thoroughly, though at great turning cost. This hints at the possibility of a better local collection strategy, which would improve the greedy phase of most algorithms present.

### Environment

Currently, the environment is static. Planned additions:

- Add multiple obstacle types (wall + cliff), plus associated scanners.
- Give scanners a dynamic error rate. Continuous, slow change so robots can predict.
- Robots will have to dynamically adjust behavior.

Example scenarios:
- Wall + cliff scanner start at 0% error: safe to explore.
- Wall + cliff error high: repeated scans? Wait for better conditions? Collect previously scanned tiles that are known safe?
- Wall error low + cliff error high: default to one of the above strategies? Perform lots of wall scans now, wait for low cliff error to complete scan?