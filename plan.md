# V2 Implementation Checklist

## Environment

- [X] Add `ObstacleType` enum (WALL, CLIFF)
- [X] Modify world generation to place walls and cliffs as separate types
- [X] Implement obstacle clustering by type (existing layer system supports this)
- [X] Wall collision drains energy instead of killing
- [X] Cliff collision kills instantly (or drains alot)
- [X] Configure energy drain amount for wall collision

## Confidence
- [X] Refactor `_seen` into confidence values for each tile type
- [X] Replace `IsWallAhead` with `ScanAhead`
- [X] update confidence values after each scan
- [X] If confidence is 100%, abort scan
- [X] Expose Confidence Queries
- [X] Walking onto open tile or into wall sets confidence to 100%

## Dynamic error rates

- [X] Per-obstacle false positive rate (says obstacle, actually open)
- [X] Per-obstacle false negative rate (says open, actually obstacle)
- [X] Continuous slow change over time (sine wave, random walk, or similar)
- [X] Rates independent per obstacle type
- [X] Rates are global (same for all tiles at a given tick)


## New API — sensor information

- [X] `GetErrorRates(ObstacleType) → float falsePos, float falseNeg`
- [X] `GetErrorDeltas(ObstacleType) → float fpDelta, float fnDelta`
- [X] `GetTileConfidence(x, y) → int[]` (indexed by obstacle type)
- [X] `GetExpectedInfoGain(x, y, ObstacleType) → InfoGain`

## New API — helpers

- [ ] `TurnToDirection(Direction d)` — shortest rotation
- [ ] `TileVisited(x, y)` — auto-tracked
- [X] Scan memory / confidence tracking (internal bookkeeping)

## New API — state machine

- [ ] `StateMachine` class
- [ ] `sm.Add(name, function)` — register a mode
- [ ] `sm.SetMode(name)` — activate a mode
- [ ] `sm.Tick()` — execute current mode

## Display

- [ ] Distinct colors for wall vs cliff tiles
- [ ] Sensor error rate HUD display
- [ ] Tile confidence visualization (optional)
- [ ] Current mode label in HUD (optional)

## Parameters (robot_params.h)

- [ ] `WALL_COLLISION_COST` — energy drained on wall hit
- [ ] `CLIFF_KILLS` — bool, cliff collision is fatal
- [ ] Error rate wave parameters (amplitude, frequency, phase offset per scanner)
- [ ] Error rate floor/ceiling bounds

## Documentation

- [ ] Update README with V2 overview
- [ ] Update API reference with new functions
- [ ] Add state machine usage example
- [ ] Add sensor state strategy table
- [ ] Document student progression (V1 → V2 upgrade path)

## Benchmark

- [ ] Rerun all LLM solutions on V2 environment
- [ ] Test baseline: all solutions with static 0% error (should match V1 scores)
- [ ] Test with low error rates (5%)
- [ ] Test with high error rates (30%+)
- [ ] Test with asymmetric rates (one scanner good, one bad)
- [ ] Compare state-machine solutions vs non-adaptive solutions