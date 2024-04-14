# All notable changes to this project will be documented in this file as per Keepachangelog. We'll follow semantic versioning.

```
## [Unreleased] hash number

or

## [Major.Minor.Patch] - date
### Added
- For new features
### Changed
- for changes in existing functionality
### Deprecated
- for soon to-be removed features
### Removed
- For now removed features
### Fixed
- For any bug fixes
### Security
- in case of vulnerabilities

```
## [Unreleased] - HEAD
### Added
- Fibonacci sequence to allow certain functions to be rate-limited

### Changed
- Limit IncrementalSstarKD_tree animation updates according to fibonacci seq
- Also limit printing to fibonacci seq
- Use logging tool

## [Unreleased] - history changed
### Changed
- Separated out Kd-Tree based features as well as animation from virtual class. Now virtual class is more barebones, i.e user needs to implement `neighbors` and `cost` function.
- Virtual class also automatically calls S*-BS by default (can be changed). The initialization arguments are simply the `graph`, `terminals`, and `variant` desired.
- Moved random seed setting into `main` portion of the code.

## [Unreleased] - 4f6f6b
### Added
- Added more info to the previous commit's changelog
- Initialize best path costs from S*-results
- Print message whenever we are updating a path

## [Unreleased] - 8a0cc1
### Added
- Path detection has been placed into its own member function. We do path detection in two places `add()` and `rewire()`
- Dont need to rebuild KD-Tree every sample, do it based on fib sequence.
### Changed
- Moved virtual class' member functions around so that they more closely follow the ordering in the abstract class
- Commented out Dijkstra for now (speeed), but it can be uncommented as necessary
### Fixed
- Path detection routine needs to be added to the `add()` phase and not just during `rewire()`. This will fix issue #3 where two terminals may not discover shorter paths. Once in a while though, sometimes paths will be detected between non-neighboring voronoi regions. 

## [Unreleased] - 45f5446
### Added
- Shortest path detection done during DFS as children nodes are updated.
### Changed
- Adding grey nodes will now use the `_update_plot_node()` routine.
### Fixed
- Edge color now correctly shown with respect to a root. However, still haven't figured out how some node colors are off.
- `rewire()` between different root sets was causing issues due to not updating roots correctly. I'm now accouting for this in DFS.

## [Unreleased] - d8778249
### Added
- Moved all edge and node plotting into their own member functions.
- Adding logging config to `config` module (WIP).
### Changed
- Use `.get( , -1)` on `terminal_indices` to return a default value of -1 for non-rooted samples.
- Only call `AnimateV2.update()` once per iteration to improve speed
- RandomGraph radius set to 0.5.
- ~~Within `rewire()`, during "# add neighbor to pt's children set", added else statement for less confusion.~~ Reverted this.
### Fixed
- Made sure root sets are completely disjoint! During `add()`, update the parent node's children set too.

## [Unreleased] - 3612ef18
### Added
- Keep track of leaf nodes (addition/removal) during `rewire()`/`add()` respectively.
- Added separate colors to each tree edge.
### Changed
- Moved `self._edges_to_plot.add((pt,n))` to `rewire()` instead of during `propagation()`.
- Within `rewire()`, during "# add neighbor to pt's children set", added else statement for less confusion.
- Commented out `reconstruct_path()` function. It seems redundant?
- Shortest paths update is now a function to be called. 
- `Rewire()` no longer needs to check whether neighbor `n` is in gcosts.


