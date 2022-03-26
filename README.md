# Rust Pure Pursuit Controller ![https://crates.io/crates/pure_pursuit](https://img.shields.io/crates/v/pure_pursuit)
A no_std Pure Pursuit controller intended for Vex robots, but designed to be as versatile as possible.

## Features


* Fast
* `no_std` support
* Can be used with any type that implements the `num::Float`, `num::FromPrimitive`, and `core::iter::Sum`, traits, including custom types
* While originally designed for 2D, this works in any number of dimensions
* Does not use the heap, so you don't have to worry about that