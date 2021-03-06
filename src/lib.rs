#![feature(array_zip)]
#![feature(generic_const_exprs)]
#![allow(dead_code)]
#![allow(incomplete_features)]
#![no_std]
#![deny(missing_docs)]

//! # Rust Pure Pursuit Controller ![https://crates.io/crates/pure_pursuit](https://img.shields.io/crates/v/pure_pursuit)
// A no_std Pure Pursuit controller intended for Vex robots, but designed to be as versatile as possible.

mod api;
pub mod prelude;

#[cfg(test)]
mod tests {
    use crate::prelude::{PathBuilder, Pathematics, Waypoint};

    #[test]
    fn pathematics_builder() {
        let path_builder = Pathematics::<f64, 2>::builder()
            .with_radius(3f64)
            .with_point(Waypoint {
                dimensions: [0f64, 0f64],
            })
            .with_point(Waypoint {
                dimensions: [2f64, 6f64],
            })
            .with_point(Waypoint {
                dimensions: [7f64, 4f64],
            });
    }

    #[test]
    fn pathematics_struct() {
        let path = Pathematics::<f64, 2>::builder()
            .with_radius(3f64)
            .with_point(Waypoint {
                dimensions: [0f64, 0f64],
            })
            .with_point(Waypoint {
                dimensions: [2f64, 6f64],
            })
            .with_point(Waypoint {
                dimensions: [7f64, 4f64],
            })
            .build();
    }

    // This test ensures that if the position of the robot is not on the path it will go straight to the start
    #[test]
    fn pathematics_test_start_out_of_range() {
        let mut path = Pathematics::<f64, 2>::builder()
            .with_radius(1f64)
            .with_point(Waypoint {
                dimensions: [0f64, 0f64],
            })
            .with_point(Waypoint {
                dimensions: [2f64, 6f64],
            })
            .with_point(Waypoint {
                dimensions: [7f64, 4f64],
            })
            .build();

        let mut position = Waypoint::<f64, 2> {
            dimensions: [
                -1f64,
                -1f64,
            ]
        };

        let mut target_expected = Waypoint::<f64, 2> {
            dimensions: [
                0f64,
                0f64,
            ]
        };

        assert_eq!(path.step(position), target_expected);
    }

    // This test ensures that if the robot starts within the radius of the path it will target the path
    #[test]
    fn pathematics_test_start_in_range() {
        let mut path = Pathematics::<f64, 2>::builder()
            .with_radius(2f64)
            .with_point(Waypoint {
                dimensions: [0f64, 0f64],
            })
            .with_point(Waypoint {
                dimensions: [2f64, 0f64],
            })
            .with_point(Waypoint {
                dimensions: [7f64, 4f64],
            })
            .build();

        let mut position = Waypoint::<f64, 2> {
            dimensions: [
                -1f64,
                0f64,
            ]
        };

        let mut target_expected = Waypoint::<f64, 2> {
            dimensions: [
                1f64,
                0f64,
            ]
        };

        assert_eq!(path.step(position), target_expected);
    }
}
