#![no_std]
use pure_pursuit::prelude::*;
use libc_print::std_name::println;
fn main() {
    const kP: f64 = 0.15;

    let mut path = path_builder::<f64, 2>()
        .with_radius(0.7f64)
        .with_point(Waypoint {
            dimensions: [0f64, 0f64],
        })
        .with_point(Waypoint {
            dimensions: [2f64, 0f64],
        })
        .with_point(Waypoint {
            dimensions: [7f64, 4f64],
        })
        .with_point(Waypoint {
            dimensions: [-6f64, 3f64],
        })
        .with_point(Waypoint {
            dimensions: [-2f64, 0f64],
        })
        .build();

    let mut robot_position = Waypoint::<f64, 2>{
        dimensions: [0f64, 0f64]
    };

    let mut target: Waypoint<f64, 2> = path.step(robot_position);

    while robot_position.pythag(&target) > 0.1 {
        target = path.step(robot_position);
        robot_position = Waypoint {
            dimensions: [
                robot_position.dimensions[0] +
                    ((target.dimensions[0] - robot_position.dimensions[0]) * kP),
                robot_position.dimensions[1] +
                    ((target.dimensions[1] - robot_position.dimensions[1]) * kP)
            ]};

        println!("({:?},{:?})", robot_position.dimensions[0], robot_position.dimensions[1]);
    }
}