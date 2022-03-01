use pure_pursuit::prelude::*;
fn main() {
    const kP: f64 = 0.4;
    const kD: f64 = 0.02;

    let mut path = Pathematics::<f64, 2>::builder()
        .with_radius(0.5f64)
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

    let mut robot_position = Waypoint::<f64, 2>{
        dimensions: [0f64, 0f64]
    };

    let mut target: Waypoint<f64, 2> = path.step(robot_position);

    let mut Position_h = robot_position;

    for i in 0..50 {
        target = path.step(robot_position);
        robot_position = Waypoint {
            dimensions: [
                robot_position.dimensions[0] +
                    ((target.dimensions[0] - robot_position.dimensions[0]) * kP),
                robot_position.dimensions[1] +
                    ((target.dimensions[1] - robot_position.dimensions[1]) * kP)
            ]};

        println!("({:?},{:?}), {:?}", target.dimensions[0], target.dimensions[1], path.current_segment);
    }
}