use crate::prelude::Waypoint;

/// Trait for structures which take a current position and spit out a target position
pub trait PathTargetFinder<T, const D: usize>
    where T: num::Float + num::FromPrimitive + core::iter::Sum
{
    /// Gets the target position for the controller
    ///
    /// position: The current position of the robot
    /// return: The position to target
    fn get_target_point(&mut self, position: Waypoint<T, D>) -> Waypoint<T, D>;
}