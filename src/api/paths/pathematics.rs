//! The Pathematics struct and its builder

use crate::api::paths::Waypoint;

pub trait RadiusState {}
pub struct Radius<T>(T);
pub struct NoRadius;
impl<T> RadiusState for Radius<T> {}
impl RadiusState for NoRadius {}


/// The builder for the Pathematics struct
///
/// This builder is generic over 4 parameters:
/// - A `num::Float` type which is used to store the point values, usually an f64 or similar type
/// - A typestate which, at compile time, ensures the resulting Pathematics struct has a radius assigned
/// - A dimension number, which is passed onto the Pathematics and eventually to the Waypoints, which determines
///     how many dimensions the controller runs in (i.e. 2D, 3D, etc.)
/// - A length number, which increases with each added waypoint. This is primarily here so that we can know the
///     size at compile time and avoid heap allocation. This ***must*** be 0 when the builder is constructed
///
/// Example using a PathBuilder to create a 2D path with f64s:
/// ```
/// let path = path_builder::<f64, 2>()
///             .with_radius(3f64)
///             .with_point(Waypoint {
///                 dimensions: [2f64, 1f64],
///             })
///             .with_point(Waypoint {
///                 dimensions: [5f64, 8f64],
///             }).build();
/// ```
#[derive(Debug)]
pub struct PathBuilder<T, E: RadiusState, const D: usize, const L: usize>
where
    T: num::Float + num::FromPrimitive + core::iter::Sum,
{
    radius: E,
    points: [Waypoint<T, D>; L],
}

/// The Pure Pursuit calculation structure
///
/// This struct is constructed with a PathBuilder and uses the given path and pursuit radius to find
/// the optimal point to pursue. It should not be constructed directly.
#[derive(Debug)]
pub struct Pathematics<T, const D: usize, const L: usize>
where
    T: num::Float + num::FromPrimitive + core::iter::Sum,
{
    points: [Waypoint<T, D>; L],
    radius: T,
    /// The current segment being used to pursue. Will automatically update as needed.
    pub current_segment: Option<usize>,
}

impl<T, const D: usize, const L: usize> PathBuilder<T, NoRadius, D, L>
where
    T: num::Float + core::iter::Sum + num::FromPrimitive,
{
    /// Consumes the builder and returns it with a radius added
    ///
    /// This function MUST be used in order to get a builder which can be turned into a Pathematics
    /// struct. Failure to do this will result in a compile time error (which is better than a runtime error).
    ///
    /// Example:
    /// ```
    /// let pathbuilder = path_builder::<f64, 2>().with_radius(3f64);
    /// ```
    pub fn with_radius(self, r: T) -> PathBuilder<T, Radius<T>, D, L> {
        PathBuilder {
            radius: Radius(r),
            points: self.points,
        }
    }
}

impl<T, const D: usize, const L: usize> PathBuilder<T, Radius<T>, D, L>
where
    T: num::Float + core::iter::Sum + num::FromPrimitive,
{
    /// Consumes the builder and returns a Pathematics struct
    pub fn build(self) -> Pathematics<T, D, L> {
        Pathematics {
            radius: self.radius.0,
            current_segment: None,
            points: self.points,
        }
    }
    /// Adds radius to the builder
    pub fn with_radius(self, r: T) -> Self {
        Self {
            radius: Radius(r),
            ..self
        }
    }
}

impl<T, E: RadiusState, const D: usize, const L: usize> PathBuilder<T, E, D, L>
where
    T: num::Float + core::iter::Sum + num::FromPrimitive,
{
    /// Consumes the builder and returns it with the point passed added
    ///
    /// Don't forget to set the radius too!
    ///
    /// Example:
    /// ```
    /// use pure_pursuit::prelude::Waypoint;
    /// let pathbuilder = path_builder::<f64, 2>().with_point(Waypoint {dimensions: [20f64, 3f64]});
    /// ```
    pub fn with_point(self, point: Waypoint<T, D>) -> PathBuilder<T, E, D, { L + 1 }> {
        let mut evil_point_concatenated: [Waypoint<T, D>; L + 1] = [Waypoint::default(); L+1];
        for i in 0..L {
            evil_point_concatenated[i] = self.points[i];
        }
        evil_point_concatenated[L] = point;
        PathBuilder {
            points: evil_point_concatenated,
            radius: self.radius,
        }
    }
}

impl<T, const D: usize, const L: usize> Pathematics<T, D, L>
    where
        T: num::Float + core::iter::Sum + num::FromPrimitive,
{
    /// Gets the next point to follow and updates what segment it is on currently if necessary
    /// position: A waypoint containing the current position of the robot along the same coordinate system
    pub fn step(&mut self, position: Waypoint<T, D>) -> Waypoint<T, D> {
        match self.current_segment {
            None => {
                // If its within the lookahead radius of the starting point, start pathfinding
                if position.pythag(self.points.get(0).unwrap()) <= self.radius {
                    self.current_segment = Some(0);
                    self.step(position)
                }
                // Otherwise, proceed toward the starting position
                // Ideally the starting position would be within your lookahead radius as soon as you
                // start using this pathfinder, but if its not I still want this to be safe so this
                // is how I'm solving that
                else {
                    *self.points.get(0).unwrap()
                }
            }
            Some(x) => {
                // If it's close to the end, of the segment, proceed to the next segment
                // but if its close to the end of the whole path, proceed to the endpoint
                if position.pythag(self.points.get(x + 1).unwrap()) < self.radius {
                    // If this is not the last segment of the path, proceed to the next segment
                    if x < self.points.len() - 2 {
                        self.current_segment = Some(x + 1);
                        self.step(position)
                    }
                    // if x < self.points.len() - 2 {
                    // If this _is_ the last segment of the path, proceed straight to the end
                    else {
                        *self.points.last().unwrap()
                    } // else
                }
                // if position.pythag(self.points.get(x + 1).unwrap()) < self.radius {
                else {
                    // But if you arent close to the end of the current segment, theres other stuff to
                    // do still
                    let p = self.points.get(x).unwrap();
                    let q = self.points.get(x + 1).unwrap();

                    // Calculate a, b, and c for finding t, where r(t) = p when t = 0 and r(t) = q when t = 1
                    // so we can find r and solve for the points where the radius intersects the current segment
                    // yes this is the quadratic formula
                    let a: T = p
                        .dimensions
                        .zip(q.dimensions)
                        .map(|(pn, qn)| (pn - qn).powi(2))
                        .iter()
                        .cloned()
                        .sum::<T>();

                    let b: T = p
                        .dimensions
                        .zip(q.dimensions)
                        .zip(position.dimensions)
                        .map(|((pn, qn), sn)| (pn - sn) * (qn - pn))
                        .iter()
                        .cloned()
                        .sum::<T>()
                        * T::from_i8(2).unwrap();

                    let c: T = p
                        .dimensions
                        .zip(position.dimensions)
                        .map(|(pn, sn)| (pn - sn).powi(2))
                        .iter()
                        .cloned()
                        .sum::<T>()
                        - self.radius.powi(2);

                    // Find the greatest potential value for t
                    // which will be the intersection closer to the destination point of the current segment
                    let t: T = (-b + (b.powi(2) - T::from_i8(4).unwrap() * a * c).sqrt())
                            / (T::from_i8(2).unwrap() * a);

                    let rt: [T; D] = p
                        .dimensions
                        .zip(q.dimensions)
                        .map(|(pn, qn)| pn + t * (qn - pn));
                    Waypoint { dimensions: rt }
                } // else {
            } // Some(x) => {
        } // match self.current_segment {
    } // fn step(&mut self, position: Waypoint<T, D>) -> Waypoint<T, D> {
}

/// Create a builder for the Pathematics struct
///
/// This function is generic over 2 parameters:
/// - A `num::Float` type which is used to store the point values, usually an f64 or similar type
/// - A dimension number, which is passed onto the Pathematics and eventually to the Waypoints, which determines
///     how many dimensions the controller runs in (i.e. 2D, 3D, etc.)
///
/// Example using a PathBuilder to create a 2D path with f64s:
/// ```
/// let path = path_builder::<f64, 2>()
///             .with_radius(3f64)
///             .with_point(Waypoint {
///                 dimensions: [2f64, 1f64],
///             })
///             .with_point(Waypoint {
///                 dimensions: [5f64, 8f64],
///             }).build();
/// ```
pub fn path_builder<T: num::Float + num::FromPrimitive + core::iter::Sum, const D: usize>() -> PathBuilder<T, NoRadius, D, 0> {
    PathBuilder {
        radius: NoRadius,
        points: [],
    }
}
