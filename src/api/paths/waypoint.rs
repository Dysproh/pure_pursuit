/// The Waypoint struct
///
/// This structure represents a point. Like PathBuilder and Pathematics, it is generic over
/// `T` and `D`, which determine the type it stores as well as the number of dimensions it can represent.
///
/// Example:
/// ```
/// use pure_pursuit::prelude::Waypoint;
///
/// let pos = Waypoint {
///     dimensions: [2f64, 1f64]
/// };
/// ```
///
/// Constructing one of these may seem unnecessarily verbose, and thats because it is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Waypoint<T, const D: usize>
where
    T: num::Float + core::iter::Sum,
{
    /// The dimensions the waypoint stores
    ///
    /// These can be X and Y, XYZ, WXYZ, or any other number of dimensions, so long as it matches
    pub dimensions: [T; D],
}

// pythagorean theorem method so I don't have to write it out every time
impl<T, const D: usize> Waypoint<T, D>
where
    T: num::Float + core::iter::Sum,
{
    /// This method finds the distance between 2 waypoints using the Pythagorean theorem
    ///
    /// Example:
    ///
    /// ```
    /// use pure_pursuit::prelude::Waypoint;
    /// let lhs = Waypoint {
    ///     dimensions: [2f64, 1f64]
    /// };
    /// let rhs = Waypoint {
    ///     dimensions: [5f64, 8f64]
    /// };
    ///
    /// let distance = lhs.pythag(&rhs);
    /// ```
    pub fn pythag(&self, rhs: &Waypoint<T, D>) -> T {
        self.dimensions
            .iter()
            .zip(rhs.dimensions.iter())
            .map(|(&lhs, &rhs)| (lhs - rhs).powi(2))
            .sum::<T>()
            .sqrt()
    }
}

impl<T, const D: usize> Default for Waypoint<T, D>
where
    T: num::Float + core::iter::Sum,
{
    fn default() -> Self {
        Self {
            dimensions: [T::zero(); D],
        }
    }
}
