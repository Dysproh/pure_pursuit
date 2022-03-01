#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Waypoint<T, const D: usize>
where
    T: num::Float + core::iter::Sum,
{
    pub dimensions: [T; D],
}

// pythagorean theorem method so I don't have to write it out every time
impl<T, const D: usize> Waypoint<T, D>
where
    T: num::Float + core::iter::Sum,
{
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
