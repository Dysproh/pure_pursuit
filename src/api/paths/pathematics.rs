use crate::api::paths::Waypoint;

pub trait RadiusState {}
pub struct Radius<T>(T);
pub struct NoRadius;
impl<T> RadiusState for Radius<T> {}
impl RadiusState for NoRadius {}

#[derive(Debug)]
pub struct PathBuilder<T, E: RadiusState, const D: usize>
where
    T: num::Float + num::FromPrimitive + core::iter::Sum,
{
    radius: E,
    points: Vec<Waypoint<T, D>>,
}

#[derive(Debug)]
pub struct Pathematics<T, const D: usize>
where
    T: num::Float + num::FromPrimitive + core::iter::Sum,
{
    points: Vec<Waypoint<T, D>>,
    radius: T,
    current_segment: Option<usize>,
}

impl<T, const D: usize> PathBuilder<T, NoRadius, D>
where
    T: num::Float + core::iter::Sum + num::FromPrimitive,
{
    /// Consumes the builder and returns it with a radius added
    pub fn with_radius(self, r: T) -> PathBuilder<T, Radius<T>, D> {
        PathBuilder {
            radius: Radius(r),
            points: self.points,
        }
    }
}

impl<T, const D: usize> PathBuilder<T, Radius<T>, D>
where
    T: num::Float + core::iter::Sum + num::FromPrimitive,
{
    /// Consumes the builder and returns a Pathematics struct
    pub fn build(self) -> Pathematics<T, D> {
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

impl<T, E: RadiusState, const D: usize> PathBuilder<T, E, D>
where
    T: num::Float + core::iter::Sum + num::FromPrimitive,
{
    pub fn with_point(mut self, point: Waypoint<T, D>) -> Self {
        self.points.push(point);
        self
    }
}

impl<T, const D: usize> Pathematics<T, D>
where
    T: num::Float + core::iter::Sum + num::FromPrimitive,
{
    pub fn builder() -> PathBuilder<T, NoRadius, D> {
        PathBuilder {
            radius: NoRadius,
            points: Vec::new(),
        }
    }

    /// Gets the next point to follow and updates what segment it is on currently if necessary
    /// position: A waypoint containing the current position of the robot along the same coordinate system
    pub(crate) fn step(&mut self, position: Waypoint<T, D>) -> Waypoint<T, D> {
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
                    let t: T = T::max(
                        (-b + (b.powi(2) - T::from_i8(4).unwrap() * a * c).sqrt())
                            / (T::from_i8(2).unwrap() * a),
                        (-b - (b.powi(2) - T::from_i8(4).unwrap() * a * c).sqrt())
                            / (T::from_i8(2).unwrap() * a),
                    );

                    let rt: [T; D] = p
                        .dimensions
                        .zip(q.dimensions)
                        .map(|(pn, qn)| pn + t * (pn + qn));
                    Waypoint { dimensions: rt }
                } // else {
            } // Some(x) => {
        } // match self.current_segment {
    } // fn step(&mut self, position: Waypoint<T, D>) -> Waypoint<T, D> {
}
