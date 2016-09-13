extern crate nalgebra;

use nalgebra::{Vector2, Norm};

use collisions::collidable::{Collidable, CollidableShape};
use collisions::util::*;

pub type Collision = (Collidable, Collidable, usize, Vector2<f32>);

pub struct Quadtree
{
	pub vertx: [f32; 4],
	pub verty: [f32; 4],
	pub quads: Vec<Quadtree>,
	pub items: Vec<Collidable>,
}

impl Quadtree
{
	pub fn new(vertx: &[f32], verty: &[f32], depth: usize) -> Quadtree
	{
		let mut quads: Vec<Quadtree> = vec![];
		if depth > 0
		{
			let mut subvert = [0.; 32];

			// x
			subvert[0] 		= vertx[0];
	        subvert[1] 		= subvert[0];
	        subvert[2] 		= (vertx[2] + vertx[1]) / 2.;
	        subvert[3] 		= subvert[2];
	        subvert[4] 		= subvert[1];
	        subvert[5] 		= subvert[4];
	        subvert[6] 		= subvert[2];
	        subvert[7] 		= subvert[2];
	        subvert[8] 		= subvert[7];
	        subvert[9] 		= subvert[6];
	        subvert[10] 	= vertx[2];
	        subvert[11] 	= subvert[10];
	        subvert[12] 	= subvert[3];
	        subvert[13] 	= subvert[2];
	        subvert[14] 	= subvert[11];
	        subvert[15] 	= vertx[3];

			// y
			subvert[16] 	= verty[0];
			subvert[17] 	= (verty[1] + verty[0]) / 2.;
			subvert[18] 	= subvert[17];
			subvert[19]		= subvert[16];
			subvert[20] 	= subvert[17];
			subvert[21] 	= verty[1];
			subvert[23] 	= subvert[20];
			subvert[22] 	= verty[2];
			subvert[24] 	= subvert[23];
			subvert[25] 	= subvert[22];
			subvert[26] 	= subvert[25];
			subvert[27] 	= subvert[24];
			subvert[28] 	= subvert[19];
			subvert[29] 	= subvert[18];
			subvert[30] 	= subvert[27];
			subvert[31] 	= verty[3];

			quads.push(Quadtree::new(&subvert[..4], &subvert[16..20], depth-1));
			quads.push(Quadtree::new(&subvert[4..8], &subvert[20..24], depth-1));
			quads.push(Quadtree::new(&subvert[8..12], &subvert[24..28], depth-1));
	        quads.push(Quadtree::new(&subvert[12..16], &subvert[28..32], depth-1));
		}
		let mut vx = [0.; 4];
		vx[..4].clone_from_slice(vertx);
		let mut vy = [0.; 4];
		vy[..4].clone_from_slice(verty);
		Quadtree
		{
			vertx: vx,
			verty: vy,
			quads: quads,
			items: vec![],
		}
	}

	pub fn insert(&mut self, item: Collidable)
	{
		if !self.quads.is_empty()
		{
		    if item.collidable_shape as u8 == CollidableShape::Circle as u8
			{
				for i in 0..4
				{
					if poly_contains_pnt(4, &self.quads[i].vertx, &self.quads[i].verty, item.centrex, item.centrey)
					&&
					dist_poly_circ(4, &self.quads[i].vertx, &self.quads[i].verty, item.centrex, item.centrey) > item.radius
					{
						self.quads[i].insert(item);
						return;
					}
				}
			}
			else
			{
				for i in 0..4
				{
					if poly_contains_poly(4, &self.quads[i].vertx, &self.quads[i].verty, item.nvert, &item.vertx, &item.verty)
					{
						self.quads[i].insert(item);
						return;
					}
				}
			}
		}
		self.items.push(item);
	}

	pub fn check_collisions(&self, collisions: &mut Vec<(Collidable, Collidable, usize, Vector2<f32>)>) -> Vec<&Collidable>
	{
		let mut all_items = vec![];
		if !self.quads.is_empty()
		{
			for i in 0..self.quads.len()
			{
				let quad_items = self.quads[i].check_collisions(collisions);
				for j in 0..quad_items.len()
				{
					all_items.push(quad_items[j]);
					for k in 0..self.items.len()
					{
						possible_collision_between(quad_items[j], &self.items[k], collisions);
					}
				}
			}
		}
		for i in 0..self.items.len()
		{
			all_items.push(&self.items[i]);
			for j in i+1..self.items.len()
			{
				possible_collision_between(&self.items[i], &self.items[j], collisions);
			}
		}
		all_items
	}
}

pub fn possible_collision_between(a: &Collidable, b: &Collidable, collisions: &mut Vec<Collision>)
{
	match a.collidable_shape
	{
		CollidableShape::Circle =>
		{
			match b.collidable_shape
			{
				CollidableShape::Circle =>
				{
					if dist_circ_circ(a.centrex, a.centrey, b.centrex, b.centrey) < a.radius + b.radius
					{
						let apos = Vector2::new(a.centrex, a.centrey);
						let bpos = Vector2::new(b.centrex, b.centrey);
						let delta = bpos - apos;
						let dunit = if delta.norm() > 0. { nalgebra::normalize(&delta) } else { nalgebra::zero() };
						let s = dunit * a.radius;
						collisions.push((a.clone(), b.clone(), 0, s));
					}
				}
				CollidableShape::Polygon =>
				{
					match dist_poly_circ__face__support(b.nvert, &b.vertx, &b.verty, a.centrex, a.centrey)
					{
						(d, f, s) =>
						{
							if d <= a.radius || poly_contains_pnt(b.nvert, &b.vertx, &b.verty, a.centrex, a.centrey)
							{
								collisions.push((b.clone(), a.clone(), f, s));
							}
						}
					}
				}
			}
		}
		CollidableShape::Polygon =>
		{
			match b.collidable_shape
			{
				CollidableShape::Circle =>
				{
					match dist_poly_circ__face__support(a.nvert, &a.vertx, &a.verty, b.centrex, b.centrey)
					{
						(d, f, s) =>
						{
							if d <= b.radius || poly_contains_pnt(a.nvert, &a.vertx, &a.verty, b.centrex, b.centrey)
							{
								collisions.push((a.clone(), b.clone(), f, s));
							}
						}
					}
				}
				CollidableShape::Polygon =>
				{
					match penetration__face__support(a.nvert, &a.vertx, &a.verty, &a.normx, &a.normy, b.nvert, &b.vertx, &b.verty)
					{
						(da, fa, sa) =>
						{
							match penetration__face__support(b.nvert, &b.vertx, &b.verty, &b.normx, &b.normy, a.nvert, &a.vertx, &a.verty)
							{
								(db, fb, sb) =>
								{
									if da < 0. && db < 0.
									{
										if da > db
										{
											collisions.push((a.clone(), b.clone(), fa, sa));
										} else
										{
											collisions.push((b.clone(), a.clone(), fb, sb));
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
}
// CollidableShape::Arc =>
// {
// 	if dist_circ_circ(a.centrex, a.centrey, b.centrex, b.centrey) < a.radius + b.radius
// 	{
// 		let apos = Vector2::new(a.centrex, a.centrey);
// 		let bpos = Vector2::new(b.centrex, b.centrey);
// 		let delta = bpos - apos;
// 		let bdir = Vector2::new(b.dirx, b.diry);
// 		let angle = bdir.norm().dot(&-delta.norm()).acos();
//
// 		if angle <= b.radians/2.
// 		{
// 			let dunit = if delta.norm() > 0. { nalgebra::normalize(&delta) } else { nalgebra::zero() };
// 			let s = dunit * a.radius;
// 			collisions.push((a.clone(), b.clone(), 0, s));
// 		}
// 	}
// }
