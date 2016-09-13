extern crate nalgebra;

use nalgebra::{Vector1, Vector2, Rotation2, Rotate};

use std::f32::consts;

use collisions::util::{calc_normx, calc_normy};

pub enum CollidableType
{
    Landscape,
    Actor,
    Object,
}

#[derive(Copy, Clone, Debug)]
pub enum CollidableShape
{
    Circle,
    Polygon,
}

#[derive(Clone, Debug)]
pub struct Collidable
{
	pub collidable_type: u8,
    pub collidable_shape: CollidableShape,
	pub collidable_id: usize,

	// CIRCLE
	pub centrex: f32,
	pub centrey: f32,
	pub radius: f32,

	// POLYGON
	pub nvert: usize,
	pub vertx: Vec<f32>,
	pub verty: Vec<f32>,
	pub normx: Vec<f32>,
	pub normy: Vec<f32>,
}

impl Collidable
{
	pub fn new_circle(t: u8, i: usize, cx: f32, cy: f32, r: f32) -> Collidable
	{
		Collidable
		{
			collidable_type: t,
            collidable_shape: CollidableShape::Circle,
			collidable_id: i,

			centrex: cx,
			centrey: cy,
			radius: r,

			nvert: 0,
			vertx: vec![],
			verty: vec![],
			normx: vec![],
			normy: vec![],
		}
	}

    pub fn new_arc(t: u8, id: usize, cx: f32, cy: f32, r: f32, dirx: f32, diry: f32, rad: f32) -> Collidable
    {
        let centre = Vector2::new(cx, cy);

        let minor_arc = 2.*consts::PI/16.;
        let nvert = (rad / minor_arc).floor() as usize + 2;

        let mut vertx = Vec::with_capacity(nvert);
        let mut verty = Vec::with_capacity(nvert);

        vertx.push(centre.x);
        verty.push(centre.y);

        let mut dir = Rotation2::new(Vector1::new(-rad/2.)).rotate(&nalgebra::normalize(&Vector2::new(dirx, diry))) * r;
        let mut p = centre + dir;

        for _ in 0..nvert-2
        {
            vertx.push(p.x);
            verty.push(p.y);
            dir = Rotation2::new(Vector1::new(minor_arc)).rotate(&dir);
            p = centre + dir;
        }
        dir = Rotation2::new(Vector1::new(rad/2.)).rotate(&nalgebra::normalize(&Vector2::new(dirx, diry))) * r;
        p = centre + dir;
        vertx.push(p.x);
        verty.push(p.y);

        let normx = calc_normx(nvert, &verty);
        let normy = calc_normy(nvert, &vertx);

        Collidable
        {
            collidable_type: t,
            collidable_shape: CollidableShape::Polygon,
            collidable_id: id,

            centrex: cx,
            centrey: cy,
            radius: r,

            nvert: nvert,
            vertx: vertx,
            verty: verty,
            normx: normx,
            normy: normy,
        }
    }

	pub fn new_poly(t: u8, i: usize, nvert: usize, vertx: Vec<f32>, verty: Vec<f32>) -> Collidable
	{
		let normx = calc_normx(nvert, &verty);
		let normy = calc_normy(nvert, &vertx);
		Collidable
		{
			collidable_type: t,
            collidable_shape: CollidableShape::Polygon,
			collidable_id: i,

			centrex: 0.,
			centrey: 0.,
			radius: 0.,

			nvert: nvert,
			vertx: vertx,
			verty: verty,
			normx: normx,
			normy: normy,
		}
	}

	pub fn new_square(t: u8, i: usize, x: f32, y: f32, w: f32, h: f32) -> Collidable
	{
		let nvert: usize = 4;
	    let mut vertx = Vec::with_capacity(nvert);
	    vertx.push(x);
	    vertx.push(x + w);
	    vertx.push(x + w);
	    vertx.push(x);
		let mut verty = Vec::with_capacity(nvert);
		verty.push(y);
		verty.push(y);
		verty.push(y + h);
		verty.push(y + h);
		let normx = calc_normx(nvert, &verty);
		let normy = calc_normy(nvert, &vertx);
		Collidable
		{
			collidable_type: t,
            collidable_shape: CollidableShape::Polygon,
			collidable_id: i,

			centrex: 0.,
			centrey: 0.,
			radius: 0.,

			nvert: nvert,
			vertx: vertx,
			verty: verty,
			normx: normx,
			normy: normy,
		}
	}

	pub fn update_normals(&mut self)
	{
		self.normx = calc_normx(self.nvert, &self.verty);
		self.normy = calc_normy(self.nvert, &self.vertx);
	}
}
