extern crate nalgebra;

use nalgebra::{Vector2};
use std::f32;

pub fn poly_contains_pnt(nvert: usize, vertx: &[f32], verty: &[f32], pntx: f32, pnty: f32) -> bool
{
	let mut j = nvert-1;
	let mut c = false;
    for i in 0..nvert
	{
        if ((verty[i]>pnty) != (verty[j]>pnty)) &&
            (pntx < (vertx[j]-vertx[i]) * (pnty-verty[i]) / (verty[j]-verty[i]) + vertx[i])
		{
            c = !c;
        }
		j = i;
	}
    return c;
}

pub fn poly_contains_poly(na: usize, ax: &[f32], ay: &[f32], nb: usize, bx: &[f32], by: &[f32]) -> bool
{
    let mut c = true;
    for i in 0..nb
	{
        if !poly_contains_pnt(na, ax, ay, bx[i], by[i])
		{
            c = false;
        }
    }
    return c;
}


pub fn sqrf(x: f32) -> f32
{
	x * x
}

pub fn dist_sqrdf(vx: f32, vy: f32, wx: f32, wy: f32) -> f32
{
    sqrf(vx - wx) + sqrf(vy - wy)
}

pub fn dist_line_pnt(lax: f32, lay: f32, lbx: f32, lby: f32, px: f32, py: f32) -> f32
{
    let length_sqr = dist_sqrdf(lax, lay, lbx, lby);
    if length_sqr == 0. { return dist_sqrdf(px, py, lax, lay); }
    let t = ((px - lax) * (lbx - lax) + (py - lay) * (lby - lay)) / length_sqr;
    if t < 0. { return dist_sqrdf(px, py, lax, lay).sqrt(); }
    if t > 1. { return dist_sqrdf(px, py, lbx, lby).sqrt(); }
    let cx = lax + t * (lbx - lax);
    let cy = lay + t * (lby - lay);
    dist_sqrdf(px, py, cx, cy).sqrt()
}

pub fn dist_line_pnt__closest_pnt(lax: f32, lay: f32, lbx: f32, lby: f32, px: f32, py: f32) -> (f32, f32, f32)
{
	let mut closest_x = px;
	let mut closest_y = py;
    let length_sqr = dist_sqrdf(lax, lay, lbx, lby);
    if length_sqr == 0. { return (dist_sqrdf(px, py, lax, lay), closest_x, closest_y); }
    let t = ((px - lax) * (lbx - lax) + (py - lay) * (lby - lay)) / length_sqr;
    if t < 0.
	{
		closest_x = lax;
		closest_y = lay;
        return (dist_sqrdf(px, py, lax, lay).sqrt(), closest_x, closest_y);
    }
    if t > 1.
	{
		closest_x = lbx;
		closest_y = lby;
        return (dist_sqrdf(px, py, lbx, lby).sqrt(), closest_x, closest_y);
    }
    let cx = lax + t * (lbx - lax);
    let cy = lay + t * (lby - lay);
    closest_x = cx;
	closest_y = cy;
    (dist_sqrdf(px, py, cx, cy).sqrt(), closest_x, closest_y)
}

pub fn dist_poly_circ(nvert: usize, vertx: &[f32], verty: &[f32], circx: f32, circy: f32) -> f32
{
    let mut distance = f32::MAX;
    let mut temp: f32;;
    for i in 0..nvert-1
	{
        temp = dist_line_pnt(vertx[i], verty[i], vertx[i+1], verty[i+1], circx, circy);
        if temp < distance { distance = temp; }
    }
    temp = dist_line_pnt(vertx[nvert-1], verty[nvert-1], vertx[0], verty[0], circx, circy);
    if temp < distance { distance = temp; }
    distance
}

pub fn dist_poly_circ__face__support(nvert: usize, vertx: &[f32], verty: &[f32], circx: f32, circy: f32) -> (f32, usize, Vector2<f32>)
{
	let mut closest_x = 0.;
	let mut closest_y = 0.;
	let mut face_index = 0;
    let mut distance = f32::MAX;
    let mut temp: f32;
    for i in 0..nvert-1
	{
		let cx: f32;
		let cy: f32;
        temp = match dist_line_pnt__closest_pnt(vertx[i], verty[i], vertx[i+1], verty[i+1], circx, circy)
		{
			(d, x, y) =>
			{
				cx = x;
				cy = y;
				d
			}
		};
        if temp < distance
		{
            distance = temp;
            closest_x = cx;
			closest_y = cy;
            face_index = i;
        }
    }
	let cx: f32;
	let cy: f32;
    temp = match dist_line_pnt__closest_pnt(vertx[nvert-1], verty[nvert-1], vertx[0], verty[0], circx, circy)
	{
		(d, x, y) =>
		{
			cx = x;
			cy = y;
			d
		}
	};
    if temp < distance
	{
        distance = temp;
        closest_x = cx;
		closest_y = cy;
        face_index = nvert-1;
    }
    (distance, face_index, Vector2::new(closest_x, closest_y))
}

pub fn dist_circ_circ(ax: f32, ay: f32, bx: f32, by: f32) -> f32
{
    ((ax - bx) * (ax - bx) + (ay - by) * (ay - by)).sqrt()
}

pub fn calc_normx(nvert: usize, verty: &[f32]) -> Vec<f32>
{
	let mut normx = vec![0.; nvert];
	for i in 0..nvert-1
	{
        normx[i] = verty[i+1] - verty[i];
	}
    normx[nvert-1] = verty[0] - verty[nvert-1];
	normx
}

pub fn calc_normy(nvert: usize, vertx: &[f32]) -> Vec<f32>
{
	let mut normy = vec![0.; nvert];
	for i in 0..nvert-1
	{
		normy[i] = -vertx[i+1] + vertx[i];
	}
	normy[nvert-1] = -vertx[0] + vertx[nvert-1];
	normy
}

pub fn support_pnt(nvert: usize, vertx: &[f32], verty: &[f32], dir: Vector2<f32>) -> Vector2<f32>
{
	let mut best_proj = f32::MIN;
	let mut best_vert = nalgebra::zero();
	for i in 0..nvert
	{
		let vert = Vector2::new(vertx[i], verty[i]);
		let proj = nalgebra::dot(&vert, &dir);
        if proj > best_proj
		{
            best_vert = vert;
            best_proj = proj;
        }
	}
	best_vert
}

pub fn penetration__face__support(nverta: usize, vertxa: &[f32], vertya: &[f32], normxa: &[f32], normya: &[f32], nvertb: usize, vertxb: &[f32], vertyb: &[f32]) -> (f32, usize, Vector2<f32>)
{
    let mut best_dist = f32::MIN;
    let mut best_face = 0;
    let mut best_supp = nalgebra::zero();
    for i in 0..nverta
	{
        let n = Vector2::new(normxa[i], normya[i]);
        let s = support_pnt(nvertb, vertxb, vertyb, -n);
      	let v = Vector2::new(vertxa[i], vertya[i]);
        let d = nalgebra::dot(&n, &(s - v));
        if d > best_dist
		{
            best_dist = d;
            best_face = i;
            best_supp = s;
        }
    }
    (best_dist, best_face, best_supp)
}
