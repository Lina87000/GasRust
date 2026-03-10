use macroquad::prelude::*;
use std::collections::HashMap;

const N_PARTICLES: usize = 10000;
const R: f64 = 3.0;

const V_MIN: f64 = 15.0;
const V_MAX: f64 = 100.0;

const CELL_SIZE: f64 = R * 4.0;

#[derive(Clone)]
struct Particles {
    x: Vec<f64>,
    y: Vec<f64>,
    vx: Vec<f64>,
    vy: Vec<f64>,
}

impl Particles {
    fn new() -> Particles {
        Particles {
            x: vec![0.0; N_PARTICLES],
            y: vec![0.0; N_PARTICLES],
            vx: vec![0.0; N_PARTICLES],
            vy: vec![0.0; N_PARTICLES],
        }
    }

    fn update_pos(&mut self, index: usize, x_new: f64, y_new: f64) {
        if index < self.x.len() {
            self.x[index] = x_new;
            self.y[index] = y_new;
        }
    }

    fn update_vel(&mut self, index: usize, vx_new: f64, vy_new: f64) {
        if index < self.vx.len() {
            self.vx[index] = vx_new;
            self.vy[index] = vy_new;
        }
    }

    fn init(&mut self) {
        rand::srand(miniquad::date::now() as u64);
        
        // Calculate lattice dimensions
        let spacing = R * 2.5;  // Space between particle centers (adjust for density)
        let cols = ((screen_width() as f64 - 2.0 * R) / spacing).floor() as usize;
        let rows = ((screen_height() as f64 - 2.0 * R) / spacing).floor() as usize;
        
        let mut particle_index = 0;
        
        // Place particles in a regular grid
        'outer: for row in 0..rows {
            for col in 0..cols {
                if particle_index >= N_PARTICLES {
                    break 'outer;
                }
                
                // Calculate grid position with offset to center the grid
                let x = R + spacing / 2.0 + col as f64 * spacing;
                let y = R + spacing / 2.0 + row as f64 * spacing;
                
                // Random velocity (still randomized for interesting dynamics)
                let theta = rand::gen_range(0.0, 2.0 * std::f64::consts::PI);
                let speed = rand::gen_range(V_MIN, V_MAX);
                let vx = f64::cos(theta) * speed;
                let vy = f64::sin(theta) * speed;
                
                self.update_pos(particle_index, x, y);
                self.update_vel(particle_index, vx, vy);
                
                particle_index += 1;
            }
        }
        
        // If we have leftover particles (grid couldn't fit all), place them randomly
        while particle_index < N_PARTICLES {
            let x = rand::gen_range(R, screen_width() as f64 - R);
            let y = rand::gen_range(R, screen_height() as f64 - R);
            
            let theta = rand::gen_range(0.0, 2.0 * std::f64::consts::PI);
            let speed = rand::gen_range(V_MIN, V_MAX);
            let vx = f64::cos(theta) * speed;
            let vy = f64::sin(theta) * speed;
            
            self.update_pos(particle_index, x, y);
            self.update_vel(particle_index, vx, vy);
            
            particle_index += 1;
        }
    }

    fn collision(&mut self, index: usize) {
        //Flip x velocity if particle reaches left and right walls.
        if self.x[index] < R {
            self.vx[index] *= -1.0;
            self.x[index] = R;
        } else if self.x[index] > screen_width() as f64 - R {
            self.vx[index] *= -1.0;
            self.x[index] = screen_width() as f64 - R;
        }

        //Flip y velocity if particle reaches top and bottom walls.
        if self.y[index] < R {
            self.vy[index] *= -1.0;
            self.y[index] = R;
        } else if self.y[index] > screen_height() as f64 - R {
            self.vy[index] *= -1.0;
            self.y[index] = screen_height() as f64 - R;
        }
    }

fn particle_collisions_spatial(&mut self) {
        // Build spatial grid
        let mut grid: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        
        for i in 0..N_PARTICLES {
            let cell_x = (self.x[i] / CELL_SIZE) as i32;
            let cell_y = (self.y[i] / CELL_SIZE) as i32;
            grid.entry((cell_x, cell_y))
                .or_insert_with(Vec::new)
                .push(i);
        }
        
        // Check collisions only within nearby cells
        const MIN_DIST_SQ: f64 = (R * 2.0) * (R * 2.0);
        
        for i in 0..N_PARTICLES {
            let cell_x = (self.x[i] / CELL_SIZE) as i32;
            let cell_y = (self.y[i] / CELL_SIZE) as i32;
            
            // Check current cell and 8 neighboring cells
            for dx in -1..=1 {
                for dy in -1..=1 {
                    let check_cell = (cell_x + dx, cell_y + dy);
                    
                    if let Some(cell_particles) = grid.get(&check_cell) {
                        for &j in cell_particles {
                            if j <= i { continue; }  // Avoid duplicate checks
                            
                            let dx = self.x[j] - self.x[i];
                            let dy = self.y[j] - self.y[i];
                            let dist_sq = dx * dx + dy * dy;
                            
                            if dist_sq < MIN_DIST_SQ && dist_sq > 0.001 {
                                let dist = dist_sq.sqrt();
                                
                                // Normalize collision vector
                                let nx = dx / dist;
                                let ny = dy / dist;
                                
                                // Relative velocity
                                let dvx = self.vx[i] - self.vx[j];
                                let dvy = self.vy[i] - self.vy[j];
                                
                                // Relative velocity in collision normal direction
                                let dvn = dvx * nx + dvy * ny;
                                
                                if dvn > 0.0 {
                                    self.vx[i] -= dvn * nx;
                                    self.vy[i] -= dvn * ny;
                                    self.vx[j] += dvn * nx;
                                    self.vy[j] += dvn * ny;
                                    
                                    // Separate overlapping particles
                                    let min_dist = R * 2.0;
                                    let overlap = min_dist - dist;
                                    let separation = overlap / 2.0;
                                    self.x[i] -= separation * nx;
                                    self.y[i] -= separation * ny;
                                    self.x[j] += separation * nx;
                                    self.y[j] += separation * ny;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}


fn draw(p: &Particles, colour: Color) {
    const CIRCLE_SIDES: u8 = 6;
    
    for i in 0..p.x.len() {
        draw_poly(
            p.x[i] as f32,
            p.y[i] as f32,
            CIRCLE_SIDES,
            R as f32,
            0.0,
            colour,
        );
    }
}

fn simulate(p: &mut Particles, dt: f64) {
    for _i in 0..p.x.len() {
        p.x[_i] += p.vx[_i] * dt;
        p.y[_i] += p.vy[_i] * dt;
        p.collision(_i);
    }
    p.particle_collisions_spatial();
}

fn kinetic_energy(p: &Particles) -> f64 {
    let mut ke = 0.0;
    for i in 0..p.x.len() {
        ke += 0.5 * (p.vx[i] * p.vx[i] + p.vy[i] * p.vy[i]);
    }
    ke
}

#[macroquad::main("2D Gas simulation")]
async fn main() {
    let mut p = Particles::new();
    p.init();

    println!("Screen Height: {:.0}", screen_height());

    loop {
        clear_background(WHITE);
        
        let dt = get_frame_time() as f64;
        let fps = 1.0 / dt;
        
        draw(&p, RED);
        draw_text("Gas2D Simulation", 20.0, 20.0, 30.0, DARKGRAY);
        draw_text(&format!("FPS : {:.0}", fps), 20.0, 50.0, 30.0, DARKGRAY);
        draw_text(&format!("Particles : {:.0}", N_PARTICLES), screen_width() - 350.0, 20.0, 30.0, DARKGRAY);

        let ke = kinetic_energy(&p);
        draw_text(&format!("Kinetic Energy : {:.0}", ke), 20.0, 80.0, 30.0, DARKGRAY);

        

        simulate(&mut p, dt);

        next_frame().await;
    }
}