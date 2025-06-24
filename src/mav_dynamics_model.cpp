#include "../inc/mav_dynamics_model.hpp"
#include <cmath>
#include <tuple>

MAVDynamicsModel::MAVDynamicsModel(float mass, float Jx, float Jy, float Jz,
                                   float Jxz, Vector3 init_pos,
                                   Vector3 init_rot, Vector3 init_vel,
                                   Vector3 init_ang_vel) {
  m_mass = mass;

  m_Jx = Jx;
  m_Jy = Jy;
  m_Jz = Jz;
  m_Jxz = Jxz;

  m_gamma = m_Jx * m_Jz - m_Jxz * m_Jxz;
  m_gamma1 = (m_Jxz * (m_Jx - m_Jy + m_Jz)) / m_gamma;
  m_gamma2 = (m_Jz * (m_Jz - m_Jy) + m_Jxz * m_Jxz) / m_gamma;
  m_gamma3 = m_Jz / m_gamma;
  m_gamma4 = m_Jxz / m_gamma;
  m_gamma5 = (m_Jz - m_Jx) / m_Jy;
  m_gamma6 = m_Jxz / m_Jy;
  m_gamma7 = ((m_Jx - m_Jy) * m_Jx + m_Jxz * m_Jxz) / m_gamma;
  m_gamma8 = m_Jx / m_gamma;

  m_pn = init_pos.x;
  m_pe = init_pos.z;
  m_pd = -init_pos.y;

  m_u = init_vel.x;
  m_v = init_vel.z;
  m_w = -init_vel.y;

  m_p = init_ang_vel.x;
  m_q = init_ang_vel.z;
  m_r = -init_ang_vel.y;

  float psi = init_rot.x;
  float phi = init_rot.z;
  float theta = -init_rot.y;
  m_e0 = cos(psi / 2.0) * cos(theta / 2.0) * cos(phi / 2.0) +
         sin(psi / 2.0) * sin(theta / 2.0) * sin(phi / 2.0);
  m_ex = cos(psi / 2.0) * cos(theta / 2.0) * sin(phi / 2.0) -
         sin(psi / 2.0) * sin(theta / 2.0) * cos(phi / 2.0);
  m_ey = cos(psi / 2.0) * sin(theta / 2.0) * cos(phi / 2.0) +
         sin(psi / 2.0) * cos(theta / 2.0) * sin(phi / 2.0);
  m_ez = sin(psi / 2.0) * cos(theta / 2.0) * cos(phi / 2.0) -
         cos(psi / 2.0) * sin(theta / 2.0) * sin(phi / 2.0);
}

Vector3 MAVDynamicsModel::get_pos() {
  Vector3 out;
  out.x = m_pn;
  out.y = -m_pd;
  out.z = m_pe;
  return out;
}

Vector3 MAVDynamicsModel::get_rot() {
  Vector3 out;
  out.x = atan2f(2.0f * (m_e0 * m_ex + m_ey * m_ez),
                 m_e0 * m_e0 + m_ez * m_ez - m_ex * m_ex - m_ey * m_ey);
  out.y = asinf(2 * (m_e0 * m_ey - m_ex * m_ez));
  out.z = atan2f(2.0f * (m_e0 * m_ez + m_ex * m_ey),
                 m_e0 * m_e0 + m_ex * m_ex - m_ey * m_ey - m_ez * m_ez);
  return out;
}

void MAVDynamicsModel::apply_force(float dt) {
  float u_wind = get_u_wind();
  float v_wind = get_v_wind();
  float w_wind = get_w_wind();

  float u_r = m_u - u_wind;
  float v_r = m_v - v_wind;
  float w_r = m_w - w_wind;

  float airspeed = get_airspeed(u_r, v_r, w_r);
  auto [thrust, torque] = get_thrust_and_torque(airspeed, m_throttle);
  float angle_of_attack = get_angle_of_attack(u_r, w_r);
  float side_slip = get_sideslip(u_r, v_r, w_r);
  float aero_coef = 0.5 * m_air_density * std::pow(airspeed, 2) * m_planform_area;
  float drag_coef = get_drag_coefficent(angle_of_attack);
  float lift_coef = get_lift_coefficent(angle_of_attack);

  float C_X = -drag_coef * cosf(angle_of_attack) + lift_coef * sinf(angle_of_attack);
  float C_Xq = -m_C_Dq * cosf(angle_of_attack) + m_C_Lq * sinf(angle_of_attack);
  float C_X_delta_e = -m_C_D_delta_e * cosf(angle_of_attack) + m_C_L_delta_e* sinf(angle_of_attack);
  float C_Z = -drag_coef * sinf(angle_of_attack) - lift_coef * cosf(angle_of_attack);
  float C_Zq = -m_C_Dq * sinf(angle_of_attack) - m_C_Lq * cosf(angle_of_attack);
  float C_Z_delta_e = -m_C_D_delta_e * sinf(angle_of_attack) - m_C_L_delta_e* cosf(angle_of_attack);

	float fx = (m_mass * m_gravity * 2 * (m_ex * m_ez - m_ey * m_e0)) +
		thrust +
		aero_coef * (C_X + C_Xq * (m_mean_chord / (2 * airspeed)) * m_q) +
                aero_coef * (C_Z_delta_e * m_elevator_deflection);
	float fy = (m_mass * m_gravity * 2 * (m_ey * m_ez + m_ex * m_e0)) + 
                   aero_coef * (m_C_Y0 + m_C_Y_beta * side_slip + m_C_Yp * (m_wingspan / (2 * airspeed)) * m_p + m_C_Yr * (m_wingspan / (2 * airspeed)) * m_r) +
                   aero_coef * (m_C_Y_delta_a * get_aileron_deflection() + m_C_Y_delta_r * m_rudder_deflection);
	float fz = (m_mass * m_gravity * (m_e0 * m_e0 + m_ez * m_ez - m_ex * m_ex - m_ey * m_ey)) + 
                   aero_coef * (C_Z * C_Zq * (m_mean_chord / (2 * airspeed)) * m_q) +
                   aero_coef * (C_Z_delta_e * m_elevator_deflection);
	float Mx = ;
	float My = ;
	float Mz = ;

	euler_step(dt, fx, fy, fz, Mx, My, Mz);
}

void MAVDynamicsModel::euler_step(float dt, float fx, float fy, float fz,
                                  float Mx, float My, float Mz) {
  float pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot, e0_dot, ex_dot, ey_dot,
      ez_dot, p_dot, q_dot, r_dot;

  pn_dot = (m_e0 * m_e0 + m_ex * m_ex - m_ey * m_ey - m_ez * m_ez) * m_u +
           2 * (m_ex * m_ey - m_e0 * m_ez) * m_v +
           2 * (m_ex * m_ez + m_e0 * m_ey) * m_w;
  pe_dot = 2 * (m_ex * m_ey + m_e0 * m_ez) * m_u +
           (m_e0 * m_e0 - m_ex * m_ex + m_ey * m_ey - m_ez * m_ez) * m_v +
           2 * (m_ey * m_ez - m_e0 * m_ex) * m_w;
  pd_dot = 2 * (m_ex * m_ez - m_e0 * m_ey) * m_u +
           2 * (m_ey * m_ez + m_e0 * m_ex) * m_v +
           (m_e0 * m_e0 - m_ex * m_ex - m_ey * m_ey + m_ez * m_ez) * m_w;

  u_dot = (m_r * m_v - m_q * m_w) + (fx / m_mass);
  v_dot = (m_p * m_w - m_r * m_u) + (fy / m_mass);
  w_dot = (m_q * m_u - m_p * m_v) + (fz / m_mass);

  e0_dot = (-m_p * m_ex + -m_q * m_ey + -m_r * m_ez) / 2.0f;
  ex_dot = (m_p * m_e0 + m_r * m_ey + -m_q * m_ez) / 2.0f;
  ey_dot = (m_q * m_e0 + -m_r * m_ex + m_p * m_ez) / 2.0f;
  ez_dot = (m_r * m_e0 + m_q * m_ex + -m_p * m_ey) / 2.0f;

  p_dot = (m_gamma1 * m_p * m_q - m_gamma2 * m_q * m_r) +
          (m_gamma3 * Mx + m_gamma4 * Mz);
  q_dot =
      (m_gamma5 * m_p * m_r - m_gamma6 * (m_p * m_p - m_r * m_r)) + (My / m_Jy);
  r_dot = (m_gamma7 * m_p * m_q - m_gamma1 * m_q * m_r) +
          (m_gamma4 * Mx + m_gamma8 * Mz);

  m_pn += dt * pn_dot;
  m_pe += dt * pe_dot;
  m_pd += dt * pd_dot;

  m_u += dt * u_dot;
  m_v += dt * v_dot;
  m_w += dt * w_dot;

  m_e0 += dt * e0_dot;
  m_ex += dt * ex_dot;
  m_ey += dt * ey_dot;
  m_ez += dt * ez_dot;

  m_p += dt * p_dot;
  m_q += dt * q_dot;
  m_r += dt * r_dot;
}

std::tuple<float, float> MAVDynamicsModel::get_thrust_and_torque(float throttle, float airspeed) {
  float volts_in = m_max_volts_motor * throttle;
  float propeller_speed = get_propeller_speed(volts_in, airspeed);
  float thrust = get_thrust(propeller_speed, airspeed);
  float torque = get_torque(propeller_speed, airspeed);
  return {thrust, torque};
}

float MAVDynamicsModel::get_propeller_speed(float volts_in, float airspeed) {
  float a = ((m_air_density * std::pow(m_propeller_diameter, 5)) /
             std::pow(2 * M_PI, 2)) *
            m_C_Q0;
  float b = ((m_air_density * std::pow(m_propeller_diameter, 4)) / (2 * M_PI)) *
                m_C_Q1 * airspeed +
            ((m_KQ * m_KV) / m_motor_winding_resistance);
  float c = m_air_density * std::pow(m_propeller_diameter, 3) * m_C_Q2 *
                std::pow(airspeed, 2) -
            (m_KQ / m_motor_winding_resistance) * volts_in +
            m_KQ * m_no_load_current;
  return (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);
}

float MAVDynamicsModel::get_thrust(float propeller_speed, float airspeed){
	return ((m_air_density * std::pow(m_propeller_diameter, 4) * m_C_T0) / (4 * std::pow(M_PI, 2))) * std::pow(propeller_speed, 2) + 
		((m_air_density * std::pow(m_propeller_diameter, 3) * m_C_T1 * airspeed) / (2 * M_PI)) * propeller_speed + 
		(m_air_density * std::pow(m_propeller_diameter, 2) * m_C_T2 * std::pow(airspeed, 2));
}

float MAVDynamicsModel::get_torque(float propeller_speed, float airspeed){
	return ((m_air_density * std::pow(m_propeller_diameter, 5) * m_C_Q0) / (4 * std::pow(M_PI, 2))) * std::pow(propeller_speed, 2) + 
		((m_air_density * std::pow(m_propeller_diameter, 4) * m_C_Q1 * airspeed) / (2 * M_PI)) * propeller_speed + 
		(m_air_density * std::pow(m_propeller_diameter, 3) * m_C_Q2 * std::pow(airspeed, 2));
}

float MAVDynamicsModel::get_drag_coefficent(float angle_of_attack){
  return (m_C_Dp + (std::pow(m_C_L0, 2) / (M_PI * m_oswald_efficency * m_wing_aspect_ratio))) +
        ((2 * m_C_L0 * m_C_La) / (M_PI * m_oswald_efficency * m_wing_aspect_ratio)) * angle_of_attack +
        (std::pow(m_C_La, 2) / (M_PI * m_oswald_efficency * m_wing_aspect_ratio)) * std::pow(angle_of_attack, 2);
}

float MAVDynamicsModel::get_lift_coefficent(float angle_of_attack){
  float sigmoid = (1 + std::pow(M_Ef, -m_M * (angle_of_attack - m_alpha0)) + std::pow(M_Ef, m_M * (angle_of_attack + m_alpha0))) / 
                  ((1 + std::pow(M_Ef, -m_M * (angle_of_attack - m_alpha0))) * (1 + std::pow(M_Ef, m_M * (angle_of_attack + m_alpha0))));
  return (1 - sigmoid) * (m_C_L0 + m_C_La * angle_of_attack) + sigmoid * (2 * (1 ? std::signbit(angle_of_attack) : 0) * std::pow(sinf(angle_of_attack), 2) * cosf(angle_of_attack));
}

float MAVDynamicsModel::get_airspeed(float u_r, float v_r, float w_r) {
  return sqrtf(u_r * u_r + v_r * v_r + w_r * w_r);
}

float MAVDynamicsModel::get_angle_of_attack(float u_r, float w_r) {
  return atanf(w_r / u_r);
}

float MAVDynamicsModel::get_sideslip(float u_r, float v_r, float w_r) {
  return atanf(v_r / sqrtf(u_r * u_r + w_r * w_r));
}
