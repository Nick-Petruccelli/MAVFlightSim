#include "../inc/mav_dynamics_model.hpp"
#include <cmath>
#include <tuple>
#include <rapidjson/document.h>
#include <fstream>
#include <iostream>
#include <vector>

const float SOFTENING_FACTOR = 0.00000000000001;

MAVDynamicsModel::MAVDynamicsModel(std::string config_path) {
  std::cout << "Initializing Simulation" << std::endl;

  std::ifstream ifs(config_path.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
  std::ifstream::pos_type file_size = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  std::vector<char> bytes(file_size);
  ifs.read(bytes.data(), file_size);
  std::string json = std::string(bytes.data(), file_size);

  rapidjson::Document config_doc;
  config_doc.Parse(json.c_str());

  validate_config_file(config_doc);
  set_sim_params(config_doc);
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
  float u_wind = 0;//get_u_wind();
  float v_wind = 0;//get_v_wind();
  float w_wind = 0;//get_w_wind();

  float u_r = m_u - u_wind;
  float v_r = m_v - v_wind;
  float w_r = m_w - w_wind;

  float airspeed = get_airspeed(u_r, v_r, w_r);
  auto [thrust, torque] = get_thrust_and_torque(m_throttle, airspeed);
  float angle_of_attack = get_angle_of_attack(u_r, w_r);
  float side_slip = get_sideslip(u_r, v_r, w_r);
  float aero_coef = 0.5 * m_air_density * std::pow(airspeed, 2) * m_planform_area;
  float drag_coef = get_drag_coefficent(angle_of_attack);
  float lift_coef = get_lift_coefficent(angle_of_attack);
  float aileron_defection = get_aileron_deflection();

  float C_X = -drag_coef * cosf(angle_of_attack) + lift_coef * sinf(angle_of_attack);
  float C_Xq = -m_C_Dq * cosf(angle_of_attack) + m_C_Lq * sinf(angle_of_attack);
  float C_X_delta_e = -m_C_D_delta_e * cosf(angle_of_attack) + m_C_L_delta_e* sinf(angle_of_attack);
  float C_Z = -drag_coef * sinf(angle_of_attack) - lift_coef * cosf(angle_of_attack);
  float C_Zq = -m_C_Dq * sinf(angle_of_attack) - m_C_Lq * cosf(angle_of_attack);
  float C_Z_delta_e = -m_C_D_delta_e * sinf(angle_of_attack) - m_C_L_delta_e* cosf(angle_of_attack);

  std::cout << "airspeed: " << airspeed << std::endl;
  std::cout << "thrust: " << thrust << std::endl;
  std::cout << "torque: " << torque << std::endl;
  std::cout << "angle_of_attack: " << angle_of_attack << std::endl;
  std::cout << "side_slip: " << side_slip << std::endl;
  std::cout << "aero_coef: " << aero_coef << std::endl;
  std::cout << "drag_coef: " << drag_coef << std::endl;
  std::cout << "lift_coef: " << lift_coef << std::endl;
  std::cout << "aileron_deflection: " << aileron_defection << std::endl;

  std::cout << "C_X: " << C_X << std::endl;
  std::cout << "C_Xq: " << C_Xq << std::endl;
  std::cout << "C_X_delta_e: " << C_X_delta_e << std::endl;
  std::cout << "C_Z: " << C_Z << std::endl;
  std::cout << "C_Zq: " << C_Zq << std::endl;
  std::cout << "C_Z_delta_e: " << C_Z_delta_e << std::endl;
  std::cout << "C_Y_delta_a: " << m_C_Y_delta_a << std::endl;
  std::cout << "C_Mx_delta_a: " << m_C_Mx_delta_a << std::endl;
  std::cout << "C_Mz_delta_a: " << m_C_Mz_delta_a << std::endl;

	float fx = (m_mass * m_gravity * 2 * (m_ex * m_ez - m_ey * m_e0)) +
		thrust +
		aero_coef * (C_X + C_Xq * (m_mean_chord / (2 * airspeed)) * m_q) +
                aero_coef * (C_Z_delta_e * m_elevator_deflection);
	float fy = (m_mass * m_gravity * 2 * (m_ey * m_ez + m_ex * m_e0)) + 
                   aero_coef * (m_C_Y0 + m_C_Y_beta * side_slip + m_C_Yp * (m_wingspan / (2 * airspeed)) * m_p + m_C_Yr * (m_wingspan / (2 * airspeed)) * m_r) +
                   aero_coef * (m_C_Y_delta_a * aileron_defection + m_C_Y_delta_r * m_rudder_deflection);
	float fz = (m_mass * m_gravity * (m_e0 * m_e0 + m_ez * m_ez - m_ex * m_ex - m_ey * m_ey)) + 
                   aero_coef * (C_Z * C_Zq * (m_mean_chord / (2 * airspeed)) * m_q) +
                   aero_coef * (C_Z_delta_e * m_elevator_deflection);
	float Mx = aero_coef * (m_wingspan * (m_C_Mx_0 + m_C_Mx_beta * side_slip + m_C_Mx_p * (m_wingspan / (2 * airspeed)) * m_p + m_C_Mx_r * (m_wingspan / (2 * airspeed)) * m_r)) +
                   aero_coef * (m_wingspan * (m_C_Mx_delta_a * aileron_defection + m_C_Mx_delta_r * m_rudder_deflection)) - torque;
	float My = aero_coef * (m_mean_chord * (m_C_My_0 + m_C_My_alpha * angle_of_attack + m_C_My_q * (m_mean_chord / (2 * airspeed)) * m_q)) +
                   aero_coef * (m_mean_chord * m_C_My_delta_e * m_elevator_deflection);
	float Mz = aero_coef * (m_wingspan * (m_C_Mz_0 + m_C_Mz_beta * side_slip + m_C_Mz_p * (m_wingspan / (2 * airspeed)) * m_p + m_C_Mz_r * (m_wingspan / (2 * airspeed)) * m_r)) +
                   aero_coef * (m_wingspan * (m_C_Mz_delta_a * aileron_defection + m_C_Mz_delta_r * m_rudder_deflection));

        std::cout << "force XYZ: " << fx << " " << fy << " " << fz << std::endl;
        std::cout << "moments XYZ: " << Mx << " " << My << " " << Mz << std::endl;
	euler_step(dt, fx, fy, fz, Mx, My, Mz);
        std::cout << "XYZ: " << m_pn << " " << m_pe << " " << -m_pd << std::endl;
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

void MAVDynamicsModel::validate_config_file(rapidjson::Document& config_doc) {
  std::cout << "Validating Config" << std::endl;

  assert(config_doc.IsObject());

  assert(config_doc.HasMember("init_conditions") && config_doc["init_conditions"].IsObject());
  assert(config_doc.HasMember("environment") && config_doc["environment"].IsObject());
  assert(config_doc.HasMember("physical_params") && config_doc["environment"].IsObject());
  assert(config_doc.HasMember("motor_params") && config_doc["environment"].IsObject());
  assert(config_doc.HasMember("aerodynamic_coefficents") && config_doc["environment"].IsObject());

  assert(config_doc["init_conditions"].HasMember("position") && config_doc["init_conditions"]["position"].IsArray());
  assert(config_doc["init_conditions"].HasMember("rotation") && config_doc["init_conditions"]["rotation"].IsArray());
  assert(config_doc["init_conditions"].HasMember("velocity") && config_doc["init_conditions"]["velocity"].IsArray());
  assert(config_doc["init_conditions"].HasMember("angular_velocity") && config_doc["init_conditions"]["angular_velocity"].IsArray());
  assert(config_doc["init_conditions"].HasMember("aileron_r_deflection") && config_doc["init_conditions"]["aileron_r_deflection"].IsFloat());
  assert(config_doc["init_conditions"].HasMember("aileron_l_deflection") && config_doc["init_conditions"]["aileron_l_deflection"].IsFloat());
  assert(config_doc["init_conditions"].HasMember("rudder_deflection") && config_doc["init_conditions"]["rudder_deflection"].IsFloat());
  assert(config_doc["init_conditions"].HasMember("elevator_deflection") && config_doc["init_conditions"]["elevator_deflection"].IsFloat());
  assert(config_doc["init_conditions"].HasMember("throttle") && config_doc["init_conditions"]["throttle"].IsFloat());

  assert(config_doc["environment"].HasMember("gravity") && config_doc["environment"]["gravity"].IsFloat());
  assert(config_doc["environment"].HasMember("air_density") && config_doc["environment"]["air_density"].IsFloat());

  assert(config_doc["physical_params"].HasMember("mass") && config_doc["physical_params"]["mass"].IsFloat());
  assert(config_doc["physical_params"].HasMember("Jx") && config_doc["physical_params"]["Jx"].IsFloat());
  assert(config_doc["physical_params"].HasMember("Jy") && config_doc["physical_params"]["Jy"].IsFloat());
  assert(config_doc["physical_params"].HasMember("Jz") && config_doc["physical_params"]["Jz"].IsFloat());
  assert(config_doc["physical_params"].HasMember("Jxz") && config_doc["physical_params"]["Jxz"].IsFloat());
  assert(config_doc["physical_params"].HasMember("S") && config_doc["physical_params"]["S"].IsFloat());
  assert(config_doc["physical_params"].HasMember("wingspan") && config_doc["physical_params"]["wingspan"].IsFloat());
  assert(config_doc["physical_params"].HasMember("mean_chord") && config_doc["physical_params"]["mean_chord"].IsFloat());
  assert(config_doc["physical_params"].HasMember("oswald_efficency") && config_doc["physical_params"]["oswald_efficency"].IsFloat());

  assert(config_doc["motor_params"].HasMember("volts_max") && config_doc["motor_params"]["volts_max"].IsFloat());
  assert(config_doc["motor_params"].HasMember("prop_diameter") && config_doc["motor_params"]["prop_diameter"].IsFloat());
  assert(config_doc["motor_params"].HasMember("Kv") && config_doc["motor_params"]["Kv"].IsFloat());
  assert(config_doc["motor_params"].HasMember("KQ") && config_doc["motor_params"]["KQ"].IsFloat());
  assert(config_doc["motor_params"].HasMember("winding_resistance") && config_doc["motor_params"]["winding_resistance"].IsFloat());
  assert(config_doc["motor_params"].HasMember("i0") && config_doc["motor_params"]["i0"].IsFloat());
  assert(config_doc["motor_params"].HasMember("C_Q2") && config_doc["motor_params"]["C_Q2"].IsFloat());
  assert(config_doc["motor_params"].HasMember("C_Q1") && config_doc["motor_params"]["C_Q1"].IsFloat());
  assert(config_doc["motor_params"].HasMember("C_Q0") && config_doc["motor_params"]["C_Q0"].IsFloat());
  assert(config_doc["motor_params"].HasMember("C_T2") && config_doc["motor_params"]["C_T2"].IsFloat());
  assert(config_doc["motor_params"].HasMember("C_T1") && config_doc["motor_params"]["C_T1"].IsFloat());
  assert(config_doc["motor_params"].HasMember("C_T0") && config_doc["motor_params"]["C_T0"].IsFloat());

  assert(config_doc["aerodynamic_coefficents"].HasMember("C_L0") && config_doc["aerodynamic_coefficents"]["C_L0"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_D0") && config_doc["aerodynamic_coefficents"]["C_D0"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_My_0") && config_doc["aerodynamic_coefficents"]["C_My_0"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_L_alpha") && config_doc["aerodynamic_coefficents"]["C_L_alpha"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_D_alpha") && config_doc["aerodynamic_coefficents"]["C_D_alpha"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_My_alpha") && config_doc["aerodynamic_coefficents"]["C_My_alpha"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Lq") && config_doc["aerodynamic_coefficents"]["C_Lq"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Dq") && config_doc["aerodynamic_coefficents"]["C_Dq"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_My_q") && config_doc["aerodynamic_coefficents"]["C_My_q"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_L_delta_e") && config_doc["aerodynamic_coefficents"]["C_L_delta_e"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_D_delta_e") && config_doc["aerodynamic_coefficents"]["C_D_delta_e"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_My_delta_e") && config_doc["aerodynamic_coefficents"]["C_My_delta_e"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("M") && config_doc["aerodynamic_coefficents"]["M"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("alpha0") && config_doc["aerodynamic_coefficents"]["alpha0"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Dp") && config_doc["aerodynamic_coefficents"]["C_Dp"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Y0") && config_doc["aerodynamic_coefficents"]["C_Y0"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Mx_0") && config_doc["aerodynamic_coefficents"]["C_Mx_0"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Mx_0") && config_doc["aerodynamic_coefficents"]["C_Mx_0"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Y_beta") && config_doc["aerodynamic_coefficents"]["C_Y_beta"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Mx_beta") && config_doc["aerodynamic_coefficents"]["C_Mx_beta"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Mz_beta") && config_doc["aerodynamic_coefficents"]["C_Mz_beta"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Yp") && config_doc["aerodynamic_coefficents"]["C_Yp"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Mx_beta") && config_doc["aerodynamic_coefficents"]["C_Mx_beta"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Mz_beta") && config_doc["aerodynamic_coefficents"]["C_Mz_beta"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Yr") && config_doc["aerodynamic_coefficents"]["C_Yr"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Mx_r") && config_doc["aerodynamic_coefficents"]["C_Mx_r"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Mz_r") && config_doc["aerodynamic_coefficents"]["C_Mz_r"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Y_delta_a") && config_doc["aerodynamic_coefficents"]["C_Y_delta_a"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Mx_delta_a") && config_doc["aerodynamic_coefficents"]["C_Mx_delta_a"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Mz_delta_a") && config_doc["aerodynamic_coefficents"]["C_Mz_delta_a"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Y_delta_r") && config_doc["aerodynamic_coefficents"]["C_Y_delta_r"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Mx_delta_r") && config_doc["aerodynamic_coefficents"]["C_Mx_delta_r"].IsFloat());
  assert(config_doc["aerodynamic_coefficents"].HasMember("C_Mz_delta_r") && config_doc["aerodynamic_coefficents"]["C_Mz_delta_r"].IsFloat());
}

void MAVDynamicsModel::set_sim_params(rapidjson::Document& config_doc) {
  std::cout << "Setting Simulation Parameters" << std::endl;

  m_gravity = config_doc["environment"]["gravity"].GetFloat();
  m_air_density = config_doc["environment"]["air_density"].GetFloat();

  m_mass = config_doc["physical_params"]["mass"].GetFloat();
  m_Jx = config_doc["physical_params"]["Jx"].GetFloat();
  m_Jy = config_doc["physical_params"]["Jy"].GetFloat();
  m_Jz = config_doc["physical_params"]["Jz"].GetFloat();
  m_Jxz = config_doc["physical_params"]["Jxz"].GetFloat();
  m_planform_area = config_doc["physical_params"]["S"].GetFloat();
  m_wingspan = config_doc["physical_params"]["wingspan"].GetFloat();
  m_mean_chord = config_doc["physical_params"]["mean_chord"].GetFloat();
  m_wing_aspect_ratio = std::pow(m_wingspan, 2) / m_planform_area;
  m_oswald_efficency = config_doc["physical_params"]["oswald_efficency"].GetFloat();

  m_max_volts_motor = config_doc["motor_params"]["volts_max"].GetFloat();
  m_propeller_diameter = config_doc["motor_params"]["prop_diameter"].GetFloat();
  m_KV = config_doc["motor_params"]["Kv"].GetFloat();
  m_KQ = config_doc["motor_params"]["KQ"].GetFloat();
  m_motor_winding_resistance = config_doc["motor_params"]["winding_resistance"].GetFloat();
  m_no_load_current = config_doc["motor_params"]["i0"].GetFloat();
  m_C_Q2 = config_doc["motor_params"]["C_Q2"].GetFloat();
  m_C_Q1 = config_doc["motor_params"]["C_Q1"].GetFloat();
  m_C_Q0 = config_doc["motor_params"]["C_Q0"].GetFloat();
  m_C_T2 = config_doc["motor_params"]["C_T2"].GetFloat();
  m_C_T1 = config_doc["motor_params"]["C_T1"].GetFloat();
  m_C_T0 = config_doc["motor_params"]["C_T0"].GetFloat();

  //aerodynamic coefficents
  m_C_L0 = config_doc["aerodynamic_coefficents"]["C_L0"].GetFloat();
  m_C_D0 = config_doc["aerodynamic_coefficents"]["C_D0"].GetFloat();
  m_C_My_0 = config_doc["aerodynamic_coefficents"]["C_My_0"].GetFloat();
  m_C_L_alpha = config_doc["aerodynamic_coefficents"]["C_L_alpha"].GetFloat();
  m_C_D_alpha = config_doc["aerodynamic_coefficents"]["C_D_alpha"].GetFloat();
  m_C_My_alpha = config_doc["aerodynamic_coefficents"]["C_My_alpha"].GetFloat();
  m_C_Lq = config_doc["aerodynamic_coefficents"]["C_Lq"].GetFloat();
  m_C_Dq = config_doc["aerodynamic_coefficents"]["C_Dq"].GetFloat();
  m_C_My_q = config_doc["aerodynamic_coefficents"]["C_My_q"].GetFloat();
  m_C_L_delta_e = config_doc["aerodynamic_coefficents"]["C_L_delta_e"].GetFloat();
  m_C_D_delta_e = config_doc["aerodynamic_coefficents"]["C_D_delta_e"].GetFloat();
  m_C_My_delta_e = config_doc["aerodynamic_coefficents"]["C_My_delta_e"].GetFloat();
  m_M = config_doc["aerodynamic_coefficents"]["M"].GetFloat();
  m_alpha0 = config_doc["aerodynamic_coefficents"]["alpha0"].GetFloat();
  m_C_Dp = config_doc["aerodynamic_coefficents"]["C_Dp"].GetFloat();
  m_C_Y0 = config_doc["aerodynamic_coefficents"]["C_Y0"].GetFloat();
  m_C_Mz_0 = config_doc["aerodynamic_coefficents"]["C_Mz_0"].GetFloat();
  m_C_Mz_0 = config_doc["aerodynamic_coefficents"]["C_Mz_0"].GetFloat();
  m_C_Y_beta = config_doc["aerodynamic_coefficents"]["C_Y_beta"].GetFloat();
  m_C_Mx_beta = config_doc["aerodynamic_coefficents"]["C_Mx_beta"].GetFloat();
  m_C_Mz_beta = config_doc["aerodynamic_coefficents"]["C_Mz_beta"].GetFloat();
  m_C_Yp = config_doc["aerodynamic_coefficents"]["C_Yp"].GetFloat();
  m_C_Mx_p = config_doc["aerodynamic_coefficents"]["C_Mx_p"].GetFloat();
  m_C_Mz_p = config_doc["aerodynamic_coefficents"]["C_Mz_p"].GetFloat();
  m_C_Yr = config_doc["aerodynamic_coefficents"]["C_Yr"].GetFloat();
  m_C_Mx_r = config_doc["aerodynamic_coefficents"]["C_Mx_r"].GetFloat();
  m_C_Mz_r = config_doc["aerodynamic_coefficents"]["C_Mz_r"].GetFloat();
  m_C_Y_delta_a = config_doc["aerodynamic_coefficents"]["C_Y_delta_a"].GetFloat();
  m_C_Mx_delta_a = config_doc["aerodynamic_coefficents"]["C_Mx_delta_a"].GetFloat();
  m_C_Mz_delta_a= config_doc["aerodynamic_coefficents"]["C_Mz_delta_a"].GetFloat();
  m_C_Y_delta_r = config_doc["aerodynamic_coefficents"]["C_Y_delta_r"].GetFloat();
  m_C_Mx_delta_r = config_doc["aerodynamic_coefficents"]["C_Mx_delta_r"].GetFloat();
  m_C_Mz_delta_r= config_doc["aerodynamic_coefficents"]["C_Mz_delta_r"].GetFloat();


  m_gamma = m_Jx * m_Jz - m_Jxz * m_Jxz;
  m_gamma1 = (m_Jxz * (m_Jx - m_Jy + m_Jz)) / m_gamma;
  m_gamma2 = (m_Jz * (m_Jz - m_Jy) + m_Jxz * m_Jxz) / m_gamma;
  m_gamma3 = m_Jz / m_gamma;
  m_gamma4 = m_Jxz / m_gamma;
  m_gamma5 = (m_Jz - m_Jx) / m_Jy;
  m_gamma6 = m_Jxz / m_Jy;
  m_gamma7 = ((m_Jx - m_Jy) * m_Jx + m_Jxz * m_Jxz) / m_gamma;
  m_gamma8 = m_Jx / m_gamma;

  const rapidjson::Value& init_pos = config_doc["init_conditions"]["position"].GetArray();
  m_pn = init_pos[0].GetFloat();
  m_pe = init_pos[1].GetFloat();
  m_pd = -init_pos[2].GetFloat();

  std::cout << "XYZ: " << m_pn << " " << m_pe << " " << -m_pd << std::endl;
  const rapidjson::Value& init_vel = config_doc["init_conditions"]["velocity"].GetArray();
  m_u = init_vel[0].GetFloat();
  m_v = init_vel[1].GetFloat();
  m_w = -init_vel[2].GetFloat();

  const rapidjson::Value& init_ang_vel = config_doc["init_conditions"]["angular_velocity"].GetArray();
  m_p = init_ang_vel[0].GetFloat();
  m_q = init_ang_vel[1].GetFloat();
  m_r = -init_ang_vel[2].GetFloat();

  const rapidjson::Value& init_rot = config_doc["init_conditions"]["rotation"].GetArray();
  float psi = init_rot[0].GetFloat();
  float phi = init_rot[1].GetFloat();
  float theta = -init_rot[2].GetFloat();
  m_e0 = cos(psi / 2.0) * cos(theta / 2.0) * cos(phi / 2.0) +
         sin(psi / 2.0) * sin(theta / 2.0) * sin(phi / 2.0);
  m_ex = cos(psi / 2.0) * cos(theta / 2.0) * sin(phi / 2.0) -
         sin(psi / 2.0) * sin(theta / 2.0) * cos(phi / 2.0);
  m_ey = cos(psi / 2.0) * sin(theta / 2.0) * cos(phi / 2.0) +
         sin(psi / 2.0) * cos(theta / 2.0) * sin(phi / 2.0);
  m_ez = sin(psi / 2.0) * cos(theta / 2.0) * cos(phi / 2.0) -
         cos(psi / 2.0) * sin(theta / 2.0) * sin(phi / 2.0);

  m_r_aileron_deflection = config_doc["init_conditions"]["aileron_r_deflection"].GetFloat();
  m_l_aileron_deflection = config_doc["init_conditions"]["aileron_l_deflection"].GetFloat();
  m_rudder_deflection = config_doc["init_conditions"]["rudder_deflection"].GetFloat();
  m_elevator_deflection = config_doc["init_conditions"]["elevator_deflection"].GetFloat();
  m_throttle = config_doc["init_conditions"]["throttle"].GetFloat();
}

std::tuple<float, float> MAVDynamicsModel::get_thrust_and_torque(float throttle, float airspeed) {
  float volts_in = m_max_volts_motor * throttle;
  float propeller_speed = get_propeller_speed(volts_in, airspeed);
  std::cout << "volts_max: " << m_max_volts_motor << std::endl;
  std::cout << "throttle: " << throttle << std::endl;
  std::cout << "volts_in: " << volts_in << std::endl;
  std::cout << "propeller_speed: " << propeller_speed << std::endl;
  float thrust = get_thrust(propeller_speed, airspeed);
  float torque = get_torque(propeller_speed, airspeed);
  return {thrust, torque};
}

float MAVDynamicsModel::get_propeller_speed(float volts_in, float airspeed) {
  float a = ((m_air_density * std::pow(m_propeller_diameter, 5)) / std::pow(2 * M_PI, 2)) * m_C_Q0;
  float b = ((m_air_density * std::pow(m_propeller_diameter, 4)) / (2 * M_PI)) * m_C_Q1 * airspeed + ((m_KQ * m_KV) / m_motor_winding_resistance);
  float c = m_air_density * std::pow(m_propeller_diameter, 3) * m_C_Q2 * std::pow(airspeed, 2) - (m_KQ / m_motor_winding_resistance) * volts_in + m_KQ * m_no_load_current;
  return (-b + sqrtf(b * b - 4 * a * c)) / (2 * a);
}

float MAVDynamicsModel::get_thrust(float propeller_speed, float airspeed){
	float thrust = ((m_air_density * std::pow(m_propeller_diameter, 4) * m_C_T0) / (4 * std::pow(M_PI, 2))) * std::pow(propeller_speed, 2) + 
		((m_air_density * std::pow(m_propeller_diameter, 3) * m_C_T1 * airspeed) / (2 * M_PI)) * propeller_speed + 
		(m_air_density * std::pow(m_propeller_diameter, 2) * m_C_T2 * std::pow(airspeed, 2));
        return thrust;
}

float MAVDynamicsModel::get_torque(float propeller_speed, float airspeed){
	return ((m_air_density * std::pow(m_propeller_diameter, 5) * m_C_Q0) / (4 * std::pow(M_PI, 2))) * std::pow(propeller_speed, 2) + 
		((m_air_density * std::pow(m_propeller_diameter, 4) * m_C_Q1 * airspeed) / (2 * M_PI)) * propeller_speed + 
		(m_air_density * std::pow(m_propeller_diameter, 3) * m_C_Q2 * std::pow(airspeed, 2));
}

float MAVDynamicsModel::get_drag_coefficent(float angle_of_attack){
  assert(m_oswald_efficency != 0);
  assert(m_wing_aspect_ratio != 0);
  assert(M_PI * m_oswald_efficency * m_wing_aspect_ratio != 0);
  return (m_C_Dp + (std::pow(m_C_L0, 2) / (M_PI * m_oswald_efficency * m_wing_aspect_ratio))) +
        ((2 * m_C_L0 * m_C_L_alpha) / (M_PI * m_oswald_efficency * m_wing_aspect_ratio)) * angle_of_attack +
        (std::pow(m_C_L_alpha, 2) / (M_PI * m_oswald_efficency * m_wing_aspect_ratio)) * std::pow(angle_of_attack, 2);
}

float MAVDynamicsModel::get_lift_coefficent(float angle_of_attack){
  float sigmoid = (1 + std::pow(M_Ef, -m_M * (angle_of_attack - m_alpha0)) + std::pow(M_Ef, m_M * (angle_of_attack + m_alpha0))) / 
                  ((1 + std::pow(M_Ef, -m_M * (angle_of_attack - m_alpha0))) * (1 + std::pow(M_Ef, m_M * (angle_of_attack + m_alpha0))));
  return (1 - sigmoid) * (m_C_L0 + m_C_L_alpha * angle_of_attack) + sigmoid * (2 * (1 ? std::signbit(angle_of_attack) : 0) * std::pow(sinf(angle_of_attack), 2) * cosf(angle_of_attack));
}

float MAVDynamicsModel::get_aileron_deflection(){
  return 0.0;
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
