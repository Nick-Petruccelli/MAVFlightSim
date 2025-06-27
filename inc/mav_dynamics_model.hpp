#pragma once
#include <raylib.h>
#include <raymath.h>
#include <string>
#include <rapidjson/document.h>

class MAVDynamicsModel {
public:
	MAVDynamicsModel(std::string config_path = std::string("resources/config.json"));
	void apply_force(float dt);
	Vector3 get_pos();
	Vector3 get_rot();
private:
	void validate_config_file(rapidjson::Document& config_doc);
	void set_sim_params(rapidjson::Document& config_doc);

	std::tuple<float, float> get_thrust_and_torque(float airspeed, float throttle);
	float get_propeller_speed(float volts_in, float airspeed);
	float get_thrust(float propeller_speed, float airspeed);
	float get_torque(float propeller_speed, float airspeed);
	float get_drag_coefficent(float angle_of_attack);
	float get_lift_coefficent(float angle_of_attack);
	float get_aileron_deflection();
	float get_airspeed(float u_r, float v_r, float w_r);
	float get_angle_of_attack(float u_r, float w_r);
	float get_sideslip(float u_r, float v_r, float w_r);

	float get_wind_u();
	float get_wind_v();
	float get_wind_w();

	void euler_step(float dt, float fx, float fy, float fz, float Mx, float My, float Mz);

	//Environ params
	float m_gravity = 9.8;
	float m_air_density;

	//Airframe Params
	float m_mass;
	float m_Jx;
	float m_Jy;
	float m_Jz;
	float m_Jxz;
	float m_oswald_efficency;
	float m_wingspan;
	float m_mean_chord;
	float m_planform_area;
	float m_wing_aspect_ratio;
	float m_gamma;
	float m_gamma1;
	float m_gamma2;
	float m_gamma3;
	float m_gamma4;
	float m_gamma5;
	float m_gamma6;
	float m_gamma7;
	float m_gamma8;

	//Aerodynamic coefficents
	float m_C_L0;
	float m_C_D0;
	float m_C_My_0;
	float m_C_L_alpha;
	float m_C_D_alpha;
	float m_C_My_alpha;
	float m_C_Lq;
	float m_C_L_delta_e;
	float m_C_Dp;
	float m_C_Dq;
	float m_C_D_delta_e;
	float m_C_Y0;
	float m_C_Yr;
	float m_C_Yp;
	float m_C_Y_beta;
	float m_C_Y_delta_r;
	float m_C_Y_delta_a;
	float m_alpha0;
	float m_M;
	float m_C_Mx_0;
	float m_C_Mx_r;
	float m_C_Mx_p;
	float m_C_Mx_beta;
	float m_C_Mx_delta_r;
	float m_C_Mx_delta_a;
	float m_C_My_q;
	float m_C_My_delta_e;
	float m_C_Mz_0;
	float m_C_Mz_p;
	float m_C_Mz_r;
	float m_C_Mz_beta;
	float m_C_Mz_delta_a;
	float m_C_Mz_delta_r;


	//Motor and propeller params
	float m_throttle = 0;
	float m_max_volts_motor;
	float m_C_Q0;
	float m_C_Q1;
	float m_C_Q2;
	float m_C_T0;
	float m_C_T1;
	float m_C_T2;
	float m_KQ;
	float m_KV;
	float m_motor_winding_resistance;
	float m_no_load_current;
	float m_propeller_diameter;

	//Control surface state vars
	float m_elevator_deflection;
	float m_r_aileron_deflection;
	float m_l_aileron_deflection;
	float m_rudder_deflection;

	//State vars
	float m_pn;
	float m_pe;
	float m_pd;

	float m_u;
	float m_v;
	float m_w;

	float m_e0;
	float m_ex;
	float m_ey;
	float m_ez;

	float m_p;
	float m_q;
	float m_r;
};
